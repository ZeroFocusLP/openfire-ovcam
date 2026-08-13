// Host test for the OV2640 camera shim: bridge seqlock + DFRobot API semantics.
// Build: g++ -std=c++17 -O2 -I../lib/DFRobotIRPositionEx_OV2640 test_shim.cpp \
//        ../lib/DFRobotIRPositionEx_OV2640/ov2640_bridge.c -o test_shim && ./test_shim
#include <cstdio>
#include <cstring>
#include <thread>
#include <atomic>
#include "DFRobotIRPositionEx.h"

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { ++failures; \
    printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static ov2640_bridge_frame_t mk(uint8_t count, uint32_t seq) {
    ov2640_bridge_frame_t f{};
    f.count = count; f.frame_seq = seq; f.frame_w = 240; f.frame_h = 176;
    for (int i = 0; i < count; ++i) {
        f.x16[i] = (uint16_t)(100 * (i + 1));   // 100,200,300,400 in x16 units
        f.y16[i] = (uint16_t)( 80 * (i + 1));
        f.area4[i] = (uint8_t)(3 * (i + 1));
    }
    return f;
}

int main() {
    printf("== OV2640 shim host tests ==\n");

    DFRobotIRPositionEx cam;   // trivial ctor: no bus
    CHECK(cam.begin(400000, DFRobotIRPositionEx::DataFormat_Basic,
                    DFRobotIRPositionEx::Sensitivity_Default), "begin() failed");

    // (1) cold start: no frame ever published -> DataMismatch (v11: the
    // "nothing new to do" idiom OpenFIRE skips silently), nothing seen
    CHECK(cam.basicAtomic() == DFRobotIRPositionEx::Error_DataMismatch, "cold atomic err");
    CHECK(cam.seen() == 0, "cold seen=%u, want 0", cam.seen());

    // (2) publish 2 points -> read back exact values + seen mask
    ov2640_bridge_frame_t f = mk(2, 1);
    ov2640_bridge_publish(&f);
    CHECK(cam.basicAtomic() == DFRobotIRPositionEx::Error_Success, "atomic err");
    CHECK(cam.seen() == 0b11, "seen=%u want 3", cam.seen());
    // outputs scaled native-x16 -> 1023x767: x = x16*1023/(240*16-1), then
    // X pre-mirrored to the DFRobot convention (OV2640_IRPOS_MIRROR_X).
    int ex0 = 1023 - (int)(100u * 1023u / 3839u), ey0 = (int)(80u * 767u / 2815u);
    int ex1 = 1023 - (int)(200u * 1023u / 3839u), ey1 = (int)(160u * 767u / 2815u);
    CHECK(cam.x(0) == ex0 && cam.y(0) == ey0, "p0 %d,%d want %d,%d", cam.x(0), cam.y(0), ex0, ey0);
    CHECK(cam.x(1) == ex1 && cam.y(1) == ey1, "p1 %d,%d want %d,%d", cam.x(1), cam.y(1), ex1, ey1);
    CHECK(cam.x(2) == 1023 && cam.y(2) == 1023, "unseen point not at idle idiom");
    CHECK(cam.xPositions()[1] == ex1, "array accessor mismatch");
    CHECK(cam.size(0) == 3 && cam.size(3) == 0, "sizes wrong");

    // (2b) v11 NEW-FRAME GATING — the smoothness fix. Re-polling WITHOUT a new
    // published frame must report Error_DataMismatch and leave positions
    // untouched, so OpenFIRE's One Euro filter is driven once per camera frame
    // (with a real dt) instead of thousands of times per second on duplicates.
    for (int i = 0; i < 50; ++i) {
        CHECK(cam.basicAtomic() == DFRobotIRPositionEx::Error_DataMismatch,
              "duplicate poll %d did not report DataMismatch", i);
        CHECK(cam.x(0) == ex0 && cam.y(0) == ey0, "duplicate poll %d moved a point", i);
        CHECK(cam.seen() == 0b11, "duplicate poll %d changed seen", i);
    }
    // ...and the very next PUBLISHED frame is reported exactly once
    { ov2640_bridge_frame_t nf = mk(3, 7);
      ov2640_bridge_publish(&nf);
      CHECK(cam.basicAtomic() == DFRobotIRPositionEx::Error_Success, "new frame not reported");
      CHECK(cam.seen() == 0b111, "new frame seen=%u want 7", cam.seen());
      CHECK(cam.basicAtomic() == DFRobotIRPositionEx::Error_DataMismatch,
            "same frame reported twice"); }

    // (3) size clamps to the DFRobot 0..15 range
    f = mk(1, 2); f.area4[0] = 200;
    ov2640_bridge_publish(&f);
    cam.extendedAtomic();
    CHECK(cam.size(0) == 15, "size clamp: %d want 15", cam.size(0));

    // (4) NoSeen variant refreshes positions but preserves the seen mask
    f = mk(4, 3);
    ov2640_bridge_publish(&f);
    cam.basicAtomic();                       // seen = 0b1111
    f = mk(1, 4);
    ov2640_bridge_publish(&f);
    cam.availableBasicNoSeen();
    CHECK(cam.seen() == 0b1111, "NoSeen clobbered seen: %u", cam.seen());
    CHECK(cam.x(0) == ex0, "NoSeen did not refresh positions");

    // (5) seqlock under contention: hammer publishes on one thread, reads on
    // another; every read must be internally consistent (x==2*y relation held
    // per our generator) and seq must never go backwards.
    std::atomic<bool> stop{false};
    std::thread writer([&stop] {
        uint32_t s = 10;
        while (!stop.load(std::memory_order_relaxed)) {
            ov2640_bridge_frame_t w{};
            w.count = 4; w.frame_seq = ++s;
            uint16_t base = (uint16_t)(s * 7u % 2000u);
            for (int i = 0; i < 4; ++i) { w.x16[i] = base + i; w.y16[i] = (uint16_t)((base + i) * 2u); }
            ov2640_bridge_publish(&w);
        }
    });
    uint32_t last = 0; int torn = 0; long judged = 0;
    for (int n = 0; n < 200000; ++n) {
        ov2640_bridge_frame_t r{};
        uint32_t seq = ov2640_bridge_read(&r);
        // Only judge frames the stress writer produced (seq >= 11): before the
        // OS schedules the writer thread, reads legitimately return test (4)'s
        // leftover frame, whose data doesn't satisfy the y==2x relation. The
        // first version counted ~15-24k of those as "torn" — a test bug that
        // cost a debugging round (the v1/v2 bridge tears it exposed were real,
        // but the v3 ring was innocent).
        if (seq < 11) continue;
        judged++;
        if (seq < last) torn++;              // seq regression = broken publish
        last = seq;
        for (int i = 0; i < r.count; ++i)
            if (r.y16[i] != (uint16_t)(r.x16[i] * 2u)) torn++;   // torn copy (bridge is native units)
    }
    stop = true; writer.join();
    CHECK(torn == 0, "bridge: %d torn/regressed reads (%ld judged)", torn, judged);
    CHECK(judged > 100000, "stress under-ran: only %ld judged reads", judged);

    if (failures) { printf("== %d FAILURE(S) ==\n", failures); return 1; }
    printf("== ALL PASS ==\n");
    return 0;
}
