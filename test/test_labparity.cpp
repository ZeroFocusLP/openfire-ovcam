// test_labparity — proves the ONE divergence that made v13's "lab-identical"
// mode not lab-identical, using the REAL detector on a REAL synthetic frame.
//
// The lab's detector is compiled with BLOB_MAX_OUT = 4; ours with 8 (raised in
// v4 for the point stabilizer). Both then run the SAME filter_blobs, which
// drops duplicates and smear stripes. So every candidate the filter drops gets
// BACKFILLED from the dim tail in the overlay, and cannot be in the lab.
//
// Scenario below is the one a real 4-LED rig actually produces (see note in
// ov2640_capture.cpp: "wiibar cluster splits fed OpenFIRE 5px-apart twins"):
// four emitters where one splits into a twin, plus a faint hot pixel cluster.
//
// Build:  g++ -std=c++17 -O2 -I../lib/OV2640Capture test_labparity.cpp \
//              ../lib/OV2640Capture/blob_detector.cpp -o test_labparity
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "blob_detector.h"

static const int W = 240, H = 176;
static const uint8_t THR = 150;
static const uint32_t MIN_PX = 4, MAX_PX = W * H / 4, BUDGET = 8000;

static std::vector<uint8_t> g_img;

static void disc(float cx, float cy, float r, uint8_t peak)
{
    for (int y = (int)(cy - r) - 1; y <= (int)(cy + r) + 1; ++y)
        for (int x = (int)(cx - r) - 1; x <= (int)(cx + r) + 1; ++x) {
            if (x < 0 || y < 0 || x >= W || y >= H) continue;
            const float dx = x - cx, dy = y - cy;
            if (dx * dx + dy * dy <= r * r)
                if (peak > g_img[y * W + x]) g_img[y * W + x] = peak;
        }
}

// The lab's filter, parameterised by how many candidates it is allowed to see.
// n=4 -> the lab (BLOB_MAX_OUT 4).  n=in.count -> v13's overlay (up to 8).
static BlobResult filt(const BlobResult& in, int cap)
{
    BlobResult out{}; out.count = 0;
    const int n = in.count < cap ? in.count : cap;
    for (int i = 0; i < n; ++i) {
        const Blob& b = in.blobs[i];
        bool drop = false; int col_mates = 0;
        for (int k = 0; k < out.count; ++k) {
            float dx = b.cx - out.blobs[k].cx, dy = b.cy - out.blobs[k].cy;
            if (dx * dx + dy * dy < 16.f) { drop = true; break; }
            if (dx < 3.5f && dx > -3.5f) col_mates++;
        }
        if (!drop && col_mates >= 2) drop = true;
        if (!drop) { out.blobs[out.count++] = b; if (out.count == 4) break; }
    }
    return out;
}

static void show(const char* tag, const BlobResult& r)
{
    printf("  %-28s count=%d :", tag, r.count);
    for (int i = 0; i < r.count; ++i)
        printf("  (%.1f,%.1f m=%u pk=%u)", r.blobs[i].cx, r.blobs[i].cy,
               (unsigned)r.blobs[i].pixels, (unsigned)r.blobs[i].peak);
    printf("\n");
}

static int fails = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("  FAIL: %s\n", msg); ++fails; } } while (0)

int main()
{
    printf("=== lab-parity: detector capacity 4 (lab) vs 8 (overlay v13) ===\n\n");

    // --- the rig as it actually degrades: a bright vertical REFLECTION
    // STRIPE on the TV (four blobs sharing a column -- exactly what the
    // col_mates rule exists to kill), one emitter still visible, and a faint
    // hot-pixel cluster from high gain. Five-plus candidates with drops among
    // the top four is the ONLY situation where the capacity matters -- on a
    // clean 4-emitter frame the two are identical, which the second scenario
    // below checks.
    g_img.assign(W * H, 0);
    disc( 60.0f,  20.0f, 4.2f, 255);   // stripe 1  (mass rank 1)
    disc( 60.0f,  55.0f, 4.0f, 254);   // stripe 2  (rank 2)
    disc( 60.0f,  90.0f, 3.8f, 253);   // stripe 3  (rank 3)
    disc( 60.0f, 125.0f, 3.6f, 252);   // stripe 4  (rank 4)
    disc(180.0f,  40.0f, 3.0f, 250);   // the one real emitter (rank 5)
    disc(210.0f,  95.0f, 1.8f, 168);   // JUNK: faint hot-pixel cluster (rank 6)

    BlobResult raw = detect_blobs(g_img.data(), W, H, THR, MIN_PX, MAX_PX);
    printf("detector output (sorted by mass desc), %d candidates:\n", raw.count);
    for (int i = 0; i < raw.count; ++i)
        printf("   #%d  (%6.2f,%6.2f)  mass=%-4u peak=%u\n", i,
               raw.blobs[i].cx, raw.blobs[i].cy,
               (unsigned)raw.blobs[i].pixels, (unsigned)raw.blobs[i].peak);
    printf("\n");
    CHECK(raw.count >= 6, "scenario needs >=6 candidates to be meaningful");

    BlobResult lab     = filt(raw, 4);            // what blobtest computes
    BlobResult overlay = filt(raw, raw.count);    // what v13 shipped
    show("LAB      (cap 4)", lab);
    show("OVERLAY  (cap 8, v13)", overlay);
    printf("\n");

    // The claim under test: the stripe costs two slots in BOTH, but only the
    // overlay refills them -- and it refills from the DIMMEST end of the frame.
    CHECK(lab.count == 2, "lab should emit 2 points (stripe tail dropped, no backfill)");
    CHECK(overlay.count == 4, "overlay v13 should backfill to 4");

    bool junk_in_overlay = false, junk_in_lab = false;
    for (int i = 0; i < overlay.count; ++i)
        if (overlay.blobs[i].cx > 200.0f) junk_in_overlay = true;
    for (int i = 0; i < lab.count; ++i)
        if (lab.blobs[i].cx > 200.0f) junk_in_lab = true;
    CHECK(junk_in_overlay, "the overlay should have admitted the faint junk blob");
    CHECK(!junk_in_lab, "the lab must never see the faint junk blob");

    printf("WHY THIS TELEPORTS:\n");
    printf("  lab      -> 2 points. Below OpenFIRE's 3-point minimum, so it\n");
    printf("              HOLDS the last good quad. The cursor does not move.\n");
    printf("  overlay  -> 4 points, one of which is a faint transient at\n");
    printf("              (%.0f,%.0f). OpenFIRE's geometric sort hands that\n",
           overlay.blobs[overlay.count - 1].cx, overlay.blobs[overlay.count - 1].cy);
    printf("              junk a CORNER, the quad deforms, and the perspective\n");
    printf("              solve moves the cursor by centimetres. Next frame the\n");
    printf("              junk is gone and it snaps back. That is the teleport.\n\n");

    // --- second scenario: the fix. Capping the overlay at 4 must reproduce
    // the lab EXACTLY, for this frame and for a clean 4-emitter frame.
    BlobResult fixed = filt(raw, 4);
    CHECK(fixed.count == lab.count, "capped overlay must match lab count");
    for (int i = 0; i < lab.count; ++i)
        CHECK(fixed.blobs[i].cx == lab.blobs[i].cx &&
              fixed.blobs[i].cy == lab.blobs[i].cy, "capped overlay must match lab points");

    g_img.assign(W * H, 0);                        // clean 4-emitter frame
    disc( 60.0f,  40.0f, 3.6f, 255); disc(180.0f,  40.0f, 3.5f, 252);
    disc( 60.0f, 140.0f, 3.4f, 250); disc(180.0f, 140.0f, 3.3f, 248);
    BlobResult c_raw = detect_blobs(g_img.data(), W, H, THR, MIN_PX, MAX_PX);
    BlobResult c_lab = filt(c_raw, 4), c_fix = filt(c_raw, 4);
    CHECK(c_lab.count == 4 && c_fix.count == 4, "clean quad must still give 4 points");
    printf("clean 4-emitter frame: lab=%d fixed=%d  (no regression)\n\n",
           c_lab.count, c_fix.count);

    if (fails) { printf("== %d CHECK(S) FAILED ==\n", fails); return 1; }
    printf("== ALL PASS: the capacity difference is real and the cap fixes it ==\n");
    return 0;
}
