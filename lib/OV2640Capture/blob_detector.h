// Blob detector: threshold + run-length connected components + subpixel centroids.
// Pure C++ (no Arduino deps) — host-testable.
//
// TWO APIs, one engine:
//  - blobstream_*(): incremental row feeding for PHASE-2 STREAMING — called from
//    the camera driver's copy-task as DMA chunks land, so blob results are ready
//    ~0 rows after the frame ends (no separate scan pass, cache-hot rows).
//    Single instance, not reentrant. feed() processes any newly completed rows.
#pragma once
#include <cstdint>
#include <cstddef>

struct Blob {
    float cx, cy;        // intensity-weighted centroid (subpixel)
    uint32_t pixels;     // area
    uint64_t mass;       // sum of intensities (brightness ranking)
    uint8_t  peak;       // max raw pixel value in the blob (255 = clipped)
    uint32_t sat;        // pixels at >=254 — how much of the blob is flat-topped.
                         // Drives the auto-unclip servo: peak alone says "touched
                         // the ceiling once"; sat says "the centroid profile is
                         // actually destroyed".
    int16_t  bx0, bx1;   // bounding box, inclusive [bx0..bx1] x [by0..by1].
    int16_t  by0, by1;   // pixels/(w*h) = fill factor: a round dot is ~0.78,
                         // streaks/smears/split flames are low — cheap shape gate.
};

// v4: capacity raised 4 -> 8 for the point stabilizer. With only 4 slots a
// bright transient EVICTED a real LED inside the detector, before any
// downstream logic could rank candidates by track history — the teleport had
// already happened here. 8 lets the stabilizer see impostor AND victim.
constexpr int BLOB_MAX_OUT = 8;

struct BlobResult {
    Blob blobs[BLOB_MAX_OUT];  // brightest blobs, sorted by mass desc
    int count;                 // how many valid (0..BLOB_MAX_OUT)
};

// ---- streaming API ----
// px_budget: FLOOD GUARD — max bright pixels accumulated per frame; 0 = unlimited.
// On ESP32 the stream runs inside the camera driver's copy-task, whose DMA-chunk
// service deadline is ~450us: a flooded frame (low threshold / long exposure in a
// lit room) used to cost ~4-5ms of centroid math there, starving the DMA event
// queue -> EV-EOF-OVF storms -> every frame short -> capture death (field data,
// v30-v33). With a budget, a flooded frame aborts scanning early (remaining
// chunks cost ~nothing), finish() reports 0 blobs, and blobstream_flooded()
// tells the UI WHY the map is empty instead of the camera dying.
void blobstream_begin(int w, uint8_t threshold, uint32_t min_px, uint32_t max_px,
                      uint32_t px_budget = 0);
// base: frame buffer start; bytes_total: bytes valid so far (monotonic within a frame)
void blobstream_feed(const uint8_t* base, size_t bytes_total);
BlobResult blobstream_finish();
bool blobstream_flooded();   // true if the LAST finished frame blew its px budget

// ---- one-shot wrapper: TEST ONLY (test/test_labparity.cpp) ----------------
// Not called by the firmware; --gc-sections drops it from the image.
BlobResult detect_blobs(const uint8_t* frame, int w, int h,
                        uint8_t threshold, uint32_t min_px, uint32_t max_px);
