// quad_resolver.h — persistent 4-corner identity + rigid reconstruction.
//
// WHY THIS EXISTS
// ---------------
// OpenFIRE's OF_Square_Advanced has NO persistent identity. It re-sorts the
// four points geometrically on every frame (OF_Square_Advanced.cpp:409-414),
// with swaps decided by ~1-unit differences. So the moment one corner drops
// during motion, the remaining three shift, the sort reassigns labels, the
// seen-mask changes, and its kinematic spring loads offset = predicted -
// observed and bleeds it off at 1 unit/frame (:500-516) -- roughly 70 frames of
// held error per event. That is the "locking is lost on one point and it throws
// off everything, point identity gets passed to the next one" failure exactly.
//
// It also cannot do better with what it is given: it sees four anonymous points
// once per frame. We see the raw blob stream at 135 fps with velocity
// continuity, so identity is cheap here and impossible there.
//
// WHY THIS IS NOT A MULTI-STAGE TRACKER
// ------------------------------------------
// That attempt failed by stacking a dozen interacting stages -- peak
// hysteresis, promotion, incumbency, seat reservation, coasting, ghosts,
// resurrection, deskew -- until no one could say which one moved a point. This
// has exactly three moving parts: associate, reconstruct, learn. Every one of
// them is closed-form and unit-tested on synthetic geometry (test/
// test_quad_resolver.cpp), with no hardware in the loop.
//
// THE IDEA THE OLD ONE MISSED
// ---------------------------
// The four emitters are RIGID relative to each other. So a missing corner is
// not a guess -- fit a similarity transform (rotation + uniform scale +
// translation) from the learned model to the corners we CAN see, and the
// missing ones fall out exactly. With 3 visible the fit is over-determined and
// essentially noise-free; with 2 it is exactly determined. OpenFIRE's
// parallelogram reconstruction is the crude special case of this.
//
// The model is RE-LEARNED continuously (slow EMA whenever all four are real),
// so LEDs taped to the frame today and repositioned tomorrow
// costs at most a second of convergence rather than a recalibration.
//
// CONTRACT: once locked, always emits 4 points, in a STABLE slot order, with a
// per-point `real` flag and a frame confidence. It never lets the count drop,
// because a dropped count is precisely what makes OpenFIRE fabricate geometry.

#pragma once
#include <stdint.h>

// Up to this many detected blobs may be offered per frame. The resolver picks
// the four that match its tracked corners; it does NOT assume the caller has
// already chosen the right four. That matters because the detector ranks blobs
// by MASS, and a bright piece of junk can outrank a real but dim LED -- so
// mass-ranked truncation upstream can discard the very point we are tracking.
// Identity beats mass ranking, which is the entire reason this file exists.
#define QUAD_MAX_IN 8

struct QuadPoint {
    float x, y;
    bool  real;      // false = reconstructed from the rigid model (or coasted)
};

struct QuadResult {
    QuadPoint p[4];
    int   count;      // 4 once locked; fewer only during cold start
    bool  locked;     // model is trusted
    int   n_real;     // how many of the 4 were actually detected this frame
    float confidence; // 0..1 — n_real/4 damped by how long we have extrapolated
};

struct QuadConfig {
    float gate;         // association radius in px, prediction -> blob
    float model_lr;     // EMA rate for re-learning the rig shape (0..1)
    int   lock_frames;  // consecutive all-4-real frames before trusting model
    float max_stretch;  // reject a similarity fit whose scale moves more than
                        // this factor in one frame (guards a bad association)
};

// Sensible defaults for a 240x176 sensor at ~135 fps. gate 20px ~= 650 deg/s
// pan margin; model_lr 1/64 converges in well under a second at frame rate.
QuadConfig quad_default_config(void);

void       quad_reset(const QuadConfig* cfg);   // cfg may be NULL -> defaults
QuadResult quad_update(const float* xs, const float* ys, int n);  // n <= QUAD_MAX_IN

// Telemetry, reset by the reader.
struct QuadStats {
    uint32_t frames;        // update() calls since last read
    uint32_t reconstructed; // points filled by the AFFINE fit (3+ real)
    uint32_t recon_sim;     // points filled by the similarity fit (2 real, no tilt)
    uint32_t env_rejects;   // fits refused as implausible for this rig
    uint32_t aniso_x100;    // deformation of the last reconstruction fit, x100
    uint32_t env_aniso_x100;// learned ceiling, x100 (grows as you play off-axis)
    uint32_t resid_x100;    // affine residual on the last 4-real frame, px x100
    uint32_t resid_max_x100;// worst since last read -- if this climbs, true
                            // perspective is exceeding the parallelogram model
    uint32_t coasted;       // points filled from velocity (model unusable)
    uint32_t reassoc;       // a slot re-acquired a blob after >=1 miss
    uint32_t dropped_blobs; // detected blobs that matched no slot
    uint32_t relearns;      // model EMA updates (all-4-real frames)
    uint32_t reseeds;       // identity re-acquired using the learned rig shape
    uint32_t lock_losses;   // correspondence abandoned so it could re-acquire
    uint32_t reshapes;      // locked-but-wrong assignment detected and rebuilt
    uint32_t worst_us;      // worst single quad_update(), microseconds
    uint32_t total_us;      // summed quad_update() time -- CPU share per second
};
QuadStats quad_take_stats(void);
