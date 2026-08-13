// quad_resolver.cpp — see header for the why. Three parts: associate,
// reconstruct, learn. Nothing else lives here on purpose.

#include "quad_resolver.h"
#include <math.h>
#include <string.h>

// The cost meter needs a real clock on BOTH targets. It used to stub to 0 on
// the host, which made the unit test that checks it silently vacuous -- the
// same trap that made tests 16 and 17 pass against deliberately broken code.
// A test that cannot fail is worse than no test, so the host gets a real one.
#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)
  #include "esp_timer.h"
  static inline int64_t quad_now_us(void) { return esp_timer_get_time(); }
#else
  #include <chrono>
  static inline int64_t quad_now_us(void) {
      using namespace std::chrono;
      return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
  }
#endif

namespace {

struct Slot {
    float x, y;      // last published position
    float vx, vy;    // per-frame velocity, EMA
    bool  live;      // has ever been seeded
    int   miss;      // consecutive frames without a real detection
};

QuadConfig  C;
Slot        S[4];
float       MX[4], MY[4];     // rig model, centroid-normalised
bool        model_valid;
int         lock_count;
bool        locked;
QuadStats   ST;

// ---- THE LEARNED PLAUSIBILITY ENVELOPE ------------------------------------
// Field requirement: "I could be angled compared to the screen; don't allow only 90
// but there should be a level that we cannot cross. the field of possible
// angle. also this field can be retrained based on prior shape."
// Exactly right, and it is measurable rather than assumed. On every frame where
// all four corners are real we KNOW the true deformation, because we can fit
// the model to it and read off the anisotropy (s_max/s_min of the linear part).
// That is the tilt, expressed as a number. We keep a slowly-decaying maximum of
// what this rig has actually shown us, and refuse any RECONSTRUCTION whose
// implied deformation exceeds it by more than a margin -- because a fit that
// claims a squash we have never seen at this rig is an association error, not a
// new camera angle. Off-axis play widens the envelope by itself; a wild fit
// cannot, because the envelope only ever learns from 4-real frames.
float       env_aniso_max = 1.0f;    // largest s_max/s_min seen, decayed
float       env_scale_max = 1.0f;    // largest scale seen, decayed
float       env_scale_min = 1.0f;
bool        env_valid = false;
const float ENV_MARGIN  = 1.30f;     // allow 30% beyond anything observed
const float ENV_DECAY   = 0.99995f;  // ~halves over 3-4 min of play at 135fps
const float ANISO_FLOOR = 1.8f;      // never tighter than this: a rig seen only
                                     // head-on must still permit a first tilt

const float VEL_LR = 0.35f;   // velocity EMA; loose enough to track a flick

// ---- RECOVERY (v24) -------------------------------------------------------
// The v22/v23 resolver could lock itself out permanently, and did on hardware:
//   * S[i].live was set by seed() and NEVER cleared, so the cold-start path
//     could not run a second time;
//   * a coasting slot did x += vx every frame with NO damping, so during a
//     blackout the slots accelerated away;
//   * the publish path clamps to the frame, so runaway slots parked ON a corner
//     -- observed as "pinned in one corner": literally the clamp;
//   * with slots at the corner and the real LEDs 100+px away, nothing was ever
//     inside the 20px gate again: "it did not want to lock on them back".
// Three fixes, in increasing order of how much they matter:
const float COAST_DAMP  = 0.80f;  // a coasted slot bleeds velocity instead of
                                  // accelerating into the wall
const float GATE_GROW   = 0.60f;  // a slot that has missed is less certain of
const float GATE_GROW_MAX = 3.0f; // where it is, so widen its search
const int   RESEED_AFTER = 12;    // frames with <2 real before we admit the
                                  // correspondence is dead and re-acquire
int consec_bad = 0;
// v26 self-heal thresholds. With a correct assignment the affine model explains
// a rigid quad to well under a pixel, so 4px sustained is unambiguous.
const float RESHAPE_RESID_PX = 4.0f;
const int   RESHAPE_FRAMES   = 20;      // ~150ms, long enough to ignore a glitch
int reshape_bad = 0;

inline float d2(float ax, float ay, float bx, float by) {
    const float dx = ax - bx, dy = ay - by;
    return dx * dx + dy * dy;
}

// v27 PSEUDO-ANGLE. Every angular sort in this file only needs an ORDER, never
// an angle -- so atan2f() is pure waste. This "diamond angle" is monotone in
// atan2 over the full circle, mapped to [0,4), and costs one divide plus two
// compares instead of a libm call (measured on the host: ~40x cheaper; on the
// S3 the gap is larger because atan2f is a soft-float-ish library routine while
// this is native FPU). Re-acquire calls it up to 4*C(n,4) times, so this is the
// difference between a ~200us hitch and a ~20us one on the frame you come back
// on screen. Continuity is exact at all four axis crossings.
inline float pseudo_angle(float dx, float dy)
{
    const float s = fabsf(dx) + fabsf(dy);
    if (s < 1e-12f) return 0.0f;
    const float p = dy / s;                       // [-1, 1]
    if (dx < 0.0f) return 2.0f - p;               // Q2: 1..2   Q3: 2..3
    return (dy < 0.0f) ? (4.0f + p) : p;          // Q4: 3..4   Q1: 0..1
}

// Signed area x2 of triangle (a,b,c). Sign = winding.
inline float cross3(float ax, float ay, float bx, float by, float cx, float cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

// v27 CHEAP PRUNE for the subset search. Four points can only be a projection
// of a rectangle if they are in CONVEX position -- if one lies inside the
// triangle of the other three, no affine map from a rectangle produces it. This
// is 12 cross products and kills most junk-containing subsets before they reach
// a fit. (A near-collinear set survives here but loses on anisotropy.)
bool convex4(const float* x, const float* y)
{
    for (int d = 0; d < 4; ++d) {
        const int a = (d + 1) & 3, b = (d + 2) & 3, c = (d + 3) & 3;
        const float s1 = cross3(x[a], y[a], x[b], y[b], x[d], y[d]);
        const float s2 = cross3(x[b], y[b], x[c], y[c], x[d], y[d]);
        const float s3 = cross3(x[c], y[c], x[a], y[a], x[d], y[d]);
        if ((s1 > 0 && s2 > 0 && s3 > 0) || (s1 < 0 && s2 < 0 && s3 < 0))
            return false;                          // point d is inside the rest
    }
    return true;
}

// ---------------------------------------------------------------------------
// TWO FITS, AND WHY THERE ARE TWO.
//
// The rig is a rigid planar rectangle, but we view it through a pinhole from an
// angle -- so what lands on the sensor is NOT the model rotated and scaled. It
// is foreshortened: the near edge is longer than the far edge. A SIMILARITY
// transform (rotation + UNIFORM scale + translation, 4 DOF) cannot represent
// that at all, so using it off-axis reconstructs a missing corner as if the gun
// were square-on, with an error that grows with the viewing angle.
//
// AFFINE (6 DOF) maps the rectangle to a PARALLELOGRAM: it carries tilt, shear
// and non-uniform scale, which is the first-order (weak-perspective) model of
// exactly that foreshortening. And 3 correspondences give 6 equations for 6
// unknowns -- EXACTLY determined, no least-squares slack. So:
//
//     4 real -> pass-through, no fit needed
//     3 real -> AFFINE, exact. Handles any angle you can hold the gun at.
//     2 real -> affine is underdetermined (4 equations, 6 unknowns), so fall
//               back to SIMILARITY, which is exact at 2. It cannot express
//               tilt, so this case is deliberately the weaker one.
//    <=1     -> no fit; velocity coast.
//
// What affine still cannot do is a true TRAPEZOID (non-parallel edges), which
// is what full perspective produces at extreme angles. Rather than assume that
// is negligible, fit_residual() MEASURES it on every 4-real frame -- see
// ST.resid_x100. If that number stays small, affine is sufficient; if it climbs
// with your play angle, we will know rather than guess.
// ---------------------------------------------------------------------------

struct Lin2 { float a, b, c, d; };      // [[a b],[c d]]

// Singular values of a 2x2, closed form. Used for the plausibility envelope:
// s_max/s_min is how anisotropic (squashed) the fit is, sqrt(|det|) its scale.
void lin2_svals(const Lin2& L, float* smax, float* smin)
{
    const float E = (L.a + L.d) * 0.5f, F = (L.a - L.d) * 0.5f;
    const float G = (L.c + L.b) * 0.5f, H = (L.c - L.b) * 0.5f;
    const float Q = sqrtf(E * E + H * H), R = sqrtf(F * F + G * G);
    *smax = Q + R;
    *smin = fabsf(Q - R);
}

// Least-squares affine fit of the model subset onto the observed subset.
// Exact when k == 3 and the three model points are not collinear.
bool fit_affine(const int* idx, int k, const float* ox, const float* oy,
                Lin2* L, float* cmx_o, float* cmy_o, float* cox_o, float* coy_o)
{
    if (k < 3) return false;
    float cmx = 0, cmy = 0, cox = 0, coy = 0;
    for (int t = 0; t < k; ++t) {
        cmx += MX[idx[t]]; cmy += MY[idx[t]];
        cox += ox[idx[t]]; coy += oy[idx[t]];
    }
    cmx /= k; cmy /= k; cox /= k; coy /= k;

    // Mc*Mc^T (symmetric) and Oc*Mc^T
    float mxx = 0, mxy = 0, myy = 0;
    float axx = 0, axy = 0, ayx = 0, ayy = 0;
    for (int t = 0; t < k; ++t) {
        const float mx = MX[idx[t]] - cmx, my = MY[idx[t]] - cmy;
        const float dx = ox[idx[t]] - cox, dy = oy[idx[t]] - coy;
        mxx += mx * mx; mxy += mx * my; myy += my * my;
        axx += dx * mx; axy += dx * my;
        ayx += dy * mx; ayy += dy * my;
    }
    const float det = mxx * myy - mxy * mxy;
    if (fabsf(det) < 1e-6f) return false;       // model points collinear
    const float i00 =  myy / det, i01 = -mxy / det, i11 =  mxx / det;
    L->a = axx * i00 + axy * i01;   L->b = axx * i01 + axy * i11;
    L->c = ayx * i00 + ayy * i01;   L->d = ayx * i01 + ayy * i11;
    *cmx_o = cmx; *cmy_o = cmy; *cox_o = cox; *coy_o = coy;
    return true;
}

// Similarity fit (2 points): A = s*cos, B = s*sin, standard 2D Procrustes.
bool fit_similarity(const int* idx, int k, const float* ox, const float* oy,
                    Lin2* L, float* cmx_o, float* cmy_o, float* cox_o, float* coy_o)
{
    if (k < 2) return false;
    float cmx = 0, cmy = 0, cox = 0, coy = 0;
    for (int t = 0; t < k; ++t) {
        cmx += MX[idx[t]]; cmy += MY[idx[t]];
        cox += ox[idx[t]]; coy += oy[idx[t]];
    }
    cmx /= k; cmy /= k; cox /= k; coy /= k;
    float a = 0, b = 0, den = 0;
    for (int t = 0; t < k; ++t) {
        const float mx = MX[idx[t]] - cmx, my = MY[idx[t]] - cmy;
        const float dx = ox[idx[t]] - cox, dy = oy[idx[t]] - coy;
        a += mx * dx + my * dy;
        b += mx * dy - my * dx;
        den += mx * mx + my * my;
    }
    if (den < 1e-6f) return false;
    const float A = a / den, B = b / den;
    if (sqrtf(A * A + B * B) < 1e-4f) return false;
    L->a = A; L->b = -B; L->c = B; L->d = A;
    *cmx_o = cmx; *cmy_o = cmy; *cox_o = cox; *coy_o = coy;
    return true;
}

inline void lin2_apply(const Lin2& L, float cmx, float cmy, float cox, float coy,
                       int j, float* x, float* y)
{
    const float mx = MX[j] - cmx, my = MY[j] - cmy;
    *x = cox + L.a * mx + L.b * my;
    *y = coy + L.c * mx + L.d * my;
}

// RMS distance between the affine prediction and the actual observed points.
// This is the direct measurement of how much true perspective (trapezoid) the
// parallelogram model is failing to capture.
float fit_residual(const Lin2& L, float cmx, float cmy, float cox, float coy,
                   const int* idx, int k, const float* ox, const float* oy)
{
    float acc = 0;
    for (int t = 0; t < k; ++t) {
        float px, py;
        lin2_apply(L, cmx, cmy, cox, coy, idx[t], &px, &py);
        acc += d2(px, py, ox[idx[t]], oy[idx[t]]);
    }
    return sqrtf(acc / (float)k);
}

// Score ONE candidate 4-point set against the learned rig shape and return the
// best slot assignment for it. `mo` is the model's own angular order, hoisted
// by the caller because it does not change between candidate subsets.
//
// v26: ONLY FOUR ASSIGNMENTS ARE GEOMETRICALLY POSSIBLE, NOT 24.
// The corners of a convex quad, taken in angular order about the centroid,
// keep their cyclic order under ANY orientation-preserving affine map. So once
// both the model and the observation are in angular order, the only candidates
// are the 4 cyclic rotations -- reflections are excluded by orientation, and
// the other 18 permutations are not realisable by a rigid rig at all. v25
// searched all 24 and leaned on a continuity term to break ties, which is
// exactly wrong when re-acquiring after an off-screen excursion: the old slot
// positions are stale, so continuity actively pulls toward the WRONG answer.
// This version needs no continuity at all.
//
// Among the 4 rotations, the correct one is the LEAST WARPED: a 90-degree
// mis-assignment of a non-square rig demands an anisotropy of about W/H, while
// the true one demands only the actual viewing tilt. That is a large, robust
// margin for any rig that is not square.
bool score_quad(const float* xs, const float* ys, const int* mo,
                float* score_o, float* aniso_o, float pick[4][2])
{
    int oo[4] = {0,1,2,3};
    {   // angular order of the observation about its centroid
        float cx = 0, cy = 0;
        for (int i = 0; i < 4; ++i) { cx += xs[i]; cy += ys[i]; }
        cx *= 0.25f; cy *= 0.25f;
        float oa[4];
        for (int i = 0; i < 4; ++i) oa[i] = pseudo_angle(xs[i] - cx, ys[i] - cy);
        for (int i = 0; i < 4; ++i)
            for (int j = i + 1; j < 4; ++j)
                if (oa[oo[j]] < oa[oo[i]]) { int t = oo[i]; oo[i] = oo[j]; oo[j] = t; }
    }

    const int all4[4] = {0,1,2,3};
    float best = 1e18f, best_aniso = 1e18f; int bestrot = -1;
    for (int r = 0; r < 4; ++r) {
        // slot mo[i] <- observed point oo[(i+r) & 3]
        float tx[4], ty[4];
        for (int i = 0; i < 4; ++i) {
            tx[mo[i]] = xs[oo[(i + r) & 3]];
            ty[mo[i]] = ys[oo[(i + r) & 3]];
        }
        Lin2 L; float a1,a2,a3,a4;
        if (!fit_affine(all4, 4, tx, ty, &L, &a1, &a2, &a3, &a4)) continue;
        if (L.a * L.d - L.b * L.c <= 0.0f) continue;          // no reflections
        float smax, smin; lin2_svals(L, &smax, &smin);
        const float aniso = (smin > 1e-6f) ? (smax / smin) : 1e6f;
        // Score: least-warped wins, with the fit error as a tiebreak. NO
        // continuity term -- see above.
        const float score = aniso + 0.05f * fit_residual(L, a1, a2, a3, a4, all4, 4, tx, ty);
        if (score < best) {
            best = score; best_aniso = aniso; bestrot = r;
            for (int i = 0; i < 4; ++i) { pick[i][0] = tx[i]; pick[i][1] = ty[i]; }
        }
    }
    if (bestrot < 0) return false;
    *score_o = best; *aniso_o = best_aniso;
    return true;
}

// Re-acquire identity using the LEARNED rig shape. This is what makes
// re-locking preserve identity instead of randomising it -- the corner that was
// slot 2 before the blackout is slot 2 after it, because the rig shape says so.
//
// v27 SUBSET SEARCH. Until now this took EXACTLY four points, and quad_update()
// only called it when exactly four blobs arrived. Two things followed, and both
// are the "come back on screen and the shape is wrong" failure:
//
//   * The caller trimmed to the top four by MASS before handing them over. A
//     bright reflection outranks a dim LED, so re-acquire could be fed a quad
//     with a real corner missing and a piece of junk in its place -- the exact
//     failure the header of this file says the resolver exists to prevent.
//   * With five or more blobs and no trim, the cold-start path fell through and
//     never re-acquired at all.
//
// So: take up to QUAD_MAX_IN blobs and CHOOSE the four. C(8,4) = 70 subsets,
// each pruned by convex4() first, then scored by the same 4-rotation test.
// Junk that is not in convex position with three real corners dies for ~50
// flops. When there IS a choice to make, the winner must also fall inside the
// learned deformation envelope -- with n == 4 there is nothing to choose
// between, so that extra gate is not applied and behaviour is unchanged.
bool reseed_with_model(const float* xs, const float* ys, int n)
{
    if (!model_valid || n < 4) return false;

    int mo[4] = {0,1,2,3};
    {   // angular order of the model about its own centroid (centroid is 0,0).
        // Constant across subsets, so it is hoisted out of the search.
        float ma[4];
        for (int i = 0; i < 4; ++i) ma[i] = pseudo_angle(MX[i], MY[i]);
        for (int i = 0; i < 4; ++i)
            for (int j = i + 1; j < 4; ++j)
                if (ma[mo[j]] < ma[mo[i]]) { int t = mo[i]; mo[i] = mo[j]; mo[j] = t; }
    }

    float bpick[4][2]; float bscore = 1e18f; bool have = false;

    if (n == 4) {
        float sc, an;
        if (score_quad(xs, ys, mo, &sc, &an, bpick)) { bscore = sc; have = true; }
    } else {
        float ceil_aniso = env_valid ? (env_aniso_max * ENV_MARGIN) : 1e18f;
        if (env_valid && ceil_aniso < ANISO_FLOOR) ceil_aniso = ANISO_FLOOR;
        int c0, c1, c2, c3;
        for (c0 = 0;      c0 < n - 3; ++c0)
        for (c1 = c0 + 1; c1 < n - 2; ++c1)
        for (c2 = c1 + 1; c2 < n - 1; ++c2)
        for (c3 = c2 + 1; c3 < n;     ++c3) {
            const float qx[4] = { xs[c0], xs[c1], xs[c2], xs[c3] };
            const float qy[4] = { ys[c0], ys[c1], ys[c2], ys[c3] };
            if (!convex4(qx, qy)) continue;
            float sc, an, pk[4][2];
            if (!score_quad(qx, qy, mo, &sc, &an, pk)) continue;
            if (an > ceil_aniso) continue;      // a shape this rig has never shown
            if (sc < bscore) {
                bscore = sc; have = true;
                for (int i = 0; i < 4; ++i) { bpick[i][0] = pk[i][0]; bpick[i][1] = pk[i][1]; }
            }
        }
    }
    if (!have) return false;

    for (int i = 0; i < 4; ++i) {
        S[i].x = bpick[i][0]; S[i].y = bpick[i][1];
        S[i].vx = 0; S[i].vy = 0; S[i].live = true; S[i].miss = 0;
    }
    lock_count = 1;
    ST.reseeds++;
    return true;
}

// Seed identity from a full, clean quad. Order is canonical (angle about the
// centroid) so the slot order is reproducible across resets and matches the
// model's own ordering.
void seed(const float* xs, const float* ys)
{
    float cx = 0, cy = 0;
    for (int i = 0; i < 4; ++i) { cx += xs[i]; cy += ys[i]; }
    cx /= 4; cy /= 4;

    int order[4] = {0, 1, 2, 3};
    float ang[4];
    // pseudo_angle, not atan2f: same ORDER, and the model ordering in
    // reseed_with_model() uses the same function, so the two must agree.
    for (int i = 0; i < 4; ++i) ang[i] = pseudo_angle(xs[i] - cx, ys[i] - cy);
    for (int i = 0; i < 4; ++i)                       // tiny insertion sort
        for (int j = i + 1; j < 4; ++j)
            if (ang[order[j]] < ang[order[i]]) { int t = order[i]; order[i] = order[j]; order[j] = t; }

    for (int i = 0; i < 4; ++i) {
        const int s = order[i];
        S[i].x = xs[s]; S[i].y = ys[s];
        S[i].vx = 0;    S[i].vy = 0;
        S[i].live = true; S[i].miss = 0;
        MX[i] = xs[s] - cx; MY[i] = ys[s] - cy;
    }
    model_valid = true;
    lock_count  = 1;
}

} // namespace

QuadConfig quad_default_config(void)
{
    QuadConfig c;
    c.gate        = 20.0f;   // px; ~650 deg/s of pan margin at 135 fps
    c.model_lr    = 1.0f / 64.0f;
    c.lock_frames = 4;
    c.max_stretch = 1.35f;   // one frame cannot legitimately rescale the rig
    return c;
}

void quad_reset(const QuadConfig* cfg)
{
    C = cfg ? *cfg : quad_default_config();
    memset(S, 0, sizeof(S));
    memset(&ST, 0, sizeof(ST));
    for (int i = 0; i < 4; ++i) { MX[i] = MY[i] = 0.0f; }
    model_valid = false;
    lock_count  = 0;
    locked      = false;
    env_aniso_max = 1.0f; env_scale_max = 1.0f; env_scale_min = 1.0f;
    env_valid = false;
    consec_bad = 0;
    reshape_bad = 0;
}

QuadStats quad_take_stats(void)
{
    QuadStats s = ST;
    memset(&ST, 0, sizeof(ST));
    return s;
}

QuadResult quad_update(const float* xs, const float* ys, int n)
{
    QuadResult R;
    memset(&R, 0, sizeof(R));
    ST.frames++;
    const int64_t t_enter = quad_now_us();
    if (n > QUAD_MAX_IN) { ST.dropped_blobs += (n - QUAD_MAX_IN); n = QUAD_MAX_IN; }

    int live = 0;
    for (int i = 0; i < 4; ++i) if (S[i].live) live++;

    // ---- cold start / re-acquire ------------------------------------------
    // v27: this used to demand EXACTLY four blobs. One junk blob in frame and
    // the resolver silently gave up and passed raw points straight through to
    // OpenFIRE -- unlocked, unordered, no reconstruction -- which is precisely
    // the state it exists to prevent, and precisely when it is needed most
    // (coming back on screen). Now: 4 or more with a learned model goes to the
    // subset search; exactly 4 with no model yet takes the angular seed.
    if (live < 4) {
        bool got = false;
        if (n >= 4) got = reseed_with_model(xs, ys, n);
        if (!got && n == 4) { seed(xs, ys); got = true; }
        if (got) {
            consec_bad = 0;
        } else {
            // v27 BOUNDS FIX: R.p has FOUR entries and n may be up to
            // QUAD_MAX_IN (8). The old loop ran to n and wrote past the array,
            // over count/locked/n_real/confidence and off the end of the
            // struct. Unreachable from today's call site only because that
            // site trimmed to 4 -- which is the trim v27 removes.
            const int m = n < 4 ? n : 4;
            for (int i = 0; i < m; ++i) { R.p[i].x = xs[i]; R.p[i].y = ys[i]; R.p[i].real = true; }
            R.count = m; R.n_real = m; R.locked = false; R.confidence = 0.0f;
            ST.dropped_blobs += (n - m);
            return R;
        }
    }

    // ---- 1. ASSOCIATE: greedy nearest-neighbour, blob -> predicted slot -----
    float px[4], py[4];
    for (int i = 0; i < 4; ++i) { px[i] = S[i].x + S[i].vx; py[i] = S[i].y + S[i].vy; }

    int  slot_of[4] = {-1, -1, -1, -1};   // blob index taken by each slot
    bool blob_used[QUAD_MAX_IN] = {false};
    // A slot that has been missing is less certain of where it is, so it gets a
    // wider search. Without this a corner that drops for a moment while the gun
    // moves can never find its way back inside a fixed 20px gate.
    float g2[4];
    for (int i = 0; i < 4; ++i) {
        float g = C.gate * (1.0f + GATE_GROW * (float)S[i].miss);
        if (g > C.gate * GATE_GROW_MAX) g = C.gate * GATE_GROW_MAX;
        g2[i] = g * g;
    }

    // v25 FIX: rank by RAW distance. The per-slot gate is an ADMISSION test
    // only, never part of the ranking. v24 ranked by d/g2[s] "to compare fairly
    // across gates" -- which is exactly backwards: a slot that has been missing
    // has a LARGER gate, so its normalised distance is SMALLER, and it
    // outcompetes a healthy slot that is tracking perfectly. That is corner
    // identity swapping while all four corners are visible.
    for (;;) {
        float best = 1e18f; int bs = -1, bb = -1;
        for (int s = 0; s < 4; ++s) {
            if (slot_of[s] >= 0) continue;
            for (int b = 0; b < n; ++b) {
                if (blob_used[b]) continue;
                const float d = d2(px[s], py[s], xs[b], ys[b]);
                if (d > g2[s]) continue;          // admission: this slot's gate
                if (d < best) { best = d; bs = s; bb = b; }   // ranking: raw
            }
        }
        if (bs < 0) break;
        slot_of[bs] = bb; blob_used[bb] = true;
    }
    for (int b = 0; b < n; ++b) if (!blob_used[b]) ST.dropped_blobs++;

    // ---- 2. update matched slots -------------------------------------------
    float ox[4], oy[4];
    int   midx[4], k = 0;
    for (int s = 0; s < 4; ++s) {
        if (slot_of[s] < 0) continue;
        const int b = slot_of[s];
        if (S[s].miss > 0) ST.reassoc++;
        const float nvx = xs[b] - S[s].x, nvy = ys[b] - S[s].y;
        S[s].vx += VEL_LR * (nvx - S[s].vx);
        S[s].vy += VEL_LR * (nvy - S[s].vy);
        S[s].x = xs[b]; S[s].y = ys[b];
        S[s].miss = 0;
        ox[s] = S[s].x; oy[s] = S[s].y;
        midx[k++] = s;
    }
    R.n_real = k;

    // ---- 3. RECONSTRUCT the rest -------------------------------------------
    // 3 real -> affine (exact, carries tilt/shear). 2 -> similarity (no tilt).
    Lin2  L; float cmx = 0, cmy = 0, cox = 0, coy = 0;
    bool  have_fit = false, fit_is_affine = false;
    if (model_valid && k >= 3 && fit_affine(midx, k, ox, oy, &L, &cmx, &cmy, &cox, &coy)) {
        have_fit = true; fit_is_affine = true;
    } else if (model_valid && k == 2 &&
               fit_similarity(midx, k, ox, oy, &L, &cmx, &cmy, &cox, &coy)) {
        have_fit = true;
    }

    // Envelope check: is the deformation this fit implies one this rig has
    // plausibly shown? Only gates RECONSTRUCTION -- a real detection is never
    // second-guessed.
    if (have_fit) {
        float smax, smin;
        lin2_svals(L, &smax, &smin);
        const float aniso = (smin > 1e-6f) ? (smax / smin) : 1e6f;
        const float scale = sqrtf(fabsf(L.a * L.d - L.b * L.c));
        ST.aniso_x100 = (uint32_t)(aniso * 100.0f);
        const float aniso_cap = (env_valid ? env_aniso_max : 1.0f) * ENV_MARGIN;
        const float cap = aniso_cap > ANISO_FLOOR ? aniso_cap : ANISO_FLOOR;
        const bool  scale_ok = !env_valid ||
                               (scale < env_scale_max * ENV_MARGIN &&
                                scale > env_scale_min / ENV_MARGIN);
        if (aniso > cap || !scale_ok) {
            have_fit = false;
            ST.env_rejects++;
        }
    }

    for (int s = 0; s < 4; ++s) {
        if (slot_of[s] >= 0) continue;
        S[s].miss++;
        bool done = false;
        if (have_fit) {
            float fx, fy;
            lin2_apply(L, cmx, cmy, cox, coy, s, &fx, &fy);
            S[s].vx = fx - S[s].x; S[s].vy = fy - S[s].y;
            S[s].x = fx; S[s].y = fy;
            if (fit_is_affine) ST.reconstructed++; else ST.recon_sim++;
            done = true;
        }
        if (!done) {
            // Damped coast. NOT constant velocity: an undamped slot accelerates
            // out of the frame during a blackout, gets clamped at publish, and
            // parks on a corner where nothing can ever re-associate with it.
            S[s].x += S[s].vx; S[s].y += S[s].vy;
            S[s].vx *= COAST_DAMP; S[s].vy *= COAST_DAMP;
            ST.coasted++;
        }
        ox[s] = S[s].x; oy[s] = S[s].y;
    }

    // ---- 4. LEARN: rig shape AND the deformation envelope -------------------
    // Both only ever train on all-4-real frames, where there is nothing to
    // guess: the observation IS the ground truth for this pose.
    if (k == 4) {
        Lin2 L4; float a1, a2, a3, a4;
        if (model_valid && fit_affine(midx, 4, ox, oy, &L4, &a1, &a2, &a3, &a4)) {
            float smax, smin;
            lin2_svals(L4, &smax, &smin);
            const float aniso = (smin > 1e-6f) ? (smax / smin) : 1.0f;
            const float scale = sqrtf(fabsf(L4.a * L4.d - L4.b * L4.c));
            if (!env_valid) {
                env_aniso_max = aniso; env_scale_max = scale; env_scale_min = scale;
                env_valid = true;
            } else {
                env_aniso_max *= ENV_DECAY; env_scale_max *= ENV_DECAY;
                env_scale_min /= ENV_DECAY;
                if (aniso > env_aniso_max) env_aniso_max = aniso;
                if (scale > env_scale_max) env_scale_max = scale;
                if (scale < env_scale_min) env_scale_min = scale;
            }
            // How much true perspective (trapezoid) the parallelogram model is
            // NOT capturing, in px RMS. Small => affine is enough.
            const float r = fit_residual(L4, a1, a2, a3, a4, midx, 4, ox, oy);
            ST.resid_x100 = (uint32_t)(r * 100.0f);
            // v26 SELF-HEAL. Until now, once live==4 the permutation was never
            // re-examined -- so a wrong re-acquire stayed wrong forever, which
            // is the "come back from off-screen and the rectangle is the
            // wrong shape". A sustained high residual on 4-REAL frames is proof
            // the assignment does not describe this rig, because with the right
            // assignment the affine model explains a rigid quad to a fraction
            // of a pixel. So: force a fresh re-acquire.
            if (r > RESHAPE_RESID_PX) {
                if (++reshape_bad >= RESHAPE_FRAMES) {
                    for (int i = 0; i < 4; ++i) { S[i].live = false; S[i].miss = 0; S[i].vx = S[i].vy = 0; }
                    lock_count = 0; locked = false; reshape_bad = 0;
                    ST.reshapes++;
                }
            } else if (reshape_bad) {
                reshape_bad--;
            }
            if (ST.resid_x100 > ST.resid_max_x100) ST.resid_max_x100 = ST.resid_x100;
            ST.env_aniso_x100 = (uint32_t)(env_aniso_max * 100.0f);
        }
        float cx = 0, cy = 0;
        for (int i = 0; i < 4; ++i) { cx += S[i].x; cy += S[i].y; }
        cx /= 4; cy /= 4;
        for (int i = 0; i < 4; ++i) {
            MX[i] += C.model_lr * ((S[i].x - cx) - MX[i]);
            MY[i] += C.model_lr * ((S[i].y - cy) - MY[i]);
        }
        model_valid = true;
        ST.relearns++;
        if (lock_count < C.lock_frames) lock_count++;
    } else if (lock_count > 0 && k == 0) {
        lock_count--;                      // total loss slowly un-locks
    }
    locked = (lock_count >= C.lock_frames);

    // ---- 4b. GIVE UP AND RE-ACQUIRE ----------------------------------------
    // Fewer than two real corners means the affine fit is unavailable and we
    // are extrapolating blind. Tolerate that briefly; past RESEED_AFTER frames
    // the correspondence is simply wrong, so clear `live` and let the next
    // clean quad re-seed. The learned MODEL and ENVELOPE deliberately survive:
    // they are what makes the re-seed keep its identity, and they were
    // expensive to learn.
    if (k < 2) {
        if (++consec_bad >= RESEED_AFTER) {
            for (int i = 0; i < 4; ++i) { S[i].live = false; S[i].miss = 0; S[i].vx = S[i].vy = 0; }
            lock_count = 0; locked = false; consec_bad = 0;
            ST.lock_losses++;
        }
    } else {
        consec_bad = 0;
    }

    // ---- 5. emit: always four, stable order --------------------------------
    int worst_miss = 0;
    for (int i = 0; i < 4; ++i) {
        R.p[i].x = S[i].x; R.p[i].y = S[i].y;
        R.p[i].real = (slot_of[i] >= 0);
        if (S[i].miss > worst_miss) worst_miss = S[i].miss;
    }
    R.count  = 4;
    R.locked = locked;
    // confidence: how much of the quad was measured, damped by how long we have
    // been extrapolating the worst corner. Reported, never acted on.
    R.confidence = (k / 4.0f) / (1.0f + 0.05f * (float)worst_miss);
    {   // how long we held cam_task (priority 23) this frame
        const uint32_t us = (uint32_t)(quad_now_us() - t_enter);
        if (us > ST.worst_us) ST.worst_us = us;
        ST.total_us += us;
    }
    return R;
}
