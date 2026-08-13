// test_quad_resolver.cpp — host tests for the rigid quad resolver.
//
//   g++ -std=c++17 -O2 -I../lib/OV2640Capture test_quad_resolver.cpp
//       ../lib/OV2640Capture/quad_resolver.cpp -o test_quad && ./test_quad
//
// Everything here is synthetic geometry, so the whole resolver is verifiable
// with no hardware. The scenarios are the ones that actually broke on the
// bench: a corner dropping mid-pan, identity surviving a rotation, and the
// model following the rig when a stand is repositioned.

#include "quad_resolver.h"
#include <cstdio>
#include <cmath>
#include <vector>

static int failures = 0;

static void check(bool ok, const char* what, double got = 0, double want = 0)
{
    if (ok) { printf("  PASS  %s\n", what); return; }
    printf("  FAIL  %s   (got %.3f, want %.3f)\n", what, got, want);
    failures++;
}
static void near(float got, float want, float tol, const char* what)
{
    check(fabsf(got - want) <= tol, what, got, want);
}

// A rigid rectangle, transformable so we can simulate real gun motion.
struct Rig {
    float w = 120, h = 80;
    void points(float cx, float cy, float ang, float s, float* xs, float* ys) const {
        const float hw = w * s / 2, hh = h * s / 2;
        const float ca = cosf(ang), sa = sinf(ang);
        const float lx[4] = {-hw, +hw, +hw, -hw};
        const float ly[4] = {-hh, -hh, +hh, +hh};
        for (int i = 0; i < 4; ++i) {
            xs[i] = cx + lx[i] * ca - ly[i] * sa;
            ys[i] = cy + lx[i] * sa + ly[i] * ca;
        }
    }
};

// Feed the resolver a full quad for n frames so the model locks.
static QuadResult warm(Rig& rig, float cx, float cy, int n = 12)
{
    float xs[4], ys[4]; QuadResult r{};
    for (int i = 0; i < n; ++i) { rig.points(cx, cy, 0, 1, xs, ys); r = quad_update(xs, ys, 4); }
    return r;
}

// Find which output slot corresponds to a given truth point (by proximity).
static int slot_for(const QuadResult& r, float x, float y)
{
    int best = -1; float bd = 1e18f;
    for (int i = 0; i < 4; ++i) {
        const float d = (r.p[i].x - x) * (r.p[i].x - x) + (r.p[i].y - y) * (r.p[i].y - y);
        if (d < bd) { bd = d; best = i; }
    }
    return best;
}

int main()
{
    Rig rig;
    float xs[4], ys[4];

    // ---------------------------------------------------------------- 1 ----
    printf("\n1. cold start then lock\n");
    quad_reset(nullptr);
    {
        rig.points(120, 88, 0, 1, xs, ys);
        QuadResult r = quad_update(xs, ys, 2);          // not enough to seed
        check(r.count == 2 && !r.locked, "2 points pre-lock pass through");
        r = warm(rig, 120, 88);
        check(r.locked && r.count == 4 && r.n_real == 4, "locks on a clean quad");
    }

    // ---------------------------------------------------------------- 2 ----
    printf("\n2. all four visible: exact pass-through, no drift\n");
    quad_reset(nullptr);
    {
        warm(rig, 120, 88);
        rig.points(120, 88, 0, 1, xs, ys);
        QuadResult r = quad_update(xs, ys, 4);
        float worst = 0;
        for (int i = 0; i < 4; ++i) {
            const int t = slot_for(r, xs[i], ys[i]);
            worst = fmaxf(worst, hypotf(r.p[t].x - xs[i], r.p[t].y - ys[i]));
        }
        check(worst < 1e-3f, "published == detected to <0.001 px", worst, 0);
        check(r.n_real == 4 && r.confidence > 0.99f, "confidence 1.0 when all real");
    }

    // ---------------------------------------------------------------- 3 ----
    printf("\n3. ONE corner drops while static -> rigid reconstruction\n");
    quad_reset(nullptr);
    {
        warm(rig, 120, 88);
        rig.points(120, 88, 0, 1, xs, ys);
        const float mx = xs[2], my = ys[2];             // hide corner 2
        float sx[3] = {xs[0], xs[1], xs[3]}, sy[3] = {ys[0], ys[1], ys[3]};
        QuadResult r = quad_update(sx, sy, 3);
        check(r.count == 4, "still emits 4");
        check(r.n_real == 3, "reports 3 real");
        const int t = slot_for(r, mx, my);
        near(r.p[t].x, mx, 0.05f, "missing corner X reconstructed");
        near(r.p[t].y, my, 0.05f, "missing corner Y reconstructed");
        check(!r.p[t].real, "missing corner flagged not-real");
    }

    // ---------------------------------------------------------------- 4 ----
    printf("\n4. corner drops DURING a pan+rotate (the bench failure)\n");
    quad_reset(nullptr);
    {
        warm(rig, 100, 88);
        float worst = 0;
        for (int f = 1; f <= 40; ++f) {
            const float cx = 100 + f * 1.5f, ang = f * 0.010f;
            rig.points(cx, 88, ang, 1, xs, ys);
            QuadResult r;
            if (f >= 10 && f <= 25) {                   // corner 1 lost for 16 frames
                float sx[3] = {xs[0], xs[2], xs[3]}, sy[3] = {ys[0], ys[2], ys[3]};
                r = quad_update(sx, sy, 3);
                const int t = slot_for(r, xs[1], ys[1]);
                worst = fmaxf(worst, hypotf(r.p[t].x - xs[1], r.p[t].y - ys[1]));
            } else {
                r = quad_update(xs, ys, 4);
            }
            if (r.count != 4) { printf("  FAIL  count dropped at frame %d\n", f); failures++; }
        }
        check(true, "count stayed 4 through all 40 frames");
        check(worst < 1.0f, "reconstructed corner tracks the pan within 1 px", worst, 0);
    }

    // ---------------------------------------------------------------- 5 ----
    printf("\n5. IDENTITY survives a corner drop (the actual bug)\n");
    quad_reset(nullptr);
    {
        warm(rig, 120, 88);
        rig.points(120, 88, 0, 1, xs, ys);
        QuadResult before = quad_update(xs, ys, 4);
        int slot0 = slot_for(before, xs[0], ys[0]);

        for (int f = 0; f < 8; ++f) {                   // corner 3 missing
            float sx[3] = {xs[0], xs[1], xs[2]}, sy[3] = {ys[0], ys[1], ys[2]};
            quad_update(sx, sy, 3);
        }
        QuadResult after = quad_update(xs, ys, 4);      // it returns
        int slot0b = slot_for(after, xs[0], ys[0]);
        check(slot0 == slot0b, "corner 0 kept its slot across the dropout");

        bool all_kept = true;
        for (int i = 0; i < 4; ++i)
            if (slot_for(before, xs[i], ys[i]) != slot_for(after, xs[i], ys[i])) all_kept = false;
        check(all_kept, "ALL four corners kept their slots (no label swap)");
        check(after.n_real == 4, "returning corner re-associates immediately");
    }

    // ---------------------------------------------------------------- 6 ----
    printf("\n6. TWO corners drop -> still exactly solvable\n");
    quad_reset(nullptr);
    {
        warm(rig, 120, 88);
        rig.points(120, 88, 0.20f, 1, xs, ys);
        quad_update(xs, ys, 4);
        float sx[2] = {xs[0], xs[2]}, sy[2] = {ys[0], ys[2]};
        QuadResult r = quad_update(sx, sy, 2);
        check(r.count == 4 && r.n_real == 2, "emits 4 from 2 real");
        float worst = 0;
        for (int i : {1, 3}) {
            const int t = slot_for(r, xs[i], ys[i]);
            worst = fmaxf(worst, hypotf(r.p[t].x - xs[i], r.p[t].y - ys[i]));
        }
        check(worst < 0.5f, "both reconstructed within 0.5 px", worst, 0);
    }

    // ---------------------------------------------------------------- 7 ----
    printf("\n7. model RE-LEARNS when a stand is repositioned\n");
    quad_reset(nullptr);
    {
        warm(rig, 120, 88, 20);
        rig.h = 110;                                     // moved a stand
        for (int f = 0; f < 400; ++f) { rig.points(120, 88, 0, 1, xs, ys); quad_update(xs, ys, 4); }
        rig.points(120, 88, 0, 1, xs, ys);
        const float mx = xs[2], my = ys[2];
        float sx[3] = {xs[0], xs[1], xs[3]}, sy[3] = {ys[0], ys[1], ys[3]};
        QuadResult r = quad_update(sx, sy, 3);
        const int t = slot_for(r, mx, my);
        const float err = hypotf(r.p[t].x - mx, r.p[t].y - my);
        check(err < 1.0f, "reconstruction uses the NEW rig shape", err, 0);
    }

    // ---------------------------------------------------------------- 8 ----
    printf("\n8. a junk blob near a corner must not steal identity\n");
    quad_reset(nullptr);
    {
        warm(rig, 120, 88);
        rig.points(120, 88, 0, 1, xs, ys);
        float jx[5], jy[5];
        for (int i = 0; i < 4; ++i) { jx[i] = xs[i]; jy[i] = ys[i]; }
        jx[4] = xs[1] + 6; jy[4] = ys[1] + 6;            // impostor beside corner 1
        QuadResult r = quad_update(jx, jy, 5);
        const int t = slot_for(r, xs[1], ys[1]);
        near(r.p[t].x, xs[1], 0.01f, "real corner won over the closer impostor");
        QuadStats st = quad_take_stats();
        check(st.dropped_blobs >= 1, "impostor counted as a dropped blob");
    }

    // ---------------------------------------------------------------- 9 ----
    printf("\n9. total blackout: grace period, then HONEST, then re-lock\n");
    quad_reset(nullptr);
    {
        warm(rig, 120, 88);
        // Policy (refined by a field lock-up): a MOMENTARY
        // total dropout must still emit 4 -- LEDs flicker and OpenFIRE must not
        // fabricate. But a SUSTAINED one means the gun is pointed away from the
        // screen, and feeding stale corners then would park the cursor
        // somewhere wrong instead of reading as off-screen. So: hold 4 for a
        // ~90ms grace period, then report honestly and stand ready to re-lock.
        QuadResult r = quad_update(nullptr, nullptr, 0);
        check(r.count == 4, "1 frame of total loss still emits 4");
        for (int f = 0; f < 8; ++f) r = quad_update(nullptr, nullptr, 0);
        check(r.count == 4, "still 4 inside the grace period (~90ms)");
        check(r.confidence < 0.6f, "confidence already collapsing", r.confidence, 0);
        for (int f = 0; f < 20; ++f) r = quad_update(nullptr, nullptr, 0);
        check(r.count == 0, "past the grace period it reports honestly, not stale");
        rig.points(120, 88, 0, 1, xs, ys);
        r = quad_update(xs, ys, 4);
        check(r.n_real == 4, "reacquires immediately when they return");
    }

    // --------------------------------------------------------------- 10 ----
    // THE ANGLED-GUN CASE. A gun held off-axis foreshortens the quad into a
    // parallelogram; similarity cannot express that, affine can. Shear the rig
    // and drop a corner: the reconstruction must still land on the truth.
    printf("\n10. ANGLED gun (sheared quad) -> affine reconstruction\n");
    quad_reset(nullptr);
    {
        auto shear = [&](float k, float sy, float* dx, float* dy) {
            rig.points(120, 88, 0, 1, dx, dy);
            for (int i = 0; i < 4; ++i) {
                const float x = dx[i] - 120, y = (dy[i] - 88) * sy;
                dx[i] = 120 + x + k * y; dy[i] = 88 + y;
            }
        };
        for (int f = 0; f < 30; ++f) { shear(0.45f, 0.70f, xs, ys); quad_update(xs, ys, 4); }
        shear(0.45f, 0.70f, xs, ys);
        const float mx = xs[2], my = ys[2];
        float sx[3] = {xs[0], xs[1], xs[3]}, sy3[3] = {ys[0], ys[1], ys[3]};
        QuadResult r = quad_update(sx, sy3, 3);
        const int t = slot_for(r, mx, my);
        const float err = hypotf(r.p[t].x - mx, r.p[t].y - my);
        check(err < 0.5f, "angled-rig corner reconstructed within 0.5 px", err, 0);
        QuadStats st = quad_take_stats();
        check(st.reconstructed >= 1 && st.recon_sim == 0, "used the AFFINE fit");
        check(st.resid_max_x100 < 50, "affine residual < 0.5px (parallelogram is enough)",
              st.resid_max_x100 / 100.0, 0);
    }

    // --------------------------------------------------------------- 11 ----
    printf("\n11. envelope LEARNS from play, and refuses the implausible\n");
    quad_reset(nullptr);
    {
        // head-on only: the rig has never shown us any tilt
        warm(rig, 120, 88, 40);
        QuadStats st = quad_take_stats();
        const uint32_t env_flat = st.env_aniso_x100;
        // Now turn off-axis. RAMPED, not snapped: a corner may not move more
        // than the association gate in one frame, and rightly so -- 20px at
        // 135fps is already 2700 px/s. (My first version of this test snapped
        // straight to the sheared pose, moved every corner ~25px in one frame,
        // and the resolver correctly refused to follow. Good behaviour, bad
        // test.) 200 frames = 1.5s, which is a brisk but human wrist turn.
        for (int f = 0; f < 200; ++f) {
            const float t = (f < 100) ? (f / 100.0f) : 1.0f;   // ease in, then hold
            rig.points(120, 88, 0, 1, xs, ys);
            for (int i = 0; i < 4; ++i) {
                const float y = ys[i] - 88;
                xs[i] += (0.55f * t) * y;
                ys[i]  = 88 + y * (1.0f - 0.28f * t);
            }
            quad_update(xs, ys, 4);
        }
        st = quad_take_stats();
        check(st.env_aniso_x100 > env_flat, "envelope widened from off-axis play",
              st.env_aniso_x100 / 100.0, env_flat / 100.0);
        check(st.env_rejects == 0, "no rejects while all four are real");
    }

    // --------------------------------------------------------------- 12 ----
    printf("\n12. a bad association cannot smuggle in a wild deformation\n");
    quad_reset(nullptr);
    {
        warm(rig, 120, 88, 40);
        rig.points(120, 88, 0, 1, xs, ys);
        // three "corners" nearly collinear -- the shape a mis-association makes
        float bx[3] = {xs[0], xs[0] + 3, xs[0] + 6};
        float by[3] = {ys[0], ys[0] + 1, ys[0] + 2};
        QuadResult r = quad_update(bx, by, 3);
        check(r.count == 4, "still emits 4 rather than collapsing");
        QuadStats st = quad_take_stats();
        check(st.env_rejects >= 1 || st.coasted >= 1,
              "implausible fit refused (envelope or degenerate guard)");
    }

    // --------------------------------------------------------------- 13 ----
    // THE BUG THAT SHIPPED. v22 could pin the quad in a corner and never
    // re-lock. Test 9 missed it because the rig was STATIONARY during the
    // blackout, so velocity was 0 and the slots stayed put. Move during the
    // blackout and the undamped coast flew the slots into the clamp.
    printf("\n13. blackout WHILE MOVING -> must not run away, must re-lock\n");
    quad_reset(nullptr);
    {
        for (int f = 0; f < 20; ++f) {          // establish motion: 3 px/frame
            rig.points(60.0f + f * 3.0f, 88, 0, 1, xs, ys);
            quad_update(xs, ys, 4);
        }
        QuadResult r{};
        for (int f = 0; f < 60; ++f) r = quad_update(nullptr, nullptr, 0);
        float far = 0;
        for (int i = 0; i < 4; ++i) far = fmaxf(far, fabsf(r.p[i].x));
        check(far < 400.0f, "coasted slots did NOT fly off (damped)", far, 0);

        // LEDs come back where they actually are, far from where a runaway
        // would have parked
        rig.points(150, 88, 0, 1, xs, ys);
        bool relocked = false;
        for (int f = 0; f < 40 && !relocked; ++f) {
            r = quad_update(xs, ys, 4);
            if (r.n_real == 4) relocked = true;
        }
        check(relocked, "RE-LOCKED onto the real LEDs after the blackout");
        QuadStats st = quad_take_stats();
        check(st.lock_losses >= 1, "reported that it had lost correspondence");
        check(st.reseeds >= 1, "re-acquired via the learned rig shape");
    }

    // --------------------------------------------------------------- 14 ----
    printf("\n14. re-lock PRESERVES corner identity\n");
    quad_reset(nullptr);
    {
        rig.points(120, 88, 0, 1, xs, ys);
        warm(rig, 120, 88, 20);
        QuadResult before = quad_update(xs, ys, 4);
        int s0 = slot_for(before, xs[0], ys[0]), s1 = slot_for(before, xs[1], ys[1]);
        for (int f = 0; f < 40; ++f) quad_update(nullptr, nullptr, 0);   // force loss
        QuadResult after{};
        for (int f = 0; f < 5; ++f) after = quad_update(xs, ys, 4);
        check(slot_for(after, xs[0], ys[0]) == s0 &&
              slot_for(after, xs[1], ys[1]) == s1,
              "corners kept their slots THROUGH a full loss + re-acquire");
    }

    // --------------------------------------------------------------- 15 ----
    printf("\n15. a corner that drops mid-pan can find its way back\n");
    quad_reset(nullptr);
    {
        warm(rig, 60, 88, 15);
        bool back = false;
        for (int f = 1; f <= 40; ++f) {
            rig.points(60 + f * 2.0f, 88, 0, 1, xs, ys);
            if (f <= 20) {                       // corner 1 gone while panning
                float sx[3] = {xs[0], xs[2], xs[3]}, sy3[3] = {ys[0], ys[2], ys[3]};
                quad_update(sx, sy3, 3);
            } else {
                QuadResult r = quad_update(xs, ys, 4);
                if (r.n_real == 4) { back = true; break; }
            }
        }
        check(back, "returning corner re-associated during motion");
    }

    // --------------------------------------------------------------- 16 ----
    // *** NOT COVERED — READ THIS BEFORE TRUSTING THE SUITE ***
    //
    // One earlier fix is NOT regression-tested; attempts to test it failed
    // them. Recording that honestly rather than shipping tests that pass either
    // way and imply coverage that does not exist:
    //
    //  (1) ASSOCIATION RANKING. v24 ranked candidate matches by d/gate^2, so a
    //      starved slot (gate widened to 3x) could out-bid a healthy slot for a
    //      blob three times closer to the healthy one. v25 ranks on raw
    //      distance and uses the per-slot gate only for admission.
    //      Why untested: reaching the state needs a starved slot whose
    //      PREDICTION is far from its blob, but the affine reconstruction keeps
    //      predictions accurate, so the distances stay near zero and the
    //      ranking never decides anything. Every scenario tried passed on the
    //      buggy code too.
    //
    //  (2) RE-SEED SYMMETRY. SUPERSEDED, and now covered. A rectangle has eight
    //      symmetries and an affine fit scores them identically (affine permits
    //      non-uniform scale AND reflection), so v24's residual-only choice was
    //      arbitrary and v25 patched it with a continuity term. v26 deleted the
    //      whole problem instead: corners in angular order keep their cyclic
    //      order under any orientation-preserving affine map, so there are only
    //      4 candidates, not 24, and no continuity term is needed (it was
    //      actively harmful after an off-screen excursion, where the old slot
    //      positions are stale). Tests 18, 19 and 21 exercise that directly.
    //
    // (1) is still argued from first principles only, with HARDWARE as its
    // verification. If it is ever suspected again, write the test first.
    //
    // Tests 20, 21 and 22 were each confirmed to FAIL against a deliberately
    // re-broken copy before being kept -- 20 fires AddressSanitizer, 21 picks
    // junk corners under the v26 gate. Test 22's ratio bound is a gross
    // backstop only; see its own comment for what it does not catch.

    // --------------------------------------------------------------- 18 ----
    // OFF-SCREEN RELOAD. Point away, come back fast at a different angle and
    // position -- the games case. The shape must come back RIGHT, not just
    // come back. v25 leaned on a continuity term here, which is stale after an
    // excursion and biased toward the wrong answer.
    printf("\n18. off-screen reload: return fast, at a new angle\n");
    quad_reset(nullptr);
    {
        rig.w = 150; rig.h = 75;
        warm(rig, 120, 88, 40);
        for (int f = 0; f < 40; ++f) quad_update(nullptr, nullptr, 0);   // off-screen

        // come back rotated and shifted, as if you snapped back onto the screen
        rig.points(105, 95, 0.30f, 0.85f, xs, ys);
        QuadResult r{};
        for (int f = 0; f < 6; ++f) r = quad_update(xs, ys, 4);
        check(r.n_real == 4, "re-acquired all four");

        // THE test: hide a corner and see whether the rebuilt one is right.
        // A wrong cyclic rotation would put it diagonally opposite.
        const float mx = xs[2], my = ys[2];
        float rx[3] = {xs[0], xs[1], xs[3]}, ry[3] = {ys[0], ys[1], ys[3]};
        r = quad_update(rx, ry, 3);
        const int t = slot_for(r, mx, my);
        const float err = hypotf(r.p[t].x - mx, r.p[t].y - my);
        check(err < 2.0f, "SHAPE is correct after the reload (right rotation)", err, 0);
        rig.w = 120; rig.h = 80;
    }

    // --------------------------------------------------------------- 19 ----
    // SELF-HEAL. Force a wrong assignment by feeding a 90-degree-rotated rig
    // for a while, then confirm the resolver notices the residual and rebuilds
    // rather than staying wrong forever (the v25 behaviour).
    printf("\n19. locked-but-wrong must self-heal\n");
    quad_reset(nullptr);
    {
        rig.w = 150; rig.h = 70;
        warm(rig, 120, 88, 40);
        // Now physically rotate the rig 90 degrees. The slots keep chasing the
        // nearest blobs, which after a 90-degree turn is a different corner
        // each -- a wrong assignment that a residual check can catch.
        for (int f = 1; f <= 120; ++f) {
            const float a = 1.5708f * (f < 60 ? f / 60.0f : 1.0f);
            rig.points(120, 88, a, 1, xs, ys);
            quad_update(xs, ys, 4);
        }
        QuadStats st = quad_take_stats();
        // Either it tracked the rotation correctly (no reshape needed) or it
        // noticed and rebuilt. What must NOT happen is a persistently bad fit.
        rig.points(120, 88, 1.5708f, 1, xs, ys);
        QuadResult r = quad_update(xs, ys, 4);
        const float mx = xs[2], my = ys[2];
        float rx[3] = {xs[0], xs[1], xs[3]}, ry[3] = {ys[0], ys[1], ys[3]};
        r = quad_update(rx, ry, 3);
        const int t = slot_for(r, mx, my);
        const float err = hypotf(r.p[t].x - mx, r.p[t].y - my);
        check(err < 3.0f, "reconstruction sane after a full 90-degree rotation", err, 0);
        printf("       (reshapes=%u, reseeds=%u -- either route is acceptable)\n",
               (unsigned)st.reshapes, (unsigned)st.reseeds);
        rig.w = 120; rig.h = 80;
    }

    // --------------------------------------------------------------- 20 ----
    // v27 BOUNDS. The cold-start passthrough used to loop to n with n up to
    // QUAD_MAX_IN while QuadResult::p has FOUR entries -- it wrote off the end
    // of the struct, over count/locked/n_real/confidence and past them. Guard:
    // never report more than 4, and never touch memory after the result.
    printf("\n20. more blobs than slots must not overflow the result\n");
    quad_reset(nullptr);
    {
        struct Box { QuadResult r; uint32_t canary; } box;
        box.canary = 0xA5A5A5A5u;
        float bx[QUAD_MAX_IN], by[QUAD_MAX_IN];
        for (int i = 0; i < QUAD_MAX_IN; ++i) { bx[i] = 10.0f + 13.0f * i; by[i] = 20.0f + 7.0f * i; }
        box.r = quad_update(bx, by, QUAD_MAX_IN);       // no model yet, junk only
        check(box.canary == 0xA5A5A5A5u, "no write past the end of QuadResult");
        check(box.r.count <= 4, "count clamped to the 4 slots", box.r.count, 4);
        check(box.r.n_real <= 4, "n_real clamped to the 4 slots", box.r.n_real, 4);
    }

    // --------------------------------------------------------------- 21 ----
    // v27 SUBSET SEARCH, and the reason it exists. The caller ranks blobs by
    // MASS, so a bright junk blob can outrank a dim real LED. Give the resolver
    // the real quad PLUS junk that would have displaced a corner under a
    // top-4-by-mass trim, and it must still pick the four real ones -- and give
    // them the same slots they held before the excursion.
    printf("\n21. junk blobs must not steal a corner during re-acquire\n");
    quad_reset(nullptr);
    {
        rig.w = 150; rig.h = 70;
        warm(rig, 120, 88, 40);

        // remember the pre-excursion slot of one physical corner
        rig.points(120, 88, 0, 1, xs, ys);
        QuadResult before = quad_update(xs, ys, 4);
        const int slot_c2 = slot_for(before, xs[2], ys[2]);

        for (int f = 0; f < 40; ++f) quad_update(nullptr, nullptr, 0);   // off screen

        // Back on screen, rotated -- with THREE junk blobs mixed in, listed
        // FIRST so a naive "take the first four" fails too.
        rig.points(112, 92, 0.35f, 0.9f, xs, ys);
        float px[7], py[7];
        px[0] =  30; py[0] =  20;      // junk: would form a much larger quad
        px[1] = 200; py[1] = 160;      // junk: opposite side of the frame
        px[2] = (xs[0]+xs[1]+xs[2]+xs[3])/4;   // junk: dead centre of the rig
        py[2] = (ys[0]+ys[1]+ys[2]+ys[3])/4;
        for (int i = 0; i < 4; ++i) { px[3+i] = xs[i]; py[3+i] = ys[i]; }

        QuadResult r{};
        for (int f = 0; f < 6; ++f) r = quad_update(px, py, 7);
        check(r.count == 4, "still emits exactly 4 with 7 blobs offered", r.count, 4);

        bool all_real_corners = true;
        for (int i = 0; i < 4; ++i) {
            float bd = 1e18f;
            for (int k = 0; k < 4; ++k) {
                const float d = hypotf(r.p[i].x - xs[k], r.p[i].y - ys[k]);
                if (d < bd) bd = d;
            }
            if (bd > 2.0f) all_real_corners = false;
        }
        check(all_real_corners, "picked the four REAL corners, not the junk");

        const int slot_now = slot_for(r, xs[2], ys[2]);
        check(slot_now == slot_c2, "corner kept its slot through junk + reload",
              slot_now, slot_c2);
        rig.w = 120; rig.h = 80;
    }

    // --------------------------------------------------------------- 22 ----
    // The subset search must not be able to hitch the camera task, and the
    // COST/s meter that reports it must actually be measuring something.
    //
    // Absolute host microseconds mean nothing on an S3, so this compares the
    // worst RE-ACQUIRE frame against the steady 4-real frame measured in the
    // same run on the same machine. That ratio is a proxy for the operation
    // count, and it is what blows up if someone reinstates a 24-permutation
    // search or drops the convex4() prune. The on-target truth is COST/s.
    printf("\n22. re-acquire cost is bounded, and the meter is alive\n");
    {
        quad_reset(nullptr);
        rig.w = 150; rig.h = 70;
        warm(rig, 120, 88, 40);

        // --- steady state: 4 real blobs, no search ---
        (void)quad_take_stats();
        for (int f = 0; f < 20000; ++f) {
            rig.points(120 + (f & 3), 88, 0.2f, 1, xs, ys);
            quad_update(xs, ys, 4);
        }
        QuadStats sst = quad_take_stats();
        const double steady_us = (double)sst.total_us / (double)sst.frames;

        // --- worst case: 8 blobs, half junk, full re-acquire every time ---
        float px[QUAD_MAX_IN], py[QUAD_MAX_IN];
        rig.points(120, 88, 0.2f, 1, xs, ys);
        for (int i = 0; i < 4; ++i) { px[i] = xs[i]; py[i] = ys[i]; }
        // junk in convex position, so convex4() cannot reject it cheaply
        px[4] =  20; py[4] =  20;  px[5] = 220; py[5] =  20;
        px[6] = 220; py[6] = 160;  px[7] =  20; py[7] = 160;
        double reacq_us = 0; int reacq_n = 0;
        for (int f = 0; f < 2000; ++f) {
            for (int k = 0; k < 40; ++k) quad_update(nullptr, nullptr, 0);  // kill the lock
            (void)quad_take_stats();
            quad_update(px, py, QUAD_MAX_IN);
            QuadStats one = quad_take_stats();
            reacq_us += (double)one.total_us; ++reacq_n;
        }
        const double search_us = reacq_us / reacq_n;
        printf("       host: steady %.2f us/frame, re-acquire %.2f us/frame (x%.1f)\n",
               steady_us, search_us, steady_us > 0 ? search_us / steady_us : 0.0);

        // The meter must not be reading zeros -- that failure mode is exactly
        // what made the PSRAM-dead instrumentation useless for two days.
        check(sst.total_us > 0, "cost meter reports non-zero for steady frames");
        check(search_us > 0.0, "cost meter reports non-zero for re-acquire frames");
        // WHAT THIS RATIO ACTUALLY CATCHES, measured, not assumed:
        //   as shipped               ~22x  (-O2)   ~48x  (-O1 +ASan)
        //   convex4() prune removed  ~39x  (-O2)
        //   24-permutation search    would be ~6x the rotation count again
        // So 150x is a GROSS-EXPLOSION backstop. It does NOT catch losing the
        // prune -- that only costs ~1.5x and hides under the ASan spread. Do
        // not read a pass here as "the prune is still there"; read the printed
        // numbers. A tighter bound would fail on the sanitiser build, which is
        // worse than an honest loose one.
        check(steady_us <= 0 || search_us / steady_us < 150.0,
              "re-acquire stays within 150x a steady frame (gross backstop)",
              steady_us > 0 ? search_us / steady_us : 0.0, 150.0);
        rig.w = 120; rig.h = 80;
    }

    printf("\n%s  (%d failure%s)\n", failures ? "TESTS FAILED" : "ALL TESTS PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
