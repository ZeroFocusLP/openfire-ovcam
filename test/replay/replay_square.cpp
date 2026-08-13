// replay_square — feed CAPTURED harness P-streams (the exact values
// OpenFIRE receives from our shim) through OpenFIRE's UNTOUCHED
// OpenFIRE_Square_Advanced solver on the host, and measure what its output
// does. If the solver's corner assignment flips / median teleports on CLEAN
// input, the aim teleports originate in solver-vs-geometry, not in detection.
//
// Build (from test/replay/, OF = path to OpenFIRE-Firmware-ESP32):
//   g++ -std=c++17 -O2 -DUSE_SQUARE_ADVANCED -I. -I$OF/lightgun/lib/OpenFIREPosition \
//       replay_square.cpp $OF/lightgun/lib/OpenFIREPosition/OpenFIRE_Square_Advanced.cpp \
//       -o replay_square && ./replay_square
#include <cstdio>
#include <cstring>
#include <cmath>
#include "OpenFIRE_Square_Advanced.h"

struct Line { int x[4], y[4]; unsigned seen; };

// ---- dataset 1: latest resting capture (v7-era rig), steady tail ----------
// P,23570..P,24638 (all 0xF, sub-pixel jitter) — the "clean at rest" stream.
static const Line DS1[] = {
{{674,279,401,443},{474,369,446,384},0xF},
{{673,278,400,443},{472,367,444,382},0xF},
{{675,281,403,446},{473,367,444,384},0xF},
{{677,283,404,447},{471,366,443,381},0xF},
{{672,278,399,442},{469,364,441,378},0xF},
{{672,277,399,442},{467,364,440,378},0xF},
{{675,280,402,444},{471,366,443,381},0xF},
{{675,281,402,444},{473,368,445,383},0xF},
{{675,280,402,444},{473,368,444,383},0xF},
{{672,278,399,442},{471,365,442,380},0xF},
{{673,279,400,443},{472,365,443,379},0xF},
{{672,277,398,442},{470,362,439,378},0xF},
{{670,274,397,440},{467,361,438,377},0xF},
{{669,274,396,439},{469,362,439,379},0xF},
{{673,277,399,443},{471,363,441,379},0xF},
{{672,277,399,442},{470,361,440,378},0xF},
{{671,276,397,441},{465,356,434,374},0xF},
};
// ---- dataset 2: earlier resting capture (T-shaped rig), steady segment ----
// P,8837..P,9636 — clean 0xF segment before the interloper events.
static const Line DS2[] = {
{{428,282,492,526},{293,640,640,687},0xF},
{{430,282,493,526},{288,633,634,679},0xF},
{{430,283,495,526},{283,629,629,675},0xF},
{{432,289,499,533},{287,634,633,678},0xF},
{{429,288,501,533},{273,618,617,662},0xF},
{{432,290,501,534},{270,615,613,658},0xF},
{{428,286,496,529},{272,617,615,660},0xF},
{{430,290,500,533},{276,622,619,664},0xF},
{{428,287,496,530},{276,622,618,663},0xF},
{{432,293,502,536},{280,628,624,668},0xF},
{{426,290,500,534},{275,622,618,662},0xF},
{{432,293,504,538},{280,628,625,669},0xF},
};
// ---- dataset 3 (control): synthetic RECTANGLE quad + 2-unit jitter --------
static Line make_rect(int f)
{
    // wide rectangle around a screen: corners TL,TR,BL,BR in shim space
    int jx = (f * 7) % 5 - 2, jy = (f * 3) % 5 - 2;   // deterministic +/-2 jitter
    Line L;
    int bx[4] = {250, 750, 250, 750};
    int by[4] = {200, 200, 600, 600};
    for (int i = 0; i < 4; ++i) { L.x[i] = bx[i] + jx * (i + 1) % 3; L.y[i] = by[i] + jy * ((i + 2) % 3); }
    L.seen = 0xF;
    return L;
}

static void run(const char* name, const Line* ds, int n, bool synth)
{
    OpenFIRE_Square sq;
    int prevMX = -1, prevMY = -1;
    int prevCorner[4][2]; bool havePrev = false;
    int teleports = 0, flips = 0;
    printf("=== %s ===\n", name);
    for (int f = 0; f < n; ++f) {
        Line L = synth ? make_rect(f) : ds[f];
        // each P-line stands for ~8 camera frames (15Hz sampling of 120fps):
        // feed it repeatedly so the solver's per-frame dynamics settle.
        for (int rep = 0; rep < 8; ++rep)
            sq.begin(L.x, L.y, L.seen);
        int mx = sq.testMedianX(), my = sq.testMedianY();
        // corner positions after assignment:
        int cx[4], cy[4];
        for (int i = 0; i < 4; ++i) { cx[i] = sq.X(i); cy[i] = sq.Y(i); }
        if (havePrev) {
            int dm = abs(mx - prevMX) + abs(my - prevMY);
            // input moved at most a few units/line -> median should too.
            // Flag anything > 60 mouse-units (~15 shim units) as a teleport.
            if (dm > 60) {
                teleports++;
                printf("  f%02d TELEPORT: median (%d,%d) -> (%d,%d)  |d|=%d\n",
                       f, prevMX, prevMY, mx, my, dm);
            }
            // corner-assignment flip: corner i jumps by more than the whole
            // input motion could explain (>200 mouse units = 50 shim units)
            for (int i = 0; i < 4; ++i) {
                int dc = abs(cx[i] - prevCorner[i][0]) + abs(cy[i] - prevCorner[i][1]);
                if (dc > 200) {
                    flips++;
                    printf("  f%02d corner %c REASSIGNED: (%d,%d) -> (%d,%d)\n",
                           f, 'A' + i, prevCorner[i][0], prevCorner[i][1], cx[i], cy[i]);
                }
            }
        }
        prevMX = mx; prevMY = my;
        for (int i = 0; i < 4; ++i) { prevCorner[i][0] = cx[i]; prevCorner[i][1] = cy[i]; }
        havePrev = true;
    }
    printf("  -> %d teleports, %d corner reassignments over %d lines\n\n",
           teleports, flips, n);
}

// ---- experiment: corner-LABEL stability under hand tremor -----------------
// The app's IR test view draws the SOLVED corners; if the A/B/C/D labeling
// flips, a corner teleports in that view even when the physical points barely
// move. Degenerate quads (points at near-equal heights) sit ON the labeling
// heuristics' decision boundaries, so tremor flips them.
static void tremor(const char* name, const int bx[4], const int by[4])
{
    OpenFIRE_Square sq;
    int prevC[4][2]; bool have = false; int flips = 0;
    for (int f = 0; f < 1500; ++f) {
        int px[4], py[4];
        for (int i = 0; i < 4; ++i) {
            // deterministic +/-3 unit tremor, different phase per point
            unsigned h = (unsigned)(f * 2654435761u) ^ (unsigned)(i * 97u);
            h ^= h >> 13; h *= 2246822519u; h ^= h >> 16;
            px[i] = bx[i] + (int)(h % 7) - 3;
            py[i] = by[i] + (int)((h >> 8) % 7) - 3;
        }
        sq.begin(px, py, 0xF);
        int cx[4], cy[4];
        for (int i = 0; i < 4; ++i) { cx[i] = sq.X(i); cy[i] = sq.Y(i); }
        if (have)
            for (int i = 0; i < 4; ++i) {
                int d = abs(cx[i] - prevC[i][0]) + abs(cy[i] - prevC[i][1]);
                if (d > 200) flips++;   // label jumped far beyond the tremor
            }
        for (int i = 0; i < 4; ++i) { prevC[i][0] = cx[i]; prevC[i][1] = cy[i]; }
        have = true;
    }
    printf("  %-44s: %d corner teleports / 1500 frames\n", name, flips);
}

int main()
{
    run("DS1: latest rig, resting (clean 0xF stream)", DS1, (int)(sizeof(DS1)/sizeof(DS1[0])), false);
    run("DS2: T-shaped rig, resting (clean 0xF segment)", DS2, (int)(sizeof(DS2)/sizeof(DS2[0])), false);
    run("CONTROL: synthetic rectangle + jitter", nullptr, 40, true);
    printf("=== corner-label stability under +/-3-unit hand tremor ===\n");
    {   // T-rig quad: THREE points at near-equal height (y 465-470)
        int bx[4] = {455, 325, 600, 270}, by[4] = {135, 468, 465, 470};
        tremor("T-RIG (3 points at equal height)", bx, by);
    }
    {   // his flat-sliver quad from the latest rig
        int bx[4] = {672, 277, 399, 442}, by[4] = {470, 361, 440, 378};
        tremor("FLAT SLIVER QUAD", bx, by);
    }
    {   // proper rectangle spanning the screen
        int bx[4] = {250, 750, 250, 750}, by[4] = {200, 200, 600, 600};
        tremor("PROPER RECTANGLE", bx, by);
    }
    return 0;
}
