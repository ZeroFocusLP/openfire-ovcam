// replay_recorded — feed LITERAL captured P-lines (the exact integers the
// shim handed to OpenFIRE) through the UNMODIFIED OpenFIRE chain:
//     Square_Advanced -> Multi One Euro -> Perspective_Advanced -> cursor
// and report the frame-to-frame cursor movement. The input is known-good
// (steady 0xF, sub-pixel drift, J/s=0), so ANY large cursor jump here is
// produced downstream of us and is reproducible on the host.
//
// Build (from test/replay/):
//   g++ -std=c++17 -O2 -DUSE_SQUARE_ADVANCED -DUSE_PERSPECTIVE_ADVANCED \
//     -DUSE_MULTI_ONE_EURO_FILTER -I. -I$OF/lightgun/lib/OpenFIREPosition \
//     replay_recorded.cpp $OF/.../OpenFIRE_Square_Advanced.cpp \
//     $OF/.../OpenFIRE_Perspective_Advanced.cpp \
//     $OF/.../OpenFIRE_Multi_One_Euro_Filter.cpp -o replay_recorded && ./replay_recorded
#include <cstdio>
#include <cmath>
#include "OpenFIRE_Square_Advanced.h"
#include "OpenFIRE_Perspective_Advanced.h"
#include "OpenFIRE_Multi_One_Euro_Filter.h"

unsigned long g_fake_micros = 0;
static constexpr int RES_X = 1920 << 2, RES_Y = 1080 << 2;

// Recorded capture, gun essentially still (from a live serial trace).
// Format: x0,y0, x1,y1, x2,y2, x3,y3   — already in shim 0..1023 / 0..767 space.
static const int FRAMES[][8] = {
    {486,492, 558,108, 6,421, 10,24}, {490,492, 558,107, 6,422, 11,24},
    {488,494, 558,109, 7,426,  9,23}, {491,492, 559,108, 7,424, 11,25},
    {489,492, 557,106, 5,421,  8,23}, {489,492, 558,107, 6,421,  9,23},
    {490,492, 557,106, 6,421,  9,22}, {491,490, 557,103, 7,420, 10,19},
    {490,490, 557,103, 6,420,  8,20}, {489,491, 556,104, 6,421,  8,22},
    {489,491, 555,104, 5,421,  7,21}, {489,492, 556,105, 4,420,  8,22},
    {491,493, 559,108, 7,424, 11,25}, {492,494, 561,113, 9,425, 14,27},
    {493,492, 561,110,10,424, 14,24}, {491,491, 560,107, 6,420, 13,21},
    {493,489, 561,105, 9,419, 13,20},
};
static const int N = (int)(sizeof(FRAMES) / sizeof(FRAMES[0]));

int main()
{
    OpenFIRE_Square sq;
    OpenFIRE_Perspective per;
    OpenFIRE_One_Euro_Multi oef;

    printf("=== replay of recorded coordinates (gun still) ===\n");
    printf("input: max per-frame movement of any single point\n");
    int max_in = 0;
    for (int f = 1; f < N; ++f)
        for (int k = 0; k < 8; ++k) {
            int dd = FRAMES[f][k] - FRAMES[f - 1][k]; int d = dd < 0 ? -dd : dd;
            if (d > max_in) max_in = d;
        }
    printf("  -> %d units (%.2f native px)\n\n", max_in, max_in * 240.0f / 1023.0f);

    // settle + calibrate on the first frame, replayed
    bool cal = false;
    float prev_x = 0, prev_y = 0; bool have = false;
    float worst = 0; int worst_f = -1;

    for (int pass = 0; pass < 3; ++pass) {          // loop the capture 3x
        for (int f = 0; f < N; ++f) {
            int px[4], py[4];
            for (int i = 0; i < 4; ++i) { px[i] = FRAMES[f][i * 2]; py[i] = FRAMES[f][i * 2 + 1]; }
            sq.begin(px, py, 0x0F);

            int Xi[4], Yi[4]; float Xo[4], Yo[4];
            for (int i = 0; i < 4; ++i) { Xi[i] = sq.X(i); Yi[i] = sq.Y(i); }
            g_fake_micros += 7353;                  // one camera frame @136fps
            oef.process(Xi, Yi, Xo, Yo);

            if (!cal && pass == 1 && f == 0) {
                per.source((float)sq.testMedianX(), (float)sq.testMedianY());
                per.deinit(0); cal = true;
            }
            if (!cal) continue;

            per.warp(Xo[0], Yo[0], Xo[1], Yo[1], Xo[2], Yo[2], Xo[3], Yo[3],
                     0, 0, (float)RES_X, 0, 0, (float)RES_Y, (float)RES_X, (float)RES_Y);
            float cx = 100.0f * per.getX() / RES_X;
            float cy = 100.0f * per.getY() / RES_Y;
            if (have && pass == 2) {
                float d = std::fabs(cx - prev_x) + std::fabs(cy - prev_y);
                if (d > worst) { worst = d; worst_f = f; }
                if (d > 0.5f)
                    printf("  JUMP at frame %2d: cursor moved %.2f%% of screen "
                           "(%.1f px on 1920) | corners A(%d,%d) B(%d,%d) C(%d,%d) D(%d,%d)\n",
                           f, d, d * 19.2f, sq.X(0), sq.Y(0), sq.X(1), sq.Y(1),
                           sq.X(2), sq.Y(2), sq.X(3), sq.Y(3));
            }
            prev_x = cx; prev_y = cy; have = true;
        }
    }
    printf("\n  worst per-frame cursor movement: %.3f%% of screen (%.1f px on 1920)"
           " at frame %d\n", worst, worst * 19.2f, worst_f);
    printf("  (his observed teleport is 4-5cm on a 27in screen = ~7%%)\n");
    return 0;
}
