#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include <chrono>
#include "screen_detector.h"

// Helper to render a white border on black image
void draw_rect_border(std::vector<uint8_t>& img, int w, int h, int x0, int y0, int x1, int y1, int thick, bool with_notch = true) {
    std::fill(img.begin(), img.end(), 10); // Ambient black level ~10

    // Top & Bottom bars
    for (int t = 0; t < thick; ++t) {
        for (int x = x0; x <= x1; ++x) {
            if (y0 + t < h) img[(y0 + t) * w + x] = 230;
            if (y1 - t >= 0) img[(y1 - t) * w + x] = 230;
        }
    }
    // Left & Right bars
    for (int t = 0; t < thick; ++t) {
        for (int y = y0; y <= y1; ++y) {
            if (x0 + t < w) img[y * w + (x0 + t)] = 230;
            if (x1 - t >= 0) img[y * w + (x1 - t)] = 230;
        }
    }
    // Top orientation marker at 35%
    if (with_notch) {
        int mx = x0 + (int)(0.35f * (x1 - x0));
        for (int dy = thick; dy < thick + 10; ++dy) {
            for (int dx = -10; dx <= 10; ++dx) {
                if (y0 + dy < h && mx + dx >= 0 && mx + dx < w) {
                    img[(y0 + dy) * w + (mx + dx)] = 230;
                }
            }
        }
    }
}

// Helper to render 4 corner fiducials
void draw_fiducials(std::vector<uint8_t>& img, int w, int h, int x0, int y0, int x1, int y1, int box) {
    std::fill(img.begin(), img.end(), 10);
    auto draw_box = [&](int cx, int cy) {
        for (int y = cy - box/2; y <= cy + box/2; ++y) {
            for (int x = cx - box/2; x <= cx + box/2; ++x) {
                if (x >= 0 && x < w && y >= 0 && y < h) img[y * w + x] = 240;
            }
        }
    };
    draw_box(x0, y0); // TL
    draw_box(x1, y0); // TR
    draw_box(x1, y1); // BR
    draw_box(x0, y1); // BL
}

int main() {
    constexpr int W = 240;
    constexpr int H = 176;
    std::vector<uint8_t> frame(W * H, 0);

    std::cout << "========================================\n";
    std::cout << "TEST 1: Rectangular Border Detection\n";
    std::cout << "========================================\n";
    int exp_x0 = 30, exp_y0 = 20, exp_x1 = 210, exp_y1 = 155;
    draw_rect_border(frame, W, H, exp_x0, exp_y0, exp_x1, exp_y1, 6, true);

    screen_result_t r1 = screen_detect(frame.data(), W, H, SCREEN_MODE_BORDER);
    std::cout << "Detected: " << (r1.valid ? "YES" : "NO") << ", Count: " << (int)r1.count << "\n";
    std::cout << "TL: (" << r1.p[0].x << ", " << r1.p[0].y << ") expected (" << exp_x0 << ", " << exp_y0 << ")\n";
    std::cout << "TR: (" << r1.p[1].x << ", " << r1.p[1].y << ") expected (" << exp_x1 << ", " << exp_y0 << ")\n";
    std::cout << "BR: (" << r1.p[2].x << ", " << r1.p[2].y << ") expected (" << exp_x1 << ", " << exp_y1 << ")\n";
    std::cout << "BL: (" << r1.p[3].x << ", " << r1.p[3].y << ") expected (" << exp_x0 << ", " << exp_y1 << ")\n";
    std::cout << "Execution time: " << r1.dt_us << " us\n";

    assert(r1.valid && r1.count == 4);
    assert(std::abs(r1.p[0].x - exp_x0) <= 2.0f);
    assert(std::abs(r1.p[0].y - exp_y0) <= 2.0f);
    assert(std::abs(r1.p[1].x - exp_x1) <= 2.0f);
    assert(std::abs(r1.p[1].y - exp_y0) <= 2.0f);
    assert(std::abs(r1.p[2].x - exp_x1) <= 2.0f);
    assert(std::abs(r1.p[2].y - exp_y1) <= 2.0f);
    assert(std::abs(r1.p[3].x - exp_x0) <= 2.0f);
    assert(std::abs(r1.p[3].y - exp_y1) <= 2.0f);
    std::cout << "-> TEST 1 PASSED!\n\n";

    std::cout << "========================================\n";
    std::cout << "TEST 2: Inverted Border (180° Flip Detection)\n";
    std::cout << "========================================\n";
    // Place notch at the bottom instead of top
    draw_rect_border(frame, W, H, exp_x0, exp_y0, exp_x1, exp_y1, 6, false);
    // Draw notch on bottom
    int b_mx = exp_x0 + (int)(0.35f * (exp_x1 - exp_x0));
    for (int dy = 6; dy < 6 + 10; ++dy) {
        for (int dx = -10; dx <= 10; ++dx) {
            frame[(exp_y1 - dy) * W + (b_mx + dx)] = 230;
        }
    }
    screen_result_t r2 = screen_detect(frame.data(), W, H, SCREEN_MODE_BORDER);
    std::cout << "Detected: " << (r2.valid ? "YES" : "NO") << ", Count: " << (int)r2.count << "\n";
    // When inverted, the screen detector flips so p[0] (TL of the screen) maps to physical bottom-right
    std::cout << "Reported TL after flip: (" << r2.p[0].x << ", " << r2.p[0].y << ")\n";
    assert(r2.valid && r2.count == 4);
    assert(std::abs(r2.p[0].y - exp_y1) <= 2.0f); // Was flipped!
    std::cout << "-> TEST 2 (Inversion Flip) PASSED!\n\n";

    std::cout << "========================================\n";
    std::cout << "TEST 3: Corner Fiducials Detection\n";
    std::cout << "========================================\n";
    draw_fiducials(frame, W, H, 40, 30, 200, 145, 12);
    screen_result_t r3 = screen_detect(frame.data(), W, H, SCREEN_MODE_FIDUCIAL);
    std::cout << "Detected: " << (r3.valid ? "YES" : "NO") << ", Count: " << (int)r3.count << "\n";
    std::cout << "TL: (" << r3.p[0].x << ", " << r3.p[0].y << ") expected (40, 30)\n";
    std::cout << "TR: (" << r3.p[1].x << ", " << r3.p[1].y << ") expected (200, 30)\n";
    std::cout << "BR: (" << r3.p[2].x << ", " << r3.p[2].y << ") expected (200, 145)\n";
    std::cout << "BL: (" << r3.p[3].x << ", " << r3.p[3].y << ") expected (40, 145)\n";
    assert(r3.valid && r3.count == 4);
    assert(std::abs(r3.p[0].x - 40.0f) <= 1.0f);
    assert(std::abs(r3.p[0].y - 30.0f) <= 1.0f);
    std::cout << "-> TEST 3 PASSED!\n\n";

    std::cout << "========================================\n";
    std::cout << "TEST 4: SPEED BENCHMARK (10,000 frames)\n";
    std::cout << "========================================\n";
    draw_rect_border(frame, W, H, exp_x0, exp_y0, exp_x1, exp_y1, 6, true);
    auto t0 = std::chrono::high_resolution_clock::now();
    constexpr int ITERS = 10000;
    for (int i = 0; i < ITERS; ++i) {
        volatile screen_result_t r = screen_detect(frame.data(), W, H, SCREEN_MODE_BORDER);
        (void)r;
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    double total_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    double us_per_frame = total_us / ITERS;
    double max_theoretical_fps = 1000000.0 / us_per_frame;

    std::cout << "Total time for " << ITERS << " frames: " << total_us / 1000.0 << " ms\n";
    std::cout << "Per-frame execution time: " << us_per_frame << " us (" << us_per_frame / 1000.0 << " ms)\n";
    std::cout << "Max theoretical detector throughput: " << (long)max_theoretical_fps << " FPS\n";

    std::cout << "\nALL TESTS PASSED SUCCESSFULLY!\n";
    return 0;
}
