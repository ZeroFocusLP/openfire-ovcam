#include "screen_detector.h"
#include <cmath>
#include <cstring>
#include <algorithm>

#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)
  #include "esp_timer.h"
  #include "esp_attr.h"
  #define SCREEN_IRAM IRAM_ATTR
  static inline uint32_t get_time_us(void) { return (uint32_t)esp_timer_get_time(); }
#else
  #include <chrono>
  #define SCREEN_IRAM
  static inline uint32_t get_time_us(void) {
      using namespace std::chrono;
      return (uint32_t)duration_cast<microseconds>(high_resolution_clock::now().time_since_epoch()).count();
  }
#endif

static uint8_t s_user_threshold = 90; // Default brightness threshold for screen edge
static screen_stats_t s_last_stats = {};

void screen_detector_init(void) {
    memset(&s_last_stats, 0, sizeof(s_last_stats));
}

void screen_detector_set_threshold(uint8_t threshold) {
    s_user_threshold = threshold;
}

uint8_t screen_detector_get_threshold(void) {
    return s_user_threshold;
}

void screen_detector_get_stats(screen_stats_t* out) {
    if (out) *out = s_last_stats;
}

namespace {

struct Point2D {
    float x, y;
};

// Line in form y = m*x + c (for horizontal lines)
struct HLine {
    float m = 0;
    float c = 0;
    int inliers = 0;
    bool valid = false;
};

// Line in form x = m*y + c (for vertical lines, avoids infinite slope)
struct VLine {
    float m = 0;
    float c = 0;
    int inliers = 0;
    bool valid = false;
};

// Subpixel edge detector along a 1D sequence
// Returns subpixel offset (0.0 .. max_len) or -1 if not found
SCREEN_IRAM float find_edge_1d(const uint8_t* p, int step, int max_len, uint8_t thr, uint8_t min_grad) {
    int prev = *p;
    for (int i = 1; i < max_len; ++i) {
        int cur = p[i * step];
        // Look for rising edge passing threshold with at least min_grad units gradient
        if (prev < thr && cur >= thr && (cur - prev >= min_grad)) {
            // Linear interpolation for subpixel position
            float frac = (float)(thr - prev) / (float)(cur - prev + 1e-4f);
            return (float)(i - 1) + frac;
        }
        prev = cur;
    }
    return -1.0f;
}

// Fit horizontal line y = m*x + c with outlier rejection
HLine fit_hline(const Point2D* pts, int n) {
    HLine res;
    if (n < 4) return res;

    float sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (int i = 0; i < n; ++i) {
        sx += pts[i].x; sy += pts[i].y;
        sxx += pts[i].x * pts[i].x;
        sxy += pts[i].x * pts[i].y;
    }
    float denom = (n * sxx - sx * sx);
    if (std::abs(denom) < 1e-5f) return res;

    float m = (n * sxy - sx * sy) / denom;
    float c = (sy - m * sx) / n;

    // Outlier rejection pass (threshold: 2.5 pixels)
    float sx2 = 0, sy2 = 0, sxx2 = 0, sxy2 = 0;
    int inliers = 0;
    for (int i = 0; i < n; ++i) {
        float err = std::abs(pts[i].y - (m * pts[i].x + c));
        if (err <= 2.5f) {
            sx2 += pts[i].x; sy2 += pts[i].y;
            sxx2 += pts[i].x * pts[i].x;
            sxy2 += pts[i].x * pts[i].y;
            inliers++;
        }
    }

    if (inliers >= 4 && inliers * 2 >= n) {
        float denom2 = (inliers * sxx2 - sx2 * sx2);
        if (std::abs(denom2) > 1e-5f) {
            res.m = (inliers * sxy2 - sx2 * sy2) / denom2;
            res.c = (sy2 - res.m * sx2) / inliers;
            res.inliers = inliers;
            res.valid = true;
            return res;
        }
    }

    res.m = m;
    res.c = c;
    res.inliers = n;
    res.valid = true;
    return res;
}

// Fit vertical line x = m*y + c with outlier rejection
VLine fit_vline(const Point2D* pts, int n) {
    VLine res;
    if (n < 4) return res;

    float sy = 0, sx = 0, syy = 0, syx = 0;
    for (int i = 0; i < n; ++i) {
        sy += pts[i].y; sx += pts[i].x;
        syy += pts[i].y * pts[i].y;
        syx += pts[i].y * pts[i].x;
    }
    float denom = (n * syy - sy * sy);
    if (std::abs(denom) < 1e-5f) return res;

    float m = (n * syx - sy * sx) / denom;
    float c = (sx - m * sy) / n;

    // Outlier rejection pass (threshold: 2.5 pixels)
    float sy2 = 0, sx2 = 0, syy2 = 0, syx2 = 0;
    int inliers = 0;
    for (int i = 0; i < n; ++i) {
        float err = std::abs(pts[i].x - (m * pts[i].y + c));
        if (err <= 2.5f) {
            sy2 += pts[i].y; sx2 += pts[i].x;
            syy2 += pts[i].y * pts[i].y;
            syx2 += pts[i].y * pts[i].x;
            inliers++;
        }
    }

    if (inliers >= 4 && inliers * 2 >= n) {
        float denom2 = (inliers * syy2 - sy2 * sy2);
        if (std::abs(denom2) > 1e-5f) {
            res.m = (inliers * syx2 - sy2 * sx2) / denom2;
            res.c = (sx2 - res.m * sy2) / inliers;
            res.inliers = inliers;
            res.valid = true;
            return res;
        }
    }

    res.m = m;
    res.c = c;
    res.inliers = n;
    res.valid = true;
    return res;
}

// Compute intersection of horizontal line (y = m_h * x + c_h)
// and vertical line (x = m_v * y + c_v)
bool intersect_lines(const HLine& h, const VLine& v, Point2D& out) {
    float det = 1.0f - h.m * v.m;
    if (std::abs(det) < 1e-4f) return false;
    out.y = (h.m * v.c + h.c) / det;
    out.x = v.m * out.y + v.c;
    return true;
}

// Bilinear pixel sample with bounds clamping
uint8_t sample_bilinear(const uint8_t* img, int w, int h, float x, float y) {
    if (x < 0) x = 0; if (x > w - 1) x = (float)(w - 1);
    if (y < 0) y = 0; if (y > h - 1) y = (float)(h - 1);
    int x0 = (int)x; int y0 = (int)y;
    int x1 = (x0 < w - 1) ? x0 + 1 : x0;
    int y1 = (y0 < h - 1) ? y0 + 1 : y0;
    float fx = x - (float)x0; float fy = y - (float)y0;

    float p00 = img[y0 * w + x0];
    float p10 = img[y0 * w + x1];
    float p01 = img[y1 * w + x0];
    float p11 = img[y1 * w + x1];

    float top = p00 * (1.0f - fx) + p10 * fx;
    float bot = p01 * (1.0f - fx) + p11 * fx;
    return (uint8_t)(top * (1.0f - fy) + bot * fy + 0.5f);
}

} // namespace

screen_result_t screen_detect(const uint8_t* img, int width, int height, screen_mode_t mode)
{
    screen_result_t res = {};
    if (!img || width <= 0 || height <= 0) return res;

    uint32_t t_start = get_time_us();

    // 0. Compute image illumination stats
    uint8_t min_p = 255, max_p = 0;
    uint32_t sum_p = 0;
    int samples = 0;
    const int total_px = width * height;
    for (int i = 0; i < total_px; i += 4) {
        uint8_t val = img[i];
        if (val < min_p) min_p = val;
        if (val > max_p) max_p = val;
        sum_p += val;
        samples++;
    }
    uint8_t avg_p = samples ? (uint8_t)(sum_p / samples) : 0;

    uint8_t thr = s_user_threshold;
    if (thr == 0) {
        // Fully auto-adaptive threshold: place threshold between scene average and peak
        if (max_p > avg_p + 15) {
            thr = (uint8_t)(avg_p + (max_p - avg_p) * 0.45f);
        } else if (max_p > min_p + 15) {
            thr = (uint8_t)(min_p + (max_p - min_p) * 0.5f);
        } else {
            thr = 60;
        }
    } else if (thr > max_p && (max_p > min_p + 15)) {
        // Auto-rescue when manual threshold exceeds camera's peak brightness
        thr = (uint8_t)(avg_p + (max_p - avg_p) * 0.45f);
    }

    s_last_stats.min_px = min_p;
    s_last_stats.max_px = max_p;
    s_last_stats.avg_px = avg_p;
    s_last_stats.active_thr = thr;

    if (mode == SCREEN_MODE_BORDER) {
        // ===================================================================
        // SINDEN-STYLE BORDER TRACKING VIA PERIMETER RAYS
        // ===================================================================
        constexpr int MAX_RAYS = 24;
        Point2D top_pts[MAX_RAYS], bot_pts[MAX_RAYS];
        Point2D left_pts[MAX_RAYS], right_pts[MAX_RAYS];
        int n_top = 0, n_bot = 0, n_left = 0, n_right = 0;

        uint8_t min_grad = (uint8_t)std::max(6, std::min(12, (int)(max_p - min_p) / 4));

        // 1. Cast vertical rays for Top and Bottom borders
        // Avoid corners (first and last 15% of width)
        const int x_start = (int)(width * 0.15f);
        const int x_end   = (int)(width * 0.85f);
        const int x_step  = (x_end - x_start) / 16;
        const int max_vscan = height / 2;

        for (int x = x_start; x <= x_end && n_top < MAX_RAYS; x += x_step) {
            // Ray down from top edge (y=0 down to height/2)
            float ey = find_edge_1d(&img[x], width, max_vscan, thr, min_grad);
            if (ey >= 0) {
                top_pts[n_top++] = { (float)x, ey };
            }
            // Ray up from bottom edge (y=height-1 up to height/2)
            float eby = find_edge_1d(&img[(height - 1) * width + x], -width, max_vscan, thr, min_grad);
            if (eby >= 0) {
                bot_pts[n_bot++] = { (float)x, (float)(height - 1) - eby };
            }
        }

        // 2. Cast horizontal rays for Left and Right borders
        const int y_start = (int)(height * 0.15f);
        const int y_end   = (int)(height * 0.85f);
        const int y_step  = (y_end - y_start) / 12;
        const int max_hscan = width / 2;

        for (int y = y_start; y <= y_end && n_left < MAX_RAYS; y += y_step) {
            // Ray right from left edge (x=0 to width/2)
            float ex = find_edge_1d(&img[y * width], 1, max_hscan, thr, min_grad);
            if (ex >= 0) {
                left_pts[n_left++] = { ex, (float)y };
            }
            // Ray left from right edge (x=width-1 to width/2)
            float erx = find_edge_1d(&img[y * width + (width - 1)], -1, max_hscan, thr, min_grad);
            if (erx >= 0) {
                right_pts[n_right++] = { (float)(width - 1) - erx, (float)y };
            }
        }

        s_last_stats.n_top = (uint8_t)n_top;
        s_last_stats.n_bot = (uint8_t)n_bot;
        s_last_stats.n_left = (uint8_t)n_left;
        s_last_stats.n_right = (uint8_t)n_right;

        // 3. Fit 4 lines
        HLine top_line = fit_hline(top_pts, n_top);
        HLine bot_line = fit_hline(bot_pts, n_bot);
        VLine left_line = fit_vline(left_pts, n_left);
        VLine right_line = fit_vline(right_pts, n_right);

        if (top_line.valid && bot_line.valid && left_line.valid && right_line.valid) {
            Point2D tl, tr, br, bl;
            if (intersect_lines(top_line, left_line, tl) &&
                intersect_lines(top_line, right_line, tr) &&
                intersect_lines(bot_line, right_line, br) &&
                intersect_lines(bot_line, left_line, bl))
            {
                // Basic geometric sanity checks:
                // Quad width and height must be positive and reasonable
                float w_top = tr.x - tl.x;
                float w_bot = br.x - bl.x;
                float h_left = bl.y - tl.y;
                float h_right = br.y - tr.y;

                if (w_top > 25.0f && w_bot > 25.0f && h_left > 20.0f && h_right > 20.0f) {
                    // Check orientation notch:
                    // Pattern Mode 1 has a marker 35% from TL along top edge, 8px inside.
                    // Test sample at 35% along top edge vs 35% along bottom edge
                    float top_test_x = tl.x + 0.35f * (tr.x - tl.x);
                    float top_test_y = tl.y + 0.35f * (tr.y - tl.y) + 8.0f;

                    float bot_test_x = bl.x + 0.35f * (br.x - bl.x);
                    float bot_test_y = bl.y + 0.35f * (br.y - bl.y) - 8.0f;

                    uint8_t top_val = sample_bilinear(img, width, height, top_test_x, top_test_y);
                    uint8_t bot_val = sample_bilinear(img, width, height, bot_test_x, bot_test_y);

                    // If bottom is significantly brighter than top, the screen is inverted 180°
                    if (bot_val > top_val + 20) {
                        // Inverted: swap TL<->BR and TR<->BL
                        res.p[0] = { br.x, br.y };
                        res.p[1] = { bl.x, bl.y };
                        res.p[2] = { tl.x, tl.y };
                        res.p[3] = { tr.x, tr.y };
                    } else {
                        // Standard orientation
                        res.p[0] = { tl.x, tl.y };
                        res.p[1] = { tr.x, tr.y };
                        res.p[2] = { br.x, br.y };
                        res.p[3] = { bl.x, bl.y };
                    }

                    res.count = 4;
                    res.valid = true;
                    res.confidence = (float)(top_line.inliers + bot_line.inliers +
                                            left_line.inliers + right_line.inliers) / 40.0f;
                    if (res.confidence > 1.0f) res.confidence = 1.0f;
                }
            }
        }
    }
    else if (mode == SCREEN_MODE_FIDUCIAL) {
        // ===================================================================
        // 4 CORNER FIDUCIAL / BRACKET TRACKING (Quadrant Peak Centroids)
        // Works with Pattern Mode 2 (corner boxes) & Mode 3 (corner brackets)
        // ===================================================================
        const int qx_mid = width / 2;
        const int qy_mid = height / 2;

        struct QuadBox { int x0, y0, x1, y1; };
        QuadBox quads[4] = {
            { 0, 0, qx_mid, qy_mid },               // TL: quadrant 0
            { qx_mid, 0, width, qy_mid },           // TR: quadrant 1
            { qx_mid, qy_mid, width, height },      // BR: quadrant 2
            { 0, qy_mid, qx_mid, height }           // BL: quadrant 3
        };

        Point2D corners[4] = {};
        bool q_found[4] = {};

        for (int q = 0; q < 4; ++q) {
            uint64_t sum_x = 0, sum_y = 0, sum_w = 0;
            uint8_t peak = 0;
            int px = -1, py = -1;

            // 1. Locate peak pixel in quadrant (skip 4px border margin)
            int y_start = std::max(quads[q].y0, 2);
            int y_end   = std::min(quads[q].y1, height - 2);
            int x_start = std::max(quads[q].x0, 2);
            int x_end   = std::min(quads[q].x1, width - 2);

            for (int y = y_start; y < y_end; y += 2) {
                const uint8_t* row = &img[y * width];
                for (int x = x_start; x < x_end; x += 2) {
                    if (row[x] > peak) {
                        peak = row[x]; px = x; py = y;
                    }
                }
            }
            s_last_stats.peak[q] = peak;

            // 2. If peak exceeds threshold, compute intensity-weighted centroid in window
            if (peak >= thr && px >= 0) {
                int wx0 = std::max(quads[q].x0, px - 14);
                int wx1 = std::min(quads[q].x1 - 1, px + 14);
                int wy0 = std::max(quads[q].y0, py - 14);
                int wy1 = std::min(quads[q].y1 - 1, py + 14);

                uint8_t local_thr = (thr > 15) ? (thr - 10) : thr;
                for (int y = wy0; y <= wy1; ++y) {
                    const uint8_t* row = &img[y * width];
                    for (int x = wx0; x <= wx1; ++x) {
                        uint8_t v = row[x];
                        if (v >= local_thr) {
                            uint32_t w = (v - local_thr + 1);
                            sum_x += (uint64_t)x * w;
                            sum_y += (uint64_t)y * w;
                            sum_w += w;
                        }
                    }
                }

                if (sum_w > 0) {
                    corners[q].x = (float)sum_x / (float)sum_w;
                    corners[q].y = (float)sum_y / (float)sum_w;
                    q_found[q] = true;
                }
            }
        }

        if (q_found[0] && q_found[1] && q_found[2] && q_found[3]) {
            // Geometric sanity check:
            // TL must be left of TR, BL must be left of BR
            // TL must be above BL, TR must be above BR
            if (corners[1].x > corners[0].x + 4.0f &&
                corners[2].x > corners[3].x + 4.0f &&
                corners[3].y > corners[0].y + 4.0f &&
                corners[2].y > corners[1].y + 4.0f)
            {
                res.p[0] = { corners[0].x, corners[0].y }; // TL
                res.p[1] = { corners[1].x, corners[1].y }; // TR
                res.p[2] = { corners[2].x, corners[2].y }; // BR
                res.p[3] = { corners[3].x, corners[3].y }; // BL
                res.count = 4;
                res.valid = true;
                res.confidence = 0.95f;
            }
        }
    }

    res.dt_us = get_time_us() - t_start;
    return res;
}
