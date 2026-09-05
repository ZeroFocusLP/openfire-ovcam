#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Screen detection modes
typedef enum {
    SCREEN_MODE_BORDER = 0,     // Sinden-style rectangular perimeter border + top notch
    SCREEN_MODE_FIDUCIAL = 1,   // 4 corner high-contrast fiducials + TL orientation beacon
} screen_mode_t;

// Detection result containing 4 ordered corners (TL, TR, BR, BL) in subpixel native coords
typedef struct {
    bool valid;
    uint8_t count;              // 4 when successfully resolved
    struct {
        float x;                // 0.0 .. FRAME_W
        float y;                // 0.0 .. FRAME_H
    } p[4];                     // p[0]=TL, p[1]=TR, p[2]=BR, p[3]=BL
    float confidence;           // 0.0 .. 1.0 quality metric
    uint32_t dt_us;             // execution duration in microseconds
} screen_result_t;

// Real-time telemetry and statistics
typedef struct {
    uint8_t min_px;
    uint8_t max_px;
    uint8_t avg_px;
    uint8_t active_thr;
    uint8_t n_top;
    uint8_t n_bot;
    uint8_t n_left;
    uint8_t n_right;
    uint8_t peak[4];            // Quadrant peaks (TL, TR, BR, BL)
} screen_stats_t;

// Initialize / reset screen detector state
void screen_detector_init(void);

// Set detection sensitivity / threshold (0 = auto-adaptive)
void screen_detector_set_threshold(uint8_t threshold);
uint8_t screen_detector_get_threshold(void);

// Retrieve latest frame statistics
void screen_detector_get_stats(screen_stats_t* out);

// Detect screen border or fiducials in a grayscale HQVGA (240x176) image buffer
screen_result_t screen_detect(const uint8_t* img, int width, int height, screen_mode_t mode);

#ifdef __cplusplus
}
#endif
