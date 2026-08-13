// ov2640_bridge.h — the ONLY contract between our capture stack and the shim.
//
// The capture side (our esp32-camera fork + blob detector, running in the
// driver's copy-task) PUBLISHES the latest frame's blobs here. The shim
// (DFRobotIRPositionEx_OV2640.cpp) CONSUMES them from OpenFIRE's loop task.
// Nothing else crosses the boundary — keeping this struct tiny is what makes
// the whole overlay updatable against upstream OpenFIRE.
//
// Concurrency: single-writer (capture task) / single-reader (OpenFIRE loop).
// The seqlock pattern below is lock-free and never blocks the writer: the
// reader retries if a write overlapped its copy. On ESP32 both tasks may sit
// on different cores; the volatile seq + memory barrier via retry-compare is
// sufficient for this struct size (tens of bytes, torn reads detected by seq).
#pragma once
#include <stdint.h>

#define OV2640_BRIDGE_MAX_PTS 4

typedef struct {
    // coordinates in the CAMERA's native pixel grid, fixed-point x16
    // (240x176 sensor -> 0..3839 / 0..2815). The shim decides the output
    // scaling; the bridge stays in native units so nothing here changes if
    // the frame size ever does.
    uint16_t x16[OV2640_BRIDGE_MAX_PTS];
    uint16_t y16[OV2640_BRIDGE_MAX_PTS];
    uint8_t  area4[OV2640_BRIDGE_MAX_PTS]; // blob area >> 4, clamped 0..255
                                           // (shim clamps to DFRobot's 0..15)
    uint8_t  count;        // 0..4 valid points, brightest-first (by mass)
    uint8_t  frame_w_log;  // reserved
    uint16_t frame_w;      // native frame size these coords live in
    uint16_t frame_h;
    uint32_t frame_seq;    // increments once per finished frame
} ov2640_bridge_frame_t;

#ifdef __cplusplus
extern "C" {
#endif

// ---- writer side (capture stack) ----
// Call once per finished frame from the capture/copy task. Copies `f` into the
// published slot. Cheap (~tens of ns).
void ov2640_bridge_publish(const ov2640_bridge_frame_t* f);

// ---- reader side (shim) ----
// Copies the latest published frame into `out`. Returns the frame_seq, or 0 if
// nothing was ever published. Lock-free; retries internally on torn reads.
uint32_t ov2640_bridge_read(ov2640_bridge_frame_t* out);

#ifdef __cplusplus
}
#endif
