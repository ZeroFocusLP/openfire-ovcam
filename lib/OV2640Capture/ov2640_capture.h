// ov2640_capture.h — the lean capture core (M6). Camera init + in-driver blob
// detection publishing into ov2640_bridge. No WiFi, no dashboard, no RTC: the
// lab (firmware/) owns tuning; this core ships the PROVEN v73 S3 boot recipe
// as fixed defaults. Started (idempotently) from the shim's begin().
#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
// Returns 0 on success (or if already started). Safe to call repeatedly.
int ov2640_capture_start(void);

// v9.2 frame-gate statistic (harness G-line / field diagnostics). Driver-side
// rejections (stitched frames, FB-SIZE) never reach the chunk callback and are
// counted by the driver's own cam_patch_* counters instead.
// v28: ok / rej_period / rej_marker are GONE. They were written only by the
// full-stack (lab=0) path and its VSYNC-period gate, both removed. One rule is
// left -- exact frame size -- and this is its counter.
extern volatile uint32_t ov2640_stat_rej_size;    // byte count != full frame

// v11.2 live tuning (bench): "thr=90&boost=0&aec=40". Applied immediately.
// v28: DIAGNOSTIC BUILD ONLY. Compiled out when LIGHTGUN_DIAG is 0, so the
// shipping firmware has no serial command surface at all. The harness build
// defines LIGHTGUN_DIAG=1 and keeps it.
#if !defined(LIGHTGUN_DIAG) || LIGHTGUN_DIAG
void ov2640_tune(const char* cmd);
#endif
#ifdef __cplusplus
}
#endif
