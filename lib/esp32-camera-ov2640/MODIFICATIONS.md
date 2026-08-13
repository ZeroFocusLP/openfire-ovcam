# Modifications to the vendored esp32-camera driver

Base: Espressif `esp32-camera` (Apache-2.0). Per Apache-2.0 §4(b), modified
files carry notice. Files changed for this project, and why:

* **driver/cam_hal.c** — the core capture-robustness patches:
  * EOF/VSYNC event-race fix: pending EOF events are drained before a frame is
    finalized, so a completion landing on the frame boundary can no longer
    splice rows from two frames (the patch block is marked in-line).
  * `PATCH_ACCEPT_SHORT_FRAMES`: deliver frames >=90% complete instead of
    dropping them (consumer-side geometry gate decides what to do with them).
  * `cam_patch_*` fault counters (chunk rejects, stitch rejects, overflow
    attribution, VSYNC->restart latency) and an optional per-chunk callback
    (`cam_patch_chunk_cb`) that lets detection run inside the copy-task.
  * `PATCH_CAM_TASK_CORE1`: pin the copy-task to core 1.
* **target/esp32s3/ll_cam.c** — ISR-grade instrumentation (VSYNC period
  measured in the ISR, EOF/VSYNC ISR counters) used by the telemetry.
* **driver/sccb-ng.c** — SCCB read-failure reporting: a failed read now
  reports failure instead of returning 0x00, so read-modify-write register
  updates cannot silently clear a whole register.
* **library.json** — build metadata only.

All other files are unmodified upstream. Original copyright headers are
retained throughout.
