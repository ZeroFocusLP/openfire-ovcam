// ov2640_capture.cpp — camera capture core. See ov2640_capture.h.
//
// Pipeline (all of it runs inside the camera driver's copy-task):
//   DMA chunk -> streaming blob detector -> duplicate/smear filter ->
//   quad resolver (persistent corner identity + rigid reconstruction) ->
//   lock-free bridge -> DFRobotIRPositionEx shim -> OpenFIRE.
//
// The boot recipe below (BOOT_*) is the measured-good configuration for the
// OV2640 "NV" module with a 700nm long-pass filter. Retune via the BOOT_*
// constants, or live over serial in the diagnostic build (see LIGHTGUN_DIAG).
#include "ov2640_capture.h"
#include "ov2640_bridge.h"
#include "blob_detector.h"
#include "quad_resolver.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"      // v16: MEM line (PSRAM presence + DRAM headroom)
#include "screen_detector.h"

// Freenove S3-WROOM CAM pin map (matches firmware/src/board_esp32s3.h)
#define P_XCLK 15
#define P_SIOD 4
#define P_SIOC 5
#define P_D7 16
#define P_D6 17
#define P_D5 18
#define P_D4 12
#define P_D3 10
#define P_D2 8
#define P_D1 9
#define P_D0 11
#define P_VSYNC 6
#define P_HREF 7
#define P_PCLK 13

// ===========================================================================
// v28 DIAGNOSTIC BUILD SWITCH
// ---------------------------------------------------------------------------
// LIGHTGUN_DIAG=1 compiles the instrumentation that found every bug in this
// project: the per-second telemetry block, the dashboard stream, and the UART0
// tune console. tools/dashboard.py and tools/health.py both parse that output,
// so the diag build is what you flash when something needs diagnosing.
//
// LIGHTGUN_DIAG=0 (the shipping default) removes all of it at compile time.
//
// WHAT IS DELIBERATELY *NOT* BEHIND THIS FLAG, and why:
//   * drain_task itself. Its first act every iteration is esp_camera_fb_get()/
//     fb_return() -- that is THE framebuffer drain. Without it the driver's
//     queue fills and capture stops. The task is load-bearing; only its print
//     bodies are diagnostic.
//   * the one-line boot banner. It is the stale-build check, and this project
//     has shipped a stale .pio more than once. One line at boot costs nothing
//     and is the only way to confirm what is actually flashed.
//   * the ISR counters in cam_hal.c. A handful of increments per interrupt,
//     unmeasurable, and cam_hal.c is the file we least want to destabilise.
// ===========================================================================
#ifndef LIGHTGUN_DIAG
#define LIGHTGUN_DIAG 0
#endif

static const int   FRAME_W = 240, FRAME_H = 176;

// ===========================================================================
// BOOT RECIPE — the measured-good configuration. Change these four, and
// nothing else, to retune.
// ---------------------------------------------------------------------------
// Verified on hardware (tools/health.py scoring 100/100): 4-point yield
// 135/135, count churn 0/s, rejected frames 0/s, ~134 fps, VSI spread 9us.
//
// Why these values: a HIGH threshold bench-tuned on a single on-axis LED
// starves a real 4-LED rig off-axis -- most frames then publish fewer than 4
// points, and OpenFIRE answers a short count by fabricating the missing
// corner and bleeding the error off over ~70 frames. Aim cannot survive that.
// Lower threshold + real exposure (aec) instead of amplification (agc/boost)
// keeps the noise floor down at the source.
//
// boost is OV2640 REG45[7:6] = two extra analog doubling stages (x4 on top of
// the PGA). Raising boost requires raising thr with it; that direction is
// fragile. Prefer exposure.
static const int BOOT_THR   = 80;   // pixel threshold (runtime: "thr=N")
static const int BOOT_AEC   = 40;   // exposure in lines (runtime: "aec=N")
static const int BOOT_AGC   = 2;    // PGA index, 0..30 (runtime: "agc=N")
static const int BOOT_BOOST = 0;    // REG45[7:6] x4 stages (runtime: "boost=N")
// ===========================================================================

static uint8_t  THR = (uint8_t)BOOT_THR;   // runtime-tunable via ov2640_tune()
static const uint32_t MIN_PX = 4;   // v13: MATCHES THE LAB. wifi_blob_map sets
                                    // g_scan_min_px = 4 whenever sysclk > 45 and
                                    // y8 is off — which is our exact case at
                                    // xclk 27 (sysclk 54). The overlay had 3.
// v15: the lab computes g_scan_max_px = w*h/4 at init (240*176/4 = 10560), not
// the 10000 literal we carried. Aligned — part of "no unexplained differences".
static const uint32_t MAX_PX = (uint32_t)(240 * 176 / 4), PX_BUDGET = 8000;


extern "C" {
    // exported by our patched cam_hal.c
    typedef void (*cam_chunk_cb_t)(const uint8_t* base, size_t len, bool frame_end);
    extern cam_chunk_cb_t cam_patch_chunk_cb;
    // v9.1: VSYNC period measured in the ISR (ll_cam.c) — immune to the
    // cam_task scheduling jitter that pollutes a task-context measurement.
    extern volatile uint32_t cam_patch_vsync_isr_period_us;
    // v12.1: the driver's own vertical-alignment instrumentation.
    // cam_patch_restart_us = worst time between the real VSYNC and
    // cam_start_frame(). ll_cam_start() begins capturing wherever the sensor
    // currently is, so every microsecond of that delay is a VERTICAL OFFSET
    // (~25us per line at 136fps); if it varies frame to frame, the picture
    // rolls vertically.
    extern volatile uint32_t cam_patch_stitch_rej;
    extern volatile uint32_t cam_patch_restart_us;
    extern volatile uint32_t cam_patch_restart_us_last;   // v12.4
    extern volatile uint32_t cam_patch_nostart;
    extern volatile uint32_t cam_patch_vs_long;
    extern volatile uint32_t cam_patch_vs_short;
    extern volatile uint32_t cam_patch_chunk_rej;        // v12.2
    extern volatile uint32_t cam_patch_chunks_expected;  // v12.2
    // v14: the same per-second fault attribution the LAB puts in its STAT line,
    // so tools/dashboard.py decodes the overlay exactly as it decodes the lab.
    extern volatile uint32_t cam_patch_ovf_vsync;
    extern volatile uint32_t cam_patch_ovf_eof;
    extern volatile uint32_t cam_patch_vs_min_us;
    extern volatile uint32_t cam_patch_vs_max_us;
    extern volatile uint32_t cam_patch_vs_count;
    extern volatile uint64_t cam_patch_vs_sum_us;
    extern volatile uint32_t cam_patch_vs_ref_us;
    // v15: 0 = the two overlay-only frame gates are OBSERVE-ONLY (lab-identical,
    // the default); 1 = they reject framebuffers as they did in v9.1/v12.2.
    extern volatile uint32_t cam_patch_gate_en;
    extern volatile uint32_t cam_patch_gap_max_us;    // v15.2 starvation meter
    extern volatile uint32_t cam_patch_gap_over1ms;
    extern volatile uint32_t cam_patch_gap_over3ms;
    extern volatile uint32_t cam_patch_vs_total;          // v15.3, free-running
    extern volatile uint32_t cam_patch_frames_delivered;
    extern volatile uint32_t cam_patch_frames_rejected;
    extern volatile uint32_t cam_patch_rej_size;      // v15.5 attribution
    extern volatile uint32_t cam_patch_rej_queue;
    // v15.1: DMA ring geometry, filled in at cam_config. This is the overrun
    // budget: dma_halfs * lines_per_half * t_LINE is how long cam_task may be
    // blocked before the DMA laps the ring and the frame is spliced.
    extern volatile uint32_t cam_patch_evq_depth;
    extern volatile uint32_t cam_patch_dma_halfs;
    extern volatile uint32_t cam_patch_dma_lines;
    // v16: chunks-per-frame histogram. cam_patch_chunk_rej only said
    // "cnt != expected"; this says WHICH WAY, and the two directions have
    // opposite causes (short = restart-late or masked/coalesced EOF interrupt;
    // long = frame ran past its VSYNC = genuine stitch).
    extern volatile uint32_t cam_patch_cnt_hist[16];
    // v17: ISR-grade VSYNC period + raw ISR event counts. Splits "the interrupt
    // is late" from "the event is dequeued out of order" -- see cam_hal.c.
    extern volatile uint32_t cam_patch_vsi_min_us;
    extern volatile uint32_t cam_patch_vsi_max_us;
    extern volatile uint32_t cam_patch_vsi_cnt;
    extern volatile uint64_t cam_patch_vsi_sum_us;
    extern volatile uint32_t cam_patch_eof_isr_total;
    extern volatile uint32_t cam_patch_vs_isr_total;
    extern volatile uint32_t cam_patch_eof_drained;   // v19 race fix
}

static volatile bool s_started = false;
static sensor_t* s_sensor = nullptr;   // v11.2: kept for live tuning
static uint32_t s_frame_seq = 0;

// ---- FRAME GATE: ONE RULE ---------------------------------------------------
// A frame that started misaligned by a few rows shifts every blob vertically
// for that frame (symptom: points teleport down and come back). The gate is
// the single rule that has ever fired here:
//     publish only if the byte count is EXACTLY one full frame (240x176).
// A short frame is missing lines because capture restarted late, so every y
// carries an unknown offset. A rejected frame is "no information": nothing is
// published and the bridge keeps last-good. Counted in ov2640_stat_rej_size.
//
// The root cause those other two rules were chasing was found and fixed in the
// driver instead (the v19 EOF/VSYNC race in cam_hal.c) -- which is why they
// stopped firing.
//
// t_LINE: frame period / total lines. CIF timing = 336 lines (datasheet Fig 15);
// at 136fps that is 7353/336 = 21.9us per line.
#if LIGHTGUN_DIAG
static const float T_LINE_US = 21.9f;
#endif

// v9.2: the ONE surviving gate statistic. ok/rej_period/rej_marker were
// only ever written by the full-stack (lab=0) path, removed in v28.
volatile uint32_t ov2640_stat_rej_size = 0;

// v12 IN-SITU DEBUG STREAM. Every capture so far came from the HARNESS — never
// from the real combined firmware. If the teleport is produced by something
// that only exists in the combined build (task timing, the display's I2C, the
// wireless stack, OpenFIRE's own state), the harness can never show it.
// printf() on this board goes to UART0 (the CH340 COM port), NOT to the native
// USB CDC that the OpenFIRE app speaks on — so this is safe to leave running
// while the gun is docked and aiming. Enable with "dbg=1" (ov2640_tune).
// Prints EXACTLY what the shim will hand OpenFIRE, in its 0..1023 / 0..767
// space, decimated to ~10Hz.
#if LIGHTGUN_DIAG
static volatile bool s_dbg = true;   // ON by default: the combined
                                     // build has no console to enable it

// The pipeline is deliberately MINIMAL: detector -> filter -> publish, every
// frame, no gates and no corrections. A heavily-staged alternative (frame
// gates, a multi-stage point stabilizer, restart re-alignment) was A/B tested
// against this and lost decisively; it was removed rather than kept dormant.

// v15.4 ghost meter accumulators (see the GHOST METER block in on_chunk).
// (still inside #if LIGHTGUN_DIAG)
static volatile uint32_t s_ghost_over4 = 0;   // frames where the detector saw >4
static volatile uint32_t s_jump_pts   = 0;   // v15.5 points that moved >=8 lines
static volatile uint32_t s_jump_q16   = 0;   // ...of which, on the 16-line grid
static volatile float    s_jump_worst = 0;   // worst single move, lines
static volatile uint8_t  s_ghost_max   = 0;   // most blobs seen in one frame

// ---- DASHBOARD LINK (diagnostic builds) -----------------------------------
// tools/dashboard.py attaches to UART0 and plots the live point stream.
// The dashboard's whole live view is one 21-byte line per frame plus one
// STAT line per second. At 136fps that is ~2.9 KB/s on a 11.5 KB/s UART —
// 25% of the wire. What it does cost is TIME: printf() to UART0 busy-waits on
// the TX FIFO, ~1.8ms per line, so at full rate this task spends ~25% of a
// second waiting. That is harmless in the harness and NOT obviously harmless
// under OpenFIRE, so the stream is OFF by default and rate-limitable.
//   dash=1        stream B-lines + STAT (dashboard-compatible)
//   dash=0        off (default)
//   dashb=0..3    which point goes in the B-line (dashboard plots ONE)
//   dashhz=N      cap the B rate (0 = every frame; 60 halves the wire cost)
// The one thing NOT supported is SNAP (FRM + 42KB of pixels): the overlay
// never keeps a framebuffer copy — that is a lab-firmware feature by design.
// v15: 0 = off, 1 = B-lines (one point, lab wire format), 2 = Q-lines (all
// four). Q is ~50 bytes vs 21, so mode 2 starts rate-capped; dashhz=0 lifts it.
static volatile uint8_t  s_dash = 0;
static volatile uint8_t  s_dash_blob = 0;
static volatile uint32_t s_dash_min_dt_us = 0;    // 0 = publish every frame
#endif  // LIGHTGUN_DIAG (diagnostic state)
// Shadowed because the dashboard's panel is fed from the STAT line and the
// sensor's own registers are write-only through this API.
static int s_cfg_aec = BOOT_AEC, s_cfg_agc = BOOT_AGC, s_cfg_boost = BOOT_BOOST;

// UART0 ownership. printf() always lands on UART0 (the CH340 port) in both
// builds, so the dashboard can always READ us. Sending commands BACK needs
// somebody to drain UART0's RX:
//   harness  (ARDUINO_USB_CDC_ON_BOOT unset)  -> Serial IS UART0, and the
//            harness sketch already forwards every line to ov2640_tune().
//            Installing a second driver here would fight it.
//   combined (ARDUINO_USB_CDC_ON_BOOT=1)      -> Serial is the USB CDC that
//            OpenFIRE owns; UART0 is unused, so we take its RX ourselves.
// Build with -D OV_DASH_NO_UART0_RX to opt out entirely.
// v28: the tune console is DIAGNOSTIC-ONLY. In the shipping build nothing
// reads UART0 and no uart driver is installed at all.
#if LIGHTGUN_DIAG && defined(ARDUINO_USB_CDC_ON_BOOT) && \
    (ARDUINO_USB_CDC_ON_BOOT == 1) && !defined(OV_DASH_NO_UART0_RX)
  #define OV_DASH_OWN_UART0 1
  #include "driver/uart.h"
#else
  #define OV_DASH_OWN_UART0 0
#endif

// Duplicate/smear filter: drop duplicates within 4px, drop smear stripes
// (3+ blobs sharing a column), keep the first N by mass.
//
// The candidate cap matters more than it looks: with a wide cap, every
// duplicate the filter drops gets BACKFILLED from the dim tail of the mass
// ranking -- the faintest, most transient blobs in the frame -- and a
// downstream consumer that trusts count>=3 will happily lock onto that
// rotating cast of junk. Capping candidates at 4 kills that failure mode;
// res=2 widens the caps only because the quad resolver rejects junk by
// geometry instead (see the publish path).
static const int LAB_CANDIDATES = 4;

// ---- TEMPORAL COINCIDENCE GATE (legacy path, res=0) ------------------------
// Requires a blob in 2 consecutive frames before accepting it, which kills
// one-frame noise blobs. OpenFIRE takes seen()==0x0F as four real corners,
// sorts them geometrically (OF_Square_Advanced.cpp:409-414) and then holds a
// wrong assignment for ~70 frames through its kinematic spring (:500-516), so
// one junk blob for one frame is worth ~0.5s of wrong quad. The gate costs one
// frame of ACQUISITION latency and zero tracking latency.
//
// The history holds the CANDIDATES, not the survivors. That matters: if it held
// only survivors, a genuinely new LED could never get in (it fails frame 1, so
// it is never in the history for frame 2). Storing candidates means a real
// emitter is admitted on its second frame, while a one-frame blip is never
// admitted at all.
static const float COIN_R2 = 12.0f * 12.0f;   // px^2; > centroid noise (0.3px),
                                              // < LED spacing, and generous
                                              // enough for fast pans
static volatile bool     s_coin = true;       // "coin=0" disables for A/B
static volatile uint32_t s_coin_killed = 0;   // blobs dropped, per-second delta

// ---- PUBLISH-BOUNDARY AUDIT -------------------------------------------------
// The statistic that predicts OpenFIRE's behaviour is not a point map, it is
// the COUNT and its CHURN at the publish boundary: OpenFIRE answers count<4 by
// INVENTING the missing corners (OF_Square_Advanced.cpp:305-341 parallelogram
// for 3 points, :182-297 aspect-ratio synthesis for 2) and then loads its
// kinematic spring (:500-516), which bleeds the offset off at 1 unit/frame --
// one dropout buys ~70 frames of wrong quad. pub[i] = frames published with
// count i; churn = count changed vs the previous frame (what fires the spring).
// ---- v22 QUAD RESOLVER -----------------------------------------------------
// Persistent corner identity + rigid reconstruction (lib/OV2640Capture/
// quad_resolver.*). Supersedes the coincidence gate when on: the gate answered
// a dropout by REMOVING a point, which is the one thing that makes OpenFIRE
// fabricate geometry. The resolver answers it by reconstructing the corner from
// the learned rig shape and keeping the count at 4, so OpenFIRE's geometric
// re-sort and its kinematic spring never fire. "res=0" reverts to the gate.
// 0 = off (legacy coincidence gate), 1 = resolver fed the top-4 by mass,
// 2 = resolver fed up to QUAD_MAX_IN raw blobs and choosing four by geometry.
static volatile uint8_t s_resolver = 2;
static volatile float s_res_conf = 0.0f;
// v26 END-TO-END LATENCY. Stamped here at publish; the shim subtracts it the
// moment OpenFIRE actually consumes the frame, so ov2640_shim_lat_us is TRUE
// capture-to-consumption, not a guess from a budget. Plain u32 across two
// tasks: a torn read costs one bogus sample in a max, which is acceptable for
// a diagnostic and cheaper than a lock in the frame path.
// Defined here (not in the shim header — a header definition is a duplicate
// symbol in every TU past the first); declared extern in DFRobotIRPositionEx.h.
volatile uint32_t ov2640_pub_t_us       = 0;
volatile uint32_t ov2640_shim_lat_us    = 0;   // worst since read, written by the shim
volatile uint32_t ov2640_shim_lat_last_us = 0; // most recent sample
static volatile uint32_t s_pub_hist[5] = {0,0,0,0,0};
static volatile uint32_t s_pub_churn = 0;

static BlobResult coincidence_gate(const BlobResult& in)
{
    static float pcx[LAB_CANDIDATES], pcy[LAB_CANDIDATES];
    static int   pn = 0;
    BlobResult out{}; out.count = 0;
    for (int i = 0; i < in.count; ++i) {
        bool seen_before = false;
        for (int k = 0; k < pn; ++k) {
            const float dx = in.blobs[i].cx - pcx[k];
            const float dy = in.blobs[i].cy - pcy[k];
            if (dx * dx + dy * dy <= COIN_R2) { seen_before = true; break; }
        }
        if (seen_before) out.blobs[out.count++] = in.blobs[i];
        else             s_coin_killed++;
    }
    pn = in.count < LAB_CANDIDATES ? in.count : LAB_CANDIDATES;   // candidates
    for (int k = 0; k < pn; ++k) { pcx[k] = in.blobs[k].cx; pcy[k] = in.blobs[k].cy; }
    return out;
}

// v27: the caps are now parameters. This function does THREE things and only
// the first is a mass-rank trim:
//   1. keep the top `cand_cap` blobs by mass,
//   2. drop duplicates within 4px,
//   3. drop "smear stripes" -- a third blob sharing a column with two others,
//      which is what a DMA-spliced twin looks like.
// (2) and (3) are load-bearing: they are what kills the "each LED shows a
// twin above/below it" artifact. Only (1) is the part the resolver wants to skip,
// so res=2 asks for wide caps rather than bypassing the filter entirely --
// bypassing would have handed the twins straight to the resolver.
static BlobResult lab_filter_blobs(const BlobResult& in,
                                   int cand_cap = LAB_CANDIDATES, int out_cap = 4)
{
    BlobResult out{}; out.count = 0;
    if (out_cap > BLOB_MAX_OUT) out_cap = BLOB_MAX_OUT;
    if (out_cap < 1) out_cap = 1;
    const int n = in.count < cand_cap ? in.count : cand_cap;
    for (int i = 0; i < n; ++i) {
        const Blob& b = in.blobs[i];
        bool drop = false;
        int col_mates = 0;
        for (int k = 0; k < out.count; ++k) {
            float dx = b.cx - out.blobs[k].cx;
            float dy = b.cy - out.blobs[k].cy;
            if (dx * dx + dy * dy < 16.f) { drop = true; break; }   // duplicate
            if (dx < 3.5f && dx > -3.5f) col_mates++;               // same column
        }
        if (!drop && col_mates >= 2) drop = true;                   // smear stripe
        if (!drop) {
            out.blobs[out.count++] = b;
            if (out.count == out_cap) break;
        }
    }
    return out;
}

// Runs in the DRIVER's copy-task (core 1) — same pattern as the lab's
// on_camera_chunk, minus flood-guard telemetry (detector still aborts floods).
// v27: the handle of whatever task actually runs the capture path. Captured
// from inside it rather than looked up by name, because the telemetry that
// reports stack headroom runs in drain_task -- passing NULL there would have
// measured the WRONG task's stack and reported a comfortable number for a
// budget that was never at risk. No FreeRTOS config dependency either
// (xTaskGetHandle needs INCLUDE_xTaskGetHandle; this does not).
static volatile TaskHandle_t s_cam_task = nullptr;

#ifndef DEFAULT_TRACK_MODE
#define DEFAULT_TRACK_MODE TRACK_MODE_SCREEN_FIDUCIAL
#endif

static volatile track_mode_t s_track_mode = DEFAULT_TRACK_MODE;
static volatile uint32_t s_screen_detect_us = 0;

extern "C" void ov2640_set_track_mode(track_mode_t mode) {
    s_track_mode = mode;
    if (s_sensor) {
        if (mode == TRACK_MODE_IR_BLOBS) {
            // Restore IR settings
            s_sensor->set_exposure_ctrl(s_sensor, 0);
            s_sensor->set_gain_ctrl(s_sensor, 0);
            s_sensor->set_aec_value(s_sensor, 40);
            s_sensor->set_agc_gain(s_sensor, 2);
            s_cfg_aec = 40; s_cfg_agc = 2;
            THR = 80;
        } else {
            // Visible screen settings: higher exposure and gain for LCD/OLED
            s_sensor->set_exposure_ctrl(s_sensor, 0);
            s_sensor->set_gain_ctrl(s_sensor, 0);
            s_sensor->set_aec_value(s_sensor, 300);
            s_sensor->set_agc_gain(s_sensor, 10);
            s_cfg_aec = 300; s_cfg_agc = 10;
            THR = 0;
            screen_detector_set_threshold(THR);
        }
    }
}

extern "C" track_mode_t ov2640_get_track_mode(void) {
    return s_track_mode;
}

extern "C" const char* ov2640_get_track_mode_name(void) {
    switch (s_track_mode) {
        case TRACK_MODE_SCREEN_BORDER: return "border";
        case TRACK_MODE_SCREEN_FIDUCIAL: return "fiducial";
        case TRACK_MODE_IR_BLOBS: default: return "ir";
    }
}

extern "C" uint32_t ov2640_get_detect_us(void) {
    return s_screen_detect_us;
}

static void on_chunk(const uint8_t* base, size_t len, bool frame_end)
{
    if (!s_cam_task) s_cam_task = xTaskGetCurrentTaskHandle();
    static size_t prev_len = 0;
    static bool began = false;

    if (len == 0) {                                  // explicit frame start
        if (s_track_mode == TRACK_MODE_IR_BLOBS)
            blobstream_begin(FRAME_W, THR, MIN_PX, MAX_PX, PX_BUDGET);
        began = true; prev_len = 0;
        return;
    }
    if (!began || len < prev_len) {
        if (s_track_mode == TRACK_MODE_IR_BLOBS)
            blobstream_begin(FRAME_W, THR, MIN_PX, MAX_PX, PX_BUDGET);
        began = true;
    }
    if (s_track_mode == TRACK_MODE_IR_BLOBS) {
        blobstream_feed(base, len);
    }
    prev_len = len;
    if (frame_end) {
        const size_t lab_full = (size_t)FRAME_W * (size_t)FRAME_H;
        if (len != lab_full) {
            ov2640_stat_rej_size++;              // counted, never published
            began = false; prev_len = 0;
            return;                              // keep last-good, like the lab
        }

        // ---- VISIBLE LIGHT SCREEN TRACKING PATH ----
        if (s_track_mode == TRACK_MODE_SCREEN_BORDER || s_track_mode == TRACK_MODE_SCREEN_FIDUCIAL) {
            screen_mode_t smode = (s_track_mode == TRACK_MODE_SCREEN_BORDER) ? SCREEN_MODE_BORDER : SCREEN_MODE_FIDUCIAL;
            screen_result_t sres = screen_detect(base, FRAME_W, FRAME_H, smode);
            s_screen_detect_us = sres.dt_us;
            ov2640_bridge_frame_t lf = {};
            lf.frame_w = FRAME_W; lf.frame_h = FRAME_H;
            lf.count = (sres.valid ? sres.count : 0);
            for (int i = 0; i < lf.count; ++i) {
                float cx = sres.p[i].x, cy = sres.p[i].y;
                if (cx < 0) cx = 0; if (cy < 0) cy = 0;
                uint32_t x16 = (uint32_t)(cx * 16.0f + 0.5f);
                uint32_t y16 = (uint32_t)(cy * 16.0f + 0.5f);
                const uint32_t xm = FRAME_W * 16u - 1u, ym = FRAME_H * 16u - 1u;
                lf.x16[i] = (uint16_t)(x16 > xm ? xm : x16);
                lf.y16[i] = (uint16_t)(y16 > ym ? ym : y16);
                lf.area4[i] = 8;
            }
            lf.frame_seq = ++s_frame_seq;
            ov2640_pub_t_us = (uint32_t)esp_timer_get_time();
            {
                static uint8_t s_prev_pub_count = 255;
                s_pub_hist[lf.count > 4 ? 4 : lf.count]++;
                if (s_prev_pub_count != 255 && lf.count != s_prev_pub_count)
                    s_pub_churn++;
                s_prev_pub_count = lf.count;
            }
            ov2640_bridge_publish(&lf);
            began = false; prev_len = 0;
            return;
        }

        static BlobResult lr;
        static BlobResult raw_lr;
        raw_lr = blobstream_finish();
#if LIGHTGUN_DIAG
        // ---- JUMP METER (diagnostic) --------------------------------
        // Static instruments lie here: a rectangular 4-LED bar HAS two
        // same-column pairs by construction, so "same column, displaced
        // in y" describes the RIG, not a ghost. This meter measures
        // CHANGE BETWEEN FRAMES instead, which is zero for any rig held
        // still, whatever its geometry.
        // A point is matched to the previous frame by x (points keep their
        // column; the splice displaces y), and a move of >=8 lines while
        // the gun is still cannot be optics or noise -- 0.3px is the
        // measured centroid noise. quant16 counts how many of those land
        // within 2 lines of a multiple of 16, i.e. on the DMA half-buffer
        // grid: that is the splice fingerprint, and it is what separates
        // "the gun moved" from "the frame was reassembled wrong".
        {
            static float pvx[8], pvy[8]; static int pvn = 0;
            for (int a = 0; a < raw_lr.count && a < 8; ++a) {
                float best = 1e9f; int bi = -1;
                for (int b = 0; b < pvn; ++b) {
                    const float d = raw_lr.blobs[a].cx - pvx[b];
                    const float ad = d < 0 ? -d : d;
                    if (ad < best) { best = ad; bi = b; }
                }
                if (bi >= 0 && best < 4.0f) {
                    float dy = raw_lr.blobs[a].cy - pvy[bi];
                    if (dy < 0) dy = -dy;
                    if (dy >= 8.0f) {
                        s_jump_pts++;
                        if (dy > s_jump_worst) s_jump_worst = dy;
                        const float q = dy / 16.0f;
                        float r = q - (float)(int)(q + 0.5f);
                        if (r < 0) r = -r;
                        if (r * 16.0f <= 2.0f) s_jump_q16++;
                    }
                }
            }
            pvn = raw_lr.count < 8 ? raw_lr.count : 8;
            for (int a = 0; a < pvn; ++a) { pvx[a] = raw_lr.blobs[a].cx; pvy[a] = raw_lr.blobs[a].cy; }
            if (raw_lr.count > 4) s_ghost_over4++;      // still valid: >4 blobs from 4 emitters
            if (raw_lr.count > s_ghost_max) s_ghost_max = (uint8_t)raw_lr.count;
        }
#endif  // LIGHTGUN_DIAG (jump meter)
        // res=2 wants the SAME filter with WIDER caps, not no filter --
        // see lab_filter_blobs(). One call either way; the legacy paths
        // below still see a list capped at 4.
        lr = (s_resolver >= 2) ? lab_filter_blobs(raw_lr, BLOB_MAX_OUT, QUAD_MAX_IN)
                               : lab_filter_blobs(raw_lr);
        ov2640_bridge_frame_t lf = {};
        lf.frame_w = FRAME_W; lf.frame_h = FRAME_H;
        if (s_resolver) {
            // v22: identity + rigid reconstruction. Emits a stable 4 once
            // locked, so OpenFIRE never sees a count change.
            //
            // v27 INPUT WIDTH. res=1 keeps the historical top-4-by-MASS
            // trim; res=2 (default) hands the resolver up to QUAD_MAX_IN
            // filtered blobs and lets it CHOOSE the four by geometry.
            //
            // Why that matters: mass ranking and corner identity disagree.
            // A bright reflection outranks a dim LED, so the top-4 trim can
            // hand re-acquire a quad with a real corner missing and junk in
            // its place -- and re-acquire has no way to know. It then locks
            // that wrong shape and holds it, which is the "come back on
            // screen and the rectangle is the wrong shape" report. Given
            // all 8, the subset search rejects any four that are not in
            // convex position and any whose implied deformation exceeds
            // what this rig has ever shown, so junk-containing subsets lose
            // on geometry rather than winning on brightness.
            //
            // NOTE it is still the FILTERED list, just a wider one: the
            // duplicate and smear-stripe rejections are what stopped the
            // DMA twins, and handing raw_lr over would have undone them.
            // LAB_CANDIDATES stays 4 for the legacy res=0 path, which has
            // no way to reject junk except by not seeing it.
            float qx[QUAD_MAX_IN], qy[QUAD_MAX_IN];
            int qn = lr.count; if (qn > QUAD_MAX_IN) qn = QUAD_MAX_IN;
            for (int i = 0; i < qn; ++i) { qx[i] = lr.blobs[i].cx; qy[i] = lr.blobs[i].cy; }
            const QuadResult q = quad_update(qx, qy, qn);
            s_res_conf = q.confidence;
            lf.count = (uint8_t)(q.count > 4 ? 4 : q.count);
            for (int i = 0; i < lf.count; ++i) {
                float cx = q.p[i].x, cy = q.p[i].y;
                if (cx < 0) cx = 0;
                if (cy < 0) cy = 0;
                uint32_t x16 = (uint32_t)(cx * 16.0f + 0.5f);
                uint32_t y16 = (uint32_t)(cy * 16.0f + 0.5f);
                const uint32_t xm = FRAME_W * 16u - 1u, ym = FRAME_H * 16u - 1u;
                lf.x16[i] = (uint16_t)(x16 > xm ? xm : x16);
                lf.y16[i] = (uint16_t)(y16 > ym ? ym : y16);
                // area is not tracked per corner and nothing downstream
                // reads size() on the square path (verified), so a nominal
                // value keeps the field well-formed without inventing data.
                // area doubles as the REAL/RECONSTRUCTED marker for the
                // dashboard: 8 = measured, 4 = synthesised by the resolver.
                // Nothing downstream reads size() on the square path
                // (verified), so the field is free for this.
                lf.area4[i] = q.p[i].real ? 8 : 4;
            }
            lf.frame_seq = ++s_frame_seq;
            ov2640_pub_t_us = (uint32_t)esp_timer_get_time();
            {
                static uint8_t s_prev_pub_count = 255;
                s_pub_hist[lf.count > 4 ? 4 : lf.count]++;
                if (s_prev_pub_count != 255 && lf.count != s_prev_pub_count)
                    s_pub_churn++;
                s_prev_pub_count = lf.count;
            }
            ov2640_bridge_publish(&lf);
            began = false; prev_len = 0;
            return;
        }
        // v18 legacy path: kill one-frame junk by REMOVING it.
        if (s_coin) lr = coincidence_gate(lr);
        lf.count = (uint8_t)(lr.count > 4 ? 4 : lr.count);
        for (int i = 0; i < lf.count; ++i) {
            float cx = lr.blobs[i].cx, cy = lr.blobs[i].cy;
            if (cx < 0) cx = 0;
            if (cy < 0) cy = 0;
            uint32_t x16 = (uint32_t)(cx * 16.0f + 0.5f);
            uint32_t y16 = (uint32_t)(cy * 16.0f + 0.5f);
            uint32_t xm = FRAME_W * 16u - 1u, ym = FRAME_H * 16u - 1u;
            lf.x16[i] = (uint16_t)(x16 > xm ? xm : x16);
            lf.y16[i] = (uint16_t)(y16 > ym ? ym : y16);
            uint32_t a4 = lr.blobs[i].pixels >> 4;
            lf.area4[i] = (uint8_t)(a4 > 255 ? 255 : a4);
        }
        lf.frame_seq = ++s_frame_seq;
        {   // v20: measure exactly what OpenFIRE will be handed.
            static uint8_t s_prev_pub_count = 255;
            s_pub_hist[lf.count > 4 ? 4 : lf.count]++;
            if (s_prev_pub_count != 255 && lf.count != s_prev_pub_count)
                s_pub_churn++;
            s_prev_pub_count = lf.count;
        }
        ov2640_bridge_publish(&lf);
        began = false; prev_len = 0;
    }
}

#if OV_DASH_OWN_UART0
// Drain UART0's RX and hand whole lines to the tuner. The dashboard sends
// "k=v&k=v\n", which is exactly what ov2640_tune() already parses.
static void dash_rx_poll(void)
{
    static char ln[96];
    static int  n = 0;
    uint8_t b;
    while (uart_read_bytes(UART_NUM_0, &b, 1, 0) == 1) {
        if (b == '\n' || b == '\r') {
            if (n) { ln[n] = 0; n = 0; ov2640_tune(ln); }
        } else if (n < (int)sizeof(ln) - 1) {
            ln[n++] = (char)b;
        } else {
            n = 0;                                   // overlong: resync
        }
    }
}
#endif

// One dashboard-format fix line per published frame. Native camera pixels
// x10 — the same units the blob-test firmware streams, so both builds are
// comparable in one viewer. Q carries ALL FOUR points, because "is the quad
// itself moving, or is the solve amplifying it?" cannot be answered from one
// point. B (single point) is kept for compatibility.
#if LIGHTGUN_DIAG
static inline int dash_x10(uint16_t v16) { return (int)(((uint32_t)v16 * 10u + 8u) / 16u); }

static void dash_emit_fix(const ov2640_bridge_frame_t& f, uint32_t now_ms)
{
    if (s_dash == 1) {                               // legacy single-point form
        const uint8_t i = s_dash_blob;
        if (f.count == 0 || i >= f.count) {
            // n is still the truth: "4 points, but the one you asked for is not
            // among them" reads differently from "nothing seen at all".
            printf("B,%lu,%u,-1,-1\n", (unsigned long)now_ms, (unsigned)f.count);
            return;
        }
        printf("B,%lu,%u,%d,%d\n", (unsigned long)now_ms, (unsigned)f.count,
               dash_x10(f.x16[i]), dash_x10(f.y16[i]));
        return;
    }
    int x[4], y[4];
    for (int i = 0; i < 4; ++i) {
        const bool have = (i < f.count);
        x[i] = have ? dash_x10(f.x16[i]) : -1;
        y[i] = have ? dash_x10(f.y16[i]) : -1;
    }
    printf("Q,%lu,%u,%d,%d,%d,%d,%d,%d,%d,%d\n", (unsigned long)now_ms,
           (unsigned)f.count, x[0], y[0], x[1], y[1], x[2], y[2], x[3], y[3]);
}

// v14: the lab's STAT line, field for field, so dashboard.py's panel regexes
// (fps / restart= / vsync= / spread / lost=) all hit. Overlay-only counters
// ride at the end, where they land in the dashboard's log.
static void dash_emit_stat(uint32_t now_ms, float fps, uint32_t worst_gap_ms,
                           uint8_t n)
{
    const uint32_t rus = cam_patch_restart_us; cam_patch_restart_us = 0;
    const float line_us = T_LINE_US;
    const int off_lines = (int)(rus / line_us);
    uint32_t vmin = cam_patch_vs_min_us, vmax = cam_patch_vs_max_us;
    uint32_t vcnt = cam_patch_vs_count;
    uint64_t vsum = cam_patch_vs_sum_us;
    cam_patch_vs_min_us = 0xFFFFFFFF; cam_patch_vs_max_us = 0;
    cam_patch_vs_count = 0; cam_patch_vs_sum_us = 0;
    const uint32_t vavg = vcnt ? (uint32_t)(vsum / vcnt) : 0;
    const uint32_t vspread = (vcnt && vmax >= vmin) ? (vmax - vmin) : 0;
    // NON-DESTRUCTIVE. The lab zeroes these because it is their only reader;
    // here the harness R-line and the V-line read the SAME counters as
    // cumulative totals. Zeroing them would silently redefine two existing
    // diagnostics the moment the dashboard connects -- the exact trap that
    // cost us the "zero J-lines = the data is clean" retraction. Deltas
    // against our own snapshot instead.
    static uint32_t p_ovfv = 0, p_ovfe = 0, p_vlong = 0, p_vshrt = 0, p_nost = 0;
    const uint32_t c_ovfv = cam_patch_ovf_vsync, c_ovfe = cam_patch_ovf_eof;
    const uint32_t c_vlong = cam_patch_vs_long, c_vshrt = cam_patch_vs_short;
    const uint32_t c_nost = cam_patch_nostart;
    const uint32_t ovfv = c_ovfv - p_ovfv, ovfe = c_ovfe - p_ovfe;
    const uint32_t vlong = c_vlong - p_vlong, vshrt = c_vshrt - p_vshrt;
    const uint32_t nost = c_nost - p_nost;
    p_ovfv = c_ovfv; p_ovfe = c_ovfe; p_vlong = c_vlong;
    p_vshrt = c_vshrt; p_nost = c_nost;
    char lost[80];
    if (ovfv | ovfe | vlong | vshrt | nost)
        snprintf(lost, sizeof(lost), "lost=ovf%u/%u,skip%u,short%u,nofb%u,ref%uus",
                 (unsigned)ovfv, (unsigned)ovfe, (unsigned)vlong,
                 (unsigned)vshrt, (unsigned)nost, (unsigned)cam_patch_vs_ref_us);
    else
        snprintf(lost, sizeof(lost), "lost=0");
    // flood=/short= are LAB concepts (aborted scan / truncated frame) that the
    // overlay does not measure. They are pinned to 0 rather than repurposed:
    // reporting our frame-gate rejections through the lab's "short frames"
    // field would make two different faults read identically on one panel.
    // The overlay's own counters (rejP/rejS/rejM/chunkrej/stitch) already ride
    // the 1Hz V-line, which the dashboard logs in full -- STAT is capped at
    // 150 chars in its log view, so duplicating them here would only push the
    // lost= field off the edge.
    printf("STAT,%lu,%.1f,%.1f,%.0f,%d,thr=%d aec=%d agc=%d boost=%d y8=0 "
           "xclk=27 pdiv=3 flood=0%% short=0%% restart=%uus(~%dlines) "
           "vsync=%uus(min%u max%u spread%uus~%dlines) %s mode=%s\n",
           (unsigned long)now_ms, (double)fps, (double)worst_gap_ms, 0.0,
           (int)n, (int)THR, s_cfg_aec, s_cfg_agc, s_cfg_boost,
           (unsigned)rus, off_lines,
           (unsigned)vavg, (unsigned)(vcnt ? vmin : 0), (unsigned)vmax,
           (unsigned)vspread, (int)(vspread / line_us), lost, "LAB");
}
#endif  // LIGHTGUN_DIAG (dashboard stream)

// Drains the frame queue so the pipeline never stalls (detection already
// happened in the chunk callback; the fb itself is not needed).
static void drain_task(void*)
{
#if LIGHTGUN_DIAG
    uint32_t last_dbg = 0;
    uint32_t dash_last_seq = 0, dash_last_us = 0;
    uint32_t dash_win_ms = 0, dash_win_seq = 0, dash_gap_ms = 0, dash_prev_ms = 0;
#endif
    for (;;) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
        else vTaskDelay(1);
#if LIGHTGUN_DIAG  // tune console + dashboard + telemetry
#if OV_DASH_OWN_UART0
        dash_rx_poll();
#endif
        // ---- v14 dashboard stream (same task, same rule: never in on_chunk)
        if (s_dash) {
            ov2640_bridge_frame_t df;
            const uint32_t seq = ov2640_bridge_read(&df);
            const uint32_t us  = (uint32_t)esp_timer_get_time();
            const uint32_t ms  = us / 1000u;
            if (seq && seq != dash_last_seq) {
                if (!s_dash_min_dt_us || (uint32_t)(us - dash_last_us) >= s_dash_min_dt_us) {
                    dash_last_us = us;
                    dash_emit_fix(df, ms);
                }
                // worst frame-to-frame gap this second: the number that says
                // "we stopped seeing frames", independent of the rate cap.
                if (dash_prev_ms && (ms - dash_prev_ms) > dash_gap_ms)
                    dash_gap_ms = ms - dash_prev_ms;
                dash_prev_ms = ms;
                dash_last_seq = seq;
            }
            if (!dash_win_ms) { dash_win_ms = ms; dash_win_seq = seq; }
            else if (ms - dash_win_ms >= 1000u) {
                const float fps = (float)(seq - dash_win_seq) * 1000.0f
                                  / (float)(ms - dash_win_ms);
                dash_emit_stat(ms, fps, dash_gap_ms, df.count);
                dash_win_ms = ms; dash_win_seq = seq; dash_gap_ms = 0;
            }
        }
        // v12 debug stream — deliberately NOT in on_chunk: a printf there
        // blocks ~4ms on the UART, inside the DMA-chunk service path, which
        // is precisely what starves the driver and destroys capture. This
        // task is free to block.
        if (s_dbg) {
            const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
            // 10Hz normally; 1Hz while the dashboard is streaming, so the D
            // lines don't scroll its log faster than it can be read (the map
            // is showing the same data anyway).
            if (now - last_dbg >= (s_dash ? 1000u : 100u)) {
                last_dbg = now;
                ov2640_bridge_frame_t f;
                if (ov2640_bridge_read(&f)) {
                    const uint32_t xm = FRAME_W * 16u - 1u, ym = FRAME_H * 16u - 1u;
                    int sx[4] = {1023, 1023, 1023, 1023};
                    int sy[4] = {1023, 1023, 1023, 1023};
                    for (int i = 0; i < f.count; ++i) {  // mirror+scale = shim out
                        sx[i] = 1023 - (int)((uint32_t)f.x16[i] * 1023u / xm);
                        sy[i] = (int)((uint32_t)f.y16[i] * 767u / ym);
                    }
                    printf("D,%lu,n=%u,%d,%d,%d,%d,%d,%d,%d,%d\n",
                           (unsigned long)f.frame_seq, (unsigned)f.count,
                           sx[0], sy[0], sx[1], sy[1], sx[2], sy[2], sx[3], sy[3]);
                    // once a second: the vertical-alignment health line
                    static uint32_t last_v = 0;
                    if (now - last_v >= 1000) {
                        last_v = now;
                        // ---- v15.3 FRAME ACCOUNTING, printed FIRST ------
                        // Every frame the sensor starts must land in exactly
                        // one bucket. It currently does NOT: the sensor gives
                        // 135/s, ~26/s are rejected, and only 60-90/s are
                        // published. 20-45 frames a second were disappearing
                        // with no counter naming them, and the STAT field that
                        // might have said why (lost=) is past the width of a
                        // pasted line. So this is short, and it is first.
                        // `hole` is the number that matters: it must be 0.
                        {
                            static uint32_t p_vs = 0, p_dl = 0, p_rj = 0, p_pub = 0;
                            static uint32_t p_cs = 0, p_st2 = 0, p_sz = 0, p_qu = 0;
                            const uint32_t vs = cam_patch_vs_total;
                            const uint32_t dl = cam_patch_frames_delivered;
                            const uint32_t rj = cam_patch_frames_rejected;
                            const uint32_t pub = s_frame_seq;
                            const uint32_t d_vs = vs - p_vs, d_dl = dl - p_dl;
                            const uint32_t d_rj = rj - p_rj, d_pub = pub - p_pub;
                            printf("ACCT/s: sensor=%lu delivered=%lu rejected=%lu "
                                   "published=%lu | HOLE=%ld | why: chunk=%lu stitch=%lu size=%lu queue=%lu\n",
                                   (unsigned long)d_vs, (unsigned long)d_dl,
                                   (unsigned long)d_rj, (unsigned long)d_pub,
                                   (long)d_vs - (long)d_dl - (long)d_rj,
                                   (unsigned long)(cam_patch_chunk_rej - p_cs),
                                   (unsigned long)(cam_patch_stitch_rej - p_st2),
                                   (unsigned long)(cam_patch_rej_size - p_sz),
                                   (unsigned long)(cam_patch_rej_queue - p_qu));
                            p_vs = vs; p_dl = dl; p_rj = rj; p_pub = pub;
                            p_cs = cam_patch_chunk_rej; p_st2 = cam_patch_stitch_rej;
                            p_sz = cam_patch_rej_size;
                            p_qu = cam_patch_rej_queue;
                        }
                        {   // ---- v20 PTS: what OpenFIRE is actually handed.
                            // This, not the map, predicts the cursor. Every
                            // frame with count<4 makes OpenFIRE INVENT corners;
                            // every churn event fires its kinematic spring,
                            // which then holds the error for ~70 frames.
                            // Target: pub4 == published, churn == 0.
                            static uint32_t p_h[5] = {0,0,0,0,0}, p_ch = 0;
                            uint32_t d[5], tot = 0;
                            for (int b = 0; b < 5; ++b) {
                                const uint32_t v = s_pub_hist[b];
                                d[b] = v - p_h[b]; p_h[b] = v; tot += d[b];
                            }
                            const uint32_t ch = s_pub_churn;
                            const uint32_t d_ch = ch - p_ch; p_ch = ch;
                            const uint32_t bad = tot - d[4];
                            printf("PTS/s: pub0=%lu pub1=%lu pub2=%lu pub3=%lu pub4=%lu "
                                   "| short=%lu/%lu (%lu%%) churn=%lu/s | every short "
                                   "frame makes OpenFIRE INVENT a corner; churn fires "
                                   "its spring (~70 frames of held error)\n",
                                   (unsigned long)d[0], (unsigned long)d[1],
                                   (unsigned long)d[2], (unsigned long)d[3],
                                   (unsigned long)d[4], (unsigned long)bad,
                                   (unsigned long)tot,
                                   (unsigned long)(tot ? (bad * 100u / tot) : 0u),
                                   (unsigned long)d_ch);
                            if (s_resolver) {
                                const QuadStats qs = quad_take_stats();
                                printf("RES/s: affine=%lu sim=%lu coast=%lu reassoc=%lu "
                                       "junk=%lu envrej=%lu reseed=%lu lost=%lu conf=%.2f | tilt=%.2f "
                                       "learned_max=%.2f | perspective resid=%.2fpx "
                                       "(worst %.2f) | resid climbing => affine no "
                                       "longer enough\n",
                                       (unsigned long)qs.reconstructed,
                                       (unsigned long)qs.recon_sim,
                                       (unsigned long)qs.coasted,
                                       (unsigned long)qs.reassoc,
                                       (unsigned long)qs.dropped_blobs,
                                       (unsigned long)qs.env_rejects,
                                       (unsigned long)qs.reseeds,
                                       (unsigned long)qs.lock_losses,
                                       (double)s_res_conf,
                                       qs.aniso_x100 / 100.0,
                                       qs.env_aniso_x100 / 100.0,
                                       qs.resid_x100 / 100.0,
                                       qs.resid_max_x100 / 100.0);
                                // v27 STACK HEADROOM. The resolver runs INSIDE
                                // cam_task, which has a 4 KB stack and runs at
                                // priority 23 -- a stack overflow there is a
                                // panic, not a slowdown, and the v27 subset
                                // search added ~200 bytes of frame. Free bytes
                                // remaining, worst since boot. If this ever
                                // approaches a few hundred, raise CAM_TASK_STACK
                                // before anything else.
                                const unsigned long stack_free = s_cam_task
                                    ? (unsigned long)uxTaskGetStackHighWaterMark(
                                          (TaskHandle_t)s_cam_task) * sizeof(StackType_t)
                                    : 0UL;
                                printf("COST/s: resolver worst=%luus total=%luus "
                                       "(%.2f%% of cam_task) reshape=%lu | "
                                       "LATENCY capture->OpenFIRE: last=%luus worst=%luus | "
                                       "cam_task stack free=%lub\n",
                                       (unsigned long)qs.worst_us,
                                       (unsigned long)qs.total_us,
                                       qs.total_us / 10000.0,
                                       (unsigned long)qs.reshapes,
                                       (unsigned long)ov2640_shim_lat_last_us,
                                       (unsigned long)ov2640_shim_lat_us,
                                       stack_free);
                                ov2640_shim_lat_us = 0;
                            }
                        }
                        {   // ---- v17 ISR: the measurement that splits the last
                            // two hypotheses. VSI = VSYNC period measured IN THE
                            // ISR; the STAT/vsync= field is the same period
                            // measured in cam_task. If VSI is tight and the task
                            // one is bimodal ~690us, the interrupt is punctual
                            // and the EVENT is being attributed to the wrong
                            // frame (ordering). If VSI is ALSO ~690us, the
                            // interrupt itself is late (masking / ISR latency).
                            // eof/s must read ~11 x fps, else EOFs are coalescing.
                            static uint32_t p_eof = 0, p_vs = 0;
                            const uint32_t vmin = cam_patch_vsi_min_us;
                            const uint32_t vmax = cam_patch_vsi_max_us;
                            const uint32_t vcnt = cam_patch_vsi_cnt;
                            const uint64_t vsum = cam_patch_vsi_sum_us;
                            cam_patch_vsi_min_us = 0xFFFFFFFF; cam_patch_vsi_max_us = 0;
                            cam_patch_vsi_cnt = 0; cam_patch_vsi_sum_us = 0;
                            const uint32_t eof = cam_patch_eof_isr_total;
                            const uint32_t vst = cam_patch_vs_isr_total;
                            const uint32_t d_eof = eof - p_eof, d_vs = vst - p_vs;
                            p_eof = eof; p_vs = vst;
                            printf("ISR/s: vsync=%lu eof=%lu (expect eof=11xvsync=%lu) | "
                                   "VSI avg=%luus min=%lu max=%lu spread=%luus (~%.1f chunks) "
                                   "| tight VSI + wide STAT = ORDERING, both wide = ISR LATENCY\n",
                                   (unsigned long)d_vs, (unsigned long)d_eof,
                                   (unsigned long)(d_vs * 11u),
                                   (unsigned long)(vcnt ? (uint32_t)(vsum / vcnt) : 0),
                                   (unsigned long)(vcnt ? vmin : 0), (unsigned long)vmax,
                                   (unsigned long)((vcnt && vmax >= vmin) ? (vmax - vmin) : 0),
                                   (double)((vcnt && vmax >= vmin) ? (vmax - vmin) : 0)
                                       / (16.0 * (double)T_LINE_US));
                        }
                        {   // ---- v16 CHUNKS: the histogram the chunk gate hid.
                            // Prints only the buckets that moved this second.
                            // Healthy = everything in bucket 11 and nothing else.
                            // Buckets BELOW 11 are frames that arrived short
                            // (restart landed inside active video, or an EOF
                            // interrupt was masked/coalesced -- neither of which
                            // any ovf counter can see). Buckets ABOVE 11 are
                            // frames that ran past their VSYNC = real stitch.
                            static uint32_t p_h[16] = {0};
                            char hb[160]; int hn = 0; uint32_t tot = 0, lo = 0, hi = 0;
                            for (int b = 0; b < 16; ++b) {
                                const uint32_t v = cam_patch_cnt_hist[b];
                                const uint32_t d = v - p_h[b];
                                p_h[b] = v;
                                if (!d) continue;
                                tot += d;
                                if (b < 11) lo += d; else if (b > 11) hi += d;
                                if (hn < (int)sizeof(hb) - 24)
                                    hn += snprintf(hb + hn, sizeof(hb) - hn,
                                                   " %d:%lu", b, (unsigned long)d);
                            }
                            if (!hn) { hb[0] = ' '; hb[1] = '-'; hb[2] = 0; }
                            static uint32_t p_ck = 0, p_dr = 0;
                            const uint32_t ck = s_coin_killed;
                            const uint32_t dr = cam_patch_eof_drained;
                            printf("CHUNKS/s (cnt:frames):%s | total=%lu short=%lu "
                                   "long=%lu | drained=%lu/s | coin=%s killed=%lu/s "
                                   "| healthy = all in 11, short+long = 0\n",
                                   hb, (unsigned long)tot,
                                   (unsigned long)lo, (unsigned long)hi,
                                   (unsigned long)(dr - p_dr),
                                   s_coin ? "on" : "OFF",
                                   (unsigned long)(ck - p_ck));
                            p_ck = ck; p_dr = dr;
                        }
                        {   // v15.5 JUMP: geometry-independent. Hold still => 0.
                            static uint32_t p_o4 = 0, p_jp = 0, p_jq = 0;
                            const uint32_t o4 = s_ghost_over4, jp = s_jump_pts, jq = s_jump_q16;
                            printf("JUMP/s: moved=%lu onGrid16=%lu worst=%.1f lines "
                                   "over4=%lu maxBlobs=%u | holding still => moved must be 0\n",
                                   (unsigned long)(jp - p_jp), (unsigned long)(jq - p_jq),
                                   (double)s_jump_worst, (unsigned long)(o4 - p_o4),
                                   (unsigned)s_ghost_max);
                            p_o4 = o4; p_jp = jp; p_jq = jq;
                            s_jump_worst = 0; s_ghost_max = 0;
                        }
                        // v15.1 RATE, not just the running total. The A/B
                        // matrix in platformio.ini is measured with ONE number
                        // -- ring laps per second -- and making the human
                        // subtract two counters a second apart is how a clean
                        // experiment turns into a transcription error.
                        {
                            static uint32_t p_cr = 0, p_st = 0, p_rs = 0, p_vl = 0;
                            const uint32_t cr = cam_patch_chunk_rej, stc = cam_patch_stitch_rej;
                            const uint32_t rs = ov2640_stat_rej_size, vl = cam_patch_vs_long;
                            printf("LAPS/s: ring=%lu (chunkrej) stitch=%lu appRej=%lu "
                                   "sensorSkip=%lu | harness-alone reference ~1.2/s\n",
                                   (unsigned long)(cr - p_cr), (unsigned long)(stc - p_st),
                                   (unsigned long)(rs - p_rs), (unsigned long)(vl - p_vl));
                            p_cr = cr; p_st = stc; p_rs = rs; p_vl = vl;
                            static uint32_t p_g1 = 0, p_g3 = 0;
                            const uint32_t g1 = cam_patch_gap_over1ms, g3 = cam_patch_gap_over3ms;
                            printf("STARVE: worst gap %lu us this second (DMA needs "
                                   "servicing every ~350us) | >1ms x%lu  >3ms x%lu\n",
                                   (unsigned long)cam_patch_gap_max_us,
                                   (unsigned long)(g1 - p_g1), (unsigned long)(g3 - p_g3));
                            p_g1 = g1; p_g3 = g3; cam_patch_gap_max_us = 0;
                        }
                        // v28: rejP= dropped with the VSYNC-period gate that
                        // fed it. dashboard.py tolerates a missing key.
                        printf("V,restart_us=%lu,nostart=%lu,vs_long=%lu,"
                               "vs_short=%lu,rejS=%lu,chunks=%lu,"
                               "chunkrej=%lu,stitch=%lu\n",
                               (unsigned long)cam_patch_restart_us,
                               (unsigned long)cam_patch_nostart,
                               (unsigned long)cam_patch_vs_long,
                               (unsigned long)cam_patch_vs_short,
                               (unsigned long)ov2640_stat_rej_size,
                               (unsigned long)cam_patch_chunks_expected,
                               (unsigned long)cam_patch_chunk_rej,
                               (unsigned long)cam_patch_stitch_rej);
                        printf("Y,last_restart=%luus => y_fix=%.1f lines "
                               "(this is the vertical offset being removed)\n",
                               (unsigned long)cam_patch_restart_us_last,
                               (double)cam_patch_restart_us_last / 21.9);
                        // Running MAX: exactly ONE consumer may clear it, or
                        // each sees a fragment of the window and both under-
                        // report. When the dashboard stream is on, STAT owns it.
                        if (!s_dash) cam_patch_restart_us = 0;
                    }
                }
            }
        }
#endif  // LIGHTGUN_DIAG (per-second telemetry)
    }
}

int ov2640_capture_start(void)
{
    if (s_started) return 0;
    // Version banner: prints the constants ACTUALLY COMPILED into this flash.
    // If these numbers don't match the source on disk, the build was stale —
    // full-clean and re-upload. Added after shipping exactly that mistake.
    // v21: aec/agc/boost used to be HARDCODED LITERALS in this format string
    // ("aec=20 agc=0 ... boost=1"), so the banner kept reporting the old recipe
    // no matter what the build actually programmed. They are the real variables
    // now -- this line is the stale-build check (rule 5), it has to be true.
    // v28: KEPT IN BOTH BUILDS, deliberately. One line at boot, and the only
    // way to tell a fresh flash from a stale .pio -- which has bitten this
    // project more than once. The stab:/fgate: fields are gone with the code
    // they described; the DIAG/SHIP token says which build this is.
    printf("OV2640Capture v28 %s | thr=%u aec=%d agc=%d boost=%d xclk=27 pdiv=3 | "
           "coin=%s res=%s\n",
           LIGHTGUN_DIAG ? "DIAG" : "SHIP",
           (unsigned)THR, s_cfg_aec, s_cfg_agc, s_cfg_boost,
           s_coin ? "on" : "off",
           s_resolver == 2 ? "ON(geom8)" : (s_resolver == 1 ? "ON(top4)" : "off"));
#if LIGHTGUN_DIAG
    // v14: dashboard handshake. The " HQVGA " token sets tools/dashboard.py's
    // frame size, and parse_kv() seeds its panel from the k=v tail — otherwise
    // it would render a startup GUESS as though it were telemetry.
    printf("CFG-ACTIVE: HQVGA 240x176 gray thr=%u aec=%d agc=%d boost=%d y8=0 "
           "xclk=27 pdiv=3 dbl=1 div=0 r32=0\n",
           (unsigned)THR, s_cfg_aec, s_cfg_agc, s_cfg_boost);
    printf("dashboard: send \"dash=1\" for the B/STAT stream that "
           "tools/dashboard.py plots (off by default; ~2.9KB/s + ~25%% of this "
           "task busy-waiting on UART0 at full frame rate — use dashhz=60 to "
           "halve it). dashb=0..3 picks which point it plots. SNAP unsupported."
           "%s\n",
#if OV_DASH_OWN_UART0
           " UART0 RX is ours: dashboard commands work while OpenFIRE runs.");
#else
           " UART0 RX belongs to the sketch (harness forwards it).");
#endif
    {   // ===== v16 MEM LINE ===================================================
        // Positive confirmation that PSRAM is alive. Until v16 the board def was
        // an N8R2 (2MB QUAD psram) on N8R8 hardware (8MB OCTAL), so PSRAM failed
        // to init on every boot and the ONLY evidence was an early
        // "E (248) quad_psram: PSRAM chip is not connected" that scrolls past.
        // With CORE_DEBUG_LEVEL=2 the Arduino "PSRAM enabled" INFO line does not
        // print either, so absence of the error is not proof. This is proof.
        const size_t ps_tot = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
        const size_t ps_fre = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        const size_t dr_fre = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        const size_t dr_big = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
        printf("MEM: psram total=%uK free=%uK | internal free=%uK largest=%uK | %s\n",
               (unsigned)(ps_tot / 1024), (unsigned)(ps_fre / 1024),
               (unsigned)(dr_fre / 1024), (unsigned)(dr_big / 1024),
               ps_tot ? "PSRAM OK" : "!! PSRAM DEAD - board_build.arduino.memory_type "
                                     "must be qio_opi on this N8R8 hardware");
    }
#endif  // LIGHTGUN_DIAG (boot detail)
    quad_reset(nullptr);                             // v22: arm the quad resolver
    cam_patch_chunk_cb = on_chunk;                   // hook BEFORE init
    camera_config_t c = {};
    c.pin_pwdn = -1; c.pin_reset = -1; c.pin_xclk = P_XCLK;
    c.pin_sccb_sda = P_SIOD; c.pin_sccb_scl = P_SIOC;
    c.pin_d7 = P_D7; c.pin_d6 = P_D6; c.pin_d5 = P_D5; c.pin_d4 = P_D4;
    c.pin_d3 = P_D3; c.pin_d2 = P_D2; c.pin_d1 = P_D1; c.pin_d0 = P_D0;
    c.pin_vsync = P_VSYNC; c.pin_href = P_HREF; c.pin_pclk = P_PCLK;
    c.ledc_timer = LEDC_TIMER_0; c.ledc_channel = LEDC_CHANNEL_0;
    c.xclk_freq_hz = 27000000;                       // 136fps is ribbon-safe with pdiv=3:
                                                     // the ribbon limit is DVP BUS rate,
                                                     // not sensor rate — the divider slows
                                                     // the bus while the sensor runs 136fps
    c.pixel_format = PIXFORMAT_GRAYSCALE;
    c.frame_size = FRAMESIZE_HQVGA;
    c.fb_count = 2;
    c.fb_location = CAMERA_FB_IN_DRAM;
    c.grab_mode = CAMERA_GRAB_LATEST;
    esp_err_t e = esp_camera_init(&c);
    if (e != ESP_OK) return (int)e;
#if LIGHTGUN_DIAG
    // ===== v16: RING GEOMETRY, PRINTED AFTER init =============================
    // THIS BLOCK USED TO RUN *BEFORE* esp_camera_init(), so every number in it
    // read the uninitialised 0. Every log since v15.1 says
    //   "RING: evq=0 dma_halfs=0 lines/half=0 => overrun budget 0.00 ms"
    //   "ALIGN: frame=176 halves ring=0 halves | frame%ring=0 MISALIGNED"
    // -- and the "frame=176 halves" is just FRAME_H/1 because lines was 0, while
    // MISALIGNED printed unconditionally because halfs was 0. The ab_align table
    // in platformio.ini was therefore hand arithmetic that this build never
    // confirmed. cam_patch_dma_* are filled in by cam_config(), inside
    // esp_camera_init(), so the block belongs here.
    {
        const uint32_t halfs = cam_patch_dma_halfs, lines = cam_patch_dma_lines;
        const uint32_t fh = lines ? (FRAME_H / lines) : 0;   // halves per frame
        printf("ALIGN: frame=%lu halves ring=%lu halves | frame%%ring=%lu %s\n",
               (unsigned long)fh, (unsigned long)halfs,
               (unsigned long)(halfs ? fh % halfs : 0),
               (!halfs || !lines) ? "UNKNOWN -- driver did not publish geometry"
                 : ((fh % halfs) == 0) ? "ALIGNED (phase repeats every frame)"
                                       : "MISALIGNED -- ring phase walks every frame");
        printf("RING: evq=%lu dma_halfs=%lu lines/half=%lu => overrun budget "
               "%.2f ms (%lu lines) | gate=%s\n",
               (unsigned long)cam_patch_evq_depth, (unsigned long)halfs,
               (unsigned long)lines, (double)(halfs * lines) * T_LINE_US / 1000.0,
               (unsigned long)(halfs * lines),
               cam_patch_gate_en ? "ON (spliced frames rejected)" : "OFF");
        printf("EXPECT: dma_halfs=4 lines/half=16 evq=3 budget~1.40ms "
               "chunks/frame=11 -- if dma_halfs differs, the ring is NOT the "
               "lab's and CONFIG_CAMERA_DMA_BUFFER_SIZE_MAX is the difference\n");
    }
#endif  // LIGHTGUN_DIAG (ALIGN/RING geometry report)
    sensor_t* s = esp_camera_sensor_get();
    s_sensor = s;
    // v15 SCCB ACK CHECK + RETRY — parity with the lab's apply_sensor_regs().
    // The lab learned this the hard way (its v46): "a transient NACK on the
    // 0xFF bank-select left all later writes landing in the WRONG BANK ->
    // scrambled sensor -> crash". The overlay was throwing every return code
    // away, so a single NACK at boot produced a silently mis-programmed sensor
    // and a session of chasing ghosts downstream. Same recipe, now verified.
    int tries = 0;
    for (; s && tries < 3; ++tries) {                 // v73 recipe, fixed
        int rc = 0;
        rc |= s->set_whitebal(s, 0); rc |= s->set_special_effect(s, 0);
        rc |= s->set_lenc(s, 0);     rc |= s->set_raw_gma(s, 0);
        rc |= s->set_bpc(s, 0);      rc |= s->set_wpc(s, 0);   // v3: low gain => off (lab law agc<16)
        rc |= s->set_hmirror(s, 0);  rc |= s->set_vflip(s, 0);
        int init_aec = (s_track_mode == TRACK_MODE_IR_BLOBS) ? BOOT_AEC : 300;
        int init_agc = (s_track_mode == TRACK_MODE_IR_BLOBS) ? BOOT_AGC : 10;
        rc |= s->set_gain_ctrl(s, 0);     rc |= s->set_agc_gain(s, init_agc);  // v21
        rc |= s->set_exposure_ctrl(s, 0); rc |= s->set_aec2(s, 0);
        rc |= s->set_aec_value(s, init_aec);         // v21: see the BOOT RECIPE block
        // (period-doubling note: tracks scene darkness, not aec; the v9 frame
        // gate rejects those frames — 2 events in 12s seen on the LED bench,
        // both would be gated.)
        rc |= s->set_reg(s, 0x112, 0x02, 0x00);      // v15 (lab parity): COM7[1] test pattern OFF
        rc |= s->set_reg(s, 0x113, 0x20, 0x00);      // banding filter off
        rc |= s->set_reg(s, 0x113, 0x01, 0x00);      // v15 (lab parity): COM8[0] AEC enable bit clear
        rc |= s->set_reg(s, 0x103, 0xC0, 0x00);      // COM1: no dummy frames
        // v9 datasheet-verified frame-timing kill list (Table 13):
        rc |= s->set_reg(s, 0x12D, 0xFF, 0x00); rc |= s->set_reg(s, 0x12E, 0xFF, 0x00); // ADDVSL/H: VSYNC width +0 lines
        // NOTE 0x12A (REG2A[7:4]) is an OVERLAY-ONLY write — the lab never
        // touches it. Its reset value is 0 and we write 0, so it is a no-op in
        // practice; kept, but recorded here so the difference is not silent.
        rc |= s->set_reg(s, 0x12A, 0xF0, 0x00);      // REG2A[7:4]: line-interval adj MSBs = 0
        rc |= s->set_reg(s, 0x12B, 0xFF, 0x00);      // FRARL: line-interval adj LSBs = 0
        rc |= s->set_reg(s, 0x146, 0xFF, 0x00); rc |= s->set_reg(s, 0x147, 0xFF, 0x00); // FLL/FLH: frame length +0
        rc |= s->set_reg(s, 0x111, 0xFF, 0x80);      // CLKRC: 2x, div 1
        rc |= s->set_reg(s, 0x132, 0xFF, 0x89);      // REG32 CIF (r32=0 form)
        rc |= s->set_reg(s, 0x0D3, 0xFF, 0x03);      // v10: pdiv=3 — THE 136fps unlock
                                                     // (DVP bus slowed, sensor full rate)
        // MUST be last: REG45 low bits belong to AEC (set above); only [7:6]
        rc |= s->set_reg(s, 0x145, 0xC0, BOOT_BOOST ? 0xC0 : 0x00);   // v21: OFF
        if (rc == 0) break;
        printf("SCCB: recipe pass %d NACKed (rc=%d) - retrying\n", tries + 1, rc);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (s && tries >= 3)
        printf("!! SCCB: recipe NEVER fully ACKed - the sensor is mis-programmed "
               "and every measurement after this line is suspect\n");
    s_cfg_aec = (s_track_mode == TRACK_MODE_IR_BLOBS) ? BOOT_AEC : 300;
    s_cfg_agc = (s_track_mode == TRACK_MODE_IR_BLOBS) ? BOOT_AGC : 10;
    if (s_track_mode != TRACK_MODE_IR_BLOBS) {
        THR = 0;
        screen_detector_set_threshold(THR);
    }
#if OV_DASH_OWN_UART0
    // Take UART0's RX so the dashboard can send commands while OpenFIRE owns
    // the USB CDC. TX buffer 0 on purpose: we never write through the driver,
    // only via printf()'s polling path, so the two can never interleave a line.
    if (!uart_is_driver_installed(UART_NUM_0))
        uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
#endif
    xTaskCreatePinnedToCore(drain_task, "ov_drain", 3072, nullptr, 3, nullptr, 1);
    s_started = true;
    return 0;
}

// ---- v11.2 LIVE TUNING (bench tool) --------------------------------------
// The lab firmware has a serial console; the overlay stack had NO way to tune
// without a reflash, which made the "find the best blob size" experiment
// (jitter_sim: centroid noise dominates the cursor wobble) painfully slow.
// Accepts "k=v&k=v": thr, aec, agc, boost. Applied immediately.
// NOTE: bench use only — writes race the camera task by one frame.
#if LIGHTGUN_DIAG
extern "C" void ov2640_tune(const char* cmd)
{
    if (!cmd) return;
    const char* p = cmd;
    while (*p) {
        char key[8] = {0}; int ki = 0;
        while (*p && *p != '=' && *p != '&' && ki < 7) key[ki++] = *p++;
        if (*p != '=') { while (*p && *p != '&') ++p; if (*p) ++p; continue; }
        ++p;
        char val_str[16] = {0}; int vi = 0;
        const char* val_start = p;
        while (*p && *p != '&' && vi < 15) val_str[vi++] = *p++;
        while (*p && *p != '&') ++p;
        if (*p) ++p;

        int val = 0; bool any = false;
        const char* vp = val_start;
        while (*vp >= '0' && *vp <= '9') { val = val * 10 + (*vp++ - '0'); any = true; }

        if (!strcmp(key, "mode") || !strcmp(key, "track")) {
            if (!strcmp(val_str, "border") || (any && val == 1)) {
                ov2640_set_track_mode(TRACK_MODE_SCREEN_BORDER);
            } else if (!strcmp(val_str, "fiducial") || (any && val == 2)) {
                ov2640_set_track_mode(TRACK_MODE_SCREEN_FIDUCIAL);
            } else if (!strcmp(val_str, "ir") || (any && val == 0)) {
                ov2640_set_track_mode(TRACK_MODE_IR_BLOBS);
            }
        } else if (!strcmp(key, "auto")) {
            if (s_sensor) {
                s_sensor->set_exposure_ctrl(s_sensor, val ? 1 : 0);
                s_sensor->set_gain_ctrl(s_sensor, val ? 1 : 0);
                s_sensor->set_aec2(s_sensor, val ? 1 : 0);
            }
        } else if (!strcmp(key, "thr")) {
            if (val < 0)   val = 0;
            if (val > 250) val = 250;
            THR = (uint8_t)val;
            screen_detector_set_threshold(THR);
        } else if (!strcmp(key, "res")) {
            // v27: 0 = off, 1 = resolver on top-4-by-mass, 2 = resolver picks
            // four from up to 8 by geometry (default). A/B live.
            if (val < 0) val = 0;
            if (val > 2) val = 2;
            s_resolver = (uint8_t)val;
            quad_reset(nullptr);
        } else if (!strcmp(key, "coin")) {
            s_coin = (val != 0);       // v18 temporal coincidence gate, A/B live
        } else if (!strcmp(key, "dbg")) {
            s_dbg = (val != 0);
        } else if (!strcmp(key, "dash")) {              // v14 dashboard stream
            s_dash = (uint8_t)(val < 0 ? 0 : (val > 2 ? 2 : val));
            // Q is 2.4x the bytes of B. Starting mode 2 uncapped would put the
            // drain task at ~60% busy-wait; 60Hz is plenty to see a teleport.
            if (s_dash == 2 && s_dash_min_dt_us == 0) s_dash_min_dt_us = 1000000 / 60;
        } else if (!strcmp(key, "drvgate")) {
            // v15.1: default ON again -- see the comment at cam_patch_gate_en.
            cam_patch_gate_en = (val != 0) ? 1u : 0u;
        } else if (!strcmp(key, "dashb")) {
            s_dash_blob = (uint8_t)(val < 0 ? 0 : (val > 3 ? 3 : val));
        } else if (!strcmp(key, "dashhz")) {
            s_dash_min_dt_us = (val > 0) ? (uint32_t)(1000000 / val) : 0;
        } else if (!strcmp(key, "frame")) {
            // The dashboard's SNAP button. Answer it — a request that dies in
            // silence is the failure mode we already paid for once.
            printf("SNAPABORT: the overlay never keeps a framebuffer copy "
                   "(detection happens on the DMA chunks, the fb is returned "
                   "immediately). Pixel snapshots are a lab-firmware feature; "
                   "flash firmware/ for SNAP. B/STAT still work here.\n");
        } else if (s_sensor) {
            if      (!strcmp(key, "aec"))   { s_sensor->set_aec_value(s_sensor, val); s_cfg_aec = val; }
            else if (!strcmp(key, "agc"))   { s_sensor->set_agc_gain(s_sensor, val);  s_cfg_agc = val; }
            else if (!strcmp(key, "boost")) { s_sensor->set_reg(s_sensor, 0x145, 0xC0,
                                                                val ? 0xC0 : 0x00);
                                              s_cfg_boost = val ? 1 : 0; }
        }
    }
    // "CMD ok" prefix: dashboard.py runs parse_kv() on it, so its panel picks
    // the new values up immediately instead of waiting for the next STAT.
    printf("CMD ok (tune) | track=%s thr=%u aec=%d agc=%d boost=%d | mode=LAB dash=%u "
           "dashb=%u dashhz=%lu drvgate=%u res=%u coin=%u\n",
           ov2640_get_track_mode_name(),
           (unsigned)THR, s_cfg_aec, s_cfg_agc, s_cfg_boost,
           (unsigned)s_dash, (unsigned)s_dash_blob,
           (unsigned long)(s_dash_min_dt_us ? 1000000u / s_dash_min_dt_us : 0),
           (unsigned)cam_patch_gate_en,
           (unsigned)s_resolver, (unsigned)(s_coin ? 1 : 0));
}
#endif  // LIGHTGUN_DIAG (tune console)
