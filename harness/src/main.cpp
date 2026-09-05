// Harness — FULL-RATE anomaly recorder. A decimated point stream hides
// 1-frame artifacts, so this reads the bridge directly
// and inspects EVERY published camera frame (seq change = new frame):
//
//  J,<ms>,seq,<why>,dt=<us> | prev: x,y x,y ... | now: x,y x,y ...
//      printed for every ANOMALOUS frame transition —
//      count change, or a point moving >2.5 native px beyond the group's
//      common-mode motion (hand pans are common-mode; real artifacts are not).
//      Coordinates in NATIVE camera pixels (x16 units / 16).
//
//  G,<ms>,fps=..,polls=..,fresh=..,vsync=..us,J/s=..   (once per second)
//  R,<ms>,rejS=..,stitch=..,ovfE=..,ovfV=..
//      rejS = size-gate rejections, stitch = stitched frames the driver
//      killed, ovfE/ovfV = event-queue overflows.
//
//  P,<ms>,seen,x0,y0,...   (15Hz, shim/OpenFIRE units, as before)
//
// TUNE LIVE: type e.g. "thr=90" or "boost=0&aec=40" into the serial monitor
// (no reflash) to hunt the best blob size. "TUNE ok" echoes the new bars.
//
// Field protocol: hold the gun STILL 30s, then one slow sweep, then still
// again. Zero J-lines while still = frames are clean; J-lines tell exactly
// which point moved, how far, and whether the gates saw anything.
#include <Arduino.h>
#include "DFRobotIRPositionEx.h"
#include "ov2640_capture.h"
#include "screen_detector.h"

extern "C" {
    extern volatile uint32_t cam_patch_stitch_rej;
    extern volatile uint32_t cam_patch_ovf_eof;
    extern volatile uint32_t cam_patch_ovf_vsync;
    extern volatile uint32_t cam_patch_vsync_isr_period_us;
}

static DFRobotIRPositionEx cam;

static ov2640_bridge_frame_t prevf;
static uint32_t prev_seq = 0;
static bool have_prev = false;

// returns reason string if the transition prev->now is anomalous, else NULL
static const char* judge(const ov2640_bridge_frame_t& a, const ov2640_bridge_frame_t& b)
{
    if (a.count != b.count) return "count";
    if (b.count == 0) return NULL;
    // common-mode motion = median of per-point deltas (n<=4: use mean of
    // middle values; with tiny n a plain mean is fine for a threshold test)
    float mdx = 0, mdy = 0;
    for (int i = 0; i < b.count; ++i) {
        mdx += (float)b.x16[i] - (float)a.x16[i];
        mdy += (float)b.y16[i] - (float)a.y16[i];
    }
    mdx /= b.count; mdy /= b.count;
    for (int i = 0; i < b.count; ++i) {
        float rx = ((float)b.x16[i] - (float)a.x16[i]) - mdx;
        float ry = ((float)b.y16[i] - (float)a.y16[i]) - mdy;
        if (rx < 0) rx = -rx; if (ry < 0) ry = -ry;
        if (rx > 2.5f * 16.0f || ry > 2.5f * 16.0f) return "point";
    }
    return NULL;
}

static void print_pts(const ov2640_bridge_frame_t& f)
{
    for (int i = 0; i < f.count; ++i)
        Serial.printf(" %.1f,%.1f", f.x16[i] / 16.0f, f.y16[i] / 16.0f);
    if (!f.count) Serial.print(" -");
}

void setup() {
    Serial.begin(115200);
    delay(300);
    bool ok = cam.begin(400000, DFRobotIRPositionEx::DataFormat_Basic,
                        DFRobotIRPositionEx::Sensitivity_Default);
    Serial.printf("shim begin: %s\n", ok ? "OK (capture booted)" : "FAILED");
    Serial.printf("track mode: %s (type 'mode=border', 'mode=fiducial', or 'mode=ir' to switch)\n",
                  ov2640_get_track_mode_name());
    Serial.println("harness v3: J=anomaly (native px), G=gate counters (1Hz), P=stream (15Hz)");
}

void loop() {
    static uint32_t polls = 0, fresh = 0, frames = 0, jlines = 0;
    // v11.3 THE DECISIVE COUNTER: how many published frames carried 4 / 3 / 2 /
    // 1 / 0 points. ANY frame below 4 makes OpenFIRE (a) reconstruct the
    // missing corner by parallelogram and (b) fire its kinematic spring
    // (model_changed -> offsets loaded), which moves corners by CENTIMETRES.
    // blobtest never shows this because it has no solver. cnt3+ > 0 IS the bug.
    static uint32_t cnt[5] = {0, 0, 0, 0, 0};
    // v11.5 EDGE RISK: frames where a reported point sits within 12 NATIVE px
    // of a frame border. A point there has a TRUNCATED blob (biased, noisy
    // centroid) and is one small movement away from leaving the frame
    // entirely — which drops the count and triggers OpenFIRE's corner
    // reconstruction + kinematic spring = cm-scale jumps. Also tracks how far
    // the quad's centre sits from the frame centre (camera aim vs LED rig).
    static uint32_t edge_frames = 0;
    static int32_t  cx_sum = 0, cy_sum = 0, cx_n = 0;
    static uint32_t t0 = millis(), tp = 0;
    // v11: Error_Success now means "a NEW camera frame was consumed"; every
    // other poll reports DataMismatch (OpenFIRE skips it silently). The
    // polls/fresh ratio in the G-line IS the duplicate factor that used to
    // defeat OpenFIRE's One Euro filter — fresh should equal fps.
    if (cam.basicAtomic(DFRobotIRPositionEx::Retry_2)
            == DFRobotIRPositionEx::Error_Success) fresh++;
    polls++;

    // ---- full-rate frame inspection straight from the bridge ----
    ov2640_bridge_frame_t f;
    uint32_t seq = ov2640_bridge_read(&f);
    if (seq && seq != prev_seq) {
        if (have_prev && seq == prev_seq + 1) {      // consecutive frames only
            const char* why = judge(prevf, f);
            if (why) {
                jlines++;
                // v11.4: printed UNCONDITIONALLY (was gated on
                // availableForWrite() >= 200, which the S3's CDC never
                // reports — so J-lines were silently dropped and an EMPTY
                // J log could NOT be read as "no anomalies". Cap at 20/s
                // instead; jlines in the G line still counts every one.
                if (jlines <= 20) {
                    Serial.printf("J,%lu,%lu,%s | prev:", (unsigned long)millis(),
                                  (unsigned long)seq, why);
                    print_pts(prevf);
                    Serial.print(" | now:");
                    print_pts(f);
                    Serial.println();
                } else if (jlines == 21) {
                    Serial.println("J,... (>20 this second, see J/s in the G line)");
                }
            }
        }
        if (f.count <= 4) cnt[f.count]++;
        if (f.count) {
            bool edge = false; int32_t sx = 0, sy = 0;
            for (int i = 0; i < f.count; ++i) {
                const int nx = f.x16[i] / 16, ny = f.y16[i] / 16;
                if (nx < 12 || nx > 240 - 12 || ny < 12 || ny > 176 - 12) edge = true;
                sx += nx; sy += ny;
            }
            if (edge) edge_frames++;
            cx_sum += sx / (int)f.count; cy_sum += sy / (int)f.count; cx_n++;
        }
        prevf = f; prev_seq = seq; have_prev = true; frames++;
    }

    uint32_t now = millis();
    if (now - tp >= 66) {                            // 15Hz P-stream
        tp = now;
        {
            Serial.printf("P,%lu,0x%X,%d,%d,%d,%d,%d,%d,%d,%d\n",
                (unsigned long)now, cam.seen(),
                cam.x(0), cam.y(0), cam.x(1), cam.y(1),
                cam.x(2), cam.y(2), cam.x(3), cam.y(3));
        }
    }
    // v11.2: forward serial lines to the live tuner ("thr=90&boost=0")
    {
        static char line[96]; static size_t ln = 0;
        while (Serial.available() > 0) {
            int c = Serial.read();
            if (c < 0) break;
            if (c == '\n' || c == '\r') { if (ln) { line[ln] = 0; ov2640_tune(line); ln = 0; } }
            else if (ln < sizeof(line) - 1) line[ln++] = (char)c;
        }
    }
    if (now - t0 >= 1000) {
        // v11.4: NO availableForWrite() GATE on these once-per-second lines.
        // The previous build required >=160 bytes free before printing, but
        // the S3's CDC never reports that much and the long C-line format made
        // it worse: C/S/G were silently skipped EVERY second, forever. Only
        // the 15Hz P-lines (which need >=64) ever appeared. Diagnostics that
        // can vanish without a trace are worse than none. Lines are short now
        // and print unconditionally, once per second.
        Serial.printf("C,%lu,pts4=%lu,pts3=%lu,pts2=%lu,pts1=%lu,pts0=%lu\n",
            (unsigned long)now, (unsigned long)cnt[4], (unsigned long)cnt[3],
            (unsigned long)cnt[2], (unsigned long)cnt[1], (unsigned long)cnt[0]);
        Serial.printf("E,%lu,edge=%lu/%lu,quad_ctr=%ld,%ld (frame ctr 120,88)\n",
            (unsigned long)now, (unsigned long)edge_frames, (unsigned long)frames,
            (long)(cx_n ? cx_sum / cx_n : -1), (long)(cx_n ? cy_sum / cx_n : -1));
        Serial.printf("S,%lu,px~=%u/%u/%u/%u\n", (unsigned long)now,
            (unsigned)prevf.area4[0] * 16u, (unsigned)prevf.area4[1] * 16u,
            (unsigned)prevf.area4[2] * 16u, (unsigned)prevf.area4[3] * 16u);
        Serial.printf("G,%lu,fps=%lu,mode=%s,t_det=%luus,polls=%lu,fresh=%lu,vsync=%luus,J/s=%lu\n",
            (unsigned long)now, (unsigned long)frames,
            ov2640_get_track_mode_name(),
            (unsigned long)ov2640_get_detect_us(),
            (unsigned long)polls,
            (unsigned long)fresh, (unsigned long)cam_patch_vsync_isr_period_us,
            (unsigned long)jlines);
        screen_stats_t sstats;
        screen_detector_get_stats(&sstats);
        Serial.printf("M,%lu,min=%u,max=%u,avg=%u,thr=%u,rays=%u/%u/%u/%u,peaks=%u/%u/%u/%u\n",
            (unsigned long)now,
            sstats.min_px, sstats.max_px, sstats.avg_px, sstats.active_thr,
            sstats.n_top, sstats.n_bot, sstats.n_left, sstats.n_right,
            sstats.peak[0], sstats.peak[1], sstats.peak[2], sstats.peak[3]);
        // v28: rejP/rejM removed with the VSYNC-period and start-marker gates.
        Serial.printf("R,%lu,rejS=%lu,stitch=%lu,ovfE=%lu,ovfV=%lu\n",
            (unsigned long)now,
            (unsigned long)ov2640_stat_rej_size,
            (unsigned long)cam_patch_stitch_rej, (unsigned long)cam_patch_ovf_eof,
            (unsigned long)cam_patch_ovf_vsync);
        if (frames == 0)
            Serial.println("!! NO FRAMES PUBLISHED this second (see R line above)");
        polls = fresh = frames = jlines = 0;
        cnt[0] = cnt[1] = cnt[2] = cnt[3] = cnt[4] = 0;
        edge_frames = 0; cx_sum = cy_sum = cx_n = 0; t0 = now;
    }
    delay(2);
}
