/*!
 * DFRobotIRPositionEx.h — OV2640 backend for the OpenFIRE camera interface.
 *
 * THIRD camera backend for OpenFIRE, following the codebase's own pattern:
 * the PAJ7025 wrapper (lib/DFRobotIRPositionEx_Wrapper) already proves that
 * this class is a swappable boundary — same public interface, different sensor
 * behind it. This one is backed by an OV2640 + on-chip blob detection running
 * in the esp32-camera driver's copy-task, published through ov2640_bridge.h.
 *
 * DROP-IN CONTRACT (verified against OpenFIREcommon.cpp call sites):
 *     ctor                          — trivial (no bus: data comes from RAM)
 *     begin(clock, format, sens)    — clock/format accepted for compatibility
 *     basicAtomic(retry) / extendedAtomic(retry)
 *     xPositions() / yPositions() / seen() / x(i) / y(i) / size(i)
 *     dataFormat() / sensitivityLevel()
 * Everything OpenFIRE reads is served from the latest published camera frame;
 * "atomic" is genuinely atomic here (seqlock copy), and there is no I2C error
 * class at all — Error_Success unless no frame has ever been published.
 *
 * COORDINATE SPACE: the bridge carries native x16 subpixel units; this shim
 * scales them into the interface's output space (see the OUT_MAX defines
 * below for the space chosen and why).
 *
 * Sensitivity_e maps to nothing yet: our AGC/threshold live in the capture
 * stack (with the auto-unclip servo). Accepted and stored so profile switches
 * don't error; a future version may map it to threshold presets.
 */
#ifndef DFRobotIRPositionEx_h
#define DFRobotIRPositionEx_h

#include <stdint.h>
#include "ov2640_bridge.h"
#if defined(ESP_PLATFORM)
#include "ov2640_capture.h"   // firmware build: begin() boots the camera
#include "esp_timer.h"

// v26 latency probe. ALL THREE are DEFINED in ov2640_capture.cpp and only
// DECLARED here — a definition in a header is a duplicate symbol in every
// translation unit past the first, and C++ linkage here must match the C++
// linkage there (an `extern "C"` block around these would be an undefined
// reference, not a warning).
//   ov2640_pub_t_us       — esp_timer stamp taken at publish (capture side)
//   ov2640_shim_lat_last_us — age of the frame OpenFIRE just consumed, us
//   ov2640_shim_lat_us      — worst age since the last LAT/s report, us
extern volatile uint32_t ov2640_pub_t_us;
extern volatile uint32_t ov2640_shim_lat_us;
extern volatile uint32_t ov2640_shim_lat_last_us;
#endif

// Output coordinate space. OpenFIREConst.h hardcodes
// CamResX=1024/CamResY=768 (One-Euro beta and mouse mapping derive from it),
// even though the PAJ7025 wrapper ships feeding 4095x4095 raw. That shipping
// inconsistency apparently works, but we do not build on it: default to the
// classic 1023x767 space (quantization ~0.29 units RMS = 0.07 native px --
// negligible). If a larger space is ever verified against CamResX, raise
// these two numbers; nothing else changes.
#define OV2640_IRPOS_OUT_MAX_X 1023
#define OV2640_IRPOS_OUT_MAX_Y 767

// X MIRROR (field-proven: first live aim had left/right inverted).
// The DFRobot/PAJ sensors this class models face the PLAYER, so their X axis
// runs opposite to screen X — and OpenFIRE un-mirrors internally
// (`CamMaxX - px[i]` in OpenFIRE_Square/Diamond). Our OV2640 also faces the
// player, but delivers X already in screen orientation for this module's
// mounting, so OpenFIRE's un-mirror INVERTED it. Pre-mirroring here restores
// the DFRobot convention the solver expects. Set to 0 if a different camera
// module/mounting lands mirrored the other way (symptom: left/right swapped).
#define OV2640_IRPOS_MIRROR_X 1

class TwoWire;   // fwd-decl so the I2C-style ctor signature stays available
class SPIClass;  // (and the wrapper-style one) — both ignored internally.

class DFRobotIRPositionEx {
public:
    enum DataFormat_e {
        DataFormat_Basic    = 0,
        DataFormat_Extended = 1
    };
    enum Sensitivity_e {
        Sensitivity_Min     = 0,
        Sensitivity_Default = 0,
        Sensitivity_High    = 1,
        Sensitivity_Max     = 2
    };
    enum Errors_e {
        Error_SuccessMismatch = 1,
        Error_Success         = 0,
        Error_IICerror        = -1,
        Error_DataMismatch    = -2
    };
    enum Retry_e {
        Retry_0  = 0, Retry_0s = 1,
        Retry_1  = 2, Retry_1s = 3,
        Retry_2  = 4, Retry_2s = 5
    };

    // All ctor shapes accepted so any OpenFIRE instantiation compiles unchanged.
    DFRobotIRPositionEx() { init(); }
    explicit DFRobotIRPositionEx(TwoWire&) { init(); }
    DFRobotIRPositionEx(SPIClass*, int8_t = -1) { init(); }
    ~DFRobotIRPositionEx() {}

    bool begin(uint32_t clock = 400000,
               DataFormat_e format = DataFormat_Basic,
               Sensitivity_e sensitivity = Sensitivity_Default) {
        (void)clock;
        current_format = format;
        current_sens   = sensitivity;
#if defined(ESP_PLATFORM)
        // OpenFIRE thinks it is opening an I2C camera; actually boots our
        // whole capture pipeline (idempotent across re-begins).
        if (ov2640_capture_start() != 0) return false;
#endif
        // Success iff the capture stack is alive (it publishes from boot).
        ov2640_bridge_frame_t f;
        last_seq = ov2640_bridge_read(&f);
        return true;   // capture may still be warming up; first frames follow
    }

    void dataFormat(DataFormat_e format)          { current_format = format; }
    void sensitivityLevel(Sensitivity_e s)        { current_sens = s; }

    // ---- the per-frame calls OpenFIRE actually makes ----
    // NEW-FRAME GATING — the smoothness fix. See fetch().
    int basicAtomic(Retry_e retry = Retry_1s)     { (void)retry; return fetch(); }
    int extendedAtomic(Retry_e retry = Retry_1s)  { (void)retry; return fetch(); }

    // request/available pairs kept for interface completeness
    void requestPositionBasic()                   { pending = true; }
    void requestPositionExtended()                { pending = true; }
    bool availableBasic()          { fetch(); return true; }
    bool availableBasicNoSeen()    { fetchNoSeen(); return true; }
    bool availableExtended()       { fetch(); return true; }
    bool availableExtendedNoSeen() { fetchNoSeen(); return true; }

    // ---- accessors (exact signatures of the original) ----
    int x(int index) const                { return positionX[index]; }
    int y(int index) const                { return positionY[index]; }
    int size(int index) const             { return unpackedSizes[index]; }
    const int* xPositions() const         { return positionX; }
    const int* yPositions() const         { return positionY; }
    unsigned int seen() const             { return seenFlags; }

private:
    int positionX[4];
    int positionY[4];
    int unpackedSizes[4];
    unsigned int seenFlags;
    DataFormat_e  current_format;
    Sensitivity_e current_sens;
    uint32_t last_seq;
    bool pending;

    void init() {
        for (int i = 0; i < 4; ++i) {
            positionX[i] = 1023; positionY[i] = 1023;  // DFRobot "not seen" idiom
            unpackedSizes[i] = 0;
        }
        seenFlags = 0;
        current_format = DataFormat_Basic;
        current_sens   = Sensitivity_Default;
        last_seq = 0;
        pending  = false;
    }

    // ---- fetch: returns Error_Success ONLY when a NEW camera frame exists ----
    //
    // THE SMOOTHNESS FIX. Field report: "I expected way smoother
    // tracking; the SEN0158 was way smoother" — and he was right.
    //
    // OpenFIRE's run loop calls GetPosition() with NO pacing. With a real
    // DFRobot camera each basicAtomic() is an I2C transaction (~0.5-1ms at
    // 400kHz), which throttles the loop to roughly the sensor's own rate — so
    // their Multi One Euro filter is called about once per new reading. Its
    // tuning says so explicitly: "at 200Hz, a jump of 1 raw camera pixel =
    // 800 px/sec", with snap_base = 1000 chosen to sit just above that.
    //
    // Our shim answers from RAM in ~1us, so OpenFIRE polled at several kHz and
    // re-read the SAME frame ~40 times. The filter computes velocity as
    // (x - x_prev)/dt with dt measured BETWEEN CALLS: ~40 calls of dx=0, then
    // ONE call carrying the whole inter-frame delta over a ~200us dt. That
    // reads as tens of thousands of px/sec — 25x over the snap threshold — so
    // the adaptive cutoff pinned itself at max_cutoff (30Hz) permanently and
    // the filter stopped smoothing at all. Raw jitter went straight to the
    // cursor: exactly the "not smooth" feel, and invisible in the blob test
    // (which never runs their filter).
    //
    // Fix: report a frame ONCE. When the bridge has not advanced we return
    // Error_DataMismatch — the value OpenFIREcommon.cpp already treats as a
    // silent "nothing to do" (`else if(error != Error_DataMismatch)
    // PrintIrError();`), so it skips solver + filter for that call and no
    // error is printed. The filter then sees one sample per camera frame at
    // 136Hz with a real dt — the regime it was tuned for.
    //
    // Accessors keep the last good values, so nothing else changes.
    int fetch() {
        ov2640_bridge_frame_t f;
        uint32_t seq = ov2640_bridge_read(&f);
        if (seq == 0) { seenFlags = 0; return Error_DataMismatch; }  // not warm yet
        if (seq == last_seq) return Error_DataMismatch;              // no NEW frame
        // v26: TRUE capture-to-consumption latency, measured at the exact point
        // OpenFIRE takes the data rather than inferred from a budget. This is
        // the number that says whether perceived lag is ours or theirs.
#if defined(ESP_PLATFORM)
        {
            const uint32_t age = (uint32_t)esp_timer_get_time() - ov2640_pub_t_us;
            if (age < 1000000u) {                       // ignore boot/wrap
                ov2640_shim_lat_last_us = age;
                if (age > ov2640_shim_lat_us) ov2640_shim_lat_us = age;
            }
        }
#endif
        unsigned int flags = 0;
        for (int i = 0; i < 4; ++i) {
            if (i < f.count) {
                // native x16 -> output space (see header). frame_w/h are the
                // native pixel dims; x16 tops out at dim*16-1.
                int sx = (int)((uint32_t)f.x16[i] * OV2640_IRPOS_OUT_MAX_X
                               / (uint32_t)(f.frame_w * 16u - 1u));
#if OV2640_IRPOS_MIRROR_X
                positionX[i] = OV2640_IRPOS_OUT_MAX_X - sx;
#else
                positionX[i] = sx;
#endif
                positionY[i] = (int)((uint32_t)f.y16[i] * OV2640_IRPOS_OUT_MAX_Y
                                     / (uint32_t)(f.frame_h * 16u - 1u));
                int a = f.area4[i];
                unpackedSizes[i] = a > 15 ? 15 : a;   // DFRobot size range 0..15
                flags |= (1u << i);
            } else {
                positionX[i] = 1023; positionY[i] = 1023;
                unpackedSizes[i] = 0;
            }
        }
        seenFlags = flags;
        last_seq  = seq;
        pending   = false;
        return Error_Success;
    }

    void fetchNoSeen() {   // variant that leaves seenFlags untouched (original
        unsigned int keep = seenFlags;                 // semantics of *NoSeen)
        fetch();
        seenFlags = keep;
    }
};

#endif
