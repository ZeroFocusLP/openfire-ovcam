# Project Goal & Architectural Vision: OpenFire OVCam

> **OpenFire OVCam** is an advanced optical lightgun firmware project that brings IR-free visible screen tracking and multi-camera support to the OpenFIRE ESP32 ecosystem.

---

## 1. Core Mission & Vision

1. **IR-Free Visible Light Tracking:**
   Eliminate the need for external infrared LED bars (DolphinBar, 4-corner IR LED arrays, etc.). Enable lightguns to track directly against modern flat-panel displays (LCD, OLED, QD-OLED) and projection screens by detecting screen borders or corner fiducials.
2. **Universal OmniVision Sensor Support:**
   The legacy OV2640 sensor (released in 2005) is discontinued and obsolete. The project aims to make OpenFIRE completely sensor-agnostic, supporting modern, actively manufactured OmniVision sensors including **OV3660** (3MP) and **OV5640** (5MP) alongside OV2640.
3. **Multi-Hardware Portability:**
   Support a wide variety of ESP32-S3 boards through a modular hardware configuration layer:
   - **Freenove ESP32-S3 WROOM CAM:** Ideal for development and full-sized test rigs.
   - **Seeed Studio XIAO ESP32S3 Sense:** Ultra-compact thumb-sized board designed for clean internal integration inside commercial arcade and console lightgun shells (e.g., Namco Guncon 1, Guncon 2, Sega Stunner).
   - **Future Custom PCBs / ESP32-S3 Modules:** Easily added via declarative pin mapping tables.
4. **Preserve OpenFIRE Ecosystem Compatibility:**
   Maintain full compatibility with OpenFIRE companion tools, calibration apps, host HID protocols (DirectInput, Mouse, Guncon), and core gun physics.

---

## 2. Sensor Strategy: Modern BSI vs Legacy OV2640

| Metric | Legacy OV2640 | Modern OV3660 / OV5640 |
| :--- | :--- | :--- |
| **Availability** | Discontinued / NOS | Actively Manufactured |
| **Silicon Technology** | 2.2µm Frontside Illumination (FSI) | 1.4µm / 1.75µm Backside Illumination (BSI) |
| **Optical Readout** | Line skipping / sub-sampling | Hardware 2×2 ISP averaging / binning |
| **Image Quality** | High shot noise, pixel aliasing | Clean edges, high SNR (+6 dB), anti-aliased |
| **Target Frame Rate** | 136 FPS (via legacy `pdiv=3` hack) | **45–60 FPS** (via tuned PLL & timing) |
| **Screen Tracking Quality** | Jittery sub-pixel edges | **Superior sub-pixel corner precision** |
| **PWM / Refresh Handling** | Susceptible to rolling shutter bands | Smooth integration over 60Hz display cycles |

### Performance Philosophy:
While OV2640 achieved 136 FPS through aggressive line skipping, it suffered from high image noise and jitter. The OV3660 and OV5640 use hardware binning over their entire multi-megapixel surface. Running at **45–60 FPS** provides fluid arcade responsiveness while delivering vastly superior sub-pixel edge detection and noise immunity.

---

## 3. Hardware Architecture & Peripheral Roadmap

The reference design for the **Seeed Studio XIAO ESP32S3 Sense** is prioritized for full lightgun features:

```
                          ┌───────────────────────────┐
                          │   XIAO ESP32-S3 Sense     │
                          │        (OpenFIRE)         │
                          └─────────────┬─────────────┘
                                        │
        ┌───────────────────────────────┼──────────────────────────────┐
        │ I2C Bus (D4/D5)               │ High-Speed DVP               │ GPIO Control
        ▼                               ▼                              ▼
 ┌───────────────┐              ┌───────────────┐              ┌────────────────┐
 │ 0.96" / 0.42" │              │   OmniVision  │              │ Inputs:        │
 │  OLED Screen  │              │ Camera Sensor │              │ • Trigger (D0) │
 └──────┬────────┘              │ (OV3660/5640) │              │ • Button A(D1) │
        │ (Shared I2C)          └───────────────┘              │ • Button B(D2) │
        ▼                                                      │ • Button C(D3) │
 ┌───────────────┐                                             │ • Start   (D7) │
 │ 6-Axis Motion │                                             │ • Select (D10) │
 │  IMU Sensor   │                                             ├────────────────┤
 └───────────────┘                                             │ Outputs:       │
                                                               │ • NeoPixel(D6) │
                                                               │ • Solenoid(D8) │
                                                               │ • Rumble  (D9) │
                                                               └────────────────┘
```

### Pin Allocation Map (XIAO ESP32-S3):
* **I2C Bus (OLED & IMU):** GPIO 5 (D4 / SDA) & GPIO 6 (D5 / SCL) — *Kept strictly reserved for peripherals.*
* **Status / Muzzle LED:** GPIO 43 (D6) for WS2812B Addressable NeoPixel.
* **Player Controls:**
  - `Trigger`: GPIO 1 (D0)
  - `Button A`: GPIO 2 (D1)
  - `Button B`: GPIO 3 (D2)
  - `Button C`: GPIO 4 (D3)
  - `Start`: GPIO 44 (D7)
  - `Select`: GPIO 9 (D10)
* **Haptics & Force Feedback:**
  - `Solenoid Kick`: GPIO 7 (D8) via external MOSFET.
  - `Rumble Motor`: GPIO 8 (D9) via vibration driver.
* **Pedal:** Unmapped (`-1`) to avoid pin conflicts.

---

## 4. Software Architecture & Clean Modularity

1. **Overlay Architecture:**
   Core OpenFIRE firmware lives in `OpenFIRE-Firmware-ESP32/` as a submodule. Hardware shims and camera drivers live in top-level `lib/`:
   - `lib/OV2640Capture/`: Real-time DMA ring-buffer capture engine, centroid extraction, and screen corner solver.
   - `lib/esp32-camera-ov2640/`: Patched low-level ESP32-S3 camera HAL supporting OV2640, OV3660, and OV5640 with custom high-speed PLL configs.
   - `lib/DFRobotIRPositionEx_OV2640/`: Adapter shim presenting camera coordinates directly to OpenFIRE's coordinate engine.
2. **Build Target Hierarchy:**
   - `combined_*` targets: Production release firmware with native USB HID/CDC enabled and silent logging.
   - `diag_*` targets: Diagnostic firmware with live telemetry over UART0 and interactive tuning console.
   - `harness/`: Standalone camera verification and bench testing suite.

---

## 5. Milestone Tracker & Next Steps

- [x] **Milestone 1:** Initial OV2640 DMA capture and OpenFIRE shim.
- [x] **Milestone 2:** Visible screen corner & border detection modules with calibration tools.
- [x] **Milestone 3:** Multi-hardware & multi-sensor support (Seeed XIAO ESP32S3 + OV3660/OV5640).
- [x] **Milestone 4:** Boost OV3660 frame rate to rock-solid 45 FPS on hardware.
- [ ] **Milestone 5:** Push OV3660/OV5640 to locked 60 FPS via horizontal blanking / windowing.
- [ ] **Milestone 6:** Integrate I2C OLED display for live aiming reticle and ammo counter.
- [ ] **Milestone 7:** Integrate I2C 6-axis IMU for off-screen recovery and whip-shot motion fusion.
- [ ] **Milestone 8:** Custom injection-mold or 3D-printable shell integration guides.
