# Changelog

All notable changes, architectural decisions, and hardware integrations for **OpenFire OVCam** are documented in this file.

---

## [Unreleased] - 2026-09-05

### Added
- **Multi-Hardware Support for Seeed Studio XIAO ESP32S3 Sense**:
  - Added PlatformIO environments `combined_xiao_s3_sense` (shipping) and `diag_xiao_s3_sense` (diagnostic) in `platformio.ini`.
  - Added standalone test harness environment `[env:xiao_s3_sense]` in `harness/platformio.ini`.
  - Created modular board pinout definitions in `lib/OV2640Capture/camera_pins.h` (`P_D0`–`P_D7`, `P_XCLK`, `P_PCLK`, `P_VSYNC`, `P_HREF`, `P_SIOD`, `P_SIOC`).
  - Added board definition preset `"seeed-xiao-esp32s3"` in `OpenFIRE-Firmware-ESP32/lightgun/src/boards/OpenFIREshared.h`.
- **Multi-Camera Driver Support (OV3660 & OV5640)**:
  - Enabled OV3660 and OV5640 driver sources in `lib/esp32-camera-ov2640/` alongside legacy OV2640.
  - Implemented sensor runtime PID detection (`OV2640_PID`, `OV3660_PID`, `OV5640_PID`) in `ov2640_capture.cpp` to guard sensor-specific registers.
  - Added SCCB 16-bit register write support in `lib/esp32-camera-ov2640/driver/sccb-ng.c` for OV3660/OV5640 configuration.
- **OV3660 High-Speed PLL Tuning**:
  - Boosted OV3660 PLL multiplier in `lib/esp32-camera-ov2640/sensors/ov3660.c` from 8 to 16 (`SYSCLK = 80 MHz`, `PCLK = 20 MHz`).
  - Verified on hardware: sensor frame rate doubled from **22.2 FPS (45 ms)** to a rock-solid **45.0 FPS (22.5 ms)** with 0 dropped frames, 0 chunk errors, and 0 holes.
- **Hardware Pinout Allocation for XIAO ESP32S3**:
  - **I2C Bus (OLED & IMU)**: GPIO 5 (D4 / SDA) and GPIO 6 (D5 / SCL) reserved for display and 6-axis motion tracking.
  - **Status LED**: GPIO 43 (D6) allocated for external addressable NeoPixel.
  - **Inputs**: Trigger (GPIO 1 / D0), Button A (GPIO 2 / D1), Button B (GPIO 3 / D2), Button C (GPIO 4 / D3), Start (GPIO 44 / D7), Select (GPIO 9 / D10).
  - **Outputs (Haptics/Feedback)**: Solenoid (GPIO 7 / D8) and Rumble motor (GPIO 8 / D9) allocated to expansion pins.
  - **Pedal**: Unmapped (`-1`) to conserve GPIOs.
- **Product-validation roadmap refresh**:
  - Promoted real-game visible-screen testing to the immediate next milestone.
  - Added calibration/tuning and gameplay robustness benchmarking ahead of further camera FPS optimization.
  - Kept IMU, 60 FPS camera work, OLED, physical integration, and manufacturing preparation on the roadmap but moved them after gameplay validation.
  - Added future CI and reproducible-developer-setup milestones.
- **Documentation cleanup**:
  - Reworked the README around the current IR-free visible-screen direction while keeping conventional IR tracking documented as a fallback.
  - Clarified that `OpenFIRE-Firmware-ESP32/` is an intentional local git-ignored checkout rather than vendored code or a tracked submodule.
  - Corrected the licensing description: this repository remains GPL-3.0, OpenFIRE upstream is LGPL-2.1, and the Espressif camera-driver code retains Apache-2.0 notices.

### Verified & Tested
- **Hardware Flashing & Enumeration**:
  - Flashed XIAO ESP32S3 over USB Serial/JTAG (`/dev/cu.usbmodem1101`), verified TinyUSB CDC/HID re-enumeration as `FIRECon P1` (`/dev/cu.usbmodem1020BA0394B81`).
- **OpenFIRE Batch Serial Protocol**:
  - Implemented bidirectional packet echo acknowledgment for `sGetPins` (command 200), successfully dumping and verifying all 35 GPIO mappings.
- **Timing & Frame Rate Verification**:
  - Stock OV3660 theoretical calculation: $\frac{40\text{ MHz}}{2300 \times 783} = 22.211\text{ FPS}$ ($45,022.5\ \mu\text{s}$). Measured on hardware: `VSI avg=45022us` (exact match).
  - Boosted OV3660 theoretical calculation: $\frac{80\text{ MHz}}{2300 \times 783} = 44.42\text{ FPS}$ ($22,511\ \mu\text{s}$). Measured on hardware: `VSI avg=22511us, spread=6us, ACCT/s=45/45 delivered`.
  - Centroid detection runtime: `t_det = 1050 us` (~1.05 ms per frame, <5% CPU utilization).

---

## [0.2.0] - 2026-09-05 (Commit `8745980`)

### Added
- **Visible Light Screen Detector Module**:
  - Implemented `lib/OV2640Capture/screen_detector.cpp` and `screen_detector.h`.
  - Added support for screen corner fiducials (`TRACK_MODE_SCREEN_FIDUCIAL`) and screen border tracking (`TRACK_MODE_SCREEN_BORDER`).
  - Added dynamic brightness thresholding and quad edge gradient extraction.
- **Host Testing & Calibration Tools**:
  - Added `tools/screen_pattern.html` for displaying calibration fiducials on standard monitors and TVs.
  - Added unit test suite `test/test_screen_detector.cpp` validating corner extraction logic.
  - Added mode switching runtime commands to tuning console (`mode=ir`, `mode=fiducial`, `mode=border`).

---

## [0.1.0] - 2026-09-02 (Commit `194f35b`)

### Added
- **Initial OpenFIRE Camera Overlay**:
  - Integrated `OpenFIRE-Firmware-ESP32` as a local build dependency.
  - Implemented DMA ring buffer capture shim for OV2640 (`lib/OV2640Capture/`).
  - Implemented DFRobot IR positioning shim (`DFRobotIRPositionEx_OV2640`) translating camera coordinates into OpenFIRE bridge frames.
  - Added `tools/dashboard.py` real-time visualization and tuning utility over serial.
  - Added `docs/CALIBRATION.md` detailing room tuning, threshold adjustments, and diagnostic commands.
