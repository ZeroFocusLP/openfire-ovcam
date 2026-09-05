# OpenFire OVCam

OpenFire OVCam is an ESP32-S3 camera backend and vision layer for the OpenFIRE lightgun ecosystem.

The project started as a low-cost replacement for OpenFIRE's usual DFRobot/PAJ7025 IR positioning camera, using commodity OmniVision sensors. It has since expanded to support **IR-free visible-screen tracking** on modern displays while preserving normal OpenFIRE compatibility.

The current product direction is simple:

> **Make a low-cost open lightgun that can track modern TVs without external IR emitters, while retaining conventional 4-point IR tracking as a fallback.**

## Current status

- **OpenFIRE integration:** working through a drop-in `DFRobotIRPositionEx` shim.
- **Traditional 4-point IR mode:** working and hardware-tested with the OV2640 path.
- **Visible-screen modes:** implemented for screen-border tracking and corner/fiducial tracking.
- **Modern cameras:** OV2640, OV3660, and OV5640 driver support.
- **Seeed Studio XIAO ESP32S3 Sense:** supported as the compact reference platform.
- **OV3660:** verified at approximately **45 FPS** on hardware with stable frame delivery.
- **Diagnostics:** standalone harness, live dashboard, health scoring, runtime tuning, and host-side detector tests.

The immediate milestone is **real gameplay validation**: run an actual lightgun game using visible-screen tracking, then calibrate and tune the system based on real aiming behavior before doing further camera or product optimization.

See [`docs/GOAL.md`](docs/GOAL.md) for the roadmap and [`docs/CHANGELOG.md`](docs/CHANGELOG.md) for measured progress.

---

## Architecture

OpenFire OVCam deliberately keeps OpenFIRE separate from this repository.

```text
openfire-ovcam/
├── OpenFIRE-Firmware-ESP32/   # local checkout, intentionally git-ignored
├── lib/
│   ├── OV2640Capture/         # capture + IR blobs + visible-screen detector
│   ├── DFRobotIRPositionEx_OV2640/
│   └── esp32-camera-ov2640/   # patched camera driver
├── harness/                   # standalone camera bring-up and telemetry
├── tools/                     # dashboard, health checks, screen patterns
├── test/                      # host-side geometry / detector tests
└── platformio.ini
```

OpenFIRE itself is **not vendored and not tracked as a submodule**. The build expects a local checkout at:

```text
OpenFIRE-Firmware-ESP32/
```

That directory is intentionally listed in `.gitignore` so this repository only carries the OVCam-specific camera, vision, integration, and tooling changes.

The tested upstream revision should be checked out locally before building. At the time of writing, the known-good baseline is:

```text
f8f9bf265c4813f659bd49768378be6bf3cfee74
```

The build then compiles OpenFIRE's sources from that local directory while the project-local camera shim shadows OpenFIRE's normal positioning-camera implementation.

---

## Tracking modes

### 1. Visible screen border

A high-contrast border is displayed around the game image. The camera detects the four screen edges and reconstructs the screen quadrilateral.

This is currently the most promising IR-free path for early gameplay testing because it provides strong geometric structure and does not rely on external hardware around the television.

### 2. Visible corner / fiducial tracking

The camera detects bright corner markers or brackets shown on the display. This mode is intended to reduce the amount of visible tracking graphics once robustness against real game content is proven.

### 3. Conventional 4-point IR

The camera tracks four IR emitters arranged around the display, preserving the standard OpenFIRE-style workflow as a compatibility and fallback mode.

---

## Supported hardware

### Development platform

- **Freenove ESP32-S3 WROOM CAM**
- Useful for camera development, diagnostics, and bench testing.

### Compact reference platform

- **Seeed Studio XIAO ESP32S3 Sense**
- Intended for compact integration inside practical lightgun shells.
- Supports the project camera path plus reserved GPIO for:
  - trigger
  - A/B/C buttons
  - Start / Select
  - solenoid
  - rumble
  - NeoPixel
  - shared I2C for IMU and optional OLED

### Camera sensors

- **OV2640** — legacy high-FPS path, useful for IR tracking and historical compatibility.
- **OV3660** — current preferred modern sensor for product validation; hardware-tested at ~45 FPS.
- **OV5640** — supported in the camera driver for further experimentation.

---

## Build setup

### 1. Clone this repository

```bash
git clone https://github.com/ZeroFocusLP/openfire-ovcam.git
cd openfire-ovcam
```

### 2. Clone OpenFIRE locally into the expected ignored directory

```bash
git clone https://github.com/alessandro-satanassi/OpenFIRE-Firmware-ESP32.git
```

This must produce:

```text
openfire-ovcam/OpenFIRE-Firmware-ESP32/
```

Then check out the tested revision:

```bash
git -C OpenFIRE-Firmware-ESP32 checkout f8f9bf265c4813f659bd49768378be6bf3cfee74
```

Do not add that directory to this repository; it is intentionally excluded by `.gitignore`.

### 3. Build

Default Freenove shipping build:

```bash
pio run -t upload
```

XIAO ESP32S3 Sense shipping build:

```bash
pio run -e combined_xiao_s3_sense -t upload
```

Diagnostic variants are available as:

```bash
pio run -e diag_s3_freenove -t upload
pio run -e diag_xiao_s3_sense -t upload
```

---

## Recommended development workflow

1. **Harness first** — verify the camera and detector independently.
2. **Diagnostic OpenFIRE build** — confirm tracking behavior with the full firmware stack.
3. **Shipping build** — validate the actual gameplay path.
4. **Real game test** — evaluate aiming, stability, calibration, and tuning before optimizing further.

Useful tools:

- `tools/dashboard.py` — live camera/tracking visualization and tuning.
- `tools/health.py` — capture and tracking health scoring.
- `tools/screen_pattern.html` — visible-screen border and fiducial patterns.
- `docs/CALIBRATION.md` — detailed calibration and tuning notes.

---

## Host tests

The geometry and visible-screen detector can be tested without hardware.

Example:

```bash
g++ -std=c++17 -O2 -Ilib/OV2640Capture \
    -o test_screen_detector \
    test/test_screen_detector.cpp \
    lib/OV2640Capture/screen_detector.cpp

./test_screen_detector
```

The roadmap includes adding CI later so supported firmware targets and host tests are exercised automatically on every change.

---

## Licensing

This repository's own code is licensed under **GPL-3.0**; see [`LICENSE`](LICENSE).

OpenFIRE is a separate upstream project and is licensed under **LGPL-2.1**. It is not included in this repository. Developers clone it locally into the ignored `OpenFIRE-Firmware-ESP32/` directory and remain responsible for complying with the upstream project's license terms when redistributing binaries or derivative work.

The vendored Espressif camera-driver code under `lib/esp32-camera-ov2640/` retains its original **Apache-2.0** licensing and headers; project-specific modifications should continue to preserve those notices.
