# openfire-ov2640-overlay

An **OV2640 camera backend for [OpenFIRE](https://github.com/alessandro-satanassi/OpenFIRE-Firmware-ESP32)** on the ESP32-S3 — a drop-in replacement for the DFRobot/PAJ7025 IR camera that OpenFIRE normally requires, built from a ~$10 camera dev board and an IR-pass filter.

OpenFIRE itself is **not modified and not vendored**: you clone it next to this project and this project builds it as-is. Our camera stack shadows OpenFIRE's `DFRobotIRPositionEx` class with one that is fed by an OV2640 instead of an I²C IR camera.

## What's inside

| Piece | What it does |
|---|---|
| `lib/OV2640Capture/` | Capture core: streaming blob detector that runs **inside the camera driver's DMA chunk callback** (no framebuffer post-pass), duplicate/smear filtering, and a **quad resolver** that maintains persistent corner identity and reconstructs briefly-missing corners from a continuously-learned rigid model — so OpenFIRE always sees 4 stable, correctly-labelled points |
| `lib/DFRobotIRPositionEx_OV2640/` | The shim: OpenFIRE-compatible `DFRobotIRPositionEx` served from the camera stream via a lock-free seqlock bridge |
| `lib/esp32-camera-ov2640/` | Vendored esp32-camera driver (Apache-2.0, Espressif) with capture-robustness patches — see `lib/esp32-camera-ov2640/MODIFICATIONS.md` |
| `harness/` | Standalone bring-up firmware: the camera stack alone, no OpenFIRE, full serial telemetry — flash this first |
| `tools/` | `dashboard.py` (live point map + tuning console) and `health.py` (PASS/FAIL health scoring) |
| `test/` | Host-side unit tests (no hardware needed), including 22 scenarios for the quad resolver |

## Hardware

* **Freenove ESP32-S3-WROOM CAM** (N8R8: 8 MB flash + 8 MB octal PSRAM) or compatible ESP32-S3 board with an OV2640
* OV2640 camera module with the IR-blocking filter removed and an **IR-pass filter** (~700 nm long-pass) fitted
* 4 IR LED emitters arranged as a rectangle around the screen (OpenFIRE's 4-point square/diamond layouts)
* Wiring for trigger/buttons/solenoid/rumble per `platformio.ini` (`; ---- wiring` section — **adjust the pin defines to your gun**)

Camera pinout is the Freenove S3 CAM mapping, defined at the top of `lib/OV2640Capture/ov2640_capture.cpp`.

## Build & flash

Requires [PlatformIO](https://platformio.org/) (CLI or VS Code extension) and Python 3 with `pyserial` for the tools.

```
git clone <this repo>
cd openfire-ov2640-overlay

# OpenFIRE upstream, side by side (NOT a fork, never edited):
git clone https://github.com/alessandro-satanassi/OpenFIRE-Firmware-ESP32.git
# tested against commit f8f9bf2 (v6.2.1-7-gf8f9bf2); newer commits usually
# work because the shim boundary is narrow, but that one is known-good:
git -C OpenFIRE-Firmware-ESP32 checkout f8f9bf265c4813f659bd49768378be6bf3cfee74

pio run -t upload          # shipping build (default env: combined_s3_freenove)
```

The boot banner (USB serial or UART0, 115200) is the stale-build check — it must read:

```
OV2640Capture v28 SHIP | thr=80 aec=40 agc=2 boost=0 ...
```

If the version or `SHIP`/`DIAG` tag doesn't match what you flashed, run `pio run -t fullclean` and upload again.

Then set up OpenFIRE itself (pairing, calibration, profiles) with the standard [OpenFIRE App](https://github.com/TeamOpenFIRE/OpenFIRE-App) — from the app's point of view this is an ordinary OpenFIRE gun.

## Verification, tuning and calibration

The shipping build carries **no diagnostic serial output** (one boot banner line only). All telemetry, the live dashboard stream, and the runtime tuning console are compiled into a separate, otherwise-identical environment:

```
pio run -e diag -t upload
```

**See [`docs/CALIBRATION.md`](docs/CALIBRATION.md)** for the full workflow: reactivating the serial monitor, tuning threshold/exposure to your room with `tools/dashboard.py`, scoring the result with `tools/health.py`, making a tune permanent, and returning to the shipping build.

## Bring-up order (recommended)

1. **Harness first**: `cd harness && pio run -t upload`. This runs the camera stack alone with full telemetry — if the 4 points aren't rock-solid here, fix that before involving OpenFIRE.
2. **Diag build**: verify the same detection quality with OpenFIRE running (`pio run -e diag -t upload`, then `tools/health.py`).
3. **Shipping build**: `pio run -t upload`, calibrate in the OpenFIRE app, play.

## Host tests

The geometry code is testable without hardware:

```
g++ -std=c++17 -O2 -Ilib/OV2640Capture -o test_quad \
    test/test_quad_resolver.cpp lib/OV2640Capture/quad_resolver.cpp && ./test_quad
```

22 scenarios, including off-screen reload at a new angle, junk-blob rejection during re-acquire, and blackout-while-moving recovery.

## License

This project: **GPL-3.0** (see `LICENSE`) — it is designed to be compiled together with OpenFIRE, which is GPL. The vendored `lib/esp32-camera-ov2640/` driver remains **Apache-2.0** (Espressif and contributors); modified files are listed in `lib/esp32-camera-ov2640/MODIFICATIONS.md` and keep their original headers.
