# Guidelines for AI Agents & Contributors

This repository follows a strict documentation and architecture discipline. All AI agents (Antigravity, Claude, Copilot, Cursor, etc.) and human developers working on this codebase **must adhere to these instructions**.

---

## 1. Mandatory Documentation Discipline

Whenever you finish a task, implement a feature, or conclude an architectural discussion:

1. **Update [`docs/CHANGELOG.md`](docs/CHANGELOG.md):**
   - Append a clear, dated summary under `[Unreleased]`.
   - Document new hardware additions, pin assignments, register changes, and fixes.
   - Include both theoretical calculations and empirical hardware telemetry (measured FPS, VSYNC periods, dropped frames).
2. **Update [`docs/GOAL.md`](docs/GOAL.md):**
   - Check if any new decision deviates from or expands upon the original project goal.
   - Update the milestone tracker, hardware roadmap, or peripheral mapping accordingly.

---

## 2. Hardware & Pinout Reservation Rules

On the **Seeed Studio XIAO ESP32S3 Sense**:
* **I2C Bus (GPIO 5 / D4 and GPIO 6 / D5):**
  - **STRICTLY RESERVED for peripherals** (OLED display and 6-axis IMU).
  - **NEVER** assign buttons, solenoids, or rumble to these pins.
* **Addressable NeoPixel (GPIO 43 / D6):**
  - Reserved for muzzle flash, health, and status feedback.
* **Control Inputs (GPIO 1, 2, 3, 4, 44, 9):**
  - Assigned to Trigger, Buttons A, B, C, Start, and Select.
* **Force Feedback / Outputs (GPIO 7 / D8 and GPIO 8 / D9):**
  - Solenoid and Rumble motor control.
* **Pedal:**
  - Kept unmapped (`-1`) to conserve pins.

When adding new boards, define all camera pins in `lib/OV2640Capture/camera_pins.h` and board peripheral presets in `OpenFIRE-Firmware-ESP32/lightgun/src/boards/OpenFIREshared.h`.

---

## 3. Sensor & Camera Driver Integrity

1. **Sensor-Agnostic Code:**
   - Always check sensor PID (`s->id.PID == OV2640_PID`, `OV3660_PID`, `OV5640_PID`) before issuing sensor-specific registers.
   - 8-bit registers (e.g. `0x0D3`, bank select `0xFF`) are OV2640-only. OV3660 and OV5640 use 16-bit registers (e.g. `0x3000`–`0x5000`).
2. **Timing & Frame Rate Verification:**
   - Always calculate theoretical FPS:
     $$\text{Theoretical FPS} = \frac{\text{SYSCLK}}{\text{HTS} \times \text{VTS}}$$
   - Always verify empirical timing on real hardware using the test harness (`harness/`):
     - Check `VSI avg` (VSYNC interval in microseconds).
     - Check `ACCT/s` (`sensor`, `delivered`, `rejected`, `HOLE`).
     - Ensure chunks per frame equal 11 with `short=0` and `long=0`.
3. **Preserve Image Quality:**
   - On OV3660 and OV5640, preserve hardware 2×2 ISP binning for high sub-pixel precision in visible screen tracking.

---

## 4. Submodule & Architecture Hygiene

* The core engine lives in `OpenFIRE-Firmware-ESP32/` as a submodule.
* Keep edits to the submodule minimal and cleanly decoupled so future upstream OpenFIRE pull requests remain simple and non-breaking.
* Put capture drivers, computer vision modules, and hardware shims into `lib/` in the top-level repository.

---

## 5. Serial Protocol & USB Flashing Mechanics

* **Flashing the ESP32-S3:**
  - In OpenFIRE runtime, the device runs TinyUSB (`/dev/cu.usbmodem1020BA...`).
  - To enter the ROM bootloader for flashing, trigger a 1200-baud reset on the CDC port. The device will reappear at `/dev/cu.usbmodem1101`.
* **OpenFIRE Batch Protocol Echo:**
  - Host batch commands (such as `sGetPins` 200) send items that require an exact packet echo acknowledgment from the host. Failing to echo stalls Core 1 until timeouts expire. Always acknowledge batch packets when communicating programmatically.
