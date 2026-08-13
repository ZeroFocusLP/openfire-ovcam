# Verification & calibration

The shipping build is silent on purpose: one boot banner line, nothing else.
Every diagnostic — the per-second telemetry, the live dashboard stream, and
the runtime tuning console — lives in a separate build environment that is
identical in everything that touches capture or aim.

## 1. Reactivate the serial monitor (flash the diag build)

```
pio run -e diag -t upload
```

Confirm on the serial monitor (UART0, **115200 baud** — on the Freenove S3 CAM
this is the CH340 COM port, *not* the native-USB port OpenFIRE's app uses):

```
OV2640Capture v28 DIAG | thr=80 aec=40 agc=2 boost=0 xclk=27 pdiv=3 | coin=on res=ON(geom8)
```

`DIAG` in the banner means the console and telemetry are live. (`SHIP` means
you are on the shipping build and none of the below will respond.)

Both serial ports can be connected at once: OpenFIRE's app talks on the
native-USB CDC port while the diagnostics ride UART0, so you can watch
telemetry live while aiming.

## 2. Watch the live picture — `tools/dashboard.py`

```
pip install pyserial
python tools/dashboard.py --port COM8        # your CH340 port
```

The dashboard shows the detected points on a live map, the per-second STAT
line, and gives you a command box. It enables the point stream (`dash=1`)
automatically. What good looks like: 4 steady dots, `fps` ≈ 134, no dots
flickering in and out.

## 3. Tune to your room (runtime console)

Type `key=value` commands into the dashboard (or any serial terminal on the
same port). They apply instantly — no reflash:

| Command | Meaning | Notes |
|---|---|---|
| `thr=N` | pixel threshold (8–250) | **The main knob.** Too low: junk blobs appear. Too high: LEDs drop when off-axis |
| `aec=N` | exposure, in sensor lines | Real photons — prefer raising this over gain |
| `agc=N` | analog gain index (0–30) | Keep low; gain amplifies noise with signal |
| `boost=0/1` | extra ×4 analog gain stages | Leave 0 unless the rig is very dim; raising it usually forces `thr` up too |
| `res=2/1/0` | quad resolver: geometry-picked 8 / top-4 by mass / off | Default 2. `res=0` falls back to the plain coincidence gate — useful for A/B |
| `coin=1/0` | temporal coincidence gate (legacy path) | Only relevant when `res=0` |
| `dash=1/0`, `dashhz=N`, `dashb=0..3` | point-stream on/off, rate cap, which point the B-line carries | The dashboard manages these itself |
| `dbg=1/0` | per-second telemetry block | On by default in diag |

Tuning procedure: aim the gun at the play position, lower `thr` until all four
points hold steady at every angle you actually play at (including screen
corners), then check junk: if stray dots appear, raise `thr` slightly or lower
`agc`/`aec`. Small steps; one variable at a time.

## 4. Score it — `tools/health.py`

While the dashboard is running (it writes `dashboard_*.log` next to itself):

```
python tools/health.py                 # follows the newest dashboard log
python tools/health.py --serial COM8   # or read the port directly (no dashboard)
python tools/health.py --log dashboard_xxx.log --once   # one-shot text verdict
```

It renders a PASS/FAIL panel with a weighted quality score. The checks that
matter most: **4-point yield** (should be ~100%), **count churn 0/s**
(every churn event fires OpenFIRE's error spring for ~70 frames), rejected
frames ~0/s, and resolver confidence ~1.0.

Telemetry lines you'll see on the raw serial feed, one per second:

* `PTS/s` — published point-count histogram + churn (the aim-quality predictor)
* `RES/s` — quad resolver: reconstructions, re-acquires, tilt envelope, fit residual
* `COST/s` — resolver CPU time, capture→OpenFIRE latency, cam_task stack headroom
* `V,...` / `CHUNKS/s` / `ISR/s` — capture-layer fault counters (should all sit at ~0)

## 5. Make it permanent

Runtime tunes are **not saved** — they reset on reboot. When you've found your
values, write them into the boot recipe in
`lib/OV2640Capture/ov2640_capture.cpp`:

```c
static const int BOOT_THR   = 80;   // <- your thr
static const int BOOT_AEC   = 40;   // <- your aec
static const int BOOT_AGC   = 2;    // <- your agc
static const int BOOT_BOOST = 0;    // <- your boost
```

## 6. Back to the shipping build

```
pio run -t upload
```

Banner must now read `... v28 SHIP ...`. Calibrate in the OpenFIRE app as
usual and play. If anything ever looks off, flash `diag` again — same
firmware, plus eyes.

## Harness (camera stack alone, no OpenFIRE)

For first bring-up or hardware debugging, `harness/` builds the capture stack
with nothing else on the chip:

```
cd harness && pio run -t upload
```

It prints the same telemetry (always on — the harness is the instrument) and
forwards serial input lines to the same tuning console. If detection is not
solid here, no amount of OpenFIRE-side work will fix it.
