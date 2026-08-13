#!/usr/bin/env python3
"""IR lightgun serial dashboard v3 — WiFi-free tuning + diagnostics over USB.

Reads the v48+ firmware stream:
    B,<ms>,<n>,<x10>,<y10>      per-frame blob fix (160/s)
    STAT,<ms>,<fps>,...         once per second, carries thr/aec/agc/boost
    FRM,<w>,<h>,<ms>,<thr>      snapshot header, then w*h raw bytes, then ENDFRM
    CLK-ACT / REGS / CFG-ACTIVE diagnostics

Features: live 5s track map, snapshot viewer with threshold overlay, clickable
mode buttons, SCROLLABLE log with auto-save to file, and one-click clipboard
copy of the recent log (for pasting into a chat).

    pip install pyserial pygame
    python tools/dashboard.py --port COM8

Keys: 1 STOCK100 · 2 120rb · 3 FULL136 · f SNAP · v VIEW · r REGS · c COPY
      t/T thr · e/E aec · g boost · mouse wheel scrolls log · ENTER raw cmd · ESC quit
"""
import argparse, collections, math, re, sys, threading, time

try:
    import serial, serial.tools.list_ports
except ImportError:
    sys.exit("pip install pyserial")
try:
    import pygame
except ImportError:
    sys.exit("pip install pygame")

SIZES = {"QQVGA": (160, 120), "HQVGA": (240, 176), "QVGA": (320, 240), "CIF": (400, 296)}

# S3 XCLK LADDER (esp32s3/ll_cam.c:321 divides 160MHz by an INTEGER):
#   request 16 -> div 10 -> 16.00MHz -> sysclk 32.0 -> ~80fps
#   request 20 -> div  8 -> 20.00MHz -> sysclk 40.0 -> ~100fps   (proven good)
#   request 24 -> 24.00MHz -> sysclk 48.0 -> ~120fps  (exactly at spec)
#   request 27 -> 27.02MHz -> sysclk 54.0 -> ~135fps  (the classic's point)
# v50 patched the driver to use the LCD_CAM FRACTIONAL divider, so these are
# now real frequencies instead of silently snapping to 32MHz (which is what
# made every "160fps" run photograph as gray bands with light reading dark).
# Exposure matters too: aec=220+boost flooded at working clocks.
CMD_STOCK = "size=1&xclk=20&dbl=1&div=0&y8=0&r32=0&pdiv=0&thr=60&aec=280&agc=30&boost=1"
CMD_OC120 = "rb=1&size=1&xclk=24&dbl=1&div=0&y8=0&r32=0&pdiv=0&thr=60&aec=280&agc=30&boost=1"
# THE mode. Measured: 136 fps with the FULL sensor.
# v73 HARDENING — the modes that used to sit here, and why they are GONE:
#   FULL 150 / 145rb / 140rb (xclk 30/29/28): ALL measured fuzzy+banded with
#     bimodal VSYNC. Two datasheet reasons, either sufficient: (a) Table 8
#     requires a CLEAN input clock (duty 45-55%, rise <=5ns) and the S3's
#     fractional divider makes 28-30 by ALTERNATING /5 and /6 periods — heavy
#     jitter (27.02 is 92% uniform and works); (b) R_DVP_SP's own description
#     says "sysclk (48)" — the DSP domain is DOCUMENTED at 48MHz, we run 54
#     (works, +12%), 56+ failed. 136 is this hardware's practical ceiling.
#   r32 trap (r32=1&pdiv=3): half-frame demonstrator. Educational, breaks aim.
#   PCLK A/B: REG32 bit-7 route experiment — A is byte-identical to FULL 136,
#     B delivers half a frame. Question answered, buttons retired.
#   80 rb (xclk 16): the heal ladder reaches it automatically when needed.
#   TEST PAT: colour-bar does not survive this part's DSP (proven, twice).
# All still reachable by typing the k=v command by hand (Enter) if ever needed.
CMD_FULL104 = "rb=1&size=1&xclk=27&dbl=1&div=0&y8=0&r32=0&pdiv=3&thr=60&aec=280&agc=30&boost=1"


class State:
    def __init__(self):
        self.fixes = collections.deque(maxlen=4000)   # bounded: device-time
                                                     # pruning alone could grow forever
        self.last_rx = 0.0                           # host clock: staleness detection
        self.dims = (240, 176)
        self.log = collections.deque(maxlen=4000)
        self.last_stat = ""
        # v73: xclk / y8 / pdiv are NOT in the STAT line, so seeding them here
        # meant the panel displayed a startup guess as though it were telemetry
        # -- it read "xclk 20 req" while the board was demonstrably at 27 (the
        # 7397us sensor period only happens at sysclk 54). Start them ABSENT so
        # they render as "?" until the board actually tells us.
        self.cfg = {"thr": 128, "aec": 220, "agc": 30, "boost": 1}
        self.frame = None
        self.frame_seq = 0
        # SNAP watchdog (): a frame=1 the firmware never answers used
        # to fail SILENTLY (no frames flowing / wrong firmware flashed / snap
        # buffer wedged) — the button just "did nothing". Track the request so
        # the UI can report WHY nothing came back.
        self.snap_req_t = 0.0
        self.snap_req_seq = -1
        self.levels = ""
        self.cfg_seq = 0
        # v66 SWEEP: two 8x6 maps accumulated over a wave-the-source window.
        # swmax = brightest pixel ever seen per cell, swhit = frames whose blob
        # landed there. Bright-but-never-hit is a DETECTOR fault; dark-in-both is
        # a SENSOR/delivery fault. Kept separate so they cannot be conflated.
        self.sw_max = []
        self.sw_hit = []
        self.sw_info = ""
        self.sw_seq = 0
        self.sw_active = 0.0          # host-clock deadline while a sweep runs
        # set once the OVERLAY build identifies itself. Same wire
        # format, different firmware -- and a different sensor recipe, so the
        # lab presets must not be sent to it (see send()).
        self.overlay = False
        # v15 AIM view: the full quad per frame, (t, n, [(x,y), ...]). The
        # single-point B stream cannot answer "is the QUAD moving or is the
        # solve amplifying it?", which is the whole question.
        self.quads = collections.deque(maxlen=4000)
        self.lock = threading.Lock()


def parse_kv(raw, cfg):
    for k, v in re.findall(r"(\w+)=(-?\d+)", raw):
        if k in ("thr", "aec", "agc", "boost", "y8", "xclk", "dbl", "div", "r32",
                 "pdiv", "ss", "unclip", "agcl", "sat"):
            cfg[k] = int(v)


# ---------------------------------------------------------------- v15 AIM ---
# A PLAIN 4-point homography: camera quad -> unit screen, then the camera
# CENTRE mapped through it. That is what a lightgun computes, minus every
# correction OpenFIRE layers on top (no square solver, no kinematic spring, no
# One-Euro filter, no perspective calibration). The point is precisely that it
# has no corrections: whatever this dot does IS what the four raw points did.
# So if the dot is steady here while the gun teleports, the fault is downstream
# of the camera; if the dot teleports here, it is upstream. No third option.

def sort_quad(pts, mirror_x, w):
    """Label 4 camera points TL,TR,BL,BR. Returns None unless exactly 4."""
    if len(pts) != 4:
        return None
    q = [((w - 1.0 - x) if mirror_x else x, y) for (x, y) in pts]
    cx = sum(p[0] for p in q) / 4.0
    cy = sum(p[1] for p in q) / 4.0
    tl = tr = bl = br = None
    for p in q:
        if p[1] < cy:
            if p[0] < cx: tl = p if tl is None else tl
            else:         tr = p if tr is None else tr
        else:
            if p[0] < cx: bl = p if bl is None else bl
            else:         br = p if br is None else br
    if None in (tl, tr, bl, br):
        # Degenerate split (3 points in one quadrant = the quad has collapsed).
        # Fall back to row-then-column order rather than silently drawing a dot
        # from a labelling we do not believe.
        s = sorted(q, key=lambda p: p[1])
        top = sorted(s[:2], key=lambda p: p[0])
        bot = sorted(s[2:], key=lambda p: p[0])
        tl, tr, bl, br = top[0], top[1], bot[0], bot[1]
    return [tl, tr, bl, br]


def homography(src, dst):
    """8x8 solve for the projective map src->dst. Pure Python, no numpy."""
    A, b = [], []
    for (sx, sy), (dx, dy) in zip(src, dst):
        A.append([sx, sy, 1, 0, 0, 0, -dx * sx, -dx * sy]); b.append(dx)
        A.append([0, 0, 0, sx, sy, 1, -dy * sx, -dy * sy]); b.append(dy)
    n = 8
    for col in range(n):                                   # Gaussian elimination
        piv = max(range(col, n), key=lambda r: abs(A[r][col]))
        if abs(A[piv][col]) < 1e-9:
            return None                                    # degenerate quad
        A[col], A[piv] = A[piv], A[col]
        b[col], b[piv] = b[piv], b[col]
        d = A[col][col]
        A[col] = [v / d for v in A[col]]; b[col] /= d
        for r in range(n):
            if r == col:
                continue
            f = A[r][col]
            if f:
                A[r] = [v - f * w for v, w in zip(A[r], A[col])]
                b[r] -= f * b[col]
    h = b + [1.0]
    return h


def aim_point(pts, dims, mirror_x):
    """4 camera points -> (x, y) in 0..1 screen space, or None."""
    quad = sort_quad(pts, mirror_x, dims[0])
    if quad is None:
        return None
    h = homography(quad, [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)])
    if h is None:
        return None
    # The optical axis is the centre of the PIXEL GRID, (w-1)/2 -- not w/2.
    # It matters because the mirror flips about (w-1): using w/2 puts the axis
    # half a pixel off and makes mirrored and unmirrored views disagree.
    cx, cy = (dims[0] - 1) / 2.0, (dims[1] - 1) / 2.0
    den = h[6] * cx + h[7] * cy + h[8]
    if abs(den) < 1e-9:
        return None
    return ((h[0] * cx + h[1] * cy + h[2]) / den,
            (h[3] * cx + h[4] * cy + h[5]) / den)


def read_exact(ser, n, timeout_s=8.0):
    # returns the PARTIAL buffer on timeout (was None) so the
    # caller can diagnose WHERE the payload stopped and what got mixed in.
    # Timeout raised 4->8s: the deadline is idle-protection, not a budget.
    buf = bytearray()
    deadline = time.time() + timeout_s
    while len(buf) < n and time.time() < deadline:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
    return bytes(buf)


def printable_runs(data, min_len=6, max_runs=3):
    # extract ASCII text fragments embedded in a binary payload — if another
    # task printed mid-snapshot, this shows exactly WHO corrupted the stream
    runs, cur = [], bytearray()
    for b in data:
        if 32 <= b < 127:
            cur.append(b)
        else:
            if len(cur) >= min_len:
                runs.append(cur.decode())
                if len(runs) >= max_runs:
                    break
            cur = bytearray()
    if len(cur) >= min_len and len(runs) < max_runs:
        runs.append(cur.decode())
    return runs


def reader(ser, st, logfile):
    # EVERY iteration is wrapped: an exception here used to kill the daemon
    # thread silently, leaving the UI frozen on stale data with no indication.
    while True:
        try:
            _reader_once(ser, st, logfile)
        except Exception as e:
            with st.lock:
                st.log.append(f"[reader recovered: {type(e).__name__}: {e}]")
            try:
                ser.reset_input_buffer()
            except Exception:
                pass
            time.sleep(0.5)


def _reader_once(ser, st, logfile):
    if True:
        raw = ser.readline().decode(errors="replace").strip()
        raw = "".join(ch for ch in raw if ch.isprintable())
        if not raw:
            return

        if raw.startswith("FRM,"):                      # snapshot
            p = raw.split(",")
            ok = False
            try:
                w, h, thr = int(p[1]), int(p[2]), int(p[4])
                bx = by = None
                if len(p) >= 8 and int(p[5]) > 0:
                    bx, by = int(p[6]) / 10.0, int(p[7]) / 10.0
                # bound the payload: a garbled header used to make the reader
                # block for its whole timeout (losing ~500 fixes) or hand an
                # empty buffer to min()/max() on the main thread and crash it.
                ok = 0 < w <= 800 and 0 < h <= 600
            except (ValueError, IndexError):
                ok = False
            if not ok:
                with st.lock:
                    st.log.append("[snapshot header malformed - resyncing]")
                try:
                    ser.reset_input_buffer()
                except Exception:
                    pass
                return
            data = read_exact(ser, w * h)
            tail = ser.read_until(b"ENDFRM", 64) if len(data) == w * h else b""
            if len(data) == w * h and b"ENDFRM" in tail:
                lo, hi = min(data), max(data)
                with st.lock:
                    st.frame = (w, h, data, thr, bx, by, lo, hi)
                    st.frame_seq += 1
                    st.snap_req_t = 0.0            # watchdog: answered
                    st.log.append(f"[snapshot {w}x{h} thr={thr} range {lo}..{hi}"
                                  + (f" blob {bx:.1f},{by:.1f}]" if bx is not None else " no-blob]"))
            else:
                # leftover pixel bytes would otherwise be split on stray 0x0A
                # into hundreds of fake "log lines" (some parsing as FRM/STAT)
                try:
                    ser.reset_input_buffer()
                except Exception:
                    pass
                # diagnostics: SAY WHY instead of a generic failure.
                got, want = len(data), w * h
                frags = printable_runs(data[-2048:]) if got else []
                with st.lock:
                    st.snap_req_t = 0.0
                    st.log.append(f"[snapshot INCOMPLETE: {got}/{want} bytes"
                                  + (f", ENDFRM missing" if got == want else "")
                                  + " - input flushed]")
                    if frags:
                        st.log.append("[text found INSIDE payload (a task printed "
                                      f"mid-snapshot): {' | '.join(frags)!r}]")
                    elif got == 0:
                        st.log.append("[payload never started: firmware sent the "
                                      "header then stalled - check for SNAPABORT "
                                      "in its output]")
            return

        if raw.startswith("Q,"):                        # v15 quad stream (overlay)
            # Q,<ms>,<n>,<x10>,<y10> x4  -- ALL FOUR points, native camera px.
            # Absent points are -1,-1. Also fed into st.fixes (point 0) so the
            # existing map, fix-rate and verdict logic keep working untouched.
            p = raw.split(",")
            if len(p) == 11:
                try:
                    t, n = int(p[1]), int(p[2])
                    pts = [(int(p[3 + 2 * i]) / 10.0, int(p[4 + 2 * i]) / 10.0)
                           for i in range(4)]
                    pts = [q for q in pts if q[0] >= 0 and q[1] >= 0]
                    with st.lock:
                        st.last_rx = time.monotonic()
                        if st.quads and t < st.quads[-1][0]:
                            st.quads.clear(); st.fixes.clear()   # device rebooted
                        st.quads.append((t, n, pts))
                        while st.quads and t - st.quads[0][0] > 5000:
                            st.quads.popleft()
                        st.fixes.append((t, n, pts[0][0] if pts else -0.1,
                                         pts[0][1] if pts else -0.1))
                        while st.fixes and t - st.fixes[0][0] > 5000:
                            st.fixes.popleft()
                except (ValueError, IndexError):
                    pass
            return

        if raw.startswith("B,"):                        # fix stream (not logged)
            p = raw.split(",")
            if len(p) == 5:
                try:
                    t, n = int(p[1]), int(p[2])
                    with st.lock:
                        st.last_rx = time.monotonic()
                        if st.fixes and t < st.fixes[-1][0]:   # any backward jump = reboot
                            st.fixes.clear()            # device rebooted
                        st.fixes.append((t, n, int(p[3]) / 10.0, int(p[4]) / 10.0))
                        while st.fixes and t - st.fixes[0][0] > 5000:
                            st.fixes.popleft()
                except ValueError:
                    pass
            return

        # everything else: UI log + file (B-lines excluded to keep it readable)
        if logfile:
            try:
                logfile.write(raw + "\n")
                logfile.flush()
            except Exception:
                pass
        with st.lock:
            st.last_rx = time.monotonic()
            if raw.startswith("STAT,"):
                st.last_stat = raw
                # v71: agcl= / sat= are emitted ONLY when active/non-zero.
                # Drop stale copies first or the panel would keep showing a
                # servo state the firmware stopped reporting seconds ago.
                if " agcl=" not in raw:
                    st.cfg.pop("agcl", None)
                if " sat=" not in raw:
                    st.cfg.pop("sat", None)
                parse_kv(raw, st.cfg)
            if raw.startswith("LEVELS,"):
                st.levels = raw
            # ---- v66 SWEEP ----
            if raw.startswith("SWEEP start"):
                st.sw_max, st.sw_hit = [], []
                m = re.search(r"(\d+)ms", raw)
                st.sw_active = time.monotonic() + (int(m.group(1)) / 1000.0 if m else 5.0)
            elif raw.startswith("SWMAX|"):
                # rows arrive one at a time; a partial map is drawn as it fills
                st.sw_max = (st.sw_max + [[int(v) for v in raw[6:].split()]])[-6:]
            elif raw.startswith("SWHIT|"):
                st.sw_hit = (st.sw_hit + [[int(v) for v in raw[6:].split()]])[-6:]
            elif raw.startswith("SWEEP done"):
                st.sw_info = raw
                st.sw_active = 0.0
            elif raw.startswith("SWEEP end"):
                st.sw_seq += 1
            if "OV2640Capture v" in raw and not st.overlay:
                st.overlay = True
                st.log.append("[OVERLAY build detected - lab presets (1/2/3) "
                              "are now blocked; send dash=1 for the map]")
            st.log.append(raw[:150])
            if raw.startswith(("CMD ok", "CFG-ACTIVE:")):
                parse_kv(raw, st.cfg)
                st.cfg_seq = st.cfg_seq + 1
            if raw.startswith("CFG-ACTIVE:"):
                for name, wh in SIZES.items():
                    if (" " + name + " ") in (raw + " "):
                        st.dims = wh


class Button:
    def __init__(self, x, y, w, h, label, cmd=None, color=(53, 85, 102), fn=None):
        self.r = pygame.Rect(x, y, w, h)
        self.label, self.cmd, self.color, self.fn = label, cmd, color, fn

    def draw(self, screen, font, mouse):
        c = tuple(min(255, v + 25) for v in self.color) if self.r.collidepoint(mouse) else self.color
        pygame.draw.rect(screen, c, self.r, border_radius=4)
        t = font.render(self.label, True, (255, 255, 255))
        screen.blit(t, t.get_rect(center=self.r.center))

    def click(self, pos, send):
        if self.r.collidepoint(pos):
            if self.fn:
                self.fn()
            elif self.cmd:
                send(self.cmd)
            return True
        return False


def draw_aim(screen, font_, bfont_, big_, rect, quads, dims, mirror_x):
    """v15 AIM view: the aim dot from a plain 4-point homography, no filtering.

    Returns a dict of stats for the side panel. Everything drawn here is
    derived ONLY from the four raw camera points -- if this dot is calm and the
    gun is not, the camera side is exonerated.
    """
    X, Y, W_, H_ = rect
    out = {"dot": None, "n4": 0, "nfr": 0, "jump_max": 0.0, "jumps": 0,
           "pt_max": 0.0, "hold": 0}
    pygame.draw.rect(screen, (16, 16, 20), rect)
    # screen-shaped inset (16:9) -- this rectangle IS the display you aim at
    m = 26
    sw = W_ - 2 * m
    sh = int(sw * 9 / 16)
    if sh > H_ - 2 * m:
        sh = H_ - 2 * m
        sw = int(sh * 16 / 9)
    sx0, sy0 = X + (W_ - sw) // 2, Y + (H_ - sh) // 2 + 6
    pygame.draw.rect(screen, (34, 40, 48), (sx0, sy0, sw, sh), 2)
    for i in (1, 2, 3):
        pygame.draw.line(screen, (26, 30, 36), (sx0 + sw * i // 4, sy0),
                         (sx0 + sw * i // 4, sy0 + sh))
        pygame.draw.line(screen, (26, 30, 36), (sx0, sy0 + sh * i // 4),
                         (sx0 + sw, sy0 + sh * i // 4))

    trail, prev = [], None
    for (t, n, pts) in quads:
        out["nfr"] += 1
        if len(pts) != 4:
            out["hold"] += 1                      # <4 points: OpenFIRE holds
            continue
        out["n4"] += 1
        a = aim_point(pts, dims, mirror_x)
        if a is None:
            continue
        trail.append((t, a))
        if prev is not None:
            d = math.hypot(a[0] - prev[0], a[1] - prev[1]) * 100.0
            if d > out["jump_max"]:
                out["jump_max"] = d
            if d > 2.0:
                out["jumps"] += 1
        prev = a
    # worst per-frame movement of any SINGLE raw point, in native px --
    # the number that separates "the points moved" from "the solve amplified"
    pq = None
    for (t, n, pts) in quads:
        if len(pts) == 4:
            if pq is not None:
                for (ax, ay), (bx, by) in zip(pts, pq):
                    d = math.hypot(ax - bx, ay - by)
                    if d > out["pt_max"]:
                        out["pt_max"] = d
            pq = pts

    for i, (t, a) in enumerate(trail):
        age = min(1.0, max(0.0, (trail[-1][0] - t) / 3000.0))
        c = max(0, min(255, int(200 * max(0.10, 1 - age))))
        px_ = sx0 + a[0] * sw
        py_ = sy0 + a[1] * sh
        if sx0 - 40 <= px_ <= sx0 + sw + 40 and sy0 - 40 <= py_ <= sy0 + sh + 40:
            screen.fill((30, c, 90), (px_ - 1, py_ - 1, 3, 3))
    if trail:
        a = trail[-1][1]
        out["dot"] = a
        px_, py_ = int(sx0 + a[0] * sw), int(sy0 + a[1] * sh)
        col = (255, 90, 90) if out["jump_max"] > 2.0 else (120, 255, 160)
        pygame.draw.circle(screen, col, (px_, py_), 9, 2)
        pygame.draw.line(screen, col, (px_ - 18, py_), (px_ - 12, py_), 2)
        pygame.draw.line(screen, col, (px_ + 12, py_), (px_ + 18, py_), 2)
        pygame.draw.line(screen, col, (px_, py_ - 18), (px_, py_ - 12), 2)
        pygame.draw.line(screen, col, (px_, py_ + 12), (px_, py_ + 18), 2)

    screen.blit(bfont_.render(
        "AIM - plain 4-point homography, NO filtering (not OpenFIRE's chain)  "
        "- m flips mirror - v cycles view", True, (250, 200, 120)), (X + 6, Y + 4))
    if not quads:
        screen.blit(bfont_.render(
            "no quad stream. Press q (sends dash=2) - overlay firmware v15+ only.",
            True, (250, 140, 140)), (X + 6, Y + 24))
    elif out["n4"] == 0:
        screen.blit(bfont_.render(
            f"never saw 4 points in 5s ({out['hold']} frames below 4) - "
            "OpenFIRE would be holding its last quad", True, (250, 180, 120)),
            (X + 6, Y + 24))
    else:
        pc4 = 100.0 * out["n4"] / max(1, out["nfr"])
        screen.blit(font_.render(
            f"4-point frames {pc4:.0f}%   worst cursor jump {out['jump_max']:.2f}% of "
            f"screen   worst raw point move {out['pt_max']:.2f} px",
            True, (170, 190, 210)), (X + 6, Y + H_ - 20))
        verdict = ("RAW POINTS ARE MOVING" if out["pt_max"] > 2.0 else
                   "SOLVE AMPLIFIES" if out["jump_max"] > 2.0 else "STEADY")
        vc = ((255, 110, 110) if out["pt_max"] > 2.0 else
              (250, 200, 110) if out["jump_max"] > 2.0 else (120, 230, 150))
        screen.blit(big_.render(verdict, True, vc), (X + 6, Y + 24))
    return out


def to_clipboard(text):
    try:
        import tkinter
        r = tkinter.Tk()
        r.withdraw()
        r.clipboard_clear()
        r.clipboard_append(text)
        r.update()
        r.destroy()
        return True
    except Exception:
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200)
    a = ap.parse_args()
    port = a.port
    if not port:
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            sys.exit("no serial ports found; pass --port")
        port = ports[0].device
        print(f"using {port} (pass --port to override)")
    ser = serial.Serial(port, a.baud, timeout=0.5)

    logname = time.strftime("dashboard_%Y%m%d_%H%M%S.log")
    try:
        logfile = open(logname, "w", encoding="utf-8")
    except Exception:
        logfile = None
    st = State()
    threading.Thread(target=reader, args=(ser, st, logfile), daemon=True).start()

    def send(cmd):
        # the dashboard now also talks to the OVERLAY build
        # (OV2640Capture v14+ streams the same B/STAT lines on UART0). Its
        # sensor runs a DIFFERENT recipe -- aec 20 / agc 0 / thr 150 on 850nm
        # LEDs -- so firing a lab preset at it (aec=280 agc=30) floods the
        # sensor instantly and looks like a firmware fault. Block the presets
        # rather than let a habit key ruin a measurement; everything else,
        # including thr/aec/agc/boost nudges and dash=/lab=, still goes through.
        if st.overlay and ("xclk=" in cmd or "aec=280" in cmd):
            with st.lock:
                st.log.append(f"[BLOCKED (overlay connected): {cmd}]")
                st.log.append("[lab presets would flood the overlay's sensor. "
                              "Use t/T e/E, or Enter: thr=150 aec=20 agc=0 "
                              "boost=1 / dash=1 / dashb=0..3 / lab=0|1]")
            return
        try:
            ser.write((cmd + "\n").encode())
        except Exception as e:
            with st.lock:
                st.log.append(f"[send failed: {e}]")
            return
        with st.lock:
            st.log.append(f"> {cmd}")
            if "frame=1" in cmd:                 # arm the SNAP watchdog
                st.snap_req_t = time.monotonic()
                st.snap_req_seq = st.frame_seq
        if logfile:
            try:
                logfile.write(f"> {cmd}\n")
                logfile.flush()
            except Exception:
                pass

    def step(key, delta, lo, hi):
        # shadow the value locally: two fast presses used to read the same
        # not-yet-updated cfg and send the identical command twice.
        with st.lock:
            base = shadow.get(key, st.cfg.get(key, lo))
        v = max(lo, min(hi, base + delta))
        shadow[key] = v
        send(f"{key}={v}")

    shadow = {}
    synced = [0.0]          # v73: one-shot config query on connect
    view = ["map"]
    cam_surf = [None]
    aim_stats = [{}]
    seen_seq = [0]
    scroll = [0]          # 0 = live tail; >0 = lines scrolled back
    copied = [0.0]
    seen_sw = [0]

    # DEFAULT: NO mirror. The Q-line carries RAW camera coordinates, and raw
    # camera X is already the physically correct sense -- rotate the gun right,
    # the scene moves left in the image, the aim point moves right. (The shim
    # DOES mirror on the way to OpenFIRE, but only because OpenFIRE then
    # un-mirrors again with (CamMaxX - px) to match the PixArt convention. Two
    # flips, no net flip: OpenFIRE's internal points equal these raw ones, which
    # is the configuration confirmed correct in the field after "left and right are
    # inverted".) 'm' toggles it if the dot ever moves the wrong way.
    mirror_x = [False]

    def toggle_view():
        order = ["map", "aim", "cam", "sweep"]
        view[0] = order[(order.index(view[0]) + 1) % len(order)] if view[0] in order else "map"

    def copy_log():
        with st.lock:
            txt = "\n".join(list(st.log)[-400:])
        copied[0] = time.time() if to_clipboard(txt) else 0.0

    stretch_on = [True]

    def draw_sweep(screen, font_, bfont_, rect, sw_max, sw_hit, info, thr):
        """Two 8x6 heat grids side by side: delivered brightness vs blob hits.

        The whole point is that these are INDEPENDENT. GRID (one frame) could
        only ever answer 'was this one spot dark right now', which is why it kept
        coming back uniform while a dead square was still being reported. Here a
        cell that is bright in the left grid but black in the right one is the
        detector throwing pixels away; a cell dark in both is the sensor never
        delivering that area. Nothing else can produce those two patterns.
        """
        x0, y0, w, h = rect
        pygame.draw.rect(screen, (18, 18, 18), rect)
        if not sw_max or len(sw_max) < 6:
            msg = ("SWEEP running - wave the IR source over the WHOLE field"
                   if info == "running" else
                   "press SWEEP, then wave the IR source over the whole field for 5s")
            screen.blit(bfont_.render(msg, True, (240, 200, 120)), (x0 + 10, y0 + 10))
            return
        # v72: SWMAX rows arrive before SWHIT rows, so there is a window where
        # sw_max is complete and sw_hit is still partial. The old code guarded
        # only sw_max and indexed sw_hit unconditionally -> IndexError inside the
        # render loop -> the whole dashboard dies exactly when a sweep lands.
        def cell(g, ry, rx):
            if ry >= len(g):
                return 0
            row = g[ry]
            return row[rx] if rx < len(row) else 0
        complete = len(sw_max) >= 6 and len(sw_hit) >= 6
        gw = (w - 30) // 2
        cw, ch = gw // 8, (h - 60) // 6
        hit_max = max((max(r) for r in sw_hit if r), default=0) if sw_hit else 0
        for gi, (title, grid, top) in enumerate((
                ("DELIVERED  (brightest pixel seen)", sw_max, 255),
                ("DETECTED   (frames with a blob here)", sw_hit, max(1, hit_max)))):
            gx0 = x0 + 10 + gi * (gw + 10)
            screen.blit(bfont_.render(title, True, (180, 200, 230)), (gx0, y0 + 6))
            for ry in range(6):
                row = grid[ry] if ry < len(grid) else [0] * 8
                for rx in range(8):
                    v = row[rx] if rx < len(row) else 0
                    f = min(1.0, v / top) if top else 0.0
                    col = ((int(30 + 200 * f), int(30 + 160 * f), 40) if gi == 0
                           else (40, int(30 + 200 * f), int(60 + 120 * f)))
                    r_ = pygame.Rect(gx0 + rx * cw, y0 + 28 + ry * ch, cw - 2, ch - 2)
                    pygame.draw.rect(screen, col, r_)
                    # a cell that is bright but never detected is the finding
                    if gi == 1 and v == 0 and cell(sw_max, ry, rx) > thr:
                        pygame.draw.rect(screen, (255, 80, 80), r_, 2)
                    screen.blit(font_.render(str(v), True, (240, 240, 240) if f < .6 else (10, 10, 10)),
                                (r_.x + 4, r_.y + 3))
        if not complete:                      # verdict needs BOTH maps in full
            screen.blit(bfont_.render("receiving sweep results...", True, (200, 190, 140)),
                        (x0 + 10, y0 + h - 24))
            return
        dead = sum(1 for ry in range(6) for rx in range(8)
                   if cell(sw_max, ry, rx) <= thr and cell(sw_hit, ry, rx) == 0)
        blind = sum(1 for ry in range(6) for rx in range(8)
                    if cell(sw_max, ry, rx) > thr and cell(sw_hit, ry, rx) == 0)
        if blind:
            note, nc = f"{blind} cell(s) got light above thr but NEVER produced a blob -> detector", (255, 120, 120)
        elif dead:
            note, nc = f"{dead} cell(s) never saw light above thr={thr} -> sweep them again / sensor", (250, 200, 120)
        else:
            note, nc = "every cell both received light and produced blobs - no dead region", (140, 230, 150)
        screen.blit(bfont_.render(note, True, nc), (x0 + 10, y0 + h - 24))

    def build_cam_surface(w, h, data, thr, bx=None, by=None, lo=None, hi=None):
        # CONTRAST STRETCH: at 135fps the frame is legitimately dim (exposure is
        # capped by the frame period), so raw pixels look almost black. Stretch
        # min..max to 0..255 for the eye; pixels the DETECTOR sees (>= thr) stay
        # flat red so the two views never get confused.
        if lo is None:
            lo, hi = min(data), max(data)
        span = max(1, hi - lo)
        px = bytearray()
        for b in data:
            if b >= thr:
                px += b"\xff\x3c\x3c"
            elif stretch_on[0]:
                v = (b - lo) * 255 // span
                px += bytes((v, v, v))
            else:
                px += bytes((b, b, b))
        return pygame.image.frombuffer(bytes(px), (w, h), "RGB")

    pygame.init()
    W, H = 980, 780
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption(f"IR blob dashboard - {port} (serial, no WiFi)")
    font = pygame.font.SysFont("consolas", 14)
    bfont = pygame.font.SysFont("consolas", 13, bold=True)
    small = pygame.font.SysFont("consolas", 12)
    big = pygame.font.SysFont("consolas", 24, bold=True)
    clock = pygame.time.Clock()
    typing, typed = False, ""

    MAPX, MAPY, MAPW, MAPH = 10, 10, 600, 430
    PX = MAPX + MAPW + 14
    GREEN, ORANGE, RED, GRAY = (90, 220, 130), (250, 170, 80), (250, 90, 90), (150, 150, 150)
    DGREEN, DBLUE, DRED, DPUR = (40, 130, 70), (38, 85, 120), (135, 60, 45), (85, 70, 120)

    # v73 HARDENED LAYOUT: two rows, only modes that cannot break a working
    # session. Retired buttons and why: see the comment block above CMD_FULL104.
    y1, y2 = MAPY + MAPH + 8, MAPY + MAPH + 44
    ykeys = MAPY + MAPH + 82   # key-hint strip
    LOGY = ykeys + 20
    buttons = [
        # --- row 1: the three proven modes + diagnostics that only LOOK ---
        Button(10,  y1, 92, 30, "STOCK 100", CMD_STOCK, DGREEN),
        Button(106, y1, 86, 30, "120 rb", CMD_OC120, DBLUE),
        Button(196, y1, 96, 30, "FULL 136", CMD_FULL104, DGREEN),
        Button(296, y1, 62, 30, "SNAP", fn=lambda: send("frame=1"), color=(120, 100, 40)),
        Button(362, y1, 62, 30, "VIEW", fn=toggle_view, color=(70, 100, 70)),
        Button(428, y1, 62, 30, "SWEEP", fn=lambda: send("sweep=5000"), color=(150, 90, 30)),
        Button(494, y1, 62, 30, "GRID", fn=lambda: send("grid=1"), color=(95, 75, 40)),
        Button(560, y1, 74, 30, "LEVELS", fn=lambda: send("levels=1"), color=(110, 95, 40)),
        Button(638, y1, 62, 30, "REGS", fn=lambda: send("regs=1"), color=(120, 70, 110)),
        Button(704, y1, 88, 30, "COPY LOG", fn=copy_log, color=(60, 95, 95)),
        # --- row 2: tuning ---
        Button(10,  y2, 68, 30, "thr -10", fn=lambda: step("thr", -10, 16, 255)),
        Button(82,  y2, 68, 30, "thr +10", fn=lambda: step("thr", +10, 16, 255)),
        Button(154, y2, 68, 30, "aec -20", fn=lambda: step("aec", -20, 1, 1200)),
        Button(226, y2, 68, 30, "aec +20", fn=lambda: step("aec", +20, 1, 1200)),
        Button(298, y2, 62, 30, "agc -2", fn=lambda: step("agc", -2, 0, 30)),
        Button(364, y2, 62, 30, "agc +2", fn=lambda: step("agc", +2, 0, 30)),
        Button(430, y2, 74, 30, "BOOST x", fn=lambda: send(f"boost={0 if st.cfg.get('boost') else 1}")),
        Button(508, y2, 88, 30, "UNCLIP x",
               fn=lambda: send(f"unclip={0 if st.cfg.get('unclip', 1) else 1}"),
               color=(100, 80, 120)),
        Button(600, y2, 92, 30, "AUTO EXP", fn=lambda: send("auto=1"), color=(60, 110, 90)),
        Button(696, y2, 68, 30, "MANUAL", fn=lambda: send("auto=0"), color=(70, 80, 95)),
    ]

    while True:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                return
            if ev.type == pygame.MOUSEWHEEL:
                scroll[0] = max(0, scroll[0] + ev.y * 3)
            if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1 and not typing:
                for b in buttons:
                    if b.click(ev.pos, send):
                        break
            if ev.type == pygame.KEYDOWN:
                if typing:
                    if ev.key == pygame.K_RETURN:
                        if typed:
                            send(typed)
                        typing, typed = False, ""
                    elif ev.key == pygame.K_ESCAPE:
                        typing, typed = False, ""
                    elif ev.key == pygame.K_BACKSPACE:
                        typed = typed[:-1]
                    elif ev.unicode and ev.unicode.isprintable():
                        typed += ev.unicode
                elif ev.key == pygame.K_ESCAPE:
                    return
                elif ev.key == pygame.K_RETURN:
                    typing = True
                elif ev.key == pygame.K_1:
                    send(CMD_STOCK)
                elif ev.key == pygame.K_2:
                    send(CMD_OC120)
                elif ev.key == pygame.K_3:
                    send(CMD_FULL104)
                elif ev.key == pygame.K_f:
                    send("frame=1")
                elif ev.key == pygame.K_v:
                    toggle_view()
                elif ev.key == pygame.K_r:
                    send("regs=1")
                elif ev.key == pygame.K_l:
                    send("levels=1")
                elif ev.key == pygame.K_s:
                    stretch_on[0] = not stretch_on[0]
                    if st.frame:
                        seen_seq[0] = -1          # force re-render
                elif ev.key == pygame.K_w:
                    send("sweep=5000")
                elif ev.key == pygame.K_c:
                    copy_log()
                elif ev.key == pygame.K_PAGEUP:
                    scroll[0] += 8
                elif ev.key == pygame.K_PAGEDOWN:
                    scroll[0] = max(0, scroll[0] - 8)
                elif ev.key == pygame.K_t:
                    step("thr", +10 if ev.mod & pygame.KMOD_SHIFT else -10, 16, 255)
                elif ev.key == pygame.K_e:
                    step("aec", +20 if ev.mod & pygame.KMOD_SHIFT else -20, 1, 1200)
                elif ev.key == pygame.K_g:
                    send(f"boost={0 if st.cfg.get('boost') else 1}")
                elif ev.key == pygame.K_m:
                    mirror_x[0] = not mirror_x[0]
                elif ev.key == pygame.K_q:
                    send("dash=2")          # overlay: start the quad stream

        if synced[0] == 0.0:
            synced[0] = time.monotonic() + 1.0      # let the port settle first
        elif synced[0] > 0 and time.monotonic() > synced[0]:
            send("sync=1")      # unknown key: echoes the live cfg, changes nothing
            synced[0] = -1.0
        with st.lock:
            fixes = list(st.fixes)
            quads = list(st.quads)
            # only the visible tail of the log is copied (was: all 4000 lines,
            # every rendered frame, while holding the lock)
            nlog = len(st.log)
            log_tail = list(st.log)[max(0, nlog - scroll[0] - 40):nlog - scroll[0]] if nlog else []
            dims, stat = st.dims, st.last_stat
            levels = st.levels
            cfg = dict(st.cfg)
            frame, fseq = st.frame, st.frame_seq
            last_rx = st.last_rx
            sw_max = [list(r) for r in st.sw_max]
            sw_hit = [list(r) for r in st.sw_hit]
            sw_info, sw_seq, sw_active = st.sw_info, st.sw_seq, st.sw_active
            # SNAP watchdog (): frame=1 answered by silence used to
            # look like a dead button. Name the likely causes instead.
            if (st.snap_req_t and st.frame_seq == st.snap_req_seq
                    and time.monotonic() - st.snap_req_t > 3.0):
                st.snap_req_t = 0.0
                st.log.append("[SNAP: NO ANSWER in 3s - the firmware never sent "
                              "an FRM header. Checks: (1) is the BLOBTEST "
                              "firmware flashed? harness/overlay ignore frame=1; "
                              "(2) is the map alive / fps>0 in STAT? at fps=0 "
                              "the camera is stalled and cannot snapshot; "
                              "(3) any 'SNAP refused: no memory' above? then "
                              "rb=1 to reboot]")
        link_dead = last_rx and (time.monotonic() - last_rx) > 1.5
        if fseq != seen_seq[0] and frame:
            cam_surf[0] = pygame.transform.scale(build_cam_surface(*frame), (MAPW, MAPH))
            seen_seq[0] = fseq
            view[0] = "cam"
        if sw_seq != seen_sw[0]:
            seen_sw[0] = sw_seq
            view[0] = "sweep"          # a finished sweep is the thing to look at
        if sw_active and time.monotonic() < sw_active:
            view[0] = "sweep"

        screen.fill((10, 10, 10))
        pygame.draw.rect(screen, (22, 22, 22), (MAPX, MAPY, MAPW, MAPH))
        if view[0] == "sweep":
            draw_sweep(screen, small, bfont, (MAPX, MAPY, MAPW, MAPH),
                       sw_max, sw_hit,
                       "running" if (sw_active and time.monotonic() < sw_active) else sw_info,
                       int(cfg.get("thr", 60)))
        cam_mode = view[0] == "cam" and cam_surf[0] is not None
        if cam_mode:
            screen.blit(cam_surf[0], (MAPX, MAPY))
            f_lo, f_hi = (frame[6], frame[7]) if len(frame) >= 8 else (min(frame[2]), max(frame[2]))
            hdr_ = ("contrast-stretched x%d" % (255 // max(1, f_hi - f_lo))
                    if stretch_on[0] else "raw pixels")
            screen.blit(bfont.render(
                f"SNAPSHOT thr={frame[3]} - {hdr_} - S toggles stretch - V returns to map",
                True, (250, 200, 120)), (MAPX + 6, MAPY + 4))
            if f_hi < 32:
                warn = bfont.render(
                    f"FRAME IS BLACK (brightest pixel {f_hi}/255) - this is amplified sensor "
                    f"noise, not an image", True, (255, 90, 90))
                screen.blit(warn, (MAPX + 6, MAPY + 24))
            # v52: mark where the DETECTOR said the blob was, in this frame
            if frame and len(frame) >= 6 and frame[4] is not None:
                fw_, fh_ = frame[0], frame[1]
                mx_ = MAPX + frame[4] * MAPW / fw_
                my_ = MAPY + frame[5] * MAPH / fh_
                pygame.draw.circle(screen, (120, 220, 255), (int(mx_), int(my_)), 16, 2)
                pygame.draw.line(screen, (120, 220, 255), (mx_ - 22, my_), (mx_ - 8, my_), 2)
                pygame.draw.line(screen, (120, 220, 255), (mx_ + 8, my_), (mx_ + 22, my_), 2)
                screen.blit(bfont.render(f"detector: {frame[4]:.1f},{frame[5]:.1f}", True, (120, 220, 255)),
                            (MAPX + 6, MAPY + MAPH - 20))
        elif view[0] == "aim":
            aim_stats[0] = draw_aim(screen, small, bfont, big,
                                    (MAPX, MAPY, MAPW, MAPH), quads, dims,
                                    mirror_x[0])
        elif view[0] != "sweep":
            for i in range(1, 4):
                pygame.draw.line(screen, (40, 40, 40), (MAPX + MAPW * i // 4, MAPY),
                                 (MAPX + MAPW * i // 4, MAPY + MAPH))
                pygame.draw.line(screen, (40, 40, 40), (MAPX, MAPY + MAPH * i // 4),
                                 (MAPX + MAPW, MAPY + MAPH * i // 4))
            # v15: with the quad stream running, show ALL FOUR points on the
            # camera map, not just point 0. Colour is by SLOT, so a slot
            # swapping identity is visible as a colour jumping across the map.
            if quads:
                qsx, qsy = MAPW / dims[0], MAPH / dims[1]
                for (qt, qn, qpts) in quads[-40:]:
                    qage = min(1.0, max(0.0, (quads[-1][0] - qt) / 800.0))
                    for qi, (qx, qy) in enumerate(qpts):
                        qc = [(250, 110, 110), (250, 210, 110),
                              (120, 230, 150), (140, 180, 255)][qi % 4]
                        qc = tuple(int(v * max(0.18, 1 - qage)) for v in qc)
                        px_, py_ = MAPX + qx * qsx - 2, MAPY + qy * qsy - 2
                        if MAPX <= px_ < MAPX + MAPW and MAPY <= py_ < MAPY + MAPH:
                            screen.fill(qc, (px_, py_, 4, 4))

        now_t = fixes[-1][0] if fixes else 0
        sx, sy = MAPW / dims[0], MAPH / dims[1]
        plot_fixes = not cam_mode and view[0] != "sweep"
        nblob, max_gap, prev_t, last = 0, 0, None, None
        for (t, n, x, y) in fixes:
            if n > 0:
                nblob += 1
                if prev_t is not None:
                    max_gap = max(max_gap, t - prev_t)
                prev_t = t
                if plot_fixes:
                    age = min(1.0, max(0.0, (now_t - t) / 5000.0))
                    c = max(0, min(255, int(220 * max(0.15, 1 - age))))
                    px_, py_ = MAPX + x * sx - 1, MAPY + y * sy - 1
                    if MAPX <= px_ < MAPX + MAPW and MAPY <= py_ < MAPY + MAPH:
                        screen.fill((40, c, 60), (px_, py_, 3, 3))
                last = (t, n, x, y)
        if last and plot_fixes:
            lx, ly = MAPX + last[2] * sx, MAPY + last[3] * sy
            pygame.draw.line(screen, GREEN, (lx - 12, ly), (lx + 12, ly), 2)
            pygame.draw.line(screen, GREEN, (lx, ly - 12), (lx, ly + 12), 2)

        # include the gap from the last blob to NOW: without it a target that
        # vanished 4s ago still scored SOLID off its earlier fixes.
        if prev_t is not None and now_t > prev_t:
            max_gap = max(max_gap, now_t - prev_t)
        if link_dead:
            verdict, vc = "NO DATA (link)", RED
        elif nblob == 0:
            verdict, vc = "NO TARGET", GRAY
        elif nblob < 2:
            verdict, vc = "SINGLE HIT", ORANGE      # one fix in 5s is not a lock
        elif max_gap < 100:
            verdict, vc = "SOLID", GREEN
        elif max_gap < 500:
            verdict, vc = "INTERMITTENT", ORANGE
        else:
            verdict, vc = "LOSING TARGET", RED
        screen.blit(big.render(verdict, True, vc), (PX, 16))

        fl_m = re.search(r"flood=(\d+)%", stat)
        if fl_m and int(fl_m.group(1)) > 0:
            screen.blit(bfont.render(f"FLOOD {fl_m.group(1)}% - raise thr / lower aec", True, RED), (PX, 44))
        fps_m = re.search(r"STAT,\d+,(\d+\.?\d*)", stat)
        try:
            fps = float(fps_m.group(1)) if fps_m else 0.0
        except ValueError:
            fps = 0.0
        bpp = 1 if cfg.get("y8") else 2
        rate = dims[0] * dims[1] * bpp * fps / 1e6          # MB/s on the wire
        lines = [
            f"cam fps  : {fps:.1f}",
            f"data rate: {rate:.1f} MB/s  (avg pclk >= {rate:.1f}MHz)",
            f"fix rate : {nblob/5.0:.0f}/s (5s)",
            f"worst gap: {max_gap} ms" if max_gap else "worst gap: -",
            (lambda m: f"frame drift: {m.group(1)}us (~{m.group(2)} lines)" if m else "frame drift: -")(
                re.search(r"restart=(\d+)us\(~(-?\d+)lines\)", stat)),
            (lambda m: f"short frames: {m.group(1)}%" if m else "short frames: -")(
                re.search(r"short=(\d+)%", stat)),
            (lambda m: f"sensor period: {m.group(1)}us" if m else "sensor period: -")(
                re.search(r"vsync=(\d+)us", stat)),
            (lambda m: f"period spread: {m.group(1)}us (~{m.group(2)} lines)" if m else "period spread: -")(
                re.search(r"spread(\d+)us~(-?\d+)lines", stat)),
            # v66: which layer is losing frames. Exactly one of these should be
            # nonzero, and each points somewhere different:
            #   ovf   -> we could not queue the ISR event (cam_task too slow)
            #   long  -> the VSYNC pulse never arrived (sensor skipped it)
            #   nost  -> VSYNC fine, no free frame buffer (app too slow)
            (lambda m: (f"lost: ovf {m.group(1)}/{m.group(2)}  skip {m.group(3)}  "
                        f"nofb {m.group(5)} /s")
                       if m else ("lost: none" if "lost=0" in stat else "lost: - (pre-v66 fw)"))(
                re.search(r"lost=ovf(\d+)/(\d+),skip(\d+),short(\d+),nofb(\d+)", stat)),
            (lambda m: ("  -> " + ("EVENT QUEUE OVERFLOW" if int(m.group(1)) + int(m.group(2))
                                   else "SENSOR SKIPPED VSYNC" if int(m.group(3))
                                   else "NO FREE FRAMEBUFFER" if int(m.group(5))
                                   else "spurious VSYNC edges")) if m else "")(
                re.search(r"lost=ovf(\d+)/(\d+),skip(\d+),short(\d+),nofb(\d+)", stat)),
            f"last fix : {last[2]:.1f},{last[3]:.1f}" if last else "last fix : -",
            f"frame    : {dims[0]}x{dims[1]}",
            "",
            f"thr {cfg.get('thr','?')}   aec {cfg.get('aec','?')}",
            f"agc {cfg.get('agc','?')}   boost {'ON' if cfg.get('boost') else 'off'}",
            f"xclk {cfg.get('xclk','?')} req  y8 {cfg.get('y8','?')}  pdiv {cfg.get('pdiv','?')}",
            # v71 servo: agcl= only exists in STAT while live gain differs from
            # the setting; sat= only while a blob is actually clipping.
            ("UNCLIP: live agc " + str(cfg["agcl"]) + f" (set {cfg.get('agc','?')})"
             if "agcl" in cfg else
             ("unclip " + ("on" if cfg.get("unclip", 1) else "OFF"))),
            (f"CLIP: {cfg['sat']} px flat at 255" if cfg.get("sat") else ""),
            "",
            (("light: " + re.sub(r"^LEVELS,", "", levels)[:34]) if levels else "light: press LEVELS / l"),
        ]
        # Clip to the space above the button rows. The old code emitted a fixed
        # 24 lines and the tail was drawn UNDERNEATH the buttons, so the last
        # readings were invisible depending on window size.
        for i, ln in enumerate(lines[: max(0, (y1 - 58) // 19)]):
            screen.blit(font.render(ln, True, (200, 200, 200)), (PX, 58 + i * 19))

        mouse = pygame.mouse.get_pos()
        for b in buttons:
            b.draw(screen, bfont, mouse)
        screen.blit(small.render(
            "keys:  1 STOCK100   2 120rb   3 FULL136   f SNAP   w SWEEP   r REGS   l LEVELS   "
            "v VIEW(map/aim/cam)   q QUAD-STREAM   m MIRROR   c COPY   Enter = k=v cmd",
            True, (110, 130, 150)), (14, ykeys))

        # ---- scrollable log ----
        pygame.draw.rect(screen, (16, 16, 16), (10, LOGY, W - 20, H - LOGY - 10))
        nvis = (H - LOGY - 26) // 17
        scroll[0] = min(scroll[0], max(0, nlog - nvis))
        shown = log_tail[-nvis:] if log_tail else []
        hdr = f"LOG -> {logname}   ({nlog} lines" + (f", scrolled +{scroll[0]}" if scroll[0] else ", live") + ")"
        if link_dead:
            hdr += "   [NO SERIAL DATA]"
        if copied[0] and time.time() - copied[0] < 3:
            hdr += "   [COPIED TO CLIPBOARD]"
        screen.blit(small.render(hdr, True, (120, 170, 210) if not scroll[0] else (240, 190, 90)), (14, LOGY + 4))
        for i, ln in enumerate(shown):
            col = (150, 150, 150)
            if ln.startswith(">"):
                col = (150, 200, 150)
            elif "MISMATCH" in ln or "not applied" in ln or "OVER OV2640" in ln:
                col = (250, 120, 120)
            elif ln.startswith(("CLK-ACT", "---", "XCLK", "sysclk", "  ")):
                col = (230, 200, 130)
            elif ln.startswith("STAT,"):
                col = (110, 160, 200)
            # v73: STAT lines are wider than the window, and the field that got
            # clipped was lost= -- the one that names which layer dropped a
            # frame. thr/aec/agc/boost are already on the panel, so drop that
            # block from the LOG copy only and the tail fits.
            if ln.startswith("STAT,"):
                ln = re.sub(r"thr=\d+ aec=\d+ agc=\d+ boost=\d+ ", "", ln)
                # v74: still overflowed on the jittery lines -- and the field
                # that fell off the edge was lost=, the only one that says WHICH
                # layer dropped a frame. Drop the zero-valued indicators too;
                # they are noise when zero and stay visible when they matter.
                ln = ln.replace("flood=0% ", "").replace("short=0% ", "")
                ln = ln.replace("(~0lines)", "")
                # the OVERLAY's STAT also carries y8/xclk/pdiv (the
                # lab's does not, so this is a no-op there). They are constants
                # on the panel already; dropping them keeps lost= and mode= on
                # screen instead of clipping the fields that say what broke.
                ln = re.sub(r"y8=\d+ xclk=\d+ pdiv=\d+ ", "", ln)
            screen.blit(small.render(ln[:150], True, col), (14, LOGY + 22 + i * 17))

        if typing:
            pygame.draw.rect(screen, (30, 30, 60), (10, H - 24, W - 20, 20))
            screen.blit(font.render("> " + typed, True, (240, 240, 240)), (14, H - 22))

        pygame.display.flip()
        clock.tick(60)


if __name__ == "__main__":
    main()
