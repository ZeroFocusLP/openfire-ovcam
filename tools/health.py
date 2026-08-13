#!/usr/bin/env python3
"""
health.py -- dedicated PASS/FAIL + quality window for the openfire-overlay
             camera pipeline.

WHY THIS IS A SEPARATE PROGRAM, not a patch to dashboard.py:
  * dashboard.py is a terminal app and it already flushes its log after every
    line (tools/dashboard.py:349). Tailing that log is therefore lossless and
    needs no changes to a tool that works.
  * two processes cannot share one COM port. Tailing means you can run this
    window and the dashboard at the same time.

USAGE
  # normal case -- dashboard.py is running in another window, writing its log
  python tools/health.py

  # a specific log (also works on an OLD log, to score a past session)
  python tools/health.py --log dashboard_20260811_105831.log

  # no dashboard running: read the port directly instead
  python tools/health.py --serial COM8

  # score a finished log and print a verdict, no window (for CI / pasting)
  python tools/health.py --log dashboard_...log --once

THRESHOLDS come from the v16-v20 investigation. Every one of them is a number
we measured, not a guess -- see THRESHOLD NOTES at the bottom.
"""

import argparse, glob, os, re, sys, time
from collections import deque

# ---------------------------------------------------------------- parsing ---

PASS, WARN, FAIL, UNKNOWN = "pass", "warn", "fail", "unknown"

RE = {
    "MEM":    re.compile(r"^MEM: psram total=(\d+)K .*?\| (PSRAM OK|!! PSRAM DEAD)"),
    "RING":   re.compile(r"^RING: evq=(\d+) dma_halfs=(\d+) lines/half=(\d+)"),
    "PTS":    re.compile(r"^PTS/s: pub0=(\d+) pub1=(\d+) pub2=(\d+) pub3=(\d+) pub4=(\d+)"
                         r" \| short=(\d+)/(\d+) \((\d+)%\) churn=(\d+)/s"),
    "CHUNKS": re.compile(r"^CHUNKS/s .*?\| total=(\d+) short=(\d+) long=(\d+)"
                         r"(?: \| drained=(\d+)/s)?(?: \| coin=(\w+) killed=(\d+)/s)?"),
    "ACCT":   re.compile(r"^ACCT/s: sensor=(\d+) delivered=(\d+) rejected=(\d+) published=(\d+)"),
    "ISR":    re.compile(r"^ISR/s: vsync=(\d+) eof=(\d+) \(expect eof=11xvsync=(\d+)\)"
                         r" \| VSI avg=(\d+)us min=(\d+) max=(\d+) spread=(\d+)us"),
    "JUMP":   re.compile(r"^JUMP/s: moved=(\d+) onGrid16=(\d+) worst=([\d.]+) lines"
                         r" over4=(\d+) maxBlobs=(\d+)"),
    "LAPS":   re.compile(r"^LAPS/s: ring=(\d+) \(chunkrej\) stitch=(\d+) appRej=(\d+)"
                         r" sensorSkip=(\d+)"),
    "RES":    re.compile(r"^RES/s: affine=(\d+) sim=(\d+) coast=(\d+) reassoc=(\d+)"
                         r" junk=(\d+) envrej=(\d+) reseed=(\d+) lost=(\d+) conf=([\d.]+)"
                         r" \| tilt=([\d.]+) learned_max=([\d.]+)"
                         r" \| perspective resid=([\d.]+)px \(worst ([\d.]+)\)"),
    "STAT":   re.compile(r"^STAT,\d+,([\d.]+),"),
    "CFG":    re.compile(r"thr=(\d+) aec=(\d+) agc=(\d+) boost=(\d+)"),
}

# Sensor VSYNC skips double one measured period (~7400 -> ~14800). A window
# containing one is NOT an ISR-latency fault, so spreads at or above this are
# attributed to the sensor and excluded from the jitter check. Corroborated by
# LAPS sensorSkip.
VSI_SKIP_FLOOR_US = 6000


class Health:
    """Pure parse + score. No GUI, so it is unit-testable."""

    def __init__(self):
        self.d = {}                       # latest value per key
        self.pts_hist = deque(maxlen=30)  # (pub4, total, churn) for trend
        self.last_update = 0.0
        self.lines = 0

    # ---- ingest ----
    def feed(self, line):
        line = line.rstrip("\r\n")
        for key, rx in RE.items():
            m = rx.match(line) if key != "CFG" else None
            if m:
                self.d[key] = m.groups()
                self.last_update = time.time()
                self.lines += 1
                if key == "PTS":
                    g = m.groups()
                    self.pts_hist.append((int(g[4]), int(g[6]), int(g[8])))
                break
        m = RE["CFG"].search(line)
        if m:
            self.d["CFG"] = m.groups()

    # ---- helpers ----
    @staticmethod
    def _grade(value, ok, warn, higher_is_better=False):
        if value is None:
            return UNKNOWN
        if higher_is_better:
            return PASS if value >= ok else (WARN if value >= warn else FAIL)
        return PASS if value <= ok else (WARN if value <= warn else FAIL)

    # ---- checks: (label, shown value, state, target text, weight, critical) ----
    def checks(self):
        d, out = self.d, []

        # 1. PSRAM
        if "MEM" in d:
            tot, verdict = int(d["MEM"][0]), d["MEM"][1]
            st = PASS if (verdict == "PSRAM OK" and tot >= 8000) else FAIL
            out.append(("PSRAM", f"{tot}K {verdict}", st, "8192K, PSRAM OK", 1, True))
        else:
            out.append(("PSRAM", "-", UNKNOWN, "8192K, PSRAM OK", 1, True))

        # 2. DMA ring geometry
        if "RING" in d:
            evq, halfs, lines = (int(x) for x in d["RING"][:3])
            st = PASS if (halfs == 4 and lines == 16 and evq == 3) else WARN
            out.append(("DMA ring", f"halfs={halfs} lines={lines} evq={evq}", st,
                        "halfs=4 lines=16 evq=3", 1, False))
        else:
            out.append(("DMA ring", "-", UNKNOWN, "halfs=4 lines=16 evq=3", 1, False))

        # 3. Frame assembly -- the v19 race fix
        if "CHUNKS" in d:
            g = d["CHUNKS"]
            tot, short, lng = int(g[0]), int(g[1]), int(g[2])
            drained = g[3]
            bad = short + lng
            st = self._grade(bad, 0, 2)
            extra = f" (drained {drained}/s)" if drained else ""
            out.append(("Frame assembly", f"short={short} long={lng}{extra}", st,
                        "short+long = 0", 3, True))
        else:
            out.append(("Frame assembly", "-", UNKNOWN, "short+long = 0", 3, True))

        # 4. Driver rejections
        if "ACCT" in d:
            sen, dlv, rej, pub = (int(x) for x in d["ACCT"][:4])
            st = self._grade(rej, 2, 10)
            out.append(("Frames rejected", f"{rej}/s of {sen}", st, "<= 2/s", 2, True))
            out.append(("Sensor rate", f"{sen} fps", self._grade(sen, 130, 120, True),
                        ">= 130 fps", 1, False))
        else:
            out.append(("Frames rejected", "-", UNKNOWN, "<= 2/s", 2, True))
            out.append(("Sensor rate", "-", UNKNOWN, ">= 130 fps", 1, False))

        # 5. EOF integrity (no coalescing / no loss)
        skips = int(d["LAPS"][3]) if "LAPS" in d else 0
        if "ISR" in d:
            vs, eof, exp, avg, vmin, vmax, spread = (int(x) for x in d["ISR"][:7])
            err = abs(eof - exp) / exp * 100 if exp else None
            out.append(("EOF integrity", f"{eof} vs {exp} expected",
                        self._grade(err, 1.0, 3.0), "within 1% of 11 x vsync", 2, True))
            if spread >= VSI_SKIP_FLOOR_US:
                out.append(("VSYNC ISR jitter", f"masked by {skips} sensor skip(s)",
                            UNKNOWN, "< 100us (skip-free window)", 2, False))
            else:
                out.append(("VSYNC ISR jitter", f"spread {spread}us",
                            self._grade(spread, 100, 500), "< 100us", 2, True))
        else:
            out.append(("EOF integrity", "-", UNKNOWN, "within 1% of 11 x vsync", 2, True))
            out.append(("VSYNC ISR jitter", "-", UNKNOWN, "< 100us", 2, True))

        # 6. THE CURRENT BLOCKER: 4-point yield + corner churn
        if "PTS" in d:
            g = d["PTS"]
            pub4, bad, tot, pct_short, churn = int(g[4]), int(g[5]), int(g[6]), int(g[7]), int(g[8])
            yield_pct = (pub4 * 100.0 / tot) if tot else None
            out.append(("4-point yield", f"{pub4}/{tot} = {yield_pct:.0f}%" if tot else "-",
                        self._grade(yield_pct, 95, 80, True), ">= 95%", 5, True))
            out.append(("Corner churn", f"{churn}/s", self._grade(churn, 5, 20),
                        "<= 5/s", 4, True))
            out.append(("Invented corners", f"{pct_short}% of frames",
                        self._grade(pct_short, 5, 20), "<= 5%", 2, False))
        else:
            for lbl, tgt, w, crit in (("4-point yield", ">= 95%", 5, True),
                                      ("Corner churn", "<= 5/s", 4, True),
                                      ("Invented corners", "<= 5%", 2, False)):
                out.append((lbl, "-", UNKNOWN, tgt, w, crit))

        # 6b. QUAD RESOLVER — identity, reconstruction, and the tilt envelope.
        # These read the stream AFTER correction, i.e. exactly what OpenFIRE is
        # handed, which is the same buffer the dashboard map draws from.
        if "RES" in d:
            g = d["RES"]
            aff, sim, coast = int(g[0]), int(g[1]), int(g[2])
            envrej, reseed, lost = int(g[5]), int(g[6]), int(g[7])
            conf = float(g[8]); tilt = float(g[9]); envmax = float(g[10])
            resid, residmax = float(g[11]), float(g[12])
            out.append(("Corner identity", f"{lost} lost, {reseed} re-seeds",
                        self._grade(lost, 0, 1), "0 lost/s", 4, True))
            out.append(("Corners rebuilt", f"{aff} affine + {sim} sim /s",
                        self._grade(aff + sim, 15, 60),
                        "low = detection healthy", 2, False))
            out.append(("Blind coasting", f"{coast}/s", self._grade(coast, 0, 5),
                        "0/s (model always fits)", 3, True))
            out.append(("Tilt envelope", f"now {tilt:.2f}, learned {envmax:.2f}",
                        WARN if envrej > 10 else PASS,
                        f"{envrej} rejects/s", 1, False))
            out.append(("Perspective error", f"{resid:.2f}px (worst {residmax:.2f})",
                        self._grade(residmax, 1.0, 3.0),
                        "< 1px, else affine too weak", 3, False))
            out.append(("Resolver confidence", f"{conf:.2f}",
                        self._grade(conf, 0.85, 0.6, higher_is_better=True),
                        ">= 0.85", 2, False))
        else:
            for lbl, tgt, w, crit in (("Corner identity", "0 lost/s", 4, True),
                                      ("Blind coasting", "0/s (model always fits)", 3, True),
                                      ("Perspective error", "< 1px", 3, False)):
                out.append((lbl, "-", UNKNOWN, tgt, w, crit))

        # 7. Residual splice reaching the output
        if "JUMP" in d:
            moved, grid, worst, over4, maxb = (float(x) for x in d["JUMP"][:5])
            out.append(("Splice on 16-row grid", f"{int(grid)}/s (worst {worst:.0f} rows)",
                        self._grade(grid, 0, 3), "0/s while still", 2, False))
            out.append(("Blobs seen", f"max {int(maxb)}",
                        PASS if maxb == 4 else (WARN if maxb in (3, 5) else FAIL),
                        "exactly 4", 1, False))
        else:
            out.append(("Splice on 16-row grid", "-", UNKNOWN, "0/s while still", 2, False))
            out.append(("Blobs seen", "-", UNKNOWN, "exactly 4", 1, False))

        return out

    def score(self):
        """Weighted 0-100 over known checks. UNKNOWN is skipped, not penalised."""
        num = den = 0.0
        for _lbl, _v, st, _t, w, _c in self.checks():
            if st == UNKNOWN:
                continue
            den += w
            num += w * (1.0 if st == PASS else 0.5 if st == WARN else 0.0)
        return (num / den * 100.0) if den else None

    def verdict(self):
        cs = self.checks()
        known = [c for c in cs if c[2] != UNKNOWN]
        if not known:
            return UNKNOWN, "waiting for telemetry"
        crit_fail = [c[0] for c in cs if c[5] and c[2] == FAIL]
        if crit_fail:
            return FAIL, "FAIL: " + ", ".join(crit_fail)
        crit_warn = [c[0] for c in cs if c[5] and c[2] == WARN]
        if crit_warn:
            return WARN, "MARGINAL: " + ", ".join(crit_warn)
        if any(c[2] == FAIL for c in cs):
            return WARN, "PASS with non-critical faults"
        return PASS, "PASS"

    def hint(self):
        """One actionable next step, chosen from what is actually failing."""
        cs = {c[0]: c[2] for c in self.checks()}
        if cs.get("PSRAM") == FAIL:
            return "board_build.arduino.memory_type = qio_opi -- PSRAM is dead"
        if cs.get("Frame assembly") == FAIL or cs.get("Frames rejected") == FAIL:
            return "frame assembly broken: check CONFIG_LCD_CAM_ISR_IRAM_SAFE=1 is set"
        if cs.get("VSYNC ISR jitter") == FAIL:
            return "ISR latency high -- something on core 1 is masking interrupts"
        if cs.get("4-point yield") in (FAIL, WARN):
            return ("detection is starving the pipeline: lower thr (110 -> 90 -> 70), "
                    "then agc 4-8, and re-aim all 4 LEDs at the play position")
        if cs.get("Corner identity") == FAIL:
            return ("the resolver is losing correspondence -- corners drop for >90ms. "
                    "Check detection first (thr), then widen quad gate if you pan fast")
        if cs.get("Blind coasting") == FAIL:
            return ("extrapolating with no model fit: fewer than 2 corners visible. "
                    "This is a detection problem, not a resolver one")
        if cs.get("Perspective error") == FAIL:
            return ("affine (parallelogram) no longer explains your play angle -- "
                    "true perspective is showing. Tell Claude: needs a homography")
        if cs.get("Corner churn") in (FAIL, WARN):
            return "count is flapping: check the resolver is on (res=1)"
        if cs.get("Splice on 16-row grid") == FAIL:
            return "spliced frames still reaching output -- recheck drained/s"
        return "all green -- go shoot something"


# ---------------------------------------------------------------- sources ---

class LogTail:
    """Follow a log file, switching to a newer dashboard_*.log if one appears."""

    def __init__(self, path=None, auto=True):
        self.explicit = path
        self.auto = auto and path is None
        self.path, self.fh = None, None
        self._open(path or self._newest())

    @staticmethod
    def _newest():
        c = sorted(glob.glob("dashboard_*.log") + glob.glob("*/dashboard_*.log"),
                   key=lambda p: os.path.getmtime(p) if os.path.exists(p) else 0)
        return c[-1] if c else None

    def _open(self, path, from_end=True):
        if not path or not os.path.exists(path):
            return
        try:
            fh = open(path, "r", encoding="utf-8", errors="replace")
        except OSError:
            return
        if from_end:
            fh.seek(0, os.SEEK_END)
        if self.fh:
            self.fh.close()
        self.path, self.fh = path, fh

    def read(self):
        if self.auto:
            n = self._newest()
            if n and n != self.path:
                self._open(n, from_end=False)
        if not self.fh:
            self._open(self._newest())
            return []
        return self.fh.readlines()

    def label(self):
        return f"tailing {os.path.basename(self.path)}" if self.path else "no dashboard_*.log found"


class SerialSource:
    def __init__(self, port, baud=115200):
        import serial                      # lazy: only needed in this mode
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.buf = ""
        self.port = port

    def read(self):
        try:
            self.buf += self.ser.read(4096).decode("utf-8", "replace")
        except Exception:
            return []
        *lines, self.buf = self.buf.split("\n")
        return lines

    def label(self):
        return f"reading {self.port}"


class FileOnce:
    def __init__(self, path):
        self.lines = open(path, "r", encoding="utf-8", errors="replace").readlines()
        self.path = path

    def read(self):
        out, self.lines = self.lines, []
        return out

    def label(self):
        return f"scored {os.path.basename(self.path)}"


# -------------------------------------------------------------------- GUI ---

COLOR = {PASS: "#1f9d55", WARN: "#c98a00", FAIL: "#cc2936", UNKNOWN: "#5a6472"}
BG, FG, DIM, CARD = "#12161c", "#e8ecf1", "#8a93a0", "#1b212a"


def run_gui(src, health, stale_after=4.0):
    import tkinter as tk
    from tkinter import font as tkfont

    root = tk.Tk()
    root.title("openfire-overlay - pipeline health")
    root.configure(bg=BG)
    root.geometry("800x660")
    root.minsize(760, 600)

    f_big = tkfont.Font(family="Segoe UI", size=34, weight="bold")
    f_mid = tkfont.Font(family="Segoe UI", size=12)
    f_row = tkfont.Font(family="Consolas", size=10)
    f_sm  = tkfont.Font(family="Segoe UI", size=9)

    banner = tk.Label(root, text="WAITING", font=f_big, bg=COLOR[UNKNOWN],
                      fg="white", pady=14)
    banner.pack(fill="x")
    reason = tk.Label(root, text="", font=f_mid, bg=BG, fg=FG, pady=4)
    reason.pack(fill="x")

    qwrap = tk.Frame(root, bg=BG); qwrap.pack(fill="x", padx=14, pady=(4, 2))
    tk.Label(qwrap, text="QUALITY", font=f_sm, bg=BG, fg=DIM).pack(anchor="w")
    bar = tk.Canvas(qwrap, height=26, bg=CARD, highlightthickness=0)
    bar.pack(fill="x")
    qtxt = tk.Label(qwrap, text="--", font=f_mid, bg=BG, fg=FG)
    qtxt.pack(anchor="e")

    rows_wrap = tk.Frame(root, bg=BG); rows_wrap.pack(fill="both", expand=True,
                                                      padx=14, pady=6)
    row_widgets = []

    hint = tk.Label(root, text="", font=f_sm, bg=CARD, fg=FG, wraplength=760,
                    justify="left", anchor="w", padx=10, pady=8)
    hint.pack(fill="x", padx=14, pady=(0, 4))
    foot = tk.Label(root, text="", font=f_sm, bg=BG, fg=DIM, anchor="w")
    foot.pack(fill="x", padx=14, pady=(0, 8))

    def ensure_rows(n):
        while len(row_widgets) < n:
            fr = tk.Frame(rows_wrap, bg=BG)
            fr.pack(fill="x", pady=1)
            dot = tk.Canvas(fr, width=14, height=14, bg=BG, highlightthickness=0)
            dot.pack(side="left", padx=(0, 8))
            oid = dot.create_oval(2, 2, 12, 12, fill=COLOR[UNKNOWN], outline="")
            lbl = tk.Label(fr, text="", font=f_row, bg=BG, fg=FG, width=25, anchor="w")
            lbl.pack(side="left")
            val = tk.Label(fr, text="", font=f_row, bg=BG, fg=FG, width=31, anchor="w")
            val.pack(side="left")
            tgt = tk.Label(fr, text="", font=f_row, bg=BG, fg=DIM, anchor="w")
            tgt.pack(side="left")
            row_widgets.append((dot, oid, lbl, val, tgt))

    def tick():
        for line in src.read():
            health.feed(line)

        cs = health.checks()
        ensure_rows(len(cs))
        for (dot, oid, lbl, val, tgt), c in zip(row_widgets, cs):
            label, value, state, target, _w, crit = c
            dot.itemconfig(oid, fill=COLOR[state])
            lbl.config(text=("* " if crit else "  ") + label)
            val.config(text=value, fg=COLOR[state] if state != UNKNOWN else DIM)
            tgt.config(text=target)

        stale = health.last_update and (time.time() - health.last_update) > stale_after
        state, why = health.verdict()
        if stale:
            banner.config(text="NO DATA", bg=COLOR[UNKNOWN])
            reason.config(text="telemetry stopped - is dashboard.py still running?")
        else:
            banner.config(text={PASS: "PASS", WARN: "MARGINAL",
                                FAIL: "FAIL", UNKNOWN: "WAITING"}[state],
                          bg=COLOR[state])
            reason.config(text=why)

        s = health.score()
        bar.delete("fill")
        w = bar.winfo_width() or 1
        if s is not None:
            col = COLOR[PASS] if s >= 90 else COLOR[WARN] if s >= 60 else COLOR[FAIL]
            bar.create_rectangle(0, 0, int(w * s / 100.0), 26, fill=col,
                                 outline="", tags="fill")
            qtxt.config(text=f"{s:.0f} / 100")
        cfg = health.d.get("CFG")
        foot.config(text=f"{src.label()}   |   {health.lines} telemetry lines" +
                         (f"   |   thr={cfg[0]} aec={cfg[1]} agc={cfg[2]} boost={cfg[3]}"
                          if cfg else ""))
        hint.config(text="NEXT: " + health.hint())
        root.after(250, tick)

    tick()
    root.mainloop()


def run_once(src, health):
    for line in src.read():
        health.feed(line)
    state, why = health.verdict()
    s = health.score()
    print(f"\n=== {src.label()} ===")
    print(f"VERDICT: {why}      QUALITY: {s:.0f}/100\n" if s is not None
          else f"VERDICT: {why}\n")
    print(f"{'':2}{'check':<24}{'value':<30}{'target':<28}state")
    for label, value, state_, target, _w, crit in health.checks():
        print(f"{'* ' if crit else '  '}{label:<24}{value:<30}{target:<28}{state_.upper()}")
    print(f"\nNEXT: {health.hint()}")
    print("\n(* = critical: a FAIL here fails the whole run)")
    return 0 if health.verdict()[0] == PASS else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", help="log file to follow (default: newest dashboard_*.log)")
    ap.add_argument("--serial", help="read this COM port directly instead of a log")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--once", action="store_true",
                    help="score the file and print a verdict, no window")
    a = ap.parse_args()

    health = Health()
    if a.serial:
        src = SerialSource(a.serial, a.baud)
    elif a.once:
        if not a.log:
            sys.exit("--once needs --log")
        src = FileOnce(a.log)
    else:
        src = LogTail(a.log)
        if not src.path:
            sys.exit("no dashboard_*.log found here. Start dashboard.py first, "
                     "or pass --log / --serial.")

    if a.once:
        sys.exit(run_once(src, health))
    try:
        run_gui(src, health)
    except ImportError:
        sys.exit("tkinter is not available in this Python. Use --once, or install "
                 "the tk package for your Python build.")


# ------------------------------------------------------- THRESHOLD NOTES ----
# Every number above is measured, not assumed:
#
#  PSRAM 8192K      the board is N8R8 (octal). Before v16 the build declared
#                   N8R2 and PSRAM failed to init on every boot.
#  halfs=4 lines=16 read off the fixed RING line in v16; the lab computes the
#  evq=3            same geometry from identical code, so a mismatch means
#                   CONFIG_CAMERA_DMA_BUFFER_SIZE_MAX differs between builds.
#  short+long = 0   after the v19 EOF/VSYNC race fix this is achievable and was
#                   achieved (drained 9-11/s, chunkrej 0, stitch 0).
#  rejected <= 2/s  the clean harness measured 0-4/s; the broken combined build
#                   measured 60/s.
#  eof within 1%    eof_isr_total == 11 x vsync_isr_total exactly when healthy;
#                   a shortfall means EOF interrupts are coalescing.
#  VSI < 100us      measured 8-13us healthy. The old task-context vsync= field
#                   shows ~690us even when perfectly clean -- it is scheduling
#                   jitter, NOT a fault. Do not use it. Spreads >= 6000us mean a
#                   real sensor VSYNC skip is in the window, so the check is
#                   suspended rather than failed.
#  4-point yield    OpenFIRE invents the missing corners for any frame with
#  >= 95%           count<4 (parallelogram at 3 points, aspect-ratio synthesis
#                   at 2) so short frames are not "slightly worse", they are
#                   fabricated geometry.
#  churn <= 5/s     each change in count fires OpenFIRE's kinematic spring,
#                   which bleeds the resulting offset off at 1 unit/frame --
#                   about 70 frames of held error per event.
#  maxBlobs == 4    5 or 6 means junk is clearing the threshold; 3 or fewer
#                   means an emitter is not being seen at all.

if __name__ == "__main__":
    main()
