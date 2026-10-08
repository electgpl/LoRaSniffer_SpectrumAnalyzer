"""
LoRa Spectrum Analyzer - desktop dashboard (Electgpl)

Live viewer for the SX1262 RSSI sweep produced by LoRaSA_ESP32S3_SX1262.ino:
live trace, max-hold, linear-power average, waterfall, peak marker, cursor
readout, LoRaWAN channel grids and CSV export.

One JSON object per line over serial, a reader thread feeding a queue, and
plain Tkinter (no matplotlib, no numpy).

    pip install pyserial
    pip install Pillow          (optional, logo)

Runs without command-line arguments (e.g. from Thonny): edit the
USER CONFIGURATION block below.
"""

import json
import math
import os
import queue
import statistics
import threading
import time
import tkinter as tk
from collections import deque

import serial
from serial.tools import list_ports

try:
    from PIL import Image, ImageTk
    _HAS_PIL = True
except ImportError:
    _HAS_PIL = False

# ============================================================
#  USER CONFIGURATION
# ============================================================
PORT = "COM3"             # None = auto-detect by device id
BAUD = 921600             # ignored on native USB CDC; used with a USB-UART bridge
DEVICE_ID = "electgpl-lora-sa"

# Sweep sent to the ESP32 on connect
CFG_F0_MHZ = 902.0
CFG_F1_MHZ = 928.0
CFG_STEP_KHZ = 50.0
CFG_RBW_KHZ = 117.3
CFG_N_SAMPLES = 4

# Display
REF_DBM = -10.0           # top of the plot
RANGE_DB = 100.0          # vertical span
CAL_OFFSET_DB = 0.0       # absolute correction, measured against a known source
AVERAGE_N = 8             # exponential average constant, in sweeps
WATERFALL_HIST = 600      # sweeps kept to rebuild the waterfall
WATERFALL_PX = 2          # waterfall row height per sweep, in pixels
SPI_CLOCK_MHZ = 8.0       # must match SPI_RAW_HZ in the firmware (spur markers)

CSV_FOLDER = os.path.dirname(os.path.abspath(__file__))
LOGO_PATH = os.path.join(CSV_FOLDER, "electgpl_logo.png")
LOGO_HEIGHT_PX = 30

RBW_KHZ_VALID = [4.8, 5.8, 7.3, 9.7, 11.7, 14.6, 19.5, 23.4, 29.3, 39.0,
                 46.9, 58.6, 78.2, 93.8, 117.3, 156.2, 187.2, 234.3,
                 312.0, 373.6, 467.0]
MAX_PTS = 2048

PRESETS = [
    ("902-928", 902.0, 928.0),
    ("915-928", 915.0, 928.0),
    ("863-870", 863.0, 870.0),
    ("850-960", 850.0, 960.0),
]

# ============================================================
#  Palette
# ============================================================
C_BG      = "#111217"
C_PANEL   = "#181b1f"
C_BORDER  = "#2c3235"
C_TXT     = "#ccccdc"
C_TXT2    = "#8e8e9a"
C_GRID    = "#23262b"
C_GRID2   = "#3a4048"
C_GREEN   = "#73bf69"
C_YELLOW  = "#fade2a"
C_ORANGE  = "#ff9830"
C_RED     = "#f2495c"
C_BLUE    = "#5794f2"
C_GRAY    = "#4a4a55"
C_PURPLE  = "#c792ea"
C_CYAN    = "#4dd0e1"

F_TITLE   = ("Segoe UI", 10)
F_MED     = ("Segoe UI", 24)
F_SUB     = ("Segoe UI", 10)
F_LBL     = ("Segoe UI", 9)
F_MONO    = ("Consolas", 10)
F_AXIS    = ("Consolas", 8)

# Plot margins, shared by spectrum and waterfall so the frequency axes align
ML, MR, MT, MB = 58, 14, 10, 24


# ============================================================
#  Channel grids (LoRaWAN Regional Parameters RP002-1.0.x)
# ============================================================
def _grid_us915():
    g = [(902.3 + 0.2 * n, C_GRID2) for n in range(64)]     # UL 125 kHz
    g += [(903.0 + 1.6 * n, C_PURPLE) for n in range(8)]   # UL 500 kHz
    g += [(923.3 + 0.6 * n, C_CYAN) for n in range(8)]     # DL 500 kHz
    return g


def _grid_au915():
    g = [(915.2 + 0.2 * n, C_GRID2) for n in range(64)]
    g += [(915.9 + 1.6 * n, C_PURPLE) for n in range(8)]
    g += [(923.3 + 0.6 * n, C_CYAN) for n in range(8)]
    return g


def _grid_spi_spurs():
    """Harmonics of the firmware SPI clock (self-generated spurs)."""
    return [(n * SPI_CLOCK_MHZ, C_ORANGE)
            for n in range(int(150 / SPI_CLOCK_MHZ), int(960 / SPI_CLOCK_MHZ) + 1)]


GRIDS = {
    "none": [],
    "US915": _grid_us915(),
    "AU915": _grid_au915(),
    "SPI spurs": _grid_spi_spurs(),
    "AlarmComm 915.0": [(915.0, C_GREEN)],
}


# ============================================================
#  Waterfall colormap (256-entry LUT)
# ============================================================
def _waterfall_lut():
    stops = [(0.00, (0, 0, 8)), (0.20, (10, 20, 90)), (0.40, (20, 90, 200)),
             (0.55, (30, 190, 200)), (0.70, (110, 220, 80)),
             (0.85, (250, 220, 40)), (0.95, (250, 80, 40)),
             (1.00, (255, 240, 240))]
    lut = []
    for i in range(256):
        t = i / 255.0
        for k in range(len(stops) - 1):
            t0, c0 = stops[k]
            t1, c1 = stops[k + 1]
            if t <= t1:
                u = (t - t0) / (t1 - t0)
                c = tuple(int(c0[j] + (c1[j] - c0[j]) * u) for j in range(3))
                lut.append("#%02x%02x%02x" % c)
                break
    return lut


LUT = _waterfall_lut()


# ============================================================
#  Serial
# ============================================================
def open_port(port, timeout=0.2):
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = BAUD
    ser.timeout = timeout
    ser.dtr = False        # on the ESP32-S3 USB-Serial/JTAG, DTR/RTS can reset the chip
    ser.rts = False
    ser.open()
    return ser


def read_json(ser):
    try:
        line = ser.readline().decode("utf-8", "replace").strip()
    except serial.SerialException:
        raise
    except Exception:
        return None
    if not line:
        return None
    if not line.startswith("{"):
        print(f"[raw serial, not JSON] {line[:120]}")
        return None
    try:
        return json.loads(line)
    except json.JSONDecodeError:
        print(f"[raw serial, invalid JSON] {line[:120]}")
        return None


def detect_port():
    for p in list_ports.comports():
        try:
            with open_port(p.device, timeout=0.5) as ser:
                ser.reset_input_buffer()
                deadline = time.time() + 2.5
                while time.time() < deadline:
                    d = read_json(ser)
                    if d and d.get("device") == DEVICE_ID:
                        return p.device
        except serial.SerialException:
            continue
    return None


class Reader(threading.Thread):
    """Reads and classifies JSON lines, and sends the commands queued by the
    GUI in self.tx (only this thread touches the port)."""

    def __init__(self, port, q):
        super().__init__(daemon=True)
        self.requested_port = port
        self.q = q
        self.tx = queue.Queue()
        self.running = True

    def run(self):
        while self.running:
            port = self.requested_port or detect_port()
            if not port:
                self.q.put(("info", "Analyzer not found, retrying"))
                self.q.put(("link", None))
                time.sleep(3)
                continue
            ser = None
            try:
                ser = open_port(port)
                self.q.put(("link", port))
                self.q.put(("info", f"Connected to {port}"))
                while self.running:
                    while not self.tx.empty():
                        cmd = self.tx.get_nowait()
                        ser.write((cmd + "\n").encode("ascii"))
                        self.q.put(("info", f"> {cmd}"))
                    d = read_json(ser)
                    if d is None or d.get("device") != DEVICE_ID:
                        continue
                    if d.get("meta"):
                        self.q.put(("meta", d))
                    elif "log" in d:
                        self.q.put(("log", d))
                    elif "pk" in d:
                        self.q.put(("sweep", d))
            except serial.SerialException as e:
                self.q.put(("info", f"Port lost: {e}"))
            finally:
                try:
                    if ser:
                        ser.close()
                except Exception:
                    pass
                self.q.put(("link", None))
            time.sleep(2)


# ============================================================
#  Base widgets
# ============================================================
def panel(parent, title):
    frame = tk.Frame(parent, bg=C_PANEL, highlightbackground=C_BORDER,
                     highlightthickness=1)
    head = tk.Frame(frame, bg=C_PANEL)
    head.pack(fill="x")
    tk.Label(head, text=title.upper(), bg=C_PANEL, fg=C_TXT2,
             font=F_TITLE, anchor="w", padx=12, pady=6).pack(side="left")
    body = tk.Frame(frame, bg=C_PANEL)
    body.pack(fill="both", expand=True, padx=12, pady=(0, 10))
    return frame, head, body


class StatPanel:
    def __init__(self, parent, title):
        self.frame, self.head, body = panel(parent, title)
        self.value = tk.Label(body, text="---", bg=C_PANEL, fg=C_GRAY,
                              font=F_MED, anchor="w")
        self.value.pack(fill="x")
        self.sub = tk.Label(body, text="", bg=C_PANEL, fg=C_TXT2,
                            font=F_SUB, anchor="w")
        self.sub.pack(fill="x")

    def set(self, text, color, sub=""):
        self.value.config(text=text, fg=color)
        self.sub.config(text=sub)


def entry(parent, row, label, value, width=9):
    tk.Label(parent, text=label, bg=C_PANEL, fg=C_TXT2, font=F_LBL,
             anchor="w").grid(row=row, column=0, sticky="w", pady=2)
    var = tk.StringVar(value=str(value))
    e = tk.Entry(parent, textvariable=var, width=width, bg=C_BG, fg=C_TXT,
                 insertbackground=C_TXT, relief="flat", font=F_MONO,
                 highlightthickness=1, highlightbackground=C_BORDER,
                 highlightcolor=C_BLUE)
    e.grid(row=row, column=1, sticky="ew", pady=2, padx=(8, 0))
    return var, e


def button(parent, text, command, color=C_TXT):
    return tk.Button(parent, text=text, command=command, bg=C_GRID, fg=color,
                     activebackground=C_BORDER, activeforeground=C_TXT,
                     relief="flat", font=F_LBL, padx=8, pady=3, cursor="hand2")


def option_menu(parent, var, options):
    m = tk.OptionMenu(parent, var, *options)
    m.config(bg=C_BG, fg=C_TXT, activebackground=C_BORDER,
             activeforeground=C_TXT, relief="flat", highlightthickness=1,
             highlightbackground=C_BORDER, font=F_MONO)
    m["menu"].config(bg=C_PANEL, fg=C_TXT, activebackground=C_BLUE)
    return m


def check(parent, text, var, color):
    return tk.Checkbutton(parent, text=text, variable=var, bg=C_PANEL, fg=color,
                          selectcolor=C_BG, activebackground=C_PANEL,
                          activeforeground=color, font=F_LBL, anchor="w")


def nice_ticks(a, b, target=10):
    span = b - a
    if span <= 0:
        return []
    step = span / target
    mag = 10 ** math.floor(math.log10(step))
    for m in (1, 2, 5, 10):
        if step <= m * mag:
            step = m * mag
            break
    t = math.ceil(a / step) * step
    out = []
    while t <= b + 1e-9:
        out.append(round(t, 6))
        t += step
    return out


# ============================================================
#  Spectrum
# ============================================================
class SpectrumPanel:
    def __init__(self, parent, title, app):
        self.app = app
        self.frame, self.head, body = panel(parent, title)
        self.canvas = tk.Canvas(body, bg=C_PANEL, highlightthickness=0, height=330)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda e: self.draw())
        self.canvas.bind("<Motion>", self._move)
        self.canvas.bind("<Leave>", self._leave)
        self.cursor_x = None

    def area(self):
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        return ML, MT, max(ML + 10, w - MR), max(MT + 10, h - MB)

    def _move(self, ev):
        self.cursor_x = ev.x
        self.draw()

    def _leave(self, _ev):
        self.cursor_x = None
        self.draw()

    def draw(self):
        a = self.app
        c = self.canvas
        c.delete("all")
        x0, y0, x1, y1 = self.area()
        ref, rng = a.ref_dbm, a.range_db
        floor_db = ref - rng
        f0, f1 = a.f_start, a.f_stop
        if f1 <= f0:
            return

        def fx(f):
            return x0 + (f - f0) / (f1 - f0) * (x1 - x0)

        def dy(db):
            db = min(max(db, floor_db), ref)
            return y0 + (ref - db) / rng * (y1 - y0)

        for db in nice_ticks(floor_db, ref, 10):
            y = dy(db)
            c.create_line(x0, y, x1, y, fill=C_GRID)
            c.create_text(x0 - 6, y, text=f"{db:.0f}", fill=C_TXT2, font=F_AXIS, anchor="e")
        c.create_text(4, y0, text="dBm", fill=C_TXT2, font=F_AXIS, anchor="nw")
        for f in nice_ticks(f0, f1, 10):
            x = fx(f)
            c.create_line(x, y0, x, y1, fill=C_GRID)
            c.create_text(x, y1 + 4, text=f"{f:g}", fill=C_TXT2, font=F_AXIS, anchor="n")

        channels = [(f, col) for f, col in GRIDS.get(a.grid.get(), []) if f0 <= f <= f1]
        if channels and (x1 - x0) / max(1, len(channels)) > 2.5:
            for f, col in channels:
                x = fx(f)
                c.create_line(x, y0, x, y1, fill=col, dash=(2, 3))
        c.create_rectangle(x0, y0, x1, y1, outline=C_BORDER)

        if not a.freqs:
            c.create_text((x0 + x1) / 2, (y0 + y1) / 2, text="waiting for sweep",
                          fill=C_GRAY, font=F_SUB)
            return

        traces = []
        if a.show_avg.get() and a.avg_db:
            traces.append((a.avg_db, C_BLUE))
        if a.show_max.get() and a.max_db:
            traces.append((a.max_db, C_RED))
        if a.show_live.get() and a.live_db:
            traces.append((a.live_db, C_YELLOW))
        for vals, color in traces:
            pts = self._to_pixels(vals, fx, dy)
            if len(pts) >= 4:
                c.create_line(*pts, fill=color, width=1)

        # Peak marker on the live trace
        if a.live_db:
            i = max(range(len(a.live_db)), key=lambda k: a.live_db[k])
            x, y = fx(a.freqs[i]), dy(a.live_db[i])
            c.create_polygon(x, y - 9, x - 5, y - 16, x + 5, y - 16, fill=C_GREEN, outline="")
            c.create_text(x + 8, y - 14, text=f"{a.freqs[i]:.3f} MHz  {a.live_db[i]:.1f} dBm",
                          fill=C_GREEN, font=F_MONO, anchor="w")

        # Cursor readout
        if self.cursor_x is not None and x0 <= self.cursor_x <= x1:
            f = f0 + (self.cursor_x - x0) / (x1 - x0) * (f1 - f0)
            i = int(round((f - a.freqs[0]) / a.step_mhz))
            i = min(max(i, 0), len(a.freqs) - 1)
            xc = fx(a.freqs[i])
            c.create_line(xc, y0, xc, y1, fill=C_TXT2, dash=(1, 2))
            txt = f"{a.freqs[i]:.4f} MHz"
            if a.live_db:
                txt += f"   live {a.live_db[i]:.1f}"
            if a.max_db:
                txt += f"   max {a.max_db[i]:.1f}"
            if a.avg_db:
                txt += f"   avg {a.avg_db[i]:.1f}"
            c.create_text(x1 - 6, y0 + 6, text=txt + " dBm", fill=C_TXT, font=F_MONO, anchor="ne")

    def _to_pixels(self, vals, fx, dy):
        """With more points than pixels, decimate keeping the per-column
        maximum (display peak detector)."""
        a = self.app
        width = self.area()[2] - self.area()[0]
        pts = []
        if len(vals) <= width:
            for f, v in zip(a.freqs, vals):
                pts += (fx(f), dy(v))
            return pts
        col = {}
        for f, v in zip(a.freqs, vals):
            px = int(fx(f))
            if px not in col or v > col[px]:
                col[px] = v
        for px in sorted(col):
            pts += (px, dy(col[px]))
        return pts


# ============================================================
#  Waterfall
# ============================================================
class WaterfallPanel:
    """Tk image as wide as the plot area. Each new sweep is written as the top
    row and the rest is shifted with Tk's native 'copy' (A/B double buffer),
    so no pixels are redrawn in Python."""

    def __init__(self, parent, title, app):
        self.app = app
        self.frame, self.head, body = panel(parent, title)
        self.canvas = tk.Canvas(body, bg=C_BG, highlightthickness=0, height=220)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda e: self._schedule_rebuild())
        self.history = deque(maxlen=WATERFALL_HIST)
        self.img_a = self.img_b = None
        self.item = None
        self.w = self.h = 0
        self.pixmap = None
        self._pending = None

    def _schedule_rebuild(self):
        if self._pending:
            self.canvas.after_cancel(self._pending)
        self._pending = self.canvas.after(150, self.rebuild)

    def reset(self):
        self.history.clear()
        self.rebuild()

    def _pixel_map(self, n):
        """Point index range [lo, hi) covered by each pixel column."""
        a = self.app
        f0, f1 = a.f_start, a.f_stop
        mp = []
        for px in range(self.w):
            fa = f0 + px / self.w * (f1 - f0)
            fb = f0 + (px + 1) / self.w * (f1 - f0)
            lo = int(math.floor((fa - a.freqs[0]) / a.step_mhz + 0.5))
            hi = int(math.floor((fb - a.freqs[0]) / a.step_mhz + 0.5))
            lo = min(max(lo, 0), n - 1)
            hi = min(max(hi, lo + 1), n)
            mp.append((lo, hi))
        return mp

    def _row(self, vals):
        a = self.app
        floor_db = a.ref_dbm - a.range_db
        k = 255.0 / a.range_db
        if self.pixmap is None or len(self.pixmap) != self.w:
            self.pixmap = self._pixel_map(len(vals))
        cols = []
        for lo, hi in self.pixmap:
            v = vals[lo] if hi - lo == 1 else max(vals[lo:hi])
            idx = int((v - floor_db) * k)
            cols.append(LUT[0 if idx < 0 else (255 if idx > 255 else idx)])
        return "{" + " ".join(cols) + "}"

    def rebuild(self):
        self._pending = None
        c = self.canvas
        c.delete("all")
        cw, ch = c.winfo_width(), c.winfo_height()
        self.w = max(10, cw - ML - MR)
        self.h = max(10, ch - 4)
        self.pixmap = None
        self.img_a = tk.PhotoImage(width=self.w, height=self.h)
        self.img_b = tk.PhotoImage(width=self.w, height=self.h)
        self.item = c.create_image(ML, 2, image=self.img_a, anchor="nw")
        c.create_text(ML - 6, 4, text="now", fill=C_TXT2, font=F_AXIS, anchor="ne")
        c.create_text(ML - 6, ch - 4, text="past", fill=C_TXT2, font=F_AXIS, anchor="se")
        if self.history and self.app.freqs:
            k = WATERFALL_PX
            for j, v in enumerate(list(self.history)[-(self.h // k + 1):][::-1]):
                y = j * k
                if y >= self.h:
                    break
                self.img_a.put(self._row(v), to=(0, y, self.w, min(self.h, y + k)))

    def add(self, vals):
        self.history.append(vals)
        if self.img_a is None or not self.app.freqs:
            return
        a, b = self.img_a, self.img_b
        k = WATERFALL_PX
        b.tk.call(b, "copy", a, "-from", 0, 0, self.w, self.h - k, "-to", 0, k)
        b.put(self._row(vals), to=(0, 0, self.w, k))
        self.canvas.itemconfig(self.item, image=b)
        self.img_a, self.img_b = b, a


# ============================================================
#  Main window
# ============================================================
class Dashboard:
    def __init__(self, root, port):
        self.root = root
        self.q = queue.Queue()
        self.last_rx = 0.0
        self.paused = False

        self.freqs = []
        self.step_mhz = 0.0
        self.f_start, self.f_stop = CFG_F0_MHZ, CFG_F1_MHZ
        self.signature = None          # (f0, step, n) of the current sweep
        self.live_db = self.max_db = self.avg_db = None
        self._avg_lin = None
        self.meta = {}
        self.t_last_sweep = None
        self.rate = 0.0

        self.ref_dbm, self.range_db = REF_DBM, RANGE_DB
        self.offset_db, self.avg_n = CAL_OFFSET_DB, AVERAGE_N

        root.title("LORA SPECTRUM ANALYZER - ELECTGPL")
        root.configure(bg=C_BG)
        root.geometry("1550x930")
        root.minsize(1200, 760)

        self._header()
        self._body()

        self.reader = Reader(port, self.q)
        self.reader.start()
        self.root.after(30, self.tick)

    # ---------- layout ----------
    def _load_logo(self):
        if not os.path.exists(LOGO_PATH):
            return None
        try:
            if _HAS_PIL:
                img = Image.open(LOGO_PATH)
                r = LOGO_HEIGHT_PX / img.height
                img = img.resize((max(1, int(img.width * r)), LOGO_HEIGHT_PX), Image.LANCZOS)
                return ImageTk.PhotoImage(img)
            img = tk.PhotoImage(file=LOGO_PATH)
            f = max(1, img.height() // LOGO_HEIGHT_PX)
            return img.subsample(f, f) if f > 1 else img
        except Exception as e:
            print(f"[logo] {e}")
            return None

    def _header(self):
        bar = tk.Frame(self.root, bg=C_BG)
        bar.pack(fill="x", padx=16, pady=(14, 8))
        self.logo_img = self._load_logo()
        if self.logo_img is not None:
            tk.Label(bar, image=self.logo_img, bg=C_BG).pack(side="left", padx=(0, 12))
        tk.Label(bar, text="LORA SPECTRUM ANALYZER", bg=C_BG, fg=C_TXT,
                 font=("Segoe UI", 22)).pack(side="left")
        tk.Label(bar, text="  electgpl · ESP32-S3 · SX1262 · instantaneous RSSI (GFSK)",
                 bg=C_BG, fg=C_TXT2, font=F_SUB).pack(side="left")
        self.lbl_clock = tk.Label(bar, text="", bg=C_BG, fg=C_TXT2, font=F_MONO)
        self.lbl_clock.pack(side="right", padx=(16, 0))
        self.lbl_link = tk.Label(bar, text="connecting", bg=C_BG, fg=C_TXT2, font=F_SUB)
        self.lbl_link.pack(side="right")
        self.led = tk.Canvas(bar, width=18, height=18, bg=C_BG, highlightthickness=0)
        self.led.pack(side="right", padx=8)
        self.led_dot = self.led.create_oval(2, 2, 16, 16, fill=C_GRAY, outline="")

    def _body(self):
        cont = tk.Frame(self.root, bg=C_BG)
        cont.pack(fill="both", expand=True, padx=16, pady=(0, 6))
        cont.columnconfigure(0, weight=1)
        cont.columnconfigure(1, weight=0, minsize=290)
        cont.rowconfigure(1, weight=3)
        cont.rowconfigure(2, weight=2)

        stats = tk.Frame(cont, bg=C_BG)
        stats.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        for i in range(4):
            stats.columnconfigure(i, weight=1, uniform="s")
        self.s_peak = StatPanel(stats, "peak (live trace)")
        self.s_floor = StatPanel(stats, "noise floor (median avg)")
        self.s_sweep = StatPanel(stats, "sweep")
        self.s_cfg = StatPanel(stats, "applied configuration")
        for i, p in enumerate((self.s_peak, self.s_floor, self.s_sweep, self.s_cfg)):
            p.frame.grid(row=0, column=i, sticky="nsew",
                         padx=(0 if i == 0 else 5, 0 if i == 3 else 5))

        self.spec = SpectrumPanel(cont, "spectrum", self)
        self.spec.frame.grid(row=1, column=0, sticky="nsew", pady=(0, 10))
        self.wf = WaterfallPanel(cont, "waterfall (live trace)", self)
        self.wf.frame.grid(row=2, column=0, sticky="nsew")

        side = tk.Frame(cont, bg=C_BG)
        side.grid(row=0, column=1, rowspan=3, sticky="nsew", padx=(10, 0))
        self._controls(side)

        self.lbl_status = tk.Label(self.root, text="", bg=C_BG, fg=C_TXT2,
                                   font=F_MONO, anchor="w")
        self.lbl_status.pack(fill="x", padx=18, pady=(0, 8))

    def _controls(self, side):
        f1, _, c1 = panel(side, "sweep (sent to ESP32)")
        f1.pack(fill="x", pady=(0, 10))
        c1.columnconfigure(1, weight=1)
        self.v_f0, _ = entry(c1, 0, "Start [MHz]", CFG_F0_MHZ)
        self.v_f1, _ = entry(c1, 1, "Stop [MHz]", CFG_F1_MHZ)
        self.v_step, _ = entry(c1, 2, "Step [kHz]", CFG_STEP_KHZ)
        tk.Label(c1, text="RBW [kHz]", bg=C_PANEL, fg=C_TXT2, font=F_LBL,
                 anchor="w").grid(row=3, column=0, sticky="w", pady=2)
        self.v_rbw = tk.StringVar(value=f"{CFG_RBW_KHZ:.1f}")
        option_menu(c1, self.v_rbw, [f"{r:.1f}" for r in RBW_KHZ_VALID]).grid(
            row=3, column=1, sticky="ew", padx=(8, 0), pady=2)
        self.v_n, _ = entry(c1, 4, "Samples N/point", CFG_N_SAMPLES)
        self.v_settle, _ = entry(c1, 5, "Settle [us] 0=auto", 0)
        self.v_ppm, _ = entry(c1, 6, "XTAL corr. [ppm]", 0.0)
        self.lbl_pts = tk.Label(c1, text="", bg=C_PANEL, fg=C_TXT2, font=F_LBL, anchor="w")
        self.lbl_pts.grid(row=7, column=0, columnspan=2, sticky="w", pady=(4, 2))
        fb = tk.Frame(c1, bg=C_PANEL)
        fb.grid(row=8, column=0, columnspan=2, sticky="ew", pady=(4, 0))
        button(fb, "Apply", self.send_cfg, C_GREEN).pack(side="left")
        self.btn_pause = button(fb, "Pause", self.toggle_pause, C_YELLOW)
        self.btn_pause.pack(side="left", padx=6)
        fp = tk.Frame(c1, bg=C_PANEL)
        fp.grid(row=9, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        for name, a, b in PRESETS:
            button(fp, name, lambda a=a, b=b: self.preset(a, b)).pack(side="left", padx=(0, 4))
        for v in (self.v_f0, self.v_f1, self.v_step):
            v.trace_add("write", lambda *_: self._estimate_pts())
        self._estimate_pts()

        f2, _, c2 = panel(side, "display")
        f2.pack(fill="x", pady=(0, 10))
        c2.columnconfigure(1, weight=1)
        self.v_ref, e1 = entry(c2, 0, "Ref. level [dBm]", REF_DBM)
        self.v_range, e2 = entry(c2, 1, "Range [dB]", RANGE_DB)
        self.v_off, e3 = entry(c2, 2, "Cal. offset [dB]", CAL_OFFSET_DB)
        self.v_avgn, e4 = entry(c2, 3, "Average N [sweeps]", AVERAGE_N)
        for e in (e1, e2, e3, e4):
            e.bind("<Return>", lambda _e: self.apply_view())
        tk.Label(c2, text="Grid overlay", bg=C_PANEL, fg=C_TXT2, font=F_LBL,
                 anchor="w").grid(row=4, column=0, sticky="w", pady=2)
        self.grid = tk.StringVar(value="none")
        option_menu(c2, self.grid, list(GRIDS)).grid(row=4, column=1, sticky="ew", padx=(8, 0))
        self.grid.trace_add("write", lambda *_: self.spec.draw())

        self.show_live = tk.BooleanVar(value=True)
        self.show_max = tk.BooleanVar(value=True)
        self.show_avg = tk.BooleanVar(value=True)
        check(c2, "Live", self.show_live, C_YELLOW).grid(row=5, column=0, sticky="w")
        check(c2, "Max-hold", self.show_max, C_RED).grid(row=5, column=1, sticky="w")
        check(c2, "Average", self.show_avg, C_BLUE).grid(row=6, column=0, sticky="w")
        tk.Label(c2, text="Live detector", bg=C_PANEL, fg=C_TXT2, font=F_LBL,
                 anchor="w").grid(row=7, column=0, sticky="w", pady=(6, 2))
        self.detector = tk.StringVar(value="peak")
        option_menu(c2, self.detector, ["peak", "average"]).grid(
            row=7, column=1, sticky="ew", padx=(8, 0), pady=(6, 2))
        fb2 = tk.Frame(c2, bg=C_PANEL)
        fb2.grid(row=8, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        button(fb2, "Apply view", self.apply_view).pack(side="left")
        button(fb2, "Reset traces", self.reset_traces, C_RED).pack(side="left", padx=6)
        button(fb2, "CSV", self.save_csv, C_BLUE).pack(side="left")

        f3, _, c3 = panel(side, "firmware")
        f3.pack(fill="both", expand=True)
        self.lbl_meta = tk.Label(c3, text="no data", bg=C_PANEL, fg=C_TXT2,
                                 font=F_MONO, anchor="nw", justify="left")
        self.lbl_meta.pack(fill="both", expand=True)

    # ---------- actions ----------
    def _estimate_pts(self):
        try:
            f0, f1, s = float(self.v_f0.get()), float(self.v_f1.get()), float(self.v_step.get())
            n = int(math.floor((f1 - f0) * 1000.0 / s + 1e-6)) + 1
            col = C_TXT2 if 2 <= n <= MAX_PTS else C_RED
            self.lbl_pts.config(text=f"{n} points (max {MAX_PTS})", fg=col)
        except (ValueError, ZeroDivisionError):
            self.lbl_pts.config(text="invalid values", fg=C_RED)

    def preset(self, a, b):
        self.v_f0.set(f"{a:g}")
        self.v_f1.set(f"{b:g}")
        if b - a > 50:
            self.v_step.set("200")
            self.v_rbw.set("234.3")
        self.send_cfg()

    def send_cfg(self):
        try:
            f0, f1 = float(self.v_f0.get()), float(self.v_f1.get())
            step, rbw = float(self.v_step.get()), float(self.v_rbw.get())
            n, settle = int(self.v_n.get()), int(float(self.v_settle.get()))
            ppm = float(self.v_ppm.get())
        except ValueError:
            self.status("Invalid configuration")
            return
        if step > rbw:
            self.status(f"Warning: step {step:g} kHz > RBW {rbw:g} kHz -> gaps between points")
        self.reader.tx.put(f"SETTLE {settle}")
        self.reader.tx.put(f"PPM {ppm:.2f}")
        self.reader.tx.put(f"CFG {f0:.6f} {f1:.6f} {step:.3f} {rbw:.1f} {n}")
        if not self.paused:
            self.reader.tx.put("RUN")

    def toggle_pause(self):
        self.paused = not self.paused
        self.reader.tx.put("STOP" if self.paused else "RUN")
        self.btn_pause.config(text="Resume" if self.paused else "Pause")

    def apply_view(self):
        try:
            self.ref_dbm = float(self.v_ref.get())
            self.range_db = max(10.0, float(self.v_range.get()))
            new_off = float(self.v_off.get())
            self.avg_n = max(1, int(self.v_avgn.get()))
        except ValueError:
            self.status("Invalid display values")
            return
        if new_off != self.offset_db:
            self.offset_db = new_off
            self.reset_traces()
        self.spec.draw()
        self.wf.rebuild()

    def reset_traces(self):
        self.max_db = None
        self.avg_db = None
        self._avg_lin = None
        self.wf.reset()
        self.spec.draw()

    def save_csv(self):
        if not self.freqs:
            self.status("No sweep to save")
            return
        path = os.path.join(CSV_FOLDER, time.strftime("lora_sa_%Y%m%d_%H%M%S.csv"))
        m = self.meta
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(f"# RBW_kHz={m.get('rbw')} N={m.get('ns')} settle_us={m.get('settle_us')} "
                     f"ppm={m.get('ppm')} reg={m.get('reg')} offset_dB={self.offset_db}\n")
            fh.write("freq_MHz,live_dBm,maxhold_dBm,avg_dBm\n")
            for i, f in enumerate(self.freqs):
                v = self.live_db[i] if self.live_db else ""
                mx = self.max_db[i] if self.max_db else ""
                av = self.avg_db[i] if self.avg_db else ""
                fh.write(f"{f:.6f},{v},{mx},{av}\n")
        self.status(f"CSV saved: {path}")

    def status(self, text):
        stamp = time.strftime("%H:%M:%S")
        print(f"[{stamp}] {text}")
        self.lbl_status.config(text=f"[{stamp}] {text}")

    # ---------- data ----------
    def tick(self):
        last_sweep = None
        try:
            for _ in range(200):
                kind, d = self.q.get_nowait()
                if kind == "sweep":
                    last_sweep = d
                    self._process(d)
                elif kind == "meta":
                    self._meta(d)
                elif kind == "log":
                    self.status(f"{d.get('log')}: {d.get('text')}")
                elif kind == "info":
                    self.status(d)
                elif kind == "link":
                    self.lbl_link.config(text=f"{d}" if d else "disconnected")
                    if d:
                        self.send_cfg()
        except queue.Empty:
            pass
        if last_sweep is not None:
            self.spec.draw()
            self._stats(last_sweep)

        self.lbl_clock.config(text=time.strftime("%H:%M:%S"))
        alive = self.last_rx and time.time() - self.last_rx < 2.0
        self.led.itemconfig(self.led_dot, fill=C_GREEN if alive else C_GRAY)
        self.root.after(30, self.tick)

    def _meta(self, m):
        self.meta = m
        self.v_rbw.set(f"{m.get('rbw', 0):.1f}")      # RBW actually applied
        txt = (f"fw {m.get('fw')}  run={m.get('run')}\n"
               f"f0 {m.get('f0'):.4f} MHz\nf1 {m.get('f1'):.4f} MHz\n"
               f"step {m.get('st')} kHz  pts {m.get('n')}\n"
               f"RBW {m.get('rbw')} kHz  N {m.get('ns')}\n"
               f"settle {m.get('settle_us')} us "
               f"({'auto' if m.get('settle_auto') else 'manual'})\n"
               f"gap {m.get('gap_us')} us\nppm {m.get('ppm')}\n"
               f"image cal: {m.get('segs')} seg.\n"
               f"regulator {m.get('reg')}  boost {m.get('boost')}")
        self.lbl_meta.config(text=txt)
        self.s_cfg.set(f"{m.get('f0'):.1f}-{m.get('f1'):.1f}", C_TXT,
                       f"step {m.get('st'):g} kHz · RBW {m.get('rbw')} kHz · N={m.get('ns')}")

    def _process(self, d):
        self.last_rx = time.time()
        try:
            n = int(d["n"])
            f0 = float(d["f0"])
            st_khz = float(d["st"])
            pk = bytes.fromhex(d["pk"])
            av = bytes.fromhex(d["av"])
        except (KeyError, ValueError):
            return
        if len(pk) != n or len(av) != n:
            return
        sig = (round(f0, 6), round(st_khz, 3), n)
        if sig != self.signature:
            self.signature = sig
            self.step_mhz = st_khz / 1000.0
            self.freqs = [f0 + i * self.step_mhz for i in range(n)]
            self.f_start, self.f_stop = self.freqs[0], self.freqs[-1]
            self.max_db = self.avg_db = self._avg_lin = None
            self.wf.history.clear()
            self.wf.rebuild()

        off = self.offset_db
        pk_db = [-b / 2.0 + off for b in pk]
        av_db = [-b / 2.0 + off for b in av]
        self.live_db = pk_db if self.detector.get() == "peak" else av_db

        if self.max_db is None:
            self.max_db = list(pk_db)
        else:
            self.max_db = [a if a > b else b for a, b in zip(self.max_db, pk_db)]

        # Exponential average in linear power (mW), not in dB
        lin = [10 ** (v / 10.0) for v in av_db]
        alpha = 1.0 / self.avg_n
        if self._avg_lin is None:
            self._avg_lin = lin
        else:
            self._avg_lin = [p + alpha * (x - p) for p, x in zip(self._avg_lin, lin)]
        self.avg_db = [10 * math.log10(p) for p in self._avg_lin]

        self.wf.add(self.live_db)
        now = time.time()
        if self.t_last_sweep:
            dt = now - self.t_last_sweep
            if dt > 0:
                self.rate = 0.8 * self.rate + 0.2 / dt if self.rate else 1.0 / dt
        self.t_last_sweep = now

    def _stats(self, d):
        if not self.live_db:
            return
        i = max(range(len(self.live_db)), key=lambda k: self.live_db[k])
        self.s_peak.set(f"{self.live_db[i]:.1f} dBm", C_GREEN, f"{self.freqs[i]:.4f} MHz")
        floor_db = statistics.median(self.avg_db or self.live_db)
        rbw = float(self.meta.get("rbw", 0) or 0)
        sub = ""
        if rbw > 0:
            ktb = -174.0 + 10 * math.log10(rbw * 1e3)
            sub = f"kTB {ktb:.1f} dBm · apparent NF ~{floor_db - ktb:.1f} dB"
        self.s_floor.set(f"{floor_db:.1f} dBm", C_BLUE, sub)
        t_ms = d.get("t_us", 0) / 1000.0
        self.s_sweep.set(f"{t_ms:.0f} ms", C_YELLOW,
                         f"{self.rate:.1f} sweeps/s · {len(self.freqs)} pts · #{d.get('sw')}")


def main():
    root = tk.Tk()
    Dashboard(root, PORT)
    root.mainloop()


if __name__ == "__main__":
    main()
