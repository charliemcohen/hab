# every report figure -> docs/figures/*.png at 300 dpi. one font, black/gray, one accent color
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import numpy as np
import pandas as pd
from PIL import Image

import common as C

OUT = C.ROOT / "docs" / "figures"
OUT.mkdir(parents=True, exist_ok=True)
from matplotlib import font_manager
for _f in (C.ROOT / "fonts").glob("*.ttf"):
    font_manager.fontManager.addfont(str(_f))
SANS, MONO = "IBM Plex Sans", "IBM Plex Mono"
ACC = "#1f5fbf"  # the one accent color
INK, MUTED, PANEL, WIRE, WARN = "#1d1d1f", "#6e6e73", "#f2f3f5", "#48484a", "#b3261e"
GRAY = "#8c8c8c"
LGRAY = "#d0d0d0"
plt.rcParams.update({"font.family": SANS, "font.size": 8.5, "axes.titlesize": 9.5, "axes.titleweight": "semibold",
                     "axes.labelsize": 8.5, "axes.labelcolor": INK, "text.color": INK, "axes.edgecolor": "#3a3a3c",
                     "axes.linewidth": 0.7, "xtick.color": "#3a3a3c", "ytick.color": "#3a3a3c", "xtick.labelsize": 7.5,
                     "ytick.labelsize": 7.5, "legend.fontsize": 7.5, "legend.frameon": False,
                     "axes.spines.top": False, "axes.spines.right": False, "figure.dpi": 100, "savefig.dpi": 300})

S, counts, E = C.load_sci()
G, FX = C.load_gps2()
F1 = C.load_gps1()
K = C.load_kml2()
N = C.compute_numbers()
REL = C.RELEASE_UTC
BURST = pd.Timestamp(N["burst_t"])
GAPS = C.gps_gaps(FX)


def save(fig, name):
    fig.savefig(OUT / name, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("wrote", name)


def minutes(t):
    return (t - REL).dt.total_seconds() / 60 if hasattr(t, "dt") else (t - REL).total_seconds() / 60


def box(ax, x, y, w, h, text, fc="white", ec="black", lw=1.0, fs=8, bold=False):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.15", fc=fc, ec=ec, lw=lw))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs, fontweight="bold" if bold else "normal")


def arrow(ax, x1, y1, x2, y2, color="black", ls="-", lw=1.0):
    ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="-|>", mutation_scale=10, color=color, lw=lw, linestyle=ls))


# block diagrams: 20 drawing units per inch
def _canvas(w_in, h_in):
    fig = plt.figure(figsize=(w_in, h_in))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, w_in * 20); ax.set_ylim(0, h_in * 20); ax.axis("off")
    return fig, ax


def _panel(ax, x, y, w, h, label, note=None):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=1.8", fc=PANEL, ec="none", zorder=0))
    ax.text(x + 2.4, y + h - 2.4, label.upper(), fontsize=6.0, fontweight="semibold", color=MUTED, va="top", zorder=5)
    if note:
        ax.text(x + w - 2.4, y + h - 2.4, note, fontsize=6.0, color=MUTED, va="top", ha="right", zorder=5)


def _box(ax, x, y, w, h, title, sub=None, sub_color=None, ec=INK, lw=0.8, align="center", ts=7.2):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=1.0", fc="white", ec=ec, lw=lw, zorder=2))
    cx, ha = (x + w / 2, "center") if align == "center" else (x + 2.2, "left")
    if sub:
        ax.text(cx, y + h / 2 + 1.2, title, fontsize=ts, fontweight="semibold", ha=ha, va="center", zorder=3)
        ax.text(cx, y + h / 2 - 1.6, sub, fontsize=6.0, color=sub_color or MUTED, ha=ha, va="center", zorder=3)
    else:
        ax.text(cx, y + h / 2, title, fontsize=ts, fontweight="semibold", ha=ha, va="center", zorder=3)


def _wire(ax, pts, color=WIRE, lw=0.8, arrow=True):
    xs, ys = zip(*pts)
    ax.plot(xs, ys, color=color, lw=lw, solid_capstyle="butt", zorder=1)
    if arrow:
        ax.annotate("", xy=pts[-1], xytext=pts[-2], zorder=4,
                    arrowprops=dict(arrowstyle="-|>,head_length=0.5,head_width=0.22", color=color, lw=lw,
                                    shrinkA=0, shrinkB=0, mutation_scale=7))


def _bus(ax, x, y, text, color=None, ha="center", va="bottom"):
    ax.text(x, y, text, fontsize=5.7, family=MONO, color=color or MUTED, ha=ha, va=va, zorder=4,
            bbox=dict(fc=PANEL, ec="none", pad=0.4))


def fig01():
    fig, ax = _canvas(7.4, 3.6)
    ax.add_patch(mpatches.Circle((20, 60), 7.0, fc="white", ec=INK, lw=0.9, zorder=2))
    ax.add_patch(mpatches.Polygon([(18.6, 53.2), (21.4, 53.2), (20, 51.6)], closed=True, fc=INK, ec=INK, zorder=2))
    ax.plot([20, 20], [51.6, 47.2], color=INK, lw=0.8)
    t = np.linspace(0, np.pi, 40)
    ax.add_patch(mpatches.Polygon(np.c_[20 + 6.5 * np.cos(t), 41.6 + 5.4 * np.sin(t)], closed=True, fc=PANEL, ec=INK, lw=0.9, zorder=2))
    for x0 in (13.5, 17.2, 22.8, 26.5):
        ax.plot([x0, 20], [41.6, 36.4], color=WIRE, lw=0.5)
    for dx in (-0.45, 0.45):
        ax.plot([20 + dx, 20 + dx], [36.4, 30.6], color=INK, lw=0.8)
    ax.add_patch(mpatches.Circle((20, 30.0), 0.7, fc="white", ec=INK, lw=0.8, zorder=3))
    fl, fr, br, bl = (10, 22), (30, 22), (33, 24.5), (13, 24.5)
    ax.add_patch(mpatches.Polygon([fl, fr, br, bl], closed=True, fc="#e3e5e9", ec=INK, lw=0.8, zorder=2))
    ax.add_patch(mpatches.Polygon([fr, (30, 8), (33, 10.5), br], closed=True, fc="#d6d9de", ec=INK, lw=0.8, zorder=2))
    ax.add_patch(mpatches.Rectangle((10, 8), 20, 14, fc=PANEL, ec=INK, lw=0.9, zorder=2))
    for c, front in ((fl, True), (fr, True), (br, False), (bl, False)):
        ax.plot([20, c[0]], [29.3, c[1]], color=INK if front else WIRE, lw=0.7, ls="-" if front else (0, (2, 1.5)), zorder=3)
    lab = [(61.0, "Kaymont TX1200", "latex balloon"), (44.2, "Parachute", None),
           (33.4, "Flight train", "two lines to a ring,\nthen a four-line bridle"), (16.0, "Payload", "XPS foam box, reflective wrap")]
    for y, a, b in lab:
        ax.text(36.5, y + (1.1 if b else 0), a, fontsize=7.2, fontweight="semibold", va="center")
        if b:
            ax.text(36.5, y - 1.0, b, fontsize=6.0, color=MUTED, va="top", linespacing=1.35)
    _panel(ax, 64, 4, 82, 64, "Inside the payload")
    cards = [("Science logger", None, ["Mega 2560 + Uno", "10 sensors, I\u00b2C mux", "GPS and SD card", "about 1 record per second"]),
             ("Reaction wheel", "independent", ["Nano + BNO085 IMU", "25D motor, G2 driver", "4-spoke wheel, 75 rpm", "not logged in flight"]),
             ("Tracking, cameras", None, ["SPOT Trace tracker", "Mobius Maxi 4K", "2 \u00d7 RunCam Thumb 2"])]
    cw, gap, x = 24.6, 1.8, 66.6
    for title, tag, lines in cards:
        ax.add_patch(FancyBboxPatch((x, 22), cw, 36, boxstyle="round,pad=0,rounding_size=1.0", fc="white", ec="#d1d1d6", lw=0.7, zorder=2))
        ax.add_patch(mpatches.Rectangle((x + 2.2, 54.0), 6, 0.7, fc=ACC, ec="none", zorder=3))
        ax.text(x + 2.2, 50.6, title, fontsize=7.4, fontweight="semibold", va="center", zorder=3)
        if tag:
            ax.text(x + 2.2, 47.4, tag, fontsize=5.8, color=MUTED, va="center", zorder=3, style="italic")
        for i, ln in enumerate(lines):
            ax.text(x + 2.2, 42.4 - i * 4.4, ln, fontsize=6.1, color="#3a3a3c", va="center", zorder=3)
        x += cw + gap
    ax.add_patch(FancyBboxPatch((66.6, 9), 77.4, 8, boxstyle="round,pad=0,rounding_size=1.0", fc="white", ec="#d1d1d6", lw=0.7, zorder=2))
    ax.text(68.8, 13, "Power", fontsize=7.2, fontweight="semibold", va="center", zorder=3)
    ax.text(79.0, 13, "USB power banks \u00d72  \u00b7  LiPo for the wheel motor  \u00b7  battery holders \u00d72", fontsize=6.1, color="#3a3a3c", va="center", zorder=3)
    save(fig, "fig01_system_diagram.png")


def fig02():
    fig, axs = plt.subplots(1, 2, figsize=(7, 3.4))
    fl, fr, br, bl = (1.6, 3.0), (4.4, 3.0), (4.9, 3.5), (2.1, 3.5)
    for ax, title in zip(axs, ["Before: failed launch", "After: redesigned flight train"]):
        ax.set_xlim(0, 6.6); ax.set_ylim(0, 7); ax.axis("off")
        ax.set_title(title, fontsize=8.5, fontweight="semibold", loc="left", x=0.12)
        ax.add_patch(mpatches.Polygon([fl, fr, br, bl], closed=True, fc="#e3e5e9", ec=INK, lw=0.8))
        ax.add_patch(mpatches.Polygon([fr, (4.4, 1.0), (4.9, 1.5), br], closed=True, fc="#d6d9de", ec=INK, lw=0.8))
        ax.add_patch(mpatches.Rectangle((1.6, 1.0), 2.8, 2.0, fc=PANEL, ec=INK, lw=0.9))
        ax.text(3.0, 2.0, "payload", ha="center", va="center", fontsize=7, color=MUTED)
        ax.plot([3.25], [6.6], "o", color=INK, ms=3.5)
        ax.text(3.45, 6.6, "to parachute and balloon", fontsize=6.5, va="center", color=MUTED)
    a = axs[0]
    a.plot([3.25, 3.25], [6.55, 5.075], color=INK, lw=1.1)
    a.plot([3.25, 3.25], [4.745, 3.25], color=INK, lw=1.1)
    a.plot([3.25], [3.25], "s", color=INK, ms=3)
    for yc in (5.0, 4.82):
        a.add_patch(mpatches.Circle((3.25, yc), 0.075, fc="white", ec=INK, lw=0.9, zorder=3))
    a.plot([3.06, 3.44], [4.72, 5.10], color=WARN, lw=1.2, zorder=4); a.plot([3.06, 3.44], [5.10, 4.72], color=WARN, lw=1.2, zorder=4)
    a.text(3.6, 4.91, "swivel broke when\nthe line pulled tight", fontsize=6.5, va="center", color=WARN)
    a.text(3.25, 0.35, "one swivel carried the whole payload", ha="center", fontsize=6.5, color="#3a3a3c")
    b = axs[1]
    tie = (3.25, 5.0)
    for dx in (-0.07, 0.07):
        b.plot([3.25 + dx, 3.25 + dx], [6.55, tie[1] + 0.08], color=INK, lw=1.0)
    b.add_patch(mpatches.Circle(tie, 0.09, fc="white", ec=INK, lw=1.0, zorder=3))
    b.text(3.5, 5.8, "two lines\ninstead of one", fontsize=6.5, va="center", color=ACC)
    for c, front in ((fl, True), (fr, True), (br, False), (bl, False)):
        b.plot([tie[0], c[0]], [tie[1], c[1]], color=INK if front else WIRE, lw=0.9, ls="-" if front else (0, (3, 2)))
        b.plot([c[0], c[0]], [c[1], c[1] - 0.45], color=WIRE, lw=0.8, ls=":")
        b.plot([c[0]], [c[1]], "s", color=INK, ms=2.5)
    b.text(5.1, 4.25, "four-line bridle,\none line per\ntop corner", fontsize=6.5, va="center", color="#3a3a3c")
    b.text(5.1, 2.4, "anchored through\nthe foam at\neach corner", fontsize=6.5, va="center", color=MUTED)
    b.text(3.25, 0.35, "added lines reduce single-point risk", ha="center", fontsize=6.5, color="#3a3a3c")
    save(fig, "fig02_rigging_before_after.png")


def fig03():
    fig, ax = _canvas(7.4, 4.4)
    _panel(ax, 2, 36, 144, 50, "Science logger", "recorded to SD at about 1 Hz")
    rows = [("MCP9808 #1", "interior air", None, "ch5"), ("MCP9808 #2", "battery area \u00b7 no readings in flight", WARN, "ch1"),
            ("MCP9808 #3 \u00b7 TMP119 \u00b7 MS8607 \u00b7 AHT20 \u00b7 BMP390", "bottom exterior cluster", None, "ch2"),
            ("VEML7700 \u00b7 LTR390", "top-facing \u00b7 light and UV", None, "ch3"), ("LIS3DH", "interior deck \u00b7 acceleration", None, "ch4")]
    top, h, g = 79.5, 6.6, 1.9
    for i, (t1, t2, col, ch) in enumerate(rows):
        y = top - h - i * (h + g)
        _box(ax, 5, y, 52, h, t1, t2, sub_color=col, align="left", ts=6.8)
        _wire(ax, [(57, y + h / 2), (66, y + h / 2)], arrow=True)
        _bus(ax, 61.5, y + h / 2 + 0.35, ch)
    ybot = top - 5 * h - 4 * g
    ax.add_patch(FancyBboxPatch((66, ybot), 13, top - ybot, boxstyle="round,pad=0,rounding_size=1.0", fc="white", ec=INK, lw=0.8, zorder=2))
    my = (top + ybot) / 2
    ax.text(72.5, my + 1.6, "PCA9548", fontsize=6.8, fontweight="semibold", ha="center", va="center", zorder=3)
    ax.text(72.5, my - 1.2, "I\u00b2C mux", fontsize=6.0, color=MUTED, ha="center", va="center", zorder=3)
    ax.text(72.5, my - 3.6, "0x70", fontsize=5.7, family=MONO, color=MUTED, ha="center", va="center", zorder=3)
    _box(ax, 85, my - 6, 21, 12, "Arduino Mega 2560", "1 Hz sweep \u00b7 FW 1.2.0")
    _wire(ax, [(79, my), (85, my)]); _bus(ax, 82, my + 0.5, "I\u00b2C")
    _box(ax, 122, my - 6, 21, 12, "Arduino Uno", "logger \u00b7 SCI-UNO 1.3.0", ec=ACC, lw=1.0)
    _wire(ax, [(106, my), (122, my)], color=ACC, lw=1.1)
    _bus(ax, 114, my + 0.6, "I\u00b2C \u2192 0x42", color=ACC)
    _bus(ax, 114, my - 0.6, "CRC-16 frames", color=ACC, va="top")
    ax.text(95.5, my - 8.4, "Mega GPS fields: empty all flight", fontsize=5.7, color=MUTED, ha="center", va="center")
    _box(ax, 122, 70.5, 21, 9, "Ultimate GPS", "on the logger shield")
    _wire(ax, [(132.5, 70.5), (132.5, my + 6)]); _bus(ax, 133.6, (70.5 + my + 6) / 2, "UART", ha="left", va="center")
    _box(ax, 122, 39, 21, 9, "SD card", "SCI_000.CSV")
    _wire(ax, [(132.5, my - 6), (132.5, 48)]); _bus(ax, 133.6, (my - 6 + 48) / 2, "SPI", ha="left", va="center")
    _panel(ax, 2, 2, 96, 31, "Reaction wheel", "independent \u00b7 LiPo motor supply \u00b7 not logged")
    _box(ax, 4, 13, 13, 10, "BNO085", "IMU")
    _box(ax, 25, 13, 21, 10, "Arduino Nano", "HAB-RW 1.2.0 \u00b7 100 Hz")
    _box(ax, 56, 13, 14, 10, "G2 18v17", "motor driver")
    _box(ax, 77, 13, 18, 10, "Motor + wheel", "Pololu 25D, 20.4:1")
    _wire(ax, [(17, 18), (25, 18)]); _bus(ax, 21, 18.5, "I\u00b2C")
    _wire(ax, [(46, 18), (56, 18)]); _bus(ax, 51, 18.5, "PWM/DIR")
    _wire(ax, [(70, 18), (77, 18)])
    _wire(ax, [(86, 13), (86, 8.5), (35.5, 8.5), (35.5, 13)]); _bus(ax, 61, 8.9, "encoder A/B")
    ax.text(4, 4.2, "Telemetry: 10 Hz over USB serial only.  EEPROM keeps a summary of the last armed run.",
            fontsize=5.8, color=MUTED, va="center")
    _panel(ax, 101, 2, 45, 31, "Standalone")
    for y, t1, t2 in ((20.4, "SPOT Trace", "satellite tracker"), (12.2, "Mobius Maxi 4K", "camera \u00b7 66 min gap mid-flight"),
                      (4.0, "2 \u00d7 RunCam Thumb 2", "cameras \u00b7 off after about 2 min")):
        _box(ax, 104, y, 39, 7.2, t1, t2, align="left", ts=6.8)
    save(fig, "fig03_avionics_dataflow.png")


def fig04():
    fig, ax = plt.subplots(figsize=(7.2, 3.8))
    fx = FX[FX.t >= REL - pd.Timedelta(minutes=5)]
    Sf = S[(S.t >= REL - pd.Timedelta(minutes=5)) & (S.t <= S.t.max())]
    ax.plot(minutes(Sf.t), Sf.p_alt_m / 1000, color=LGRAY, lw=1.0, label="pressure altitude from MS8607 (estimated, fills GPS gaps)")
    t = minutes(fx.t).to_numpy(); a = fx.alt_m.to_numpy() / 1000
    brk = np.where(np.diff(fx.t).astype("timedelta64[s]").astype(float) > 60)[0]
    segs = np.split(np.arange(len(t)), brk + 1)
    for i, s in enumerate(segs):
        ax.plot(t[s], a[s], color="black", lw=1.3, label="GPS altitude (observed)" if i == 0 else None)
    for i, g in enumerate(GAPS):
        ax.plot([minutes(g["start"]), minutes(g["end"])], [g["alt_start"] / 1000, g["alt_end"] / 1000], color=GRAY, ls=":", lw=1,
                label="GPS gap (no fixes)" if i == 0 else None)
    cs, ce = minutes(pd.Timestamp(N["cam_stop_utc"])), minutes(pd.Timestamp(N["cam_resume_utc"]))
    ax.axvspan(0, cs, color=ACC, alpha=0.08, label="Mobius recording (camera clock, approx.)")
    ax.axvspan(ce, minutes(pd.Timestamp(N["spot_landing_t"])), color=ACC, alpha=0.08)
    ax.axvline(0, color=ACC, lw=1); ax.text(1, 31.5, "release\n19:42:55 UTC", fontsize=7, color=ACC, va="top")
    bm = minutes(BURST)
    ax.plot([bm], [N["burst_alt_m"] / 1000], "o", color=ACC, ms=5)
    ax.text(bm - 2, N["burst_alt_m"] / 1000 + 0.6, f"burst {N['burst_t'][11:19]} UTC\n{N['burst_alt_m']:,.0f} m ({N['burst_alt_ft']:,} ft)", fontsize=7, ha="right", color=ACC)
    ax.text(cs + 1, 2, "camera stopped\n~21:22 UTC", fontsize=6.5, color=ACC)
    ax.text(ce - 1, 2, "resumed\n~22:28", fontsize=6.5, color=ACC, ha="right")
    ax.set_xlabel("Time after release (min)"); ax.set_ylabel("Altitude, MSL (km)")
    ax.set_ylim(0, 33); ax.set_xlim(-5, 185)
    ax2 = ax.secondary_yaxis("right", functions=(lambda k: k * 1000 * C.FT_PER_M / 1000, lambda f: f * 1000 / C.FT_PER_M / 1000))
    ax2.set_ylabel("Altitude (thousand ft)")
    ax.legend(loc="upper left", bbox_to_anchor=(0.0, -0.18), ncol=2, frameon=False)
    ax.grid(alpha=0.3)
    save(fig, "fig04_altitude_time.png")


# ascent/descent rate in 500 m bins; sparse bins dropped
def rate_profile(fx, bin_m=500, min_n=8):
    d = fx.copy()
    d["dt"] = d.t.diff().dt.total_seconds()
    d["dz"] = d.alt_m.diff()
    d = d[(d.dt > 0) & (d.dt < 5)]
    d["bin"] = (d.alt_m // bin_m) * bin_m
    g = d.groupby("bin").agg(dz=("dz", "sum"), dt=("dt", "sum"), n=("dz", "size"))
    g = g[g.n >= min_n]
    return g.index.to_numpy() / 1000 + bin_m / 2000, (g.dz / g.dt).to_numpy()


def plot_profile(ax, x, h, bin_km=0.5, **kw):
    first = True
    start = 0
    for i in range(1, len(h) + 1):
        if i == len(h) or h[i] - h[i - 1] > bin_km * 1.5:
            ax.plot(x[start:i], h[start:i], marker=".", ms=3, label=kw.get("label") if first else None,
                    color=kw.get("color", "black"), lw=kw.get("lw", 1.3))
            first = False; start = i


def fig05():
    fig, axs = plt.subplots(1, 2, figsize=(7.2, 3.6), sharey=True)
    asc = FX[(FX.t >= REL) & (FX.t <= BURST)]
    h, r = rate_profile(asc)
    plot_profile(axs[0], r, h)
    axs[0].axvline(5.0, color=ACC, ls="--", lw=1); axs[0].text(5.1, 31, "5.0 m/s\n(prediction\nassumption)", fontsize=7, color=ACC, va="top")
    axs[0].axvline(N["ascent_mean_rate_mps"], color=GRAY, ls=":", lw=1)
    axs[0].text(N["ascent_mean_rate_mps"] + 0.1, 12.5, f"mean\n{N['ascent_mean_rate_mps']} m/s", fontsize=7, color=GRAY, ha="left", va="center")
    axs[0].axhspan(10.49, 21.0, color=ACC, alpha=0.07); axs[0].text(0.2, 15.5, "GPS gap", fontsize=7, color=ACC)
    axs[0].set_xlabel("Ascent rate (m/s)"); axs[0].set_ylabel("GPS altitude, MSL (km)"); axs[0].set_xlim(0, 6.5)
    axs[0].set_title("Ascent (500 m bins)")
    des = FX[FX.t > BURST]
    h, r = rate_profile(des)
    plot_profile(axs[1], -r, h)
    axs[1].axvline(7.0, color=ACC, ls="--", lw=1); axs[1].text(7.5, 12, "7.0 m/s\n(prediction\nassumption)", fontsize=7, color=ACC, va="top")
    axs[1].set_xlabel("Descent rate (m/s)"); axs[1].set_xlim(0, 50); axs[1].set_title("Descent (500 m bins, gaps left open)")
    for ax in axs:
        ax.grid(alpha=0.3); ax.set_ylim(0, 32)
    save(fig, "fig05_rate_vs_altitude.png")


def matched():
    m = pd.merge_asof(S.sort_values("t"), FX[["t", "alt_m"]].sort_values("t"), on="t", tolerance=pd.Timedelta("2s"))
    return m[m.alt_m.notna() & (m.t >= REL)]


def fig06():
    m = matched()
    asc = m[m.t <= BURST]; des = m[m.t > BURST]
    fig, ax = plt.subplots(figsize=(7.2, 4.0))
    h = np.linspace(0, 32000, 400)
    ax.plot([C.isa_pressure_hpa(C.geopotential_m(x)) for x in h], h / 1000, color=ACC, lw=1.2, label="1976 U.S. Standard Atmosphere")
    ax.plot(asc.ms_p_hpa, asc.alt_m / 1000, ".", color="black", ms=2, label="MS8607, ascent")
    ax.plot(des.ms_p_hpa, des.alt_m / 1000, ".", color=LGRAY, ms=2, label="MS8607, descent")
    ax.plot(asc.bmp_p_hpa, asc.alt_m / 1000, "x", color=GRAY, ms=3, mew=0.6, label="BMP390, ascent (stops tracking below about 100 hPa)")
    ax.set_xscale("log"); ax.set_xlabel("Pressure (hPa)"); ax.set_ylabel("GPS altitude, MSL (km)")
    ax.set_xlim(8, 1100); ax.set_ylim(0, 32); ax.grid(alpha=0.3, which="both")
    ax.annotate(f"MS8607 minimum {N['ms_p_min_hpa']} hPa at 22:08:57 UTC", xy=(N["ms_p_min_hpa"], N["burst_alt_m"] / 1000), xytext=(11, 17),
                fontsize=7, arrowprops=dict(arrowstyle="-", color=GRAY))
    ax.legend(frameon=False, loc="lower left")
    save(fig, "fig06_pressure_vs_altitude.png")


def fig07():
    fig, ax = plt.subplots(figsize=(7.2, 4.3))
    asc = S[(S.t >= REL) & (S.t <= BURST)]; des = S[(S.t > BURST) & (S.t <= pd.Timestamp(N["spot_landing_t"]))]
    ax.plot(asc.t_mcp3_c, asc.p_alt_m / 1000, color="black", lw=1.2, label="exterior, MCP9808 #3, ascent")
    ax.plot(des.t_mcp3_c, des.p_alt_m / 1000, color="black", lw=1.2, ls="--", label="exterior, MCP9808 #3, descent")
    ax.plot(asc.t_tmp119_c, asc.p_alt_m / 1000, color=GRAY, lw=0.9, label="exterior, TMP119, ascent")
    ax.plot(asc.t_mcp1_c, asc.p_alt_m / 1000, color=ACC, lw=1.2, label="interior air, MCP9808 #1, ascent")
    ax.plot(des.t_mcp1_c, des.p_alt_m / 1000, color=ACC, lw=1.2, ls="--", label="interior air, MCP9808 #1, descent")
    h = np.linspace(0, 32000, 300)
    ax.plot([C.isa_temperature_c(C.geopotential_m(x)) for x in h], h / 1000, color=LGRAY, lw=2, label="standard atmosphere", zorder=0)
    ax.plot([N["t_ext_asc_min_c"]], [N["t_ext_asc_min_palt_m"] / 1000], "o", color="black", ms=5)
    ax.annotate(f"ascent min {N['t_ext_asc_min_c']:.1f} C",
                xy=(N["t_ext_asc_min_c"], N["t_ext_asc_min_palt_m"] / 1000), xytext=(-59, 3.6), fontsize=7, ha="left",
                arrowprops=dict(arrowstyle="-", color="#444", lw=0.6))
    ax.plot([N["t_ext_min_c"]], [N["t_ext_min_palt_m"] / 1000], "o", color="black", ms=5, mfc="white")
    ax.annotate(f"descent min {N['t_ext_min_c']:.1f} C",
                xy=(N["t_ext_min_c"], N["t_ext_min_palt_m"] / 1000), xytext=(-59, 7.0), fontsize=7, ha="left",
                arrowprops=dict(arrowstyle="-", color="#444", lw=0.6))
    ax.text(14, 29.5, "exterior sensors read above\n0 C near burst on ascent:\nsolar heating, see text", fontsize=7, color="#444", va="top")
    ax.set_xlabel("Temperature (C)"); ax.set_ylabel("Pressure altitude from MS8607 (km, estimated)")
    ax.set_xlim(-65, 45); ax.set_ylim(0, 32); ax.grid(alpha=0.3)
    ax.legend(frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=3, fontsize=7)
    save(fig, "fig07_temperature_vs_altitude.png")


def fig08():
    a = FX[(FX.t >= REL) & (FX.t <= BURST)].copy()
    a["dt"] = a.t.diff().dt.total_seconds()
    a["dist"] = [0.0] + [C.haversine_m(a.lat.iloc[i - 1], a.lon.iloc[i - 1], a.lat.iloc[i], a.lon.iloc[i]) for i in range(1, len(a))]
    a = a[(a.dt > 0) & (a.dt < 5)]
    a["v"] = a.dist / a.dt
    a["bin"] = (a.alt_m // 500) * 500
    g = a.groupby("bin").v.agg(["median", lambda x: x.quantile(0.25), lambda x: x.quantile(0.75), "size"])
    g.columns = ["med", "q1", "q3", "n"]; g = g[g.n >= 20]
    g = g.reindex(np.arange(g.index.min(), g.index.max() + 500, 500))
    fig, ax = plt.subplots(figsize=(7.2, 3.8))
    h = g.index / 1000 + 0.25
    ax.fill_betweenx(h, g.q1, g.q3, where=g.med.notna().values, color=LGRAY, label="25th to 75th percentile within bin")
    ax.plot(g.med, h, color="black", lw=1.3, marker=".", ms=3, label="median horizontal speed (500 m bins)")
    ax.axhspan(10.49, 21.0, color=ACC, alpha=0.07); ax.text(1, 15.5, "GPS gap: no wind data\n10.5 to 21.0 km", fontsize=7, color=ACC)
    ax.set_xlabel("Horizontal speed over ground (m/s)"); ax.set_ylabel("GPS altitude, MSL (km)")
    ax.set_ylim(0, 32); ax.set_xlim(0, 30); ax.grid(alpha=0.3); ax.legend(frameon=False, loc="lower right")
    save(fig, "fig08_wind_vs_altitude.png")


def scale_bar(ax, lat, km=20):
    x0, x1 = ax.get_xlim(); y0, y1 = ax.get_ylim()
    dlon = km / (111.32 * np.cos(np.radians(lat)))
    xs = x0 + 0.05 * (x1 - x0); ys = y0 + 0.06 * (y1 - y0)
    ax.plot([xs, xs + dlon], [ys, ys], color="black", lw=2)
    ax.text(xs + dlon / 2, ys + 0.015 * (y1 - y0), f"{km} km", ha="center", fontsize=7)


def plot_track(ax, lat, lon, t, color="black", lw=1.2, label=None):
    brk = np.where(np.diff(t).astype("timedelta64[s]").astype(float) > 60)[0]
    first = True
    for s in np.split(np.arange(len(lat)), brk + 1):
        ax.plot(lon[s], lat[s], color=color, lw=lw, label=label if first else None); first = False


def fig09():
    fig, axs = plt.subplots(2, 1, figsize=(7.2, 6.6))
    ax = axs[0]
    plot_track(ax, F1.latitude.to_numpy(), F1.longitude.to_numpy(), F1.t.to_numpy(), label="GPS fixes (observed)")
    dt = F1.t.diff().dt.total_seconds()
    for i in F1.index[dt > 60]:
        ax.plot([F1.longitude[i - 1], F1.longitude[i]], [F1.latitude[i - 1], F1.latitude[i]], color=GRAY, ls=":", lw=1,
                label="GPS gap, straight connector" if i == F1.index[dt > 60][0] else None)
    ax.plot(N["f1_launch_lon"], N["f1_launch_lat"], "^", color=ACC, ms=7, label="launch")
    ax.plot(N["f1_peak_lon"], N["f1_peak_lat"], "o", color=ACC, ms=6, label="peak altitude (burst)")
    ax.plot(N["f1_landing_lon"], N["f1_landing_lat"], "s", color=ACC, ms=6, label="last fix (landing)")
    ax.set_title(f"Flight 1, Aug 7, 2026: {N['f1_straight_km']} km straight-line", fontsize=8.5)
    ax.set_aspect(1 / np.cos(np.radians(34.7))); ax.grid(alpha=0.3)
    ax.set_xlabel("Longitude (deg)"); ax.set_ylabel("Latitude (deg)")
    ax.legend(frameon=False, fontsize=6.5, loc="center left", bbox_to_anchor=(1.01, 0.5))
    scale_bar(ax, 34.7, 25)
    ax = axs[1]
    fx = FX[FX.t >= REL]
    plot_track(ax, fx.lat.to_numpy(), fx.lon.to_numpy(), fx.t.to_numpy(), label="GPS fixes (observed)")
    for i, g in enumerate(GAPS):
        ax.plot([g["lon_start"], g["lon_end"]], [g["lat_start"], g["lat_end"]], color=GRAY, ls=":", lw=1, label="GPS gap, straight connector" if i == 0 else None)
    sp = K["spot"]; sp = sp[(sp.t >= REL) & (sp.t <= pd.Timestamp(N["spot_landing_t"]))]
    ax.plot(sp.lon, sp.lat, "o", mfc="white", mec=ACC, ms=4, mew=0.9, label="SPOT Trace fixes (observed, ~5 min)")
    ax.plot(N["launch_lon"], N["launch_lat"], "^", color=ACC, ms=7, label="launch")
    ax.plot(N["burst_lon"], N["burst_lat"], "o", color=ACC, ms=6, label="burst")
    ax.plot(N["spot_landing_lon"], N["spot_landing_lat"], "s", color=ACC, ms=6, label="landing")
    ax.set_title(f"Flight 2, Oct 28, 2026: {N['f2_straight_km']} km straight-line", fontsize=8.5)
    ax.set_aspect(1 / np.cos(np.radians(35.0))); ax.grid(alpha=0.3)
    ax.set_xlabel("Longitude (deg)"); ax.set_ylabel("Latitude (deg)")
    ax.legend(frameon=False, fontsize=6.5, loc="center left", bbox_to_anchor=(1.01, 0.5))
    scale_bar(ax, 35.0, 25)
    fig.tight_layout()
    save(fig, "fig09_ground_tracks.png")


def fig10():
    fig, ax = plt.subplots(figsize=(7.2, 4.6))
    pp = K["pred_path"]
    ax.plot(pp.lon, pp.lat, color=ACC, lw=1.4, ls="--", label="predicted path, 19:44 run (5.0 m/s up, 7.0 m/s down, burst 32,000 m)")
    pb, pl = K["pred_burst"], K["pred_landing"]
    ax.plot(pb["lon"], pb["lat"], "o", mfc="white", mec=ACC, ms=7, mew=1.5, label="predicted burst / landing")
    ax.plot(pl["lon"], pl["lat"], "s", mfc="white", mec=ACC, ms=7, mew=1.5)
    fx = FX[FX.t >= REL]
    plot_track(ax, fx.lat.to_numpy(), fx.lon.to_numpy(), fx.t.to_numpy(), label="observed GPS track")
    for i, g in enumerate(GAPS):
        ax.plot([g["lon_start"], g["lon_end"]], [g["lat_start"], g["lat_end"]], color=GRAY, ls=":", lw=1, label="GPS gap" if i == 0 else None)
    sp = K["spot"]; sp = sp[(sp.t >= REL) & (sp.t <= pd.Timestamp(N["spot_landing_t"]))]
    ax.plot(sp.lon, sp.lat, "o", mfc="white", mec="black", ms=3.5, mew=0.8, label="SPOT fixes")
    ax.plot(N["burst_lon"], N["burst_lat"], "o", color="black", ms=6, label="actual burst / landing")
    ax.plot(N["spot_landing_lon"], N["spot_landing_lat"], "s", color="black", ms=6)
    ax.plot(N["launch_lon"], N["launch_lat"], "^", color="black", ms=7)
    ax.annotate("", xy=(N["spot_landing_lon"], N["spot_landing_lat"]), xytext=(pl["lon"], pl["lat"]), arrowprops=dict(arrowstyle="<->", color=GRAY))
    ax.text((pl["lon"] + N["spot_landing_lon"]) / 2 + 0.02, (pl["lat"] + N["spot_landing_lat"]) / 2 + 0.035, f"landing miss {N['pred_landing_vs_spot_km']} km", fontsize=7.5, ha="left")
    ax.annotate("", xy=(N["burst_lon"], N["burst_lat"]), xytext=(pb["lon"], pb["lat"]), arrowprops=dict(arrowstyle="<->", color=GRAY))
    ax.text((pb["lon"] + N["burst_lon"]) / 2 + 0.02, (pb["lat"] + N["burst_lat"]) / 2 - 0.05, f"burst miss {N['pred_burst_vs_actual_km']} km", fontsize=7.5, ha="left")
    ax.set_aspect(1 / np.cos(np.radians(35.0))); ax.grid(alpha=0.3)
    ax.set_xlabel("Longitude (deg)"); ax.set_ylabel("Latitude (deg)")
    ax.legend(frameon=False, fontsize=6.5, loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=2)
    scale_bar(ax, 35.0, 25)
    save(fig, "fig10_prediction_vs_actual.png")


def fig11():
    m = matched(); asc = m[m.t <= BURST]
    fig, axs = plt.subplots(1, 2, figsize=(7.2, 3.4), sharey=True)
    axs[0].plot(asc.veml_lux / 1000, asc.alt_m / 1000, ".", color="black", ms=1.5)
    axs[0].set_xlabel("VEML7700 illuminance (klux)"); axs[0].set_ylabel("GPS altitude, MSL (km)")
    axs[1].plot(asc.ltr_uvs_raw, asc.alt_m / 1000, ".", color="black", ms=1.5)
    axs[1].set_xlabel("LTR390 UV (raw counts)")
    for ax in axs:
        ax.axhspan(10.49, 21.0, color=ACC, alpha=0.07); ax.grid(alpha=0.3); ax.set_ylim(0, 32)
    axs[0].text(1, 15.5, "GPS gap", fontsize=7, color=ACC)
    fig.suptitle("Top-facing light sensors, ascent only; both sensors turn with the payload", fontsize=8.5)
    save(fig, "fig11_light_uv_vs_altitude.png")


def fig12():
    fig, axs = plt.subplots(2, 1, figsize=(7.2, 5.2), gridspec_kw=dict(height_ratios=[1.2, 1], hspace=0.45))
    Sf = S[(S.t >= REL - pd.Timedelta(minutes=3)) & (S.t <= pd.Timestamp(N["spot_landing_t"]) + pd.Timedelta(minutes=1))]
    ax = axs[0]
    ax.plot(minutes(Sf.t), Sf.amag, color="black", lw=0.7)
    ax.axvline(0, color=ACC, lw=1); ax.axvline(minutes(BURST), color=ACC, lw=1)
    ax.text(0.8, 2.45, f"release jolt {N['release_jolt_g']} g", fontsize=7, color=ACC)
    ax.text(minutes(BURST) - 1, 2.45, "burst", fontsize=7, color=ACC, ha="right")
    cs = minutes(pd.Timestamp(N["shake_utc"]))
    ax.axvline(cs, color=GRAY, ls="--", lw=1); ax.text(cs + 1, 2.1, f"camera shake (~{N['shake_utc'][11:16]} UTC):\nmax {N['acc_shake_window_max_g']} g in that window", fontsize=7, color="#444")
    ax.set_ylim(0, 2.7); ax.set_ylabel("|a| (g)"); ax.set_xlabel("Time after release (min)"); ax.grid(alpha=0.3)
    ax.set_xlim(-3, 185)
    ax = axs[1]
    z = S[(S.t >= BURST - pd.Timedelta(minutes=1)) & (S.t <= BURST + pd.Timedelta(minutes=5))]
    ax.plot((z.t - BURST).dt.total_seconds(), z.amag, color="black", lw=0.9, marker=".", ms=2)
    ax.axvline(0, color=ACC, lw=1)
    ax.set_xlabel(f"Seconds after burst ({N['burst_t'][11:19]} UTC)"); ax.set_ylabel("|a| (g)"); ax.grid(alpha=0.3); ax.set_ylim(0, 2.7)
    ax.text(3, 0.12, "near 0 g for a few\nseconds: free fall", fontsize=7, color="#444")
    ax.text(5, 2.45, f"peak {N['acc_post_burst_max_g']} g at {N['acc_post_burst_max_t'][11:]} UTC; values above 1.5 g from {N['acc_post_burst_first_t'][11:]} to {N['acc_post_burst_last_t'][11:]}", fontsize=7)
    save(fig, "fig12_accel_magnitude.png")


def fig13():
    R = C.load_rotation()
    fig, ax = plt.subplots(figsize=(7.2, 3.4))
    ax.axhspan(40, 52, color=ACC, alpha=0.08)
    ax.text(97, 46, "40 to 52 s band", fontsize=7, color=ACC, va="center")
    has = R[R.repeat_period_s.notna()]; none = R[R.repeat_period_s.isna()]
    ax.scatter(has.t_min, has.repeat_period_s, s=15 + 120 * (has.repeat_strength - 0.45), color="black", zorder=3,
               label="view-repeat period found (marker size = repeat strength)")
    ax.scatter(none.t_min, np.zeros(len(none)), marker="x", color=GRAY, s=25, label="window with no repeat found")
    ax.set_xlabel("Window center, minutes after camera clock 12:58:04 (within about 1 min of release)")
    ax.set_ylabel("Repeat period (s)"); ax.set_ylim(-3, 62); ax.set_xlim(-2, 100); ax.grid(alpha=0.3)
    ax.legend(frameon=False, loc="upper right", fontsize=7)
    ax.set_title("View-repeat method only. Frame-to-frame tracking failed in every window and is not used.", fontsize=8.5)
    save(fig, "fig13_rotation_repeat_period.png")


def fig14():
    a = Image.open(C._find("Flight_2__4_min.png")); b = Image.open(C._find("Flight_2__14_min.png"))
    w = a.width + b.width + 40; h = max(a.height, b.height)
    im = Image.new("RGB", (w, h), "white"); im.paste(a, (0, 0)); im.paste(b, (a.width + 40, 0))
    im.save(OUT / "fig14_tracker_example_4min_14min.png", dpi=(300, 300))
    print("wrote fig14_tracker_example_4min_14min.png")


if __name__ == "__main__":
    for f in [fig01, fig02, fig03, fig04, fig05, fig06, fig07, fig08, fig09, fig10, fig11, fig12, fig13, fig14]:
        f()
