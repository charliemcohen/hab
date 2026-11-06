# payload yaw rate from onboard video. needs numpy, matplotlib, ffmpeg on PATH
# two estimates: phase-corr dx (needs hfov) and view-repeat period (hfov-free)

import argparse
import csv
import os
import shutil
import subprocess
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

W, H = 320, 180  # analysis res; 4k is overkill for dx


def parse_time(s):
    s = str(s).strip()
    if s.startswith("-"):  # release before the footage starts
        return -parse_time(s[1:])
    parts = s.split(":")
    try:
        vals = [float(p) for p in parts]
    except ValueError:
        sys.exit(f"Can't read the time '{s}'. Use h:mm:ss, mm:ss, or seconds.")
    secs = 0.0
    for v in vals:
        secs = secs * 60 + v
    return secs


def fmt(t):
    t = max(t, 0)
    return f"{int(t // 3600)}:{int(t % 3600 // 60):02d}:{t % 60:04.1f}"


def src(video):
    return ["-f", "concat", "-safe", "0", "-i", video] if video.lower().endswith(".txt") else ["-i", video]  # clip list = one timeline


def read_frames(video, start, dur, fps):
    base = ["ffmpeg", "-nostdin", "-loglevel", "error"]
    tail = ["-ss", f"{max(start, 0):.3f}", *src(video), "-t", f"{dur:.3f}",
            "-vf", f"fps={fps},scale={W}:{H},format=gray",
            "-f", "rawvideo", "-pix_fmt", "gray", "-"]
    out = subprocess.run(base + ["-hwaccel", "auto"] + tail, capture_output=True)  # gpu decode if present
    if out.returncode != 0 or not out.stdout:
        out = subprocess.run(base + tail, capture_output=True)
    if out.returncode != 0:
        raise RuntimeError(out.stderr.decode(errors="replace").strip())
    n = len(out.stdout) // (W * H)
    return np.frombuffer(out.stdout[: n * W * H], np.uint8).reshape(n, H, W).astype(np.float32)


def _hann2d():
    return np.outer(np.hanning(H), np.hanning(W)).astype(np.float32)


def _parabolic(cm, c0, cp):
    d = cm - 2 * c0 + cp
    return 0.0 if d == 0 else 0.5 * (cm - cp) / d


def frame_shifts(frames):
    win = _hann2d()
    dxs, dys, qual = [], [], []
    a = np.fft.fft2((frames[0] - frames[0].mean()) * win)
    for frame in frames[1:]:
        b = np.fft.fft2((frame - frame.mean()) * win)
        R = b * np.conj(a)
        a = b
        mag = np.abs(R)
        # whiten only above noise floor, else small-shift bias
        R = np.where(mag > 1e-3 * mag.max(), R / np.maximum(mag, 1e-30), 0)
        R[:, 0] = 0  # kx=0 is blind to yaw (sky gradient) -> pulls dx to 0
        r = np.fft.ifft2(R).real
        iy, ix = np.unravel_index(np.argmax(r), r.shape)
        sx = _parabolic(r[iy, (ix - 1) % W], r[iy, ix], r[iy, (ix + 1) % W])
        sy = _parabolic(r[(iy - 1) % H, ix], r[iy, ix], r[(iy + 1) % H, ix])
        dx = ix + sx
        dy = iy + sy
        if dx > W / 2:
            dx -= W
        if dy > H / 2:
            dy -= H
        dxs.append(dx)
        dys.append(dy)
        qual.append((r[iy, ix] - r.mean()) / (r.std() + 1e-12))  # PSR, <10 ~ noise
    return np.array(dxs), np.array(dys), np.array(qual)


def signatures(frames):
    n = len(frames)
    # column stats only -> immune to pendulum bob in y
    strips = frames.reshape(n, H, 40, W // 40)
    f = np.concatenate([strips.mean(axis=(1, 3)), strips.std(axis=(1, 3))], axis=1)
    f = f - f.mean(axis=1, keepdims=True)
    f /= np.linalg.norm(f, axis=1, keepdims=True) + 1e-9
    return f


def lag_curve(sig):
    n = len(sig)
    maxlag = (2 * n) // 3
    C = np.ones(maxlag)
    for L in range(1, maxlag):
        C[L] = np.mean(np.sum(sig[:-L] * sig[L:], axis=1))
    return C


def find_period(C, fps, lo_s=None, hi_s=None):
    if len(C) < 5:
        return None, 0.0
    c_min = C[1:].min()
    span = 1.0 - c_min
    if span < 0.02:
        return None, 0.0
    below = np.where(C < 1.0 - 0.5 * span)[0]
    if len(below) == 0:
        return None, 0.0
    first = below[0] + 1
    if lo_s is not None:
        first = max(first, int(np.floor(lo_s * fps)))
    last = len(C) - 1
    if hi_s is not None:
        last = min(last, int(np.ceil(hi_s * fps)) + 1)
    for L in range(max(first, 1), last):
        if C[L] >= C[L - 1] and C[L] >= C[L + 1]:
            strength = (C[L] - c_min) / span
            if strength > 0.45:
                L_ref = L + _parabolic(C[L - 1], C[L], C[L + 1])
                return L_ref / fps, float(strength)
    return None, 0.0


def analyze_window(video, center, args, name, t_min=None):
    fr = args.fps / args.speed  # samples per REAL second; all time math below is real time
    win_v = args.window / args.speed
    start = center - win_v / 2
    if start < 0:  # window hangs off the start of the footage: keep only the real part
        win_v, start = win_v + start, 0.0
    frames = read_frames(video, start, win_v, args.fps) if win_v > 0 else np.zeros((0, H, W), np.float32)
    if len(frames) < 20:
        return {"window": name, "center": fmt(center), "t_min": t_min, "result": "not enough video at this time"}

    dur = (len(frames) - 1) / fr
    dx, dy, qual = frame_shifts(frames)
    net = dx.sum()
    total = np.abs(dx).sum()
    ratio = abs(net) / total if total > 0 else 0.0
    weak = float(np.mean(qual < 10))

    C = lag_curve(signatures(frames))

    deg_per_px_fov = args.hfov / W
    t = np.arange(len(dx) + 1) / fr
    heading_fov = np.concatenate([[0.0], np.cumsum(dx)]) * deg_per_px_fov
    swing_fov = heading_fov - np.polyval(np.polyfit(t, heading_fov, 1), t)
    net_deg = abs(net) * deg_per_px_fov

    # |net|/sum|dx|: 1 = one-way spin, 0 = pure swing
    if ratio > 0.7 and net_deg >= 30:
        motion = "rotating"
    elif net_deg < 30 and (swing_fov.max() - swing_fov.min()) / 2 < 10:
        motion = "nearly steady"
    elif ratio < 0.3:
        motion = "swinging"
    else:
        motion = "mixed"

    rpm_flow = (net / dur) * deg_per_px_fov / 6.0

    if motion == "rotating":
        px_per_s = abs(net) / dur
        period, strength = find_period(C, fr,
                                       lo_s=360.0 / (px_per_s * 170.0 / W),  # only search periods a 70-170 deg lens allows
                                       hi_s=360.0 / (px_per_s * 70.0 / W))
    else:
        period, strength = find_period(C, fr)
    rpm_period = None
    implied_hfov = None
    deg_per_px = deg_per_px_fov
    if motion == "rotating" and period:
        rpm_period = 60.0 / period
        px_per_rev = abs(net / dur) * period
        if px_per_rev > 0:
            deg_per_px = 360.0 / px_per_rev  # self-cal: one rev = 360 deg
            implied_hfov = deg_per_px * W

    heading = np.concatenate([[0.0], np.cumsum(dx)]) * deg_per_px
    detr = heading - np.polyval(np.polyfit(t, heading, 1), t)
    swing_amp = (detr.max() - detr.min()) / 2
    # swing period = dominant freq of detrended heading
    spec = np.abs(np.fft.rfft(detr * np.hanning(len(detr))))
    freqs = np.fft.rfftfreq(len(detr), 1.0 / fr)
    swing_period = None
    if len(spec) > 3:
        k = 1 + int(np.argmax(spec[1:-1]))
        k_ref = k + _parabolic(spec[k - 1], spec[k], spec[k + 1])
        if k_ref > 0:
            swing_period = 1.0 / (k_ref * (freqs[1] - freqs[0]))

    notes = []
    if motion == "rotating":
        if rpm_period is not None:
            best = rpm_period
            method = "repeat period (field-of-view independent)"
            if not (70 <= implied_hfov <= 170):
                notes.append(f"implied field of view {implied_hfov:.0f} deg looks wrong; "
                             "check the plot and count this window by hand")
        else:
            best = abs(rpm_flow)
            method = f"frame motion, assumes hfov={args.hfov:.0f} deg"
            notes.append("no full repeat in this window (slow spin?); try --window 120")
    elif motion == "nearly steady":
        best = abs(rpm_flow)
        method = f"frame motion, assumes hfov={args.hfov:.0f} deg"
    else:
        best = abs(rpm_flow)
        method = f"net drift from frame motion, assumes hfov={args.hfov:.0f} deg"
    if weak > 0.3:
        notes.append(f"{weak:.0%} of frame pairs matched poorly (glare, blur, or tumbling)")

    direction = "scene drifts left" if net < 0 else "scene drifts right"
    if motion == "rotating" and rpm_period is not None:
        yaw = np.sign(net) * rpm_period * 6.0
    else:
        yaw = (net / dur) * deg_per_px_fov  # + = scene drifts right

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6.5))
    ax1.plot(t, heading, lw=1.2)
    ax1.set_xlabel("Time in window (s)")
    ax1.set_ylabel("Heading change (deg)")
    summary = f"{best:.2f} rot/min" if motion == "rotating" else f"+/-{swing_amp:.0f} deg swing"
    ax1.set_title(f"{args.label}: {name} (video {fmt(center)})  ->  {motion}, {summary}")
    ax1.grid(alpha=0.3)
    lags = np.arange(len(C)) / fr
    ax2.plot(lags, C, lw=1.2)
    if period:
        ax2.axvline(period, ls="--", color="tab:red", label=f"repeat at {period:.1f} s")
        ax2.legend()
    ax2.set_xlabel("Time lag (s)")
    ax2.set_ylabel("View similarity")
    ax2.set_title("How similar the view is to itself after each time lag")
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    safe = "".join(ch if ch.isalnum() else "_" for ch in f"{args.label}_{name}")
    png = os.path.join(args.out, f"{safe}.png")
    fig.savefig(png, dpi=150)
    plt.close(fig)

    return {
        "window": name,
        "center": fmt(center),
        "t_min": t_min,
        "result": motion,
        "yaw_rate_dps": round(float(yaw), 3),
        "fov_dependent": not (motion == "rotating" and rpm_period is not None),
        "rot_per_min": round(best, 2),
        "method": method,
        "repeat_period_s": round(period, 2) if period else "",
        "repeat_strength": round(strength, 2) if period else "",
        "rpm_from_motion": round(abs(rpm_flow), 2),
        "implied_hfov_deg": round(implied_hfov, 1) if implied_hfov else "",
        "swing_amplitude_deg": round(swing_amp, 1),
        "swing_period_s": round(swing_period, 1) if (swing_period and motion != "rotating") else "",
        "direction": direction,
        "notes": "; ".join(notes),
        "plot": png,
    }


def main():
    p = argparse.ArgumentParser(description="Measure payload rotation from flight video.")
    p.add_argument("video", help="video file (mp4, mov, ...) or a list.txt of clips from check_clips.py")
    p.add_argument("--label", default="Flight", help='e.g. "Flight 1" or "Flight 2"')
    p.add_argument("--launch", help="video time of release (h:mm:ss)")
    p.add_argument("--minutes", type=float, nargs="+", default=[5, 20, 40],
                   help="minutes after launch to check (default: 5 20 40)")
    p.add_argument("--burst", help="video time of burst (h:mm:ss); adds one window just after it")
    p.add_argument("--at", nargs="+", help="video times to check instead (each is a window's middle)")
    p.add_argument("--scan", type=float, nargs=3, metavar=("START", "END", "STEP"),
                   help="minutes after launch: one window every STEP min from START to END (needs --launch)")
    p.add_argument("--window", type=float, default=None,
                   help="window length in seconds (default 60, or 90 with --scan)")
    p.add_argument("--speed", type=float, default=1.0,
                   help="playback speed of the file vs real time, e.g. 12 for a 12x timelapse. "
                        "--launch/--burst/--at are times in the file; --scan/--minutes/--window are real time")
    p.add_argument("--ascent-time", help="real release-to-burst time from GPS (h:mm:ss); checks --speed")
    p.add_argument("--fps", type=float, default=None,
                   help="frames sampled per second of the FILE (default 10, or 5 x speed for a timelapse)")
    p.add_argument("--hfov", type=float, default=120,
                   help="camera horizontal field of view in degrees, used only when the "
                        "repeat method can't run (default 120; check your lens spec)")
    p.add_argument("--out", default="rotation_results", help="output folder")
    args = p.parse_args()
    if args.window is None:
        args.window = 90.0 if args.scan else 60.0  # 90 s still catches a full turn at ~1 rot/min

    if shutil.which("ffmpeg") is None:
        sys.exit("ffmpeg isn't installed or isn't on your PATH. See the top of this file.")
    if not os.path.exists(args.video):
        sys.exit(f"Can't find the video: {args.video}")
    os.makedirs(args.out, exist_ok=True)

    native = None
    if shutil.which("ffprobe"):
        pr = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
                             "stream=avg_frame_rate", "-of", "csv=p=0", *src(args.video)], capture_output=True, text=True)
        try:
            num, den = pr.stdout.strip().split("/")
            native = float(num) / float(den)
        except (ValueError, ZeroDivisionError):
            native = None
    if args.fps is None:
        args.fps = 5.0 * args.speed if args.speed > 1 else 10.0  # ~5 samples per real s; denser buries dx in noise
    if native:
        args.fps = min(args.fps, native)
    print(f"file fps {native or 'unknown'}, sampling {args.fps:g} fps of the file "
          f"= one sample every {args.speed / args.fps:.2f} s of real time", flush=True)

    windows = []
    t0 = parse_time(args.launch) if args.launch else None
    if args.ascent_time and args.burst and t0 is not None:
        implied = parse_time(args.ascent_time) / (parse_time(args.burst) - t0)
        print(f"speed check: release-to-burst in the file implies {implied:.2f}x (using {args.speed:g}x)")
        if abs(implied / args.speed - 1) > 0.03:
            print("  WARNING: >3% mismatch. footage gap, wrong times, or wrong --speed. every rate scales with this.")
    if args.at:
        windows = [(f"video {a}", parse_time(a), None) for a in args.at]
    elif args.scan:
        if t0 is None:
            sys.exit("--scan needs --launch.")
        a, b, step = args.scan
        for m in np.arange(a, b + 1e-9, step):
            windows.append((f"+{m:g} min", t0 + m * 60 / args.speed, float(m)))
    elif t0 is not None:
        windows = [(f"+{m:g} min after launch", t0 + m * 60 / args.speed, float(m)) for m in args.minutes]
    else:
        sys.exit("Give --launch (with --minutes or --scan) or --at.")
    if args.burst and t0 is not None:
        c = parse_time(args.burst) + (30 + args.window / 2) / args.speed
        windows.append(("just after burst", c, (c - t0) * args.speed / 60))

    rows = []
    for name, center, t_min in windows:
        print(f"Analyzing {name} (file time {fmt(center)}) ...", flush=True)
        try:
            rows.append(analyze_window(args.video, center, args, name, t_min))
        except RuntimeError as e:
            rows.append({"window": name, "center": fmt(center), "t_min": t_min, "result": f"ffmpeg error: {e}"})

    # self-cal hfov from clean full turns, then rescale every hfov-dependent number
    cal = [float(r["implied_hfov_deg"]) for r in rows
           if r.get("result") == "rotating" and r.get("implied_hfov_deg", "") != ""
           and float(r.get("repeat_strength") or 0) >= 0.6 and 70 <= float(r["implied_hfov_deg"]) <= 170]
    hfov_cal = float(np.median(cal)) if cal else None
    if hfov_cal:
        k = hfov_cal / args.hfov
        for r in rows:
            if "rot_per_min" not in r:
                continue
            r["rpm_from_motion"] = round(r["rpm_from_motion"] * k, 2)
            if r["fov_dependent"]:
                r["rot_per_min"] = round(r["rot_per_min"] * k, 2)
                r["yaw_rate_dps"] = round(r["yaw_rate_dps"] * k, 3)
                r["swing_amplitude_deg"] = round(r["swing_amplitude_deg"] * k, 1)
                r["method"] = r["method"].replace(f"assumes hfov={args.hfov:.0f} deg",
                                                  f"hfov self-calibrated to {hfov_cal:.0f} deg")

    csv_path = os.path.join(args.out, f"{''.join(c if c.isalnum() else '_' for c in args.label)}_rotation.csv")
    keys = ["window", "center", "t_min", "result", "yaw_rate_dps", "rot_per_min", "method", "repeat_period_s",
            "repeat_strength", "rpm_from_motion", "implied_hfov_deg", "swing_amplitude_deg",
            "swing_period_s",
            "direction", "notes", "plot"]
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in keys})

    timed = sorted([r for r in rows if r.get("t_min") is not None and "yaw_rate_dps" in r], key=lambda r: r["t_min"])
    if len(timed) >= 3:
        mk = {"rotating": "o", "swinging": "s", "nearly steady": "^", "mixed": "x"}
        fig, ax = plt.subplots(figsize=(9, 4.5))
        for res, m in mk.items():
            pts = [(r["t_min"], r["yaw_rate_dps"]) for r in timed if r["result"] == res]
            if pts:
                ax.plot(*zip(*pts), m, ms=7, label=res, ls="none")
        ax.plot([r["t_min"] for r in timed], [r["yaw_rate_dps"] for r in timed], color="gray", lw=0.8, zorder=0)
        ax.axhline(0, color="black", lw=0.6)
        if args.burst and t0 is not None:
            ax.axvline((parse_time(args.burst) - t0) * args.speed / 60, ls="--", color="tab:red", label="burst")
        ax.set_xlabel("Minutes after release")
        ax.set_ylabel("Yaw rate (deg/s, + = scene drifts right)")
        sec = ax.secondary_yaxis("right", functions=(lambda d: d / 6.0, lambda r: r * 6.0))
        sec.set_ylabel("rot/min")
        cal_txt = f", hfov self-cal {hfov_cal:.0f} deg" if hfov_cal else f", hfov assumed {args.hfov:.0f} deg"
        ax.set_title(f"{args.label}: payload yaw rate from video ({args.window:.0f} s windows{cal_txt})")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        tl_png = os.path.join(args.out, f"{''.join(c if c.isalnum() else '_' for c in args.label)}_timeline.png")
        fig.savefig(tl_png, dpi=200)
        plt.close(fig)

    print(f"\n{args.label} rotation results")
    if hfov_cal:
        print(f"camera hfov self-calibrated from {len(cal)} full-turn window(s): {hfov_cal:.0f} deg")
    print("-" * 60)
    if len(timed) >= 3:
        for r in timed:
            extra = "" if r["result"] == "rotating" else f"  swing +/-{r['swing_amplitude_deg']:.0f} deg"
            print(f"  {r['t_min']:+7.1f} min  {r['result']:<13} {r['yaw_rate_dps']:+7.2f} deg/s"
                  f"  ({r['yaw_rate_dps'] / 6:+.2f} rot/min){extra}")
        for prev, cur in zip(timed, timed[1:]):
            if cur["result"] == "rotating" and prev["result"] != "rotating":
                print(f"\nspin onset between +{prev['t_min']:g} and +{cur['t_min']:g} min after release")
                break
        print(f"\nTimeline plot: {tl_png}")
    for r in ([] if len(timed) >= 3 else rows):
        if "rot_per_min" not in r:
            print(f"{r['window']:>24}: {r['result']}")
            continue
        line = f"{r['window']:>24}: {r['result']}, {r['rot_per_min']:.2f} rot/min"
        if r["result"] in ("swinging", "nearly steady", "mixed"):
            line += f" net drift, swinging about +/-{r['swing_amplitude_deg']:.0f} deg"
            if r.get("swing_period_s", "") != "":
                line += f" (period about {r['swing_period_s']:.1f} s)"
        line += f"  [{r['method']}]"
        print(line)
        if r["notes"]:
            print(f"{'':>26}note: {r['notes']}")
    print(f"\nTable: {csv_path}\nPlots: {args.out}/")
    print("Spot-check at least one window by counting by hand before using these numbers.")


if __name__ == "__main__":
    main()
