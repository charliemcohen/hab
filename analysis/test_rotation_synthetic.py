# regression test: 360 deg synthetic pano, 120 deg virtual cam, known yaw profiles

import csv
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "rotation_from_video.py")
W, H, HFOV, FPS = 640, 360, 120.0, 30
PW = int(round(W * 360 / HFOV))
PX_PER_DEG = PW / 360
rng = np.random.default_rng(1)


def make_panorama():
    y = np.arange(H)[:, None]
    pano = (20 + 60 * (y / H)) * np.ones((H, PW))
    k = np.fft.rfftfreq(PW) * PW
    spec = (rng.normal(size=k.size) + 1j * rng.normal(size=k.size)) / (1 + k ** 1.5)
    spec[0] = 0
    ridge = np.fft.irfft(spec, PW)
    ridge = 170 + 40 * ridge / np.abs(ridge).max()
    fy = np.fft.fftfreq(H)[:, None]
    fx = np.fft.fftfreq(PW)[None, :]
    tex = np.fft.ifft2(np.fft.fft2(rng.normal(size=(H, PW)))
                       * np.exp(-((fy * H) ** 2 + (fx * PW) ** 2) / (2 * 40 ** 2))).real
    tex /= tex.std()
    pano = np.where(y > ridge[None, :], 110 + 35 * tex, pano)
    yy, xx = np.mgrid[0:H, 0:PW]
    d = np.minimum(abs(xx - 600), PW - abs(xx - 600))
    pano += 200 * np.exp(-(d ** 2 + (yy - 60) ** 2) / (2 * 12 ** 2))
    return np.clip(pano, 0, 255)


def render(pano, heading_deg, bob_px=0.0, gain=1.0, noise=3.0):
    off = heading_deg * PX_PER_DEG
    i = int(np.floor(off))
    f = off - i
    cols = (i + np.arange(W)) % PW
    fr = pano[:, cols] * (1 - f) + pano[:, (cols + 1) % PW] * f
    fr = np.roll(fr, int(round(bob_px)), axis=0) * gain
    return np.clip(fr + rng.normal(0, noise, fr.shape), 0, 255).astype(np.uint8)


def write_video(path, frames, crf):
    p = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "gray",
                          "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-c:v", "libx264",
                          "-crf", str(crf), "-pix_fmt", "yuv420p", path], stdin=subprocess.PIPE)
    for fr in frames:
        p.stdin.write(fr.tobytes())
    p.stdin.close()
    p.wait()


def run(video, at, window, out, label):
    subprocess.run([sys.executable, SCRIPT, video, "--label", label, "--at", *at,
                    "--window", str(window), "--out", out], check=True, capture_output=True)
    with open(os.path.join(out, f"{label}_rotation.csv")) as f:
        return list(csv.DictReader(f))


def main():
    if shutil.which("ffmpeg") is None:
        sys.exit("ffmpeg isn't on your PATH. Install it, then open a NEW terminal and check: ffmpeg -version")
    pano = make_panorama()
    tmp = tempfile.mkdtemp()
    out = os.path.join(tmp, "results")

    def clean():
        for n in range(70 * FPS):
            yield render(pano, 6 * 6.0 * n / FPS)
        for n in range(70 * FPS):
            yield render(pano, 40 * np.sin(2 * np.pi * n / FPS / 8))
        for n in range(70 * FPS):
            yield render(pano, -1 * 6.0 * n / FPS)
        for n in range(70 * FPS):
            yield render(pano, 3 * np.sin(2 * np.pi * n / FPS / 5))

    def rough():
        # worst case: bob, exposure flicker, sigma=8 noise, crf 32
        for n in range(130 * FPS):
            t = n / FPS
            yield render(pano, 1 * 6.0 * t, 12 * np.sin(2 * np.pi * t / 3),
                         1 + 0.15 * np.sin(2 * np.pi * t / 7), noise=8)
        for n in range(70 * FPS):
            t = n / FPS
            yield render(pano, -3 * 6.0 * t, 15 * np.sin(2 * np.pi * t / 2.5),
                         1 + 0.2 * np.sin(2 * np.pi * t / 4), noise=8)

    print("Building test videos (about a minute)...", flush=True)
    v1 = os.path.join(tmp, "clean.mp4")
    write_video(v1, clean(), crf=23)
    v2 = os.path.join(tmp, "rough.mp4")
    write_video(v2, rough(), crf=32)
    print("Running rotation_from_video.py on them...", flush=True)

    r1 = run(v1, ["0:35", "1:45", "2:55", "4:05"], 60, out, "clean")
    r2 = run(v2, ["1:05"], 120, out, "rough_slow")
    r3 = run(v2, ["2:45"], 60, out, "rough_fast")

    # tol: 3% rate, 10% swing amplitude
    checks = [
        ("6 rot/min", r1[0], "rotating", 6.0, None),
        ("+/-40 deg swing", r1[1], "swinging", None, 40.0),
        ("1 rot/min reverse", r1[2], "rotating", 1.0, None),
        ("+/-3 deg wobble", r1[3], "nearly steady", None, None),
        ("1 rot/min, rough video", r2[0], "rotating", 1.0, None),
        ("3 rot/min, rough video", r3[0], "rotating", 3.0, None),
    ]
    failed = 0
    for name, row, motion, rpm, amp in checks:
        ok = row["result"] == motion
        got = f"{row['result']}"
        if rpm is not None:
            val = float(row["rot_per_min"])
            ok = ok and abs(val - rpm) / rpm < 0.03
            got += f", {val:.2f} rot/min"
        if amp is not None:
            val = float(row["swing_amplitude_deg"])
            ok = ok and abs(val - amp) / amp < 0.10
            got += f", +/-{val:.0f} deg"
        failed += not ok
        print(f"{'PASS' if ok else 'FAIL'}  {name:<24} got: {got}")
    print(f"\n{len(checks) - failed}/{len(checks)} passed. Plots in {out}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
