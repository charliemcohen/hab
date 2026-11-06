# orders camera clips by their own timestamps, flags gaps, writes list.txt for a lossless rejoin
import os
import re
import subprocess
import sys
from datetime import datetime, timedelta
from statistics import median

EXT = (".mp4", ".mov", ".avi", ".mkv")


def probe(path):
    out = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0",
                          "-show_entries", "stream=avg_frame_rate:format=duration:format_tags=creation_time",
                          "-of", "default=nw=1", path], capture_output=True, text=True).stdout
    info = dict(line.split("=", 1) for line in out.splitlines() if "=" in line)
    num, _, den = info.get("avg_frame_rate", "0/1").partition("/")
    try:
        fps = float(num) / float(den or 1)
    except (ValueError, ZeroDivisionError):
        fps = 0.0
    try:
        dur = float(info.get("duration", "nan"))
    except ValueError:
        dur = float("nan")
    ct = info.get("TAG:creation_time")
    try:
        start = datetime.fromisoformat(ct.replace("Z", "+00:00")).replace(tzinfo=None) if ct else None
    except ValueError:
        start = None
    return fps, dur, start


def hms(sec):
    return str(timedelta(seconds=round(sec))) if sec == sec else "?"


def main():
    args = sys.argv[1:]
    at = None
    if "--at" in args:
        i = args.index("--at"); at = args[i + 1]; del args[i:i + 2]
    folder = os.path.abspath(args[0] if args else ".")
    names = sorted(f for f in os.listdir(folder) if f.lower().endswith(EXT))
    if not names:
        sys.exit(f"no video files in {folder}")

    clips = []
    for f in names:
        p = os.path.join(folder, f)
        fps, dur, start = probe(p)
        mtime = datetime.fromtimestamp(os.path.getmtime(p))  # usually when the camera closed the file
        clips.append({"name": f, "path": p, "fps": fps, "dur": dur, "start": start, "mtime": mtime})

    base = median(c["fps"] for c in clips if c["fps"] > 0)
    typ = median(c["dur"] for c in clips if c["dur"] == c["dur"])
    skipped = [c for c in clips if c["fps"] > 2 * base or c["dur"] > 2 * typ]  # exports, not camera clips
    clips = [c for c in clips if c not in skipped]

    for c in clips:  # camera names like 20261028_122348.MOV carry the start time
        m = re.match(r"(\d{8})_(\d{6})", c["name"])
        c["nstart"] = datetime.strptime(m.group(1) + m.group(2), "%Y%m%d%H%M%S") if m else None
    starts = [c["start"] for c in clips]
    use_cam = all(starts) and len(set(starts)) == len(starts)
    use_name = all(c["nstart"] for c in clips)
    if use_name:
        for c in clips:
            c["start"] = c["nstart"]
        order = sorted(clips, key=lambda c: c["start"])
        use_cam = True
        how = "camera file names (start time in each name)"
    elif use_cam:
        order = sorted(clips, key=lambda c: c["start"])
        how = "camera timestamps"
    else:
        order = sorted(clips, key=lambda c: c["mtime"] - timedelta(seconds=c["dur"] if c["dur"] == c["dur"] else 0))
        how = "file modified times (camera timestamps missing or duplicated)"

    print(f"folder: {folder}")
    for c in skipped:
        print(f"skipped {c['name']} ({c['fps']:.0f} fps, {hms(c['dur'])}: looks like an export, not a camera clip)")
    print(f"ordered by {how}\n")
    print(f"{'#':>3}  {'file':<26}{'fps':>7}{'length':>10}  {'camera clock':<20}{'joined at':>10}  gap before")
    total, prev_end, gaps = 0.0, None, []
    for i, c in enumerate(order, 1):
        st = c["start"] if use_cam else c["mtime"] - timedelta(seconds=c["dur"])
        gap = ""
        if prev_end is not None:
            g = (st - prev_end).total_seconds()
            gap = f"{g:+.1f} s"
            if g > 5:
                gaps.append((i, c["name"], g))
                gap += "  <-- gap"
            elif g < -5:
                gap += "  <-- overlap?"
        c["joined"] = total
        c["st"] = st
        print(f"{i:>3}  {c['name'][:25]:<26}{c['fps']:>7.2f}{hms(c['dur']):>10}  {st.strftime('%Y-%m-%d %H:%M:%S'):<20}{hms(total):>10}  {gap}")
        total += c["dur"] if c["dur"] == c["dur"] else 0.0
        prev_end = st + timedelta(seconds=c["dur"] if c["dur"] == c["dur"] else 0)

    if [c["name"] for c in order] != sorted(c["name"] for c in order):
        print("\nNOTE: time order differs from file-name order. a join by name would scramble the flight.")
    print(f"\n{len(order)} clips, {hms(total)} of footage at {base:g} fps")
    print(f"first clip starts {hms(0)}, last clip ends {hms((prev_end - (order[0]['start'] if use_cam else order[0]['mtime'] - timedelta(seconds=order[0]['dur']))).total_seconds())} later")
    if gaps:
        print(f"{len(gaps)} gap(s) totaling {hms(sum(g for _, _, g in gaps))}:")
        for i, n, g in gaps:
            print(f"  before #{i} {n}: {hms(g)} missing")
    else:
        print("no gaps between clips")
    for c in skipped:
        print(f"{c['name']} is {hms(c['dur'])} long; all the clips above at {c['fps'] / base:g}x would be {hms(total * base / c['fps'])}")

    if at:
        day = order[0]["st"].date()
        want = datetime.combine(day, datetime.strptime(at, "%H:%M:%S").time())
        hit = [c for c in order if c["st"] <= want < c["st"] + timedelta(seconds=c["dur"])]
        if hit:
            c = hit[0]
            off = (want - c["st"]).total_seconds()
            print(f"\ncamera clock {at} = {c['name']} at {hms(off)} = joined timeline {hms(c['joined'] + off)}")
        else:
            print(f"\ncamera clock {at} is not inside any clip (falls in a gap or outside the recording)")

    lst = os.path.join(os.getcwd(), "list.txt")
    with open(lst, "w") as fh:
        for c in order:
            fh.write("file '" + c["path"].replace("\\", "/").replace("'", "'\\''") + "'\n")
    print(f"\nwrote {lst} in time order. rotation_from_video.py can read it directly (no 100 GB copy):")
    print('  py rotation_from_video.py list.txt --launch <joined time> ...')


if __name__ == "__main__":
    main()
