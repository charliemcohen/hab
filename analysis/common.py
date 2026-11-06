# loaders for the raw flight files + compute_numbers()
import csv
import io
import json
import math
import re
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]


def _find(name):
    for p in [ROOT / "data" / "flight2_2026-10-28" / name,
              ROOT / "data" / "flight1_2026-08-07" / name,
              ROOT / "firmware" / "HAB_RW_Controller" / name,
              ROOT / "rotation_results" / name,
              ROOT / name]:
        if p.exists():
            return p
    raise FileNotFoundError(name)


RELEASE_UTC = pd.Timestamp("2026-10-28 19:42:55")  # 2.53 g jolt on the accelerometer; GPS climbs 3 s later
FLIGHT1_PEAK_M = 31761
CAMERA_OFFSET = pd.Timedelta(minutes=15, seconds=7)  # Mobius clock runs fast by this, +/- 1 min
BAKER_CA = (35.2727, -116.0742)
FT_PER_M = 3.280839895


def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    h = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(h))


def along_track_m(lat, lon):
    lat, lon = np.asarray(lat), np.asarray(lon)
    return sum(haversine_m(lat[i - 1], lon[i - 1], lat[i], lon[i]) for i in range(1, len(lat)))


R0_ISA = 6356766.0  # USSA-1976 effective earth radius, m


# GPS height is geometric; the ISA layers want geopotential
def geopotential_m(z_m):
    return R0_ISA * z_m / (R0_ISA + z_m)


def geometric_m(h_m):
    return R0_ISA * h_m / (R0_ISA - h_m)


def isa_pressure_hpa(h_m):
    if h_m < 11000:
        T = 288.15 - 0.0065 * h_m
        return 1013.25 * (T / 288.15) ** 5.2559
    if h_m < 20000:
        return 226.32 * math.exp(-9.80665 * (h_m - 11000) / (287.05 * 216.65))
    T = 216.65 + 0.001 * (h_m - 20000)
    return 54.749 * (T / 216.65) ** (-34.163)


def isa_temperature_c(h_m):
    if h_m < 11000:
        return 288.15 - 0.0065 * h_m - 273.15
    if h_m < 20000:
        return 216.65 - 273.15
    return 216.65 + 0.001 * (h_m - 20000) - 273.15


# pressure altitude: inverted ISA, only used where GPS is missing
def isa_altitude_m(p_hpa):
    if p_hpa > 226.32:
        return 44330.8 * (1 - (p_hpa / 1013.25) ** 0.190263)
    if p_hpa > 54.749:
        return 11000 + 287.05 * 216.65 / 9.80665 * math.log(226.32 / p_hpa)
    return 20000 + 216.65 / 0.001 * ((p_hpa / 54.749) ** (-1 / 34.163) - 1)


SENSOR_BITS = "MCP1 MCP2 MCP3 TMP119 VEML LTR MS_PT MS_RH AHT LIS BMP GPS INA219 MUX".split()


def load_sci():
    path = _find("SCI_000.CSV")
    with open(path) as f:
        header = f.readline().strip().split(",")
        lines = f.readlines()
    counts = {}
    s_lines, e_rows = [], []
    for l in lines:
        t = l.split(",", 1)[0]
        counts[t] = counts.get(t, 0) + 1
        if t == "S":
            s_lines.append(l)
        elif t == "E":
            p = l.strip().split(",")
            e_rows.append(dict(rtc_utc=p[1], uno_ms=int(p[3]), uno_seq=int(p[4]), code=p[5],
                               a=p[6] if len(p) > 6 else None, b=p[7] if len(p) > 7 else None))
    S = pd.read_csv(io.StringIO("".join(s_lines)), names=header, header=None,
                    dtype={"valid": str, "oor": str, "synth": str})
    S["t"] = pd.to_datetime(S["rtc_utc"])
    # drop bench-test and placeholder rows (none in this file)
    S = S[(S["test"] == 0) & (S["synth"].apply(lambda x: int(str(x), 16)) == 0)].reset_index(drop=True)
    S["amag"] = np.sqrt(S.lis_ax_g ** 2 + S.lis_ay_g ** 2 + S.lis_az_g ** 2)
    S["valid_i"] = S["valid"].apply(lambda x: int(str(x), 16))
    S["oor_i"] = S["oor"].apply(lambda x: int(str(x), 16))
    S["p_alt_m"] = S["ms_p_hpa"].apply(lambda p: geometric_m(isa_altitude_m(p)) if pd.notna(p) else np.nan)
    return S, counts, pd.DataFrame(e_rows)


def _dm_to_deg(v, hemi):
    if not v:
        return np.nan
    x = float(v)
    d = int(x // 100)
    m = x - d * 100
    out = d + m / 60
    return -out if hemi in "SW" else out


def load_gps2():
    path = _find("SCI_000.CSV")
    rows = []
    with open(path) as f:
        for l in f:
            if not l.startswith("G,"):
                continue
            p = next(csv.reader([l.strip()]))
            nmea = p[5]
            if not nmea.startswith("$GNGGA") and not nmea.startswith("$GPGGA"):
                continue
            fld = nmea.split(",")
            try:
                rows.append(dict(
                    rtc=p[1], uno_ms=int(p[3]), gps_utc=fld[1],
                    lat=_dm_to_deg(fld[2], fld[3]), lon=_dm_to_deg(fld[4], fld[5]),
                    qual=int(fld[6]) if fld[6] else 0, sats=int(fld[7]) if fld[7] else 0,
                    hdop=float(fld[8]) if fld[8] else np.nan,
                    alt_m=float(fld[9]) if fld[9] else np.nan))
            except (ValueError, IndexError):
                continue
    G = pd.DataFrame(rows)
    rtc_t = pd.to_datetime(G["rtc"], errors="coerce")
    hms = G["gps_utc"].str.extract(r"^(\d{2})(\d{2})(\d{2}(?:\.\d+)?)$").astype(float)
    gga_t = (rtc_t.dt.normalize() + pd.to_timedelta(hms[0], unit="h") + pd.to_timedelta(hms[1], unit="m")
             + pd.to_timedelta(hms[2], unit="s"))
    gga_t = gga_t.where((gga_t - rtc_t).abs() < pd.Timedelta(hours=12))
    G["t"] = gga_t.fillna(rtc_t).astype(rtc_t.dtype)  # receiver's GGA epoch; logger clock only as fallback
    fixes = G[(G.qual > 0) & (G.t >= "2026-10-28 19:20:00")].copy().reset_index(drop=True)
    return G, fixes


# any stretch over a minute with no valid fix
def gps_gaps(fixes, min_s=60):
    dt = fixes.t.diff().dt.total_seconds()
    out = []
    for i in fixes.index[dt > min_s]:
        a, b = fixes.loc[i - 1], fixes.loc[i]
        out.append(dict(start=a.t, end=b.t, alt_start=a.alt_m, alt_end=b.alt_m,
                        lat_start=a.lat, lon_start=a.lon, lat_end=b.lat, lon_end=b.lon,
                        minutes=dt[i] / 60))
    return out


def load_gps1():
    f = pd.read_csv(_find("Aug_7_2026_GPS_Fixes.csv"))
    f["t"] = pd.to_datetime(f["time_utc"]).dt.tz_localize(None)
    return f


KML_NS = {"k": "http://www.opengis.net/kml/2.2"}


def load_kml2():
    root = ET.parse(_find("Second_Successful_Launch.kml")).getroot()
    out = {"pred_path": None, "pred_burst": None, "pred_landing": None, "pred_desc": None, "spot": None,
           "n_prediction_paths": 0}
    for pm in root.iter("{%s}Placemark" % KML_NS["k"]):
        nm = pm.find("k:name", KML_NS)
        n = nm.text.strip() if nm is not None and nm.text else ""
        if n == "Flight path":
            out["n_prediction_paths"] += 1
    for fo in root.iter("{%s}Folder" % KML_NS["k"]):
        nm = fo.find("k:name", KML_NS)
        n = nm.text.strip() if nm is not None and nm.text else ""
        if n == "Prediction":
            for pm in fo.findall("k:Placemark", KML_NS):
                pn = pm.find("k:name", KML_NS).text.strip()
                co = pm.find(".//k:coordinates", KML_NS).text.split()
                pts = [tuple(map(float, x.split(","))) for x in co]
                d = pm.find("k:description", KML_NS)
                if pn == "Flight path":
                    out["pred_path"] = pd.DataFrame(pts, columns=["lon", "lat", "alt_m"])
                    out["pred_desc"] = d.text if d is not None else ""
                elif "Burst" in pn:
                    out["pred_burst"] = dict(lon=pts[0][0], lat=pts[0][1], alt_m=pts[0][2],
                                             time=re.search(r"at (\d{4}-\d\d-\d\dT[\d:.]+Z)", d.text).group(1) if d is not None else None)
        if n == "Flight Points":
            for pm in fo.findall("k:Placemark", KML_NS):
                pn = pm.find("k:name", KML_NS).text.strip()
                co = pm.find(".//k:coordinates", KML_NS).text.strip().split(",")
                if pn.startswith("Predicted Balloon Landing"):
                    d = pm.find("k:description", KML_NS).text
                    out["pred_landing"] = dict(lon=float(co[0]), lat=float(co[1]), alt_m=float(co[2]),
                                               time=re.search(r"at (\d{4}-\d\d-\d\dT[\d:.]+Z)", d).group(1))
                if pn == "Balloon Landing":
                    out["kml_landing_point"] = dict(lon=float(co[0]), lat=float(co[1]), alt_m=float(co[2]))
        if n == "2D SPOT":
            rows = []
            for pm in fo.iter("{%s}Placemark" % KML_NS["k"]):
                co = pm.find(".//k:coordinates", KML_NS)
                d = pm.find("k:description", KML_NS)
                if co is None or d is None or not d.text:
                    continue
                m = re.search(r"(\d+/\d+/\d+)\s+(\d+:\d+:\d+)\s*([AP]M)", d.text)
                a = re.search(r"Altitude: (-?\d+) ft", d.text)
                if not m:
                    continue
                c = co.text.strip().split(",")
                rows.append(dict(t_local=f"{m.group(1)} {m.group(2)} {m.group(3)}",
                                 lat=float(c[1]), lon=float(c[0]), alt_ft=int(a.group(1)) if a else None))
            sp = pd.DataFrame(rows)
            sp["t_local"] = pd.to_datetime(sp["t_local"], format="%m/%d/%Y %I:%M:%S %p")
            sp["t"] = sp["t_local"] + pd.Timedelta(hours=5)  # SPOT export is UTC-5: the 14:44:05 fix sits on the pad, the next is airborne
            out["spot"] = sp
    return out


def load_rotation():
    a = pd.read_csv(_find("Flight_2_rotation.csv"))
    b = pd.read_csv(_find("Flight_2_upper_rotation.csv"))
    r = pd.concat([a, b]).drop_duplicates(subset=["t_min"]).sort_values("t_min").reset_index(drop=True)
    return r


# every value the report quotes, straight from the raw files
def compute_numbers():
    N = {}
    S, counts, E = load_sci()
    G, fx = load_gps2()
    f1 = load_gps1()
    K = load_kml2()

    N["s_records_total"] = int(len(S))
    N["s_records_before_release"] = int((S.t < RELEASE_UTC).sum())
    N["s_records_after_release"] = int((S.t >= RELEASE_UTC).sum())
    N["s_first_t"], N["s_last_t"] = str(S.t.min()), str(S.t.max())
    N["log_minutes"] = round((S.t.max() - S.t.min()).total_seconds() / 60, 1)
    N["log_rate_hz"] = round(len(S) / (S.t.max() - S.t.min()).total_seconds(), 2)
    N["g_rows"] = int(counts.get("G", 0))
    N["e_rows"] = int(counts.get("E", 0))
    seq_gap = E[E.code == "SEQ_GAP"]
    N["seq_gap_events"] = int(len(seq_gap))
    N["seq_gap_missing"] = int((S.seq.diff() - 1).clip(lower=0).sum())
    N["seq_gap_pct"] = round(100 * N["seq_gap_missing"] / (S.seq.max() + 1), 1)
    N["mega_records_generated"] = int(S.seq.max() + 1)
    N["test_nonzero"] = int((S["test"] != 0).sum())

    vi, oi = S.valid_i.to_numpy(dtype=np.int64), S.oor_i.to_numpy(dtype=np.int64)
    N["valid_counts"] = {b: int(((vi >> i) & 1).sum()) for i, b in enumerate(SENSOR_BITS)}
    N["oor_counts"] = {b: int(((oi >> i) & 1).sum()) for i, b in enumerate(SENSOR_BITS)}
    N["mega_gps_cols_populated"] = int(S.gps_lat.notna().sum())

    N["gps_fixes_used"] = int(len(fx))
    N["gps_fixes_before_1920_excluded"] = int(((G.qual > 0) & (G.t < "2026-10-28 19:20:00")).sum())
    pre = G[(G.qual > 0) & (G.t < "2026-10-28 19:20:00")]
    N["gps_pre1920_sats_range"] = [int(pre.sats.min()), int(pre.sats.max())]
    N["gps_pre1920_alt_range"] = [float(pre.alt_m.min()), float(pre.alt_m.max())]
    ground = fx[(fx.t >= RELEASE_UTC - pd.Timedelta(minutes=10)) & (fx.t < RELEASE_UTC)]
    N["launch_alt_m"] = round(float(ground.alt_m.median()), 1)
    N["launch_lat"], N["launch_lon"] = round(float(ground.lat.median()), 5), round(float(ground.lon.median()), 5)
    ib = fx.alt_m.idxmax()
    B = fx.loc[ib]
    N["burst_t"] = str(B.t)
    N["burst_alt_m"] = float(B.alt_m)
    N["burst_alt_ft"] = int(round(B.alt_m * FT_PER_M))
    N["burst_lat"], N["burst_lon"] = round(float(B.lat), 5), round(float(B.lon), 5)
    N["burst_sats"] = int(B.sats)
    N["first_descending_fix_t"] = str(fx.loc[ib + 1].t)
    N["ascent_minutes"] = round((B.t - RELEASE_UTC).total_seconds() / 60, 1)
    N["ascent_mean_rate_mps"] = round((B.alt_m - N["launch_alt_m"]) / (B.t - RELEASE_UTC).total_seconds(), 2)
    L = fx.iloc[-1]
    N["last_fix_t"] = str(L.t)
    N["last_fix_lat"], N["last_fix_lon"], N["last_fix_alt_m"] = round(float(L.lat), 5), round(float(L.lon), 5), float(L.alt_m)
    N["last_fix_sats"] = int(L.sats)
    N["gps_climb_start_t"] = "2026-10-28 19:42:58"
    N["release_jolt_g"] = round(float(S.loc[(S.t == RELEASE_UTC), "amag"].iloc[0]), 2)
    gaps = gps_gaps(fx)
    N["gps_gaps"] = [{k: (str(v) if isinstance(v, pd.Timestamp) else (round(float(v), 4 if "lat" in k or "lon" in k else 1) if isinstance(v, (float, np.floating)) else v))
                      for k, v in g.items()} for g in gaps]

    N["f2_straight_km"] = round(haversine_m(N["launch_lat"], N["launch_lon"], L.lat, L.lon) / 1000, 1)
    N["f2_straight_mi"] = round(N["f2_straight_km"] / 1.609344, 1)
    N["f2_launch_to_burst_km"] = round(haversine_m(N["launch_lat"], N["launch_lon"], B.lat, B.lon) / 1000, 1)
    N["f2_burst_to_last_km"] = round(haversine_m(B.lat, B.lon, L.lat, L.lon) / 1000, 1)
    inflight = fx[fx.t >= RELEASE_UTC]
    N["f2_along_track_km"] = round(along_track_m(inflight.lat, inflight.lon) / 1000, 1)
    N["f2_baker_km"] = round(haversine_m(BAKER_CA[0], BAKER_CA[1], L.lat, L.lon) / 1000, 1)
    N["f2_baker_mi"] = round(N["f2_baker_km"] / 1.609344, 1)

    sp = K["spot"]
    landed = sp[(sp.t > "2026-10-28 22:40") & (sp.t < "2026-10-28 23:30")]
    N["spot_landing_t"] = str(landed.iloc[0].t)
    N["spot_landing_lat"], N["spot_landing_lon"] = float(landed.iloc[0].lat), float(landed.iloc[0].lon)
    N["spot_landing_to_last_fix_m"] = round(haversine_m(N["spot_landing_lat"], N["spot_landing_lon"], L.lat, L.lon))
    N["spot_points_in_ascent_gap"] = int(((sp.t > gaps[0]["start"]) & (sp.t < gaps[0]["end"])).sum()) if gaps else 0
    N["spot_points_in_flight"] = int(((sp.t >= RELEASE_UTC) & (sp.t <= "2026-10-28 22:45")).sum())

    im = S.ms_p_hpa.idxmin()
    N["ms_p_min_hpa"], N["ms_p_min_t"] = float(S.loc[im, "ms_p_hpa"]), str(S.loc[im, "t"])
    N["ms_p_max_hpa"] = float(S.ms_p_hpa.max())
    N["bmp_p_min_hpa"], N["bmp_p_min_t"] = float(S.bmp_p_hpa.min()), str(S.loc[S.bmp_p_hpa.idxmin(), "t"])
    asc = S[(S.t >= RELEASE_UTC) & (S.t <= B.t)]
    div = asc[(asc.bmp_p_hpa - asc.ms_p_hpa) > 5]
    N["bmp_diverge_t"] = str(div.iloc[0].t)
    N["bmp_diverge_ms_p_hpa"] = float(div.iloc[0].ms_p_hpa)
    N["bmp_at_burst_hpa"] = float(S.loc[im, "bmp_p_hpa"])
    hi = asc[asc.ms_p_hpa < 100]
    N["bmp_band_when_ms_below_100"] = [float(hi.bmp_p_hpa.min()), float(hi.bmp_p_hpa.max())]

    m = pd.merge_asof(S.sort_values("t"), fx[["t", "alt_m"]].sort_values("t"), on="t", tolerance=pd.Timedelta("2s"))
    m = m[m.alt_m.notna() & (m.t >= RELEASE_UTC) & (m.t <= B.t)].copy()
    m["isa"] = m.alt_m.apply(lambda z: isa_pressure_hpa(geopotential_m(z)))
    m["ratio"] = m.ms_p_hpa / m.isa
    N["isa_ratio_min"], N["isa_ratio_max"] = round(float(m.ratio.min()), 3), round(float(m.ratio.max()), 3)
    N["isa_ratio_above_2km_min"] = round(float(m[m.alt_m > 2000].ratio.min()), 3)
    N["isa_ratio_above_2km_max"] = round(float(m[m.alt_m > 2000].ratio.max()), 3)

    N["t_ext_min_c"] = float(S.t_mcp3_c.min())
    N["t_ext_min_t"] = str(S.loc[S.t_mcp3_c.idxmin(), "t"])
    N["t_ext_min_p_hpa"] = float(S.loc[S.t_mcp3_c.idxmin(), "ms_p_hpa"])
    N["t_ext_min_palt_m"] = round(float(S.loc[S.t_mcp3_c.idxmin(), "p_alt_m"]))
    N["t_tmp119_min_c"] = float(S.t_tmp119_c.min())
    N["t_ms_min_c"] = float(S.ms_t_c.min())
    ia = asc.t_mcp3_c.idxmin()
    N["t_ext_asc_min_c"], N["t_ext_asc_min_t"] = float(asc.loc[ia, "t_mcp3_c"]), str(asc.loc[ia, "t"])
    N["t_ext_asc_min_p_hpa"] = float(asc.loc[ia, "ms_p_hpa"])
    N["t_ext_asc_min_palt_m"] = round(float(asc.loc[ia, "p_alt_m"]))
    top = asc[asc.ms_p_hpa < 15]
    N["t_ext_near_burst_c"] = [round(float(top.t_mcp3_c.min()), 1), round(float(top.t_mcp3_c.max()), 1)]
    N["t_isa_30km_c"] = round(isa_temperature_c(geopotential_m(30000)), 1)
    N["t_int_min_c"], N["t_int_min_t"] = float(S.t_mcp1_c.min()), str(S.loc[S.t_mcp1_c.idxmin(), "t"])
    N["t_int_at_release_c"] = float(S.loc[S.t == RELEASE_UTC, "t_mcp1_c"].iloc[0])
    N["t_int_near_burst_c"] = round(float(top.t_mcp1_c.median()), 1)

    N["ms_rh_negative_count"] = int((asc.ms_rh_pct < 0).sum())
    N["aht_rh_zero_frac_flight"] = round(float((S[S.t >= RELEASE_UTC].aht_rh_pct == 0).mean()), 2)
    N["aht_rh_ground_median"] = round(float(S[S.t < RELEASE_UTC].aht_rh_pct.median()), 1)

    N["lux_ground"] = round(float(S[S.t < RELEASE_UTC].veml_lux.median()))
    N["lux_top"] = round(float(top.veml_lux.median()))
    N["uv_ground"] = round(float(S[S.t < RELEASE_UTC].ltr_uvs_raw.median()))
    low = m[m.alt_m < 2000]
    N["uv_low_ascent"] = round(float(low.ltr_uvs_raw.median()))
    N["lux_low_ascent"] = round(float(low.veml_lux.median()))
    N["uv_top"] = round(float(top.ltr_uvs_raw.median()))
    N["veml_als_raw_max"] = int(S.veml_als_raw.max())

    after = S[(S.t > RELEASE_UTC + pd.Timedelta(seconds=15)) & (S.t <= B.t)]
    N["acc_ascent_max_g"] = round(float(after.amag.max()), 2)
    N["acc_ascent_max_t"] = str(after.loc[after.amag.idxmax(), "t"])
    N["acc_ascent_p99_g"] = round(float(after.amag.quantile(0.99)), 2)
    N["acc_ascent_n_over_1p17"] = int((after.amag > 1.17).sum())
    ff = S[(S.t >= B.t - pd.Timedelta(seconds=5)) & (S.t <= B.t + pd.Timedelta(seconds=10))]
    N["acc_freefall_first_t"] = str(ff[ff.amag < 0.1].t.min())
    N["acc_freefall_min_g"] = round(float(ff.amag.min()), 3)
    N["acc_freefall_n_below_0p2"] = int((ff.amag < 0.2).sum())
    post = S[(S.t > B.t) & (S.t <= B.t + pd.Timedelta(minutes=3))]
    N["acc_post_burst_max_g"] = round(float(post.amag.max()), 2)
    N["acc_post_burst_max_t"] = str(post.loc[post.amag.idxmax(), "t"])
    big = post[post.amag > 1.5]
    N["acc_post_burst_first_t"], N["acc_post_burst_last_t"] = str(big.t.min()), str(big.t.max())
    shake = S[(S.t >= "2026-10-28 21:20:00") & (S.t <= "2026-10-28 21:23:00")]
    N["acc_shake_window_max_g"] = round(float(shake.amag.max()), 2)
    N["acc_ground_vector"] = [round(float(v), 2) for v in S[S.t < "2026-10-28 19:40"][["lis_ax_g", "lis_ay_g", "lis_az_g"]].median()]

    def cam(hms):
        return pd.Timestamp("2026-10-28 " + hms) + pd.Timedelta(hours=7) - CAMERA_OFFSET
    N["cam_offset_s"] = int(CAMERA_OFFSET.total_seconds())
    N["cam_liftoff_frame_utc"] = str(cam("12:58:02"))
    N["cam_stop_utc"] = str(cam("14:36:59"))
    N["cam_resume_utc"] = str(cam("15:43:18"))
    N["cam_alt_frame_utc"] = str(cam("14:35:28"))
    N["cam_gap_minutes"] = round((cam("15:43:18") - cam("14:36:59")).total_seconds() / 60, 1)
    N["cam_stop_to_burst_min"] = round((B.t - cam("14:36:59")).total_seconds() / 60, 1)

    def palt_at(t):
        r = S.iloc[(S.t - t).abs().argsort().iloc[0]]
        return round(float(r.p_alt_m)), float(r.ms_p_hpa)
    N["cam_stop_palt_m"], N["cam_stop_p_hpa"] = palt_at(cam("14:36:59"))
    N["cam_resume_palt_m"], N["cam_resume_p_hpa"] = palt_at(cam("15:43:18"))
    N["cam_alt_frame_palt_m"], _ = palt_at(cam("14:35:28"))
    N["shake_utc"] = str(cam("14:36:39"))

    N["pred_desc"] = K["pred_desc"]
    pp = K["pred_path"]
    ibp = pp.alt_m.idxmax()
    N["pred_launch_lat"], N["pred_launch_lon"] = float(pp.lat[0]), float(pp.lon[0])
    N["pred_burst"] = K["pred_burst"]
    N["pred_landing"] = K["pred_landing"]
    N["pred_launch_to_burst_km"] = round(haversine_m(pp.lat[0], pp.lon[0], pp.lat[ibp], pp.lon[ibp]) / 1000, 1)
    N["pred_burst_to_landing_km"] = round(haversine_m(pp.lat[ibp], pp.lon[ibp], pp.lat.iloc[-1], pp.lon.iloc[-1]) / 1000, 1)
    N["pred_launch_to_landing_km"] = round(haversine_m(pp.lat[0], pp.lon[0], pp.lat.iloc[-1], pp.lon.iloc[-1]) / 1000, 1)
    N["pred_burst_vs_actual_km"] = round(haversine_m(K["pred_burst"]["lat"], K["pred_burst"]["lon"], B.lat, B.lon) / 1000, 1)
    N["pred_landing_vs_last_fix_km"] = round(haversine_m(K["pred_landing"]["lat"], K["pred_landing"]["lon"], L.lat, L.lon) / 1000, 1)
    N["pred_landing_vs_spot_km"] = round(haversine_m(K["pred_landing"]["lat"], K["pred_landing"]["lon"], N["spot_landing_lat"], N["spot_landing_lon"]) / 1000, 1)
    tb = pd.Timestamp(K["pred_burst"]["time"].replace("Z", ""))
    tl = pd.Timestamp(K["pred_landing"]["time"].replace("Z", ""))
    t0 = pd.Timestamp("2026-10-28 19:44:00")
    N["pred_ascent_min"] = round((tb - t0).total_seconds() / 60, 1)
    N["pred_descent_min"] = round((tl - tb).total_seconds() / 60, 1)
    N["actual_descent_min"] = round((pd.Timestamp(N["spot_landing_t"]) - B.t).total_seconds() / 60, 1)
    N["n_prediction_paths_in_kml"] = K["n_prediction_paths"]
    N["kml_landing_point_alt_m"] = K.get("kml_landing_point", {}).get("alt_m")

    desc = fx[fx.t > B.t].copy()
    seg = desc[(desc.alt_m < 7000) & (desc.alt_m > 3000)]
    N["descent_rate_3_7km_mps"] = round(float((seg.alt_m.iloc[0] - seg.alt_m.iloc[-1]) / (seg.t.iloc[-1] - seg.t.iloc[0]).total_seconds()), 1)
    tail = desc.tail(60)
    N["descent_rate_last_min_mps"] = round(float((tail.alt_m.iloc[0] - tail.alt_m.iloc[-1]) / (tail.t.iloc[-1] - tail.t.iloc[0]).total_seconds()), 1)
    seg = desc[(desc.alt_m > 25000)]
    N["descent_rate_above_25km_mps"] = round(float((seg.alt_m.iloc[0] - seg.alt_m.iloc[-1]) / (seg.t.iloc[-1] - seg.t.iloc[0]).total_seconds()), 1)

    a = fx[(fx.t >= RELEASE_UTC) & (fx.t <= B.t)].copy()
    a["dt"] = a.t.diff().dt.total_seconds()
    a["dist"] = [0.0] + [haversine_m(a.lat.iloc[i - 1], a.lon.iloc[i - 1], a.lat.iloc[i], a.lon.iloc[i]) for i in range(1, len(a))]
    a = a[(a.dt > 0) & (a.dt < 5)]
    a["v"] = a.dist / a.dt
    a["bin"] = (a.alt_m // 1000) * 1000
    w = a.groupby("bin").v.median()
    N["wind_max_mps"] = round(float(w.max()), 1)
    N["wind_max_bin_km"] = int(w.idxmax() / 1000)
    N["wind_5_10km_mps"] = round(float(w[(w.index >= 5000) & (w.index < 11000)].median()), 1)
    N["wind_23_24km_mps"] = round(float(w[(w.index >= 23000) & (w.index < 25000)].median()), 1)

    ip = f1.altitude_m.idxmax()
    P = f1.loc[ip]
    N["f1_peak_m"], N["f1_peak_ft"], N["f1_peak_t"] = int(P.altitude_m), int(round(P.altitude_m * FT_PER_M)), str(P.t)
    N["f1_peak_lat"], N["f1_peak_lon"] = float(P.latitude), float(P.longitude)
    N["f1_fixes"] = int(len(f1))
    N["f1_interval_s"] = float(f1.t.diff().dt.total_seconds().median())
    N["f1_sessions"] = int(f1.session.nunique())
    g1 = f1[f1.t < "2026-08-07 20:00"].iloc[-1]
    a1 = f1[f1.t > "2026-08-07 20:00"].iloc[0]
    N["f1_last_ground_t"], N["f1_last_ground_alt_m"] = str(g1.t), int(g1.altitude_m)
    N["f1_first_air_t"], N["f1_first_air_alt_m"] = str(a1.t), int(a1.altitude_m)
    N["f1_launch_gap_min"] = round((a1.t - g1.t).total_seconds() / 60, 1)
    L1, E1 = f1.iloc[0], f1.iloc[-1]
    N["f1_launch_lat"], N["f1_launch_lon"] = float(L1.latitude), float(L1.longitude)
    N["f1_landing_lat"], N["f1_landing_lon"], N["f1_landing_alt_m"], N["f1_last_t"] = float(E1.latitude), float(E1.longitude), int(E1.altitude_m), str(E1.t)
    N["f1_straight_km"] = round(haversine_m(L1.latitude, L1.longitude, E1.latitude, E1.longitude) / 1000, 1)
    N["f1_straight_mi"] = round(N["f1_straight_km"] / 1.609344, 1)
    N["f1_along_track_km"] = round(along_track_m(f1.latitude, f1.longitude) / 1000, 1)
    N["f1_along_track_mi"] = round(N["f1_along_track_km"] / 1.609344, 1)
    dt1 = f1.t.diff().dt.total_seconds()
    N["f1_gaps"] = [dict(start=str(f1.t[i - 1]), end=str(f1.t[i]), alt_start=int(f1.altitude_m[i - 1]), alt_end=int(f1.altitude_m[i]), minutes=round(dt1[i] / 60, 1))
                    for i in f1.index[dt1 > 60]]
    s = f1[(f1.t >= "2026-08-07 20:10") & (f1.t < "2026-08-07 21:10")]
    N["f1_rate_4_10km_mps"] = round(float((s.altitude_m.iloc[-1] - s.altitude_m.iloc[0]) / (s.t.iloc[-1] - s.t.iloc[0]).total_seconds()), 2)
    s = f1[(f1.t >= "2026-08-07 23:22") & (f1.t <= P.t)]
    N["f1_rate_29_32km_mps"] = round(float((s.altitude_m.iloc[-1] - s.altitude_m.iloc[0]) / (s.t.iloc[-1] - s.t.iloc[0]).total_seconds()), 2)
    N["f1_mean_rate_from_1940_mps"] = round(float((P.altitude_m - g1.altitude_m) / (P.t - g1.t).total_seconds()), 2)
    N["f1_mean_rate_from_2010_mps"] = round(float((P.altitude_m - g1.altitude_m) / (P.t - a1.t).total_seconds()), 2)

    J_wheel, J_payload = 3.19e-4, 0.01926
    r = J_wheel / J_payload
    N["rw_J_wheel"], N["rw_J_payload"], N["rw_ratio"] = J_wheel, J_payload, round(r, 4)
    N["rw_band_rpm"] = 30
    N["rw_authority_dps"] = round(30 * 6.0 * r, 2)
    obs_rate_dps = 360 / 50.0
    N["rw_obs_rate_dps"] = round(obs_rate_dps, 1)
    N["rw_obs_rot_per_min"] = round(60 / 50.0, 1)
    N["rw_rpm_to_stop"] = round(obs_rate_dps / r / 6.0, 1)
    N["rw_band_shortfall_x"] = round(N["rw_rpm_to_stop"] / 30, 1)
    N["rw_dump_dps"] = round(75 * 6.0 * r, 1)

    R = load_rotation()
    N["rot_windows"] = int(len(R))
    N["rot_windows_poor_match"] = int(R.notes.str.contains("matched poorly").sum())
    N["rot_windows_with_repeat"] = int(R.repeat_period_s.notna().sum())
    rr = R[R.repeat_period_s.between(40, 52)]
    N["rot_windows_40_52s"] = int(len(rr))
    N["rot_40_52_tmin_range"] = [float(rr.t_min.min()), float(rr.t_min.max())]
    N["rot_top3"] = R.nlargest(3, "repeat_strength")[["t_min", "repeat_period_s", "repeat_strength"]].round(2).values.tolist()
    return N


def save_numbers(path=None):
    N = compute_numbers()
    path = Path(path) if path else ROOT / "analysis" / "numbers.json"
    path.write_text(json.dumps(N, indent=1, default=str))
    return N


if __name__ == "__main__":
    print(json.dumps(compute_numbers(), indent=1, default=str))
