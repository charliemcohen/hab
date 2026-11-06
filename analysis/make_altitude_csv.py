# Flight 2 GPS altitude every minute from release -> docs/flight2_gps_altitude_1min.csv
import numpy as np
import pandas as pd

import common as C

G, FX = C.load_gps2()
S, _, _ = C.load_sci()
rel = C.RELEASE_UTC
end = pd.Timestamp(S.t.max())
rows = []
m = 0
while rel + pd.Timedelta(minutes=m) <= end:
    t = rel + pd.Timedelta(minutes=m)
    d = (FX.t - t).abs()
    i = d.idxmin()
    g = FX.loc[i] if d[i] <= pd.Timedelta(seconds=30) else None
    ds = (S.t - t).abs()
    j = ds.idxmin()
    srow = S.loc[j] if ds[j] <= pd.Timedelta(seconds=30) else None
    rows.append(dict(
        minutes_after_release=m,
        utc=t.strftime("%Y-%m-%d %H:%M:%S"),
        gps_fix_utc=g.t.strftime("%H:%M:%S") if g is not None else "",
        gps_alt_m=round(float(g.alt_m), 1) if g is not None else "",
        gps_alt_ft=int(round(g.alt_m * C.FT_PER_M)) if g is not None else "",
        gps_lat=round(float(g.lat), 5) if g is not None else "",
        gps_lon=round(float(g.lon), 5) if g is not None else "",
        gps_sats=int(g.sats) if g is not None else "",
        ms8607_p_hpa=float(srow.ms_p_hpa) if srow is not None else "",
        pressure_alt_m_estimated=int(round(srow.p_alt_m)) if srow is not None else "",
        note="" if g is not None else "no GPS fix within 30 s",
    ))
    m += 1
out = pd.DataFrame(rows)
path = C.ROOT / "docs" / "flight2_gps_altitude_1min.csv"
with open(path, "w") as f:
    f.write("# Flight 2 (2026-10-28) GPS altitude at one-minute intervals from release at 19:42:55 UTC.\n")
    f.write("# Source: SCI_000.CSV G rows (GGA sentences from the Uno shield GPS), MSL altitude. Pressure altitude is an estimate.\n")
    out.to_csv(f, index=False)
print("wrote", path, len(out), "rows;", int((out.gps_alt_m == "").sum()), "minutes without a GPS fix")
