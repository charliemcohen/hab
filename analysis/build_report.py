# builds docs/Cohen_HAB_Report.pdf. run make_figures.py and make_altitude_csv.py first
import os
import re
from datetime import datetime
from pathlib import Path

from PIL import Image as PILImage
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import inch
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (HRFlowable, Image, KeepTogether, PageBreak, Paragraph, SimpleDocTemplate, Spacer, Table,
                                TableStyle)

import common as C

N = C.compute_numbers()
FIG = C.ROOT / "docs" / "figures"
PHOTO = C.ROOT / "photos"
OUT = C.ROOT / "docs" / "Cohen_HAB_Report.pdf"
TMP = Path("/tmp/hab_report_img")
TMP.mkdir(exist_ok=True)

FONT_DIR = C.ROOT / "fonts"
for _n, _f in (("Plex", "IBMPlexSans-Regular"), ("Plex-Italic", "IBMPlexSans-Italic"), ("Plex-Medium", "IBMPlexSans-Medium"),
               ("Plex-SemiBold", "IBMPlexSans-SemiBold"), ("PlexMono", "IBMPlexMono-Regular")):
    pdfmetrics.registerFont(TTFont(_n, str(FONT_DIR / f"{_f}.ttf")))
pdfmetrics.registerFontFamily("Plex", normal="Plex", bold="Plex-SemiBold", italic="Plex-Italic", boldItalic="Plex-SemiBold")

INK = colors.HexColor("#1d1d1f")
ACC = colors.HexColor("#1f5fbf")
GRAY = colors.HexColor("#6e6e73")
LINE = colors.HexColor("#d1d1d6")
HAIR = colors.HexColor("#e5e5ea")
INK_HEX, ACC_HEX, GRAY_HEX = "#1d1d1f", "#1f5fbf", "#6e6e73"

body = ParagraphStyle("body", fontName="Plex", fontSize=9.0, leading=13.2, spaceAfter=5, alignment=TA_LEFT, textColor=INK,
                      allowWidows=0, allowOrphans=0)  # no lone line at the top or bottom of a page
small = ParagraphStyle("small", parent=body, fontSize=7.6, leading=10.4, textColor=GRAY, spaceAfter=3)
cap = ParagraphStyle("cap", parent=body, fontSize=7.6, leading=10.4, textColor=GRAY, spaceBefore=3, spaceAfter=9)
h1 = ParagraphStyle("h1", fontName="Plex-SemiBold", fontSize=13.5, leading=17, spaceBefore=4, spaceAfter=6, textColor=INK)
h2 = ParagraphStyle("h2", fontName="Plex-SemiBold", fontSize=9.8, leading=13, spaceBefore=8, spaceAfter=3, textColor=INK)
title = ParagraphStyle("title", fontName="Plex-SemiBold", fontSize=20, leading=24.5, spaceAfter=7, textColor=INK)
kicker = ParagraphStyle("kicker", fontName="PlexMono", fontSize=7.4, leading=10, textColor=ACC, spaceAfter=7)
sub = ParagraphStyle("sub", fontName="Plex", fontSize=8.8, leading=12.5, textColor=GRAY, spaceAfter=12)
cell = ParagraphStyle("cell", fontName="Plex", fontSize=7.5, leading=10.0, textColor=INK)
cellb = ParagraphStyle("cellb", fontName="Plex-SemiBold", fontSize=7.2, leading=9.6, textColor=GRAY)
statn = ParagraphStyle("statn", fontName="Plex-SemiBold", fontSize=15, leading=18, textColor=INK)
statl = ParagraphStyle("statl", fontName="Plex", fontSize=7.0, leading=9.2, textColor=GRAY)
bul = ParagraphStyle("bul", parent=body, leftIndent=13, bulletIndent=2, spaceAfter=2.5, bulletFontName="Plex", bulletFontSize=8)

FIG_ORDER = ["fig02_rigging_before_after.png", "fig01_system_diagram.png", "01_payload_open.jpg", "02_payload_closed.jpg",
             "05_electronics_closeup.jpg", "04_reaction_wheel.jpg", "fig03_avionics_dataflow.png", "fig04_altitude_time.png",
             "fig05_rate_vs_altitude.png", "fig06_pressure_vs_altitude.png", "fig07_temperature_vs_altitude.png", "fig08_wind_vs_altitude.png",
             "fig09_ground_tracks.png", "fig10_prediction_vs_actual.png", "fig11_light_uv_vs_altitude.png", "fig12_accel_magnitude.png",
             "03_test_bridle.jpg", "07_ground_test_rig.jpg", "fig13_rotation_repeat_period.png", "fig14_tracker_example_4min_14min.png",
             "09_liftoff.jpg", "11_altitude_frame.jpg", "10_recovery.jpg", "12_recovery_photo.jpg"]
FIGNUM = {n: i + 1 for i, n in enumerate(FIG_ORDER)}
_SEEN = []


def F(name):
    return FIGNUM[name]


_UNIT = re.compile(r"(\d) (?=(?:m/s|km|hPa|mph|rpm|min|deg|ft|mi|Hz|kg|lb|°C|m|g|s|%)(?![\w/]))")


def P(t, st=body):
    t = _UNIT.sub(r"\1&nbsp;", t)  # keep numbers on the same line as their units
    if st is h1:
        m = re.match(r"^(\d+)\.\s+(.*)$", t) or re.match(r"^(Appendix [A-Z])\.\s+(.*)$", t)
        if m:
            t = f'<font color="{ACC_HEX}">{m.group(1)}</font>&nbsp;&nbsp;&nbsp;{m.group(2)}'
    elif st is h2:
        m = re.match(r"^(\d+[a-z])\.\s+(.*)$", t)
        if m:
            t = f'<font color="{GRAY_HEX}">{m.group(1)}</font>&nbsp;&nbsp;{m.group(2)}'
    elif st is cap:
        m = re.match(r"^(?:<b>)?(Figure|Table) (\d+)\.(?:</b>)?\s+(.*)$", t, re.S)
        if m:
            t = f'<font color="{INK_HEX}"><b>{m.group(1)} {m.group(2)}</b></font>&nbsp;&nbsp;{m.group(3)}'
    para = Paragraph(t, st)
    if st in (h1, h2):
        para.keepWithNext = 1  # never strand a heading at the bottom of a page
    return para


def B(t):
    return Paragraph(t, bul, bulletText="\u2022")


FIG_SCALE = float(os.environ.get("HAB_FIG_SCALE", "0.94"))  # one size knob that holds the report to 16 pages
PHOTO_H = float(os.environ.get("HAB_PHOTO_H", "2.2"))


def fig(name, caption, width=6.3 * inch, maxh=None):
    n = FIGNUM[name]; _SEEN.append(name); assert FIG_ORDER[len(_SEEN) - 1] == name, name
    p = FIG / name if (FIG / name).exists() else PHOTO / name
    im = PILImage.open(p)
    w, h = im.size
    width *= FIG_SCALE
    H = width * h / w
    if maxh and H > maxh:
        H = maxh; width = H * w / h
    return KeepTogether([Image(str(p), width=width, height=H), P(f"<b>Figure {n}.</b> {caption}", cap)])


def photo_path(name, max_px=1400):
    im = PILImage.open(PHOTO / name)
    im.thumbnail((max_px, max_px))
    out = TMP / name
    im.convert("RGB").save(out, quality=82)
    return out


def photos2(names, captions, width=3.05 * inch):
    cells, caps = [], []
    for nm, c in zip(names, captions):
        n = FIGNUM[nm]; _SEEN.append(nm); assert FIG_ORDER[len(_SEEN) - 1] == nm, nm
        p = photo_path(nm)
        im = PILImage.open(p); w, h = im.size
        cells.append((p, w, h))
        caps.append(P(f"<b>Figure {n}.</b> {c}", cap))
    H = min([PHOTO_H * inch] + [width * h / w for _, w, h in cells])
    cells = [Image(str(p), width=H * w / h, height=H) for p, w, h in cells]
    t = Table([cells, caps], colWidths=[3.15 * inch, 3.15 * inch])
    t.setStyle(TableStyle([("VALIGN", (0, 0), (-1, 0), "BOTTOM"), ("VALIGN", (0, 1), (-1, 1), "TOP"),
                           ("LEFTPADDING", (0, 0), (-1, -1), 2), ("RIGHTPADDING", (0, 0), (-1, -1), 2)]))
    return KeepTogether([t])


def table(rows, widths, header=True, fs=7.6):
    data = [[Paragraph(str(c), cellb if (header and i == 0) else cell) for c in r] for i, r in enumerate(rows)]
    t = Table(data, colWidths=widths, repeatRows=1 if header else 0)
    st = [("VALIGN", (0, 0), (-1, -1), "TOP"),
          ("LEFTPADDING", (0, 0), (-1, -1), 4), ("RIGHTPADDING", (0, 0), (-1, -1), 6),
          ("TOPPADDING", (0, 0), (-1, -1), 2.8), ("BOTTOMPADDING", (0, 0), (-1, -1), 2.8),
          ("LINEABOVE", (0, 0), (-1, 0), 0.9, INK), ("LINEBELOW", (0, -1), (-1, -1), 0.9, INK),
          ("LINEBELOW", (0, 0), (-1, -2), 0.3, HAIR)]
    if header:
        st.append(("LINEBELOW", (0, 0), (-1, 0), 0.6, INK))
    t.setStyle(TableStyle(st))
    return t


def footer(canvas, doc):
    canvas.saveState()
    canvas.setStrokeColor(HAIR); canvas.setLineWidth(0.5)
    canvas.line(0.85 * inch, 0.62 * inch, letter[0] - 0.85 * inch, 0.62 * inch)
    canvas.setFont("Plex", 7)
    canvas.setFillColor(GRAY)
    canvas.drawString(0.85 * inch, 0.45 * inch, "Two high-altitude balloon flights  \u00b7  Charlie Cohen  \u00b7  github.com/charliemcohen/hab")
    canvas.setFont("Plex-SemiBold", 7)
    canvas.setFillColor(INK)
    canvas.drawRightString(letter[0] - 0.85 * inch, 0.45 * inch, f"{doc.page}")
    canvas.restoreState()


def fmt_t(s):
    return s[11:16] if len(s) >= 16 else s


def fmt_ts(s):
    return s[11:19]


story = []
A = story.append

g1 = N["gps_gaps"][0]
rel = "19:42:55 UTC"
burst_t = fmt_ts(N["burst_t"]) + " UTC"

A(P("ENGINEERING REPORT \u00b7 NOVEMBER 2026", kicker))
A(P(f"Two high-altitude balloon flights: payload design, flight data, and a reaction-wheel experiment", title))
A(P(f"Charlie Cohen \u00b7 Santa Monica, California \u00b7 Raw data, firmware, and analysis scripts: "
    f"<link href='https://github.com/charliemcohen/hab' color='{ACC_HEX}'>github.com/charliemcohen/hab</link>", sub))
_stats = [(f"{N['burst_alt_m']:,.0f} m", f"Flight 2 peak (GPS), {N['burst_alt_ft']:,} ft"),
          (f"{N['f1_peak_m']:,} m", f"Flight 1 peak (GPS), {N['f1_peak_ft']:,} ft"),
          (f"{N['s_records_total']:,}", "Flight 2 science records, about 1 Hz"),
          (f"{N['f2_straight_km']} km", "Flight 2, launch to last GPS fix")]
_st = Table([[Paragraph(v, statn) for v, _ in _stats], [Paragraph(l, statl) for _, l in _stats]],
            colWidths=[1.7 * inch] * 4)
_st.setStyle(TableStyle([("LINEABOVE", (0, 0), (-1, 0), 1.6, ACC), ("LEFTPADDING", (0, 0), (-1, -1), 0),
                         ("RIGHTPADDING", (0, 0), (-1, -1), 8), ("TOPPADDING", (0, 0), (-1, 0), 7),
                         ("BOTTOMPADDING", (0, 0), (-1, 0), 0), ("BOTTOMPADDING", (0, 1), (-1, 1), 2),
                         ("VALIGN", (0, 0), (-1, -1), "TOP")]))
A(_st)
A(Spacer(1, 14))
A(P(f"1. Summary", h1))
A(P(f"I launched a weather-balloon payload three times in 2026. On the first attempt, the swivel connecting the parachute to the "
    f"payload broke when I released the balloon a second time, and the balloon and parachute flew away without the payload. After I redesigned the flight train, both later flights climbed past 30 km and I "
    f"recovered both payloads. Flight 1 reached {N['f1_peak_m']:,} m ({N['f1_peak_ft']:,} ft) with a GPS logger and a camera. Flight 2 "
    f"carried ten environmental sensors, a two-Arduino logger, three cameras, and a single-axis reaction wheel, and reached "
    f"{N['burst_alt_m']:,.1f} m ({N['burst_alt_ft']:,} ft). The flight-log numbers in this report come from the raw files in the repository, "
    f"and the scripts in analysis/ regenerate every figure and calculated value. Bench values, camera timings, and my own "
    f"observations are labeled as such."))
A(P(f"The main results from Flight 2 are a near-continuous pressure, temperature, light, and acceleration record at about 1 Hz "
    f"({N['s_records_total']:,} records, {N['s_records_after_release']:,} of them after release), a GPS track with one large gap, "
    f"and a clear picture of which parts of the payload worked and which did not. The reaction wheel launched running and worked in a "
    f"post-flight ground test, but nothing recorded it in flight, so its in-flight performance is unknown."))
A(table([
    ["", "Failed launch", "Flight 1", "Flight 2"],
    ["Date", "Before Aug 2026", "Aug 7, 2026", "Oct 28, 2026"],
    ["Site", "Mojave Desert, CA", "Lucerne Dry Lake, CA", "Stoddard Valley OHV Area near Barstow, CA"],
    ["Balloon", "Latex weather balloon", "Latex weather balloon", "Kaymont TX1200"],
    ["Payload", "XPS foam box: electronics, tracker, camera", "XPS foam box: GPS logger, SPOT Trace, action camera",
     "XPS foam box: 10 sensors, Mega + Uno logger, GPS, SPOT Trace, Mobius Maxi 4K, 2 RunCam Thumb 2, reaction wheel; box about 3.89 lb before external hardware"],
    ["Peak altitude (GPS)", "Not measured (a few meters)", f"{N['f1_peak_m']:,} m ({N['f1_peak_ft']:,} ft) at {fmt_ts(N['f1_peak_t'])} UTC",
     f"{N['burst_alt_m']:,.1f} m ({N['burst_alt_ft']:,} ft) at {burst_t}"],
    ["Outcome", "Swivel broke on the second release; balloon and parachute lost", f"Recovered; {N['f1_straight_km']} km ({N['f1_straight_mi']} mi) from launch",
     f"Recovered; {N['f2_straight_km']} km ({N['f2_straight_mi']} mi) from launch, about {N['f2_baker_km']} km ({N['f2_baker_mi']} mi) east of Baker, CA"],
    ["What changed next", "Stronger swivels, two lines, and extra lines through the foam body", "Added the science logger and the reaction wheel",
     "Next: log the wheel, add a swivel or more wheel authority, fix the GPS gap and the camera dropout (section 8)"],
], [1.05 * inch, 1.55 * inch, 1.8 * inch, 2.4 * inch]))
A(P(f"Table 1. Mission summary. Altitudes are GPS altitude above mean sea level.", cap))

A(P(f"2. The failed launch and the flight-train redesign", h1))
A(P(f"The first launch failed because of the one part I had treated as simple: the connection between the parachute and the payload. "
    "I had spent my time on the electronics and the tracking, and a single swivel joined the parachute line to the payload."))
A(P("On the first "
    "release the balloon was underfilled, so the payload lifted only a few meters and drifted until I caught it. I added helium and released "
    "it again, and as the line pulled tight to lift the payload, the swivel broke. The balloon and parachute flew away without the payload. "
    "There is no flight data from that attempt, only a short video of the payload drifting before I caught it."))
A(P(f"The redesign adds redundancy where the first attempt had none and replaces the part that failed. Starting with Flight 1, I used stronger "
    f"swivels than the one that broke. Two lines now run from the flight train to the payload, and extra lines are "
    f"routed through the foam body of the box itself, so the added lines reduce the risk of losing the payload to a single attachment failure (Figure {F('fig02_rigging_before_after.png')}). "
    f"This rigging flew on both later flights without a problem, and the recovery photos (Figures {F('10_recovery.jpg')} and {F('12_recovery_photo.jpg')}) show the bridle lines still "
    "in place across the lid after landing. Next time I would load-test the whole flight train with the payload hanging "
    "from it before filling the balloon, instead of trusting part ratings on paper. I would also check free lift before release so the balloon is "
    "filled right the first time."))
A(fig("fig02_rigging_before_after.png", "Rigging before and after the failed launch (schematic, not to scale). The after panel shows "
      f"Flight 2, drawn from my description and the recovery photos (Figures {F('10_recovery.jpg')} and {F('12_recovery_photo.jpg')}).", width=5.0 * inch))

A(P(f"3. Payload hardware", h1))
A(P(f"The Flight 2 payload is a pink XPS foam box wrapped in reflective insulation and black tape (Figures {F('01_payload_open.jpg')} and {F('02_payload_closed.jpg')}). The box weighed "
    f"about 3.89 lb before the external hardware was attached; I did not record the all-up flight mass. Inside are the Arduino Mega 2560, "
    f"the Arduino Uno with its GPS logger shield, and the PCA9548 multiplexer with the sensor breakouts on STEMMA QT cables. The reaction "
    f"wheel sits over its motor and controller board. Two Nitecore USB power banks (one runs the wheel's Nano), a LiPo battery that drives the "
    f"wheel's motor through the G2 driver, and two battery holders supply power. The cameras are a Mobius Maxi 4K and two RunCam Thumb 2s. The balloon was a Kaymont TX1200. The parachute model and line specifications were not recorded."))
A(fig("fig01_system_diagram.png", "Flight 2 system block diagram. Source: my build notes and the Flight 2 firmware.", width=4.8 * inch))
A(P(f"Sensor placement comes from the mux map in the Mega firmware: the bottom exterior cluster holds MCP9808 #3, TMP119, MS8607, AHT20, "
    "and BMP390; the VEML7700 and LTR390 face up; MCP9808 #1 measures interior air; MCP9808 #2 sits in the battery area; and the LIS3DH "
    "accelerometer is on the rigid interior deck. The exterior cluster is not shielded from the sun or ventilated, which matters for the "
    "temperature results in section 5."))
A(photos2(["01_payload_open.jpg", "02_payload_closed.jpg"],
          ["Flight 2 payload with the lid off before launch: Mega (left), Uno with GPS shield (right), mux and TMP119 (upper left), "
           "reaction wheel (center), two power banks, Mobius (bottom left), RunCam (bottom right).",
           "Flight 2 payload closed and wrapped, at the Stoddard Valley launch site during balloon fill, Oct 28, 2026."]))
A(photos2(["05_electronics_closeup.jpg", "04_reaction_wheel.jpg"],
          ["Science electronics on the bench before installation: Mega 2560 R3, Uno with Adafruit GPS logger shield, PCA9548 mux, and sensor breakouts.",
           "Reaction wheel installed in the payload (top view): 4-spoke wheel over the motor, driver, and Nano."]))

A(P(f"4. Avionics and data flow", h1))
A(P(f"The Mega sweeps the sensors once a second through the multiplexer, builds one record, and sends it to the Uno over I2C as a "
    f"CRC-16 frame in 28-byte chunks with an acknowledgement for each chunk (Figure {F('fig03_avionics_dataflow.png')}). The Uno writes the record to the SD card as an "
    f"S row and also writes every checksum-valid GGA and RMC sentence from its own shield GPS as a G row. The Uno uses the RTClib library only for date "
    f"arithmetic on the GPS time; there is no clock chip on the payload. Firmware versions: Mega 1.2.0-I2C, Uno SCI-UNO-1.3.0-GPS-UNO "
    f"(printed in the first record of SCI_000.CSV), and the reaction wheel HAB-RW 1.2.0-ground."))
A(P(f"The log has {N['s_records_total']:,} S rows, {N['g_rows']:,} G rows, {N['e_rows']} E (event) rows, two B (boot) rows, and one D "
    f"(sensor detection) row. Each S row carries a test flag and three hex bitmasks: valid (sensor read succeeded), oor (value outside the "
    f"expected range but still logged), and synth (placeholder values from bench-test mode). A failed read leaves the field empty, never zero. "
    f"Every record in this file has test = 0 and synth = 0, so nothing had to be excluded. The I2C link was not lossless: the Uno logged "
    f"{N['seq_gap_events']} SEQ_GAP events totalling {N['seq_gap_missing']} missing Mega sequence numbers out of {N['mega_records_generated']:,} "
    f"generated, about {N['seq_gap_pct']}%."))
A(P(f"One thing the log does not contain is a GPS fix from the Mega. The firmware reserves gps_* columns for a receiver on the Mega's "
    f"Serial2, but gps_fix is 0 in every one of the {N['s_records_total']:,} records and the other gps_* fields are blank. All GPS positions in the science log come from the Uno shield's receiver in the G rows; SPOT Trace positions come from the KML "
    f"and are marked separately. The reaction wheel is a separate system: its Nano streams 10 Hz telemetry only over USB "
    f"serial, which nothing recorded in flight, and keeps an EEPROM summary of only its most recent armed run."))
A(fig("fig03_avionics_dataflow.png", "Flight 2 avionics and data flow. Source: HAB_Mega_Science.ino, HAB_Uno_Logger.ino, sci_link.h, HAB_RW_Controller.ino.", width=6.1 * inch))

A(P(f"5. Flight 2 results", h1))
A(P(f"5a. Altitude, release, and burst", h2))
A(P(f"Release was at {rel}. The accelerometer recorded a {N['release_jolt_g']} g jolt at that second, and GPS altitude starts climbing from "
    f"{N['launch_alt_m']:.0f} m three seconds later. The highest GPS fix was {N['burst_alt_m']:,.1f} m ({N['burst_alt_ft']:,} ft) at {burst_t} by the "
    f"receiver's own timestamp; the next fix is lower, so burst was at about that time. Two other sensors agree: the MS8607 reached its minimum of "
    f"{N['ms_p_min_hpa']} hPa at {fmt_ts(N['ms_p_min_t'])} UTC (logger clock), and starting at {fmt_ts(N['acc_freefall_first_t'])} UTC (logger clock) "
    f"{N['acc_freefall_n_below_0p2']} of the next 5 accelerometer samples read below 0.2 g, with a minimum of {N['acc_freefall_min_g']} g (the other "
    f"read about 0.6 g). That is consistent with free fall, but sampling once a second cannot show how long it lasted. The ascent took "
    f"{N['ascent_minutes']} min at a mean {N['ascent_mean_rate_mps']} m/s."))
A(P(f"The last GPS fix was at {fmt_ts(N['last_fix_t'])} UTC at {N['last_fix_lat']:.4f} N, {abs(N['last_fix_lon']):.4f} W, {N['last_fix_alt_m']:,.0f} m, "
    f"with {N['last_fix_sats']} satellites; GPS heights this late in the descent are suspect. A SPOT Trace report at "
    f"{fmt_ts(N['spot_landing_t'])} UTC, {N['spot_landing_to_last_fix_m']} m from that fix at {N['spot_landing_lat']:.5f} N, "
    f"{abs(N['spot_landing_lon']):.5f} W, still lists 10.3 mph; the next report, at 22:49:03 UTC, lists 0.0 mph at 1,238 m, and USGS elevation "
    f"data put the ground there at about 1,245 m. The payload landed between about 22:43 and 22:49 UTC; I cannot pin the exact time. The "
    f"straight-line distance from launch to the last GPS fix is {N['f2_straight_km']} km ({N['f2_straight_mi']} mi), and the path through the "
    f"retained fixes, with straight lines across the gaps, is {N['f2_along_track_km']} km, a lower bound. I excluded GPS fixes before 19:20 UTC: "
    f"the receiver had {N['gps_pre1920_sats_range'][0]} to {N['gps_pre1920_sats_range'][1]} satellites then and reported altitudes between "
    f"{N['gps_pre1920_alt_range'][0]:.0f} and {N['gps_pre1920_alt_range'][1]:.0f} m while sitting on the ground."))
A(P(f"The GPS lost its fix for {g1['minutes']} min on the way up, from {fmt_ts(g1['start'])} UTC at {g1['alt_start']:,.0f} m to "
    f"{fmt_ts(g1['end'])} UTC at {g1['alt_end']:,.0f} m, and three more times on descent (5.2, 3.1, and 10.1 min). The receiver kept sending "
    f"no-fix sentences the whole time, so it was alive but not tracking. I do not know why. The same receiver family worked to {N['f1_peak_m']:,} m on "
    f"Flight 1 and to {N['burst_alt_m']:,.0f} m later on this flight, so an altitude limit does not explain it. Where GPS is missing, I use pressure "
    f"altitude from the MS8607 through the 1976 U.S. Standard Atmosphere, and I label every such value as an estimate."))
A(fig("fig04_altitude_time.png", "Flight 2 GPS altitude versus time with release, burst, GPS gaps, and the Mobius recording window. "
      "Camera times are approximate (camera clock about 15 min 7 s fast, within about 1 min). Source: SCI_000.CSV.", width=6.0 * inch))
A(P(f"5b. Ascent and descent rate", h2))
A(P(f"The ascent rate was below the 5.0 m/s I had assumed in the prediction for almost the whole climb: about 2.8 to 3.2 m/s between 4 and "
    f"10 km and 3.8 to 4.0 m/s between 21 and 30 km (Figure {F('fig05_rate_vs_altitude.png')}). The descent started at about {N['descent_rate_above_25km_mps']} m/s above 25 km, "
    f"where the low air density makes the descent much faster, and settled to {N['descent_rate_3_7km_mps']} m/s between 7 and 3 km. GPS "
    f"heights near the end of the descent are suspect, so I do not use them to judge the landing."))
A(fig("fig05_rate_vs_altitude.png", "Ascent and descent rate from GPS altitude in 500 m bins. Lines are broken where the GPS had no fix. Source: SCI_000.CSV G rows.", width=6.0 * inch))
A(P(f"5c. Pressure against the standard atmosphere", h2))
A(P(f"On the ascent, above 2 km, with GPS height converted to geopotential height, the MS8607 tracked the 1976 U.S. Standard Atmosphere within {100*(N['isa_ratio_above_2km_min']-1):+.0f}% to "
    f"{100*(N['isa_ratio_above_2km_max']-1):+.0f}% wherever GPS altitude was available (Figure {F('fig06_pressure_vs_altitude.png')}), from {N['ms_p_max_hpa']:.0f} hPa on the ground "
    f"to {N['ms_p_min_hpa']} hPa at burst. The firmware marks anything below 10 hPa as outside the sensor's normal range, so the last readings "
    f"near burst are at the edge of what the part is rated for. The BMP390 followed the MS8607 closely until {fmt_ts(N['bmp_diverge_t'])} UTC at "
    f"{N['bmp_diverge_ms_p_hpa']:.0f} hPa and then stopped tracking: it read between {N['bmp_band_when_ms_below_100'][0]:.0f} and "
    f"{N['bmp_band_when_ms_below_100'][1]:.0f} hPa for the rest of the ascent while the MS8607 fell to 10 hPa. Its qualified range is "
    f"300 to 1250 hPa in both the firmware and the datasheet, so the divergence below about 100 hPa is consistent with operating outside its qualified range; the firmware flagged {N['oor_counts']['BMP']:,} of its readings "
    f"as out of range."))
A(fig("fig06_pressure_vs_altitude.png", "MS8607 and BMP390 pressure versus GPS altitude with the 1976 U.S. Standard Atmosphere. Source: SCI_000.CSV.", width=6.0 * inch))
A(P(f"5d. Temperature", h2))
A(P(f"The coldest reading of the flight was {N['t_ext_min_c']} °C on MCP9808 #3 at {fmt_ts(N['t_ext_min_t'])} UTC on descent, at "
    f"{N['t_ext_min_p_hpa']:.0f} hPa (pressure altitude about {N['t_ext_min_palt_m']/1000:.1f} km). That is below the MCP9808's rated range, which ends at "
    f"−40 °C, so its {N['oor_counts']['MCP3']} readings colder than −40 °C are outside the part's specification. The TMP119 and MS8607 minima were "
    f"{N['t_tmp119_min_c']:.1f} and {N['t_ms_min_c']:.1f} °C. On ascent the exterior minimum was only {N['t_ext_asc_min_c']} °C at "
    f"{N['t_ext_asc_min_p_hpa']:.0f} hPa (about {N['t_ext_asc_min_palt_m']/1000:.1f} km), and above that the exterior sensors warmed steadily "
    f"to between {N['t_ext_near_burst_c'][0]} and {N['t_ext_near_burst_c'][1]} °C near burst, where the standard atmosphere is "
    f"{N['t_isa_30km_c']} °C (Figure {F('fig07_temperature_vs_altitude.png')}). I do not believe those high-altitude readings are air temperature. The cluster hangs under the box "
    f"in full sun with no shield and no forced ventilation, and at a few hectopascals there is almost no convection to carry away absorbed sunlight. The "
    f"descent profile, taken at 8 to 40 m/s with much more airflow, reads much colder at the same pressure altitude; these are uncalibrated sensor readings, not air temperatures. The cold layer is "
    f"visible in both profiles between about 170 and 190 hPa, which is where I would expect the tropopause."))
A(P(f"Interior air (MCP9808 #1) was {N['t_int_at_release_c']:.1f} °C at release after the box sat in the sun, fell to about "
    f"{N['t_int_near_burst_c']:.0f} °C near burst, and reached its minimum of {N['t_int_min_c']} °C at {fmt_ts(N['t_int_min_t'])} UTC on "
    f"descent. The foam and the electronics' own heat kept the inside above the outside by 20 to 40 °C through the coldest part of the flight. "
    f"MCP9808 #2 in the battery area returned no readings at all (section 6), so I have no battery temperature."))
A(fig("fig07_temperature_vs_altitude.png", "Exterior and interior temperature versus pressure altitude from the MS8607 (an estimate, used because "
      "GPS was missing between 10.5 and 21 km on ascent). Source: SCI_000.CSV.", width=6.0 * inch))
A(P(f"5e. Horizontal wind from GPS", h2))
A(P(f"Treating the payload's horizontal speed over the ground as the wind, the balloon drifted at about {N['wind_5_10km_mps']} m/s between 5 and "
    f"10 km, slowed to about {N['wind_23_24km_mps']} m/s near 23 to 24 km, and then sped up again to a median of {N['wind_max_mps']} m/s in the "
    f"{N['wind_max_bin_km']} km bin just below burst (Figure {F('fig08_wind_vs_altitude.png')}). The GPS gap means there is no wind data between 10.5 and 21 km. The track "
    f"ran east-northeast the whole flight (Figure {F('fig09_ground_tracks.png')})."))
A(fig("fig08_wind_vs_altitude.png", "Horizontal speed over the ground from consecutive GPS fixes during ascent, 500 m bins. Source: SCI_000.CSV G rows.", width=5.9 * inch))
A(P(f"5f. Ground tracks", h2))
A(P(f"Flight 1 flew west from Lucerne Dry Lake and landed {N['f1_straight_km']} km ({N['f1_straight_mi']} mi) away; Flight 2 flew east-northeast from "
    f"Stoddard Valley and landed {N['f2_straight_km']} km ({N['f2_straight_mi']} mi) away. Both tracks have gaps. On the Flight 2 map I show the SPOT Trace fixes "
    f"as a separate marker: {N['spot_points_in_ascent_gap']} of them fall inside the {g1['minutes']}-min GPS gap, which is why the gap's straight "
    f"connector is a fair stand-in for the real path. The SPOT export is stamped in UTC minus 5 hours; I checked the offset against release."))
A(fig("fig09_ground_tracks.png", "Ground tracks of both flights. Observed GPS fixes are solid black, gaps are dotted connectors, SPOT fixes are open "
      "circles. Source: Aug_7_2026_GPS_Fixes.csv, SCI_000.CSV, Second_Successful_Launch.kml (SPOT folder).", width=6.0 * inch, maxh=4.6 * inch))
A(P(f"5g. Prediction against reality", h2))
pb, pl = N["pred_burst"], N["pred_landing"]
A(P(f"The prediction run I used, which assumed a 19:44 UTC launch from the site, also assumed a {N['pred_desc'].split(',')[0].split(':')[1].strip()} m/s ascent, "
    f"a 7.0 m/s descent, and burst at 32,000 m. It put burst at {pb['lat']:.4f} N, {abs(pb['lon']):.4f} W at {pb['time'][11:19]} UTC and landing at "
    f"{pl['lat']:.5f} N, {abs(pl['lon']):.5f} W at {pl['time'][11:19]} UTC. The real burst was {N['pred_burst_vs_actual_km']} km from the predicted one and "
    f"the SPOT landing position was {N['pred_landing_vs_spot_km']} km from the predicted landing, and the predicted landing time was before the real burst."))
A(P(f"The slower ascent is a plausible major contributor to the miss, though I have not separated it from forecast winds, timing, and burst "
    f"altitude. The prediction had the balloon aloft for {N['pred_ascent_min']} min on the way up; it actually took "
    f"{N['ascent_minutes']} min at {N['ascent_mean_rate_mps']} m/s, so it rode the wind for {N['ascent_minutes']-N['pred_ascent_min']:.0f} extra minutes "
    f"and drifted {N['f2_launch_to_burst_km']} km from launch to burst instead of the predicted {N['pred_launch_to_burst_km']} km. The descent legs "
    f"were close: {N['pred_burst_to_landing_km']} km in {N['pred_descent_min']} min predicted, {N['f2_burst_to_last_km']} km in "
    f"{N['actual_descent_min']} min from the highest fix to the first SPOT report near the landing site (touchdown time is uncertain). The predicted burst altitude was also 1,150 m too high. The KML file holds "
    f"{N['n_prediction_paths_in_kml']} prediction runs; only the 19:44 run is used here, and its landing placemark "
    f"carries a ground altitude of {N['kml_landing_point_alt_m']} m that was copied from the prediction, not measured; the ground at the landing "
    f"site is about 1,245 m (USGS)."))
A(fig("fig10_prediction_vs_actual.png", "Predicted path (dashed) against the observed GPS track and SPOT fixes for Flight 2. "
      "Source: Second_Successful_Launch.kml (Prediction folder) and SCI_000.CSV.", width=6.0 * inch))
A(P(f"5h. Light and UV", h2))
A(P(f"Both top-facing sensors worked for the whole flight and neither saturated (VEML7700 raw maximum {N['veml_als_raw_max']:,} of 65,535). "
    f"The VEML7700 read about {N['lux_low_ascent']/1000:.1f} klux below 2 km and {N['lux_top']/1000:.1f} klux near burst; the LTR390 UV count rose "
    f"from about {N['uv_low_ascent']} below 2 km to {N['uv_top']} near burst (Figure {F('fig11_light_uv_vs_altitude.png')}). Both sensors turned with the payload and their view of the "
    f"sun changed as it did, so I read these as usable but qualitative: the UV increase with altitude is the expected direction, and the "
    f"illuminance is nearly flat."))
A(fig("fig11_light_uv_vs_altitude.png", "VEML7700 illuminance and LTR390 UV counts versus GPS altitude on ascent. Source: SCI_000.CSV.", width=5.9 * inch))
A(P(f"5i. Acceleration", h2))
A(P(f"After the release jolt, the acceleration magnitude on ascent never exceeded {N['acc_ascent_max_g']} g (one sample at {fmt_ts(N['acc_ascent_max_t'])} UTC, "
    f"34 s after release); the 99th percentile was {N['acc_ascent_p99_g']} g and only {N['acc_ascent_n_over_1p17']} sample was above 1.17 g. The one large "
    f"event is burst: low-g readings consistent with free fall, then a cluster of 1.5 to {N['acc_post_burst_max_g']} g readings from {fmt_ts(N['acc_post_burst_first_t'])} to "
    f"{fmt_ts(N['acc_post_burst_last_t'])} UTC as the parachute and the falling balloon remains loaded the box (Figure {F('fig12_accel_magnitude.png')}). The accelerometer showed no "
    f"large acceleration around {fmt_ts(N['shake_utc'])} UTC, when the Mobius footage shakes violently just before it stops: the maximum in that "
    f"three-minute window is {N['acc_shake_window_max_g']} g. However, its 1 Hz sample rate cannot rule out brief vibration or rotation."))
A(fig("fig12_accel_magnitude.png", "Acceleration magnitude from the LIS3DH for the whole flight (top) and around burst (bottom). Source: SCI_000.CSV.", width=6.0 * inch))

_gap_min = sum(g["minutes"] for g in N["gps_gaps"])
_flight_min = (datetime.fromisoformat(N["last_fix_t"]) - datetime.fromisoformat("2026-10-28 19:42:55")).total_seconds() / 60
A(P(f"6. Sensor and camera reliability audit", h1))
A(P(f"Of the ten sensors, one failed completely, the BMP390 diverged once it left its qualified range, and the two humidity channels produced nothing "
    f"usable. The rest returned a reading in every one of the {N['s_records_total']:,} records, though several logged values outside their rated ranges (Table 2). Two of the three cameras shut down about two minutes after launch, and the third missed burst. "
    f"The GPS worked but lost its fix for {_gap_min:.0f} of the {_flight_min:.0f} minutes from release to the last GPS fix, "
    f"about {100 * _gap_min / _flight_min:.0f}% of the flight. The details are in Table 2."))
vc = N["valid_counts"]
A(table([
    ["Item", "Status", "Evidence", "Likely cause", "What I will change"],
    ["MCP9808 #1 (interior air)", "Worked", f"{vc['MCP1']:,}/{N['s_records_total']:,} valid; 37 to −19 °C", "", ""],
    ["MCP9808 #2 (battery area)", "Failed", f"{vc['MCP2']} valid readings; valid bit never set, column empty",
     "Unknown: wiring, its mux channel, or the part itself", "Bench-verify every mux channel in the flight box before closing the lid; log the detect mask on every boot"],
    ["MCP9808 #3 (exterior)", "Worked", f"{vc['MCP3']:,} valid; min {N['t_ext_min_c']} °C; {N['oor_counts']['MCP3']} readings flagged below −40 °C spec", "Readings below the part's rated range are flagged, not dropped", "Shield from sun; aspirate"],
    ["TMP119 (exterior)", "Worked", f"{vc['TMP119']:,} valid; min {N['t_tmp119_min_c']:.1f} °C; {N['oor_counts']['TMP119']} flagged out of range", "", "Same"],
    ["MS8607 pressure/temp", "Worked", f"{N['ms_p_max_hpa']:.0f} to {N['ms_p_min_hpa']} hPa; on ascent, within {100*(N['isa_ratio_above_2km_min']-1):+.0f} to {100*(N['isa_ratio_above_2km_max']-1):+.0f}% of standard atmosphere (not a calibration)", "Below 10 hPa is outside the part's normal range", "Keep as primary barometer"],
    ["MS8607 humidity", "Unusable", f"{N['ms_rh_negative_count']:,} negative readings on ascent; {N['oor_counts']['MS_RH']:,} flagged out of range", "Not isolated: cold, low pressure, and sensor response are all possible", "Drop or replace with a sensor rated for the stratosphere"],
    ["AHT20 temp/humidity", "Partial", f"Temperature tracked the others; humidity read 0.0 for {int(100*N['aht_rh_zero_frac_flight'])}% of the flight (ground median {N['aht_rh_ground_median']}%); {N['oor_counts']['AHT']} flagged out of range", "Not isolated; cold is one possibility", "Treat as temperature only"],
    ["BMP390", "Range limit", f"Tracked MS8607 to {N['bmp_diverge_ms_p_hpa']:.0f} hPa, then stayed at {N['bmp_band_when_ms_below_100'][0]:.0f} to {N['bmp_band_when_ms_below_100'][1]:.0f} hPa; minimum {N['bmp_p_min_hpa']} hPa", "Qualified range 300 to 1250 hPa", "Keep for the lower atmosphere only"],
    ["VEML7700 / LTR390", "Worked", f"{vc['VEML']:,} valid each; no saturation", "", "Add a fixed-orientation reference or a sun shield"],
    ["LIS3DH accelerometer", "Worked", f"{vc['LIS']:,} valid; release and burst both visible", "", "Raise sample rate for impact events"],
    ["Uno shield GPS", "Partial", f"{N['gps_fixes_used']:,} fixes used; {g1['minutes']} min gap on ascent (10.5 to 21.0 km) plus 18 min of descent gaps", "Unknown; receiver stayed alive but reported no fix", "Second receiver on the Mega (its columns were empty); log satellite count and fix status continuously"],
    ["Mega GPS columns", "No data", f"gps_fix = 0 in all {N['s_records_total']:,} S rows; other gps_* fields blank", "No receiver delivering fixes to the Mega's Serial2", "Connect and verify before flight"],
    ["Mega to Uno I2C link", "Partial", f"{N['seq_gap_events']} SEQ_GAP events, {N['seq_gap_missing']} records lost ({N['seq_gap_pct']}%)", "Chunk retries exhausted or Uno busy writing SD", "Buffer and retry on the Mega; log link statistics"],
    ["Mobius Maxi 4K", "Partial", f"Continuous from before release until about {fmt_t(N['cam_stop_utc'])} UTC (about {N['cam_stop_palt_m']/1000:.0f} km pressure altitude), off for {N['cam_gap_minutes']:.0f} min, resumed about {fmt_t(N['cam_resume_utc'])} UTC near {N['cam_resume_palt_m']/1000:.1f} km; burst not recorded",
     "Possibly the camera itself failing or rattling; the accelerometer shows no large acceleration during the shake, but 1 Hz sampling cannot rule out brief vibration", "Insulate and tie down the camera; add a status LED in its view"],
    ["RunCam Thumb 2 (two)", "Failed early", "Both stopped about 2 min after launch", "Not recorded; cold or power are the candidates", "Test at low temperature; power from the main banks"],
    ["SPOT Trace", "Worked", f"{N['spot_points_in_flight']} fixes in flight at about 5-min intervals; near-landing report {N['spot_landing_to_last_fix_m']} m from the last GPS fix (still listing 10.3 mph)", "", "Keep"],
], [1.15 * inch, 0.65 * inch, 1.95 * inch, 1.55 * inch, 1.5 * inch]))
A(P(f"Table 2. Reliability audit of every sensor and camera on Flight 2. Counts are from SCI_000.CSV; camera times are approximate.", cap))

A(P(f"7. Reaction wheel", h1))
A(P(f"7a. The problem", h2))
A(P(f"In the Flight 1 footage the camera's view turned slowly around the horizon for most of the ascent. I saw it; I did not measure it. A "
    "balloon payload on a line is free to twist, and the line winds and unwinds as the balloon rotates. I wanted the Flight 2 camera to hold a "
    "heading, so I built a single-axis reaction wheel to control yaw."))
A(P(f"7b. Design and control loop", h2))
A(P(f"The controller is an Arduino Nano reading an Adafruit BNO085 IMU and driving a Pololu 25D 20.4:1 geared motor with encoder through a Pololu "
    f"G2 18v17 driver. The wheel is a custom 4-spoke disk. The motor runs from its own LiPo battery through the G2 driver, and the Nano runs from a USB power bank. It is a bias-momentum design: the wheel idles at 75 rpm and makes torque by speeding up "
    f"or slowing down inside a 45 to 105 rpm band, never reversing. The control loop runs at 100 Hz, and the telemetry stream at 10 Hz. The firmware latches a fault and stops driving if the "
    f"payload tilts more than 15 degrees, and the fault stays latched until a serial R command clears it, which nothing could send in flight. The board "
    f"reported the closed-loop, sign-commissioned "
    f"configuration: BUILD_COMMISSIONING 0, SIGN_COMMISSIONED 1, HEADING_RATE_SIGN_VERIFIED 1, MDS = 1, SA = 1, BIAS = 1, firmware string "
    f"HAB-RW 1.2.0-ground, read back from the Nano with the T command after recovery (flight_run_summary.txt). I did not archive the exact binary "
    f"that flew. The source in the repository is the version I recovered, with its flags set to match that readback. With today's library versions "
    f"it compiles to 31,220 bytes, about 500 bytes over the standard Nano limit, so the flight build must have differed in a way I can't "
    f"reconstruct, such as older library versions."))
A(P(f"7c. Authority calculation", h2))
A(P(f"The wheel's moment of inertia is {N['rw_J_wheel']:.2e} kg·m² and the payload's is {N['rw_J_payload']} kg·m² (the wheel's from CAD, the payload's from a bifilar-pendulum test; the original measurement records are not in the repository), so the ratio "
    f"is {N['rw_ratio']}. The ±{N['rw_band_rpm']} rpm band around the bias therefore gives only about ±{N['rw_authority_dps']:.0f} deg/s of body-rate "
    f"authority. If the payload was turning once every 50 s, the unconfirmed video estimate in section 7f ({N['rw_obs_rate_dps']} deg/s), stopping it would need "
    f"about {N['rw_rpm_to_stop']:.0f} rpm of wheel speed change, about {N['rw_band_shortfall_x']} times the band. The balloon keeps twisting the "
    f"payload through the line during ascent, so a wheel alone will saturate unless the flight train decouples yaw with a swivel or the authority is much "
    f"larger. If the wheel spun down completely from 75 rpm, its momentum would end up in the payload, about {N['rw_dump_dps']} deg/s in an isolated-system "
    f"model. Both inertias are estimates, so these numbers are too; actual yaw and saturation in flight were not measured."))
A(P(f"7d. What worked", h2))
A(B(f"It flew installed in the Flight 2 payload (Figure {F('04_reaction_wheel.jpg')})."))
A(B(f"After recovery the board reported the closed-loop build with its motor and sensor directions checked (status readback)."))
A(B(f"At release the wheel was spinning: I saw it and heard it."))
A(B(f"The hardware and firmware survived the flight and the landing intact."))
A(B(f"After recovery, in a 1200 s ground test, the controller's heading error against its own IMU reference peaked at 1.70 deg (Table 3). "
    f"Absolute heading accuracy was not measured independently."))
A(B(f"Pre-flight bench tests verified the encoder, motor direction, and minimum PWM, and reached about 83 rpm (bench results; the logs are not in the repository)."))
A(P(f"7e. Ground test after recovery", h2))
A(P(f"The Nano's EEPROM keeps only its most recent armed run. I re-armed the wheel for a ground test after recovery, so the summary I read back "
    "describes that ground test and not the flight. The payload hung indoors from a foam board across a stepladder by a single line, a swivel, and a "
    f"four-line bridle to the box corners (Figures {F('03_test_bridle.jpg')} and {F('07_ground_test_rig.jpg')})."))
_t3 = (table([
    ["Ground test (not flight)", "Value"],
    ["Run length / time evaluated", "1200 s / 1190 s (the 10 s settle after capture is excluded)"],
    ["Peak heading error", "1.70 deg against the IMU's own reference"],
    ["Time within 2 deg / 3 deg", "1188 s / 1188 s, counted only while fully armed and unsaturated"],
    ["Time LIMITED / saturated", "2.0 s near a wheel-speed edge / 0 s"],
    ["Wheel speed range", "59.0 to 97.0 rpm (bias 75, soft band 45 to 105)"],
    ["How it ended", "Fault bit 5, F_TILT: payload tilt over 15 deg"],
], [2.0 * inch, 4.2 * inch]))
A(KeepTogether([_t3, P(f"Table 3. Ground-test EEPROM summary (Nano E command), decoded in flight_run_summary.txt.", cap)]))
A(photos2(["03_test_bridle.jpg", "07_ground_test_rig.jpg"],
          ["Four-line bridle used for the ground test: lines from the box corners cross to a zip-tie loop above the center.",
           "Post-flight ground test rig: payload hung from a foam board across a stepladder by one line, a swivel, and the four-line bridle."]))
A(P(f"7f. Video evidence (unconfirmed)", h2))
A(P(f"I measured payload yaw from the Mobius footage two ways. Frame-to-frame tracking failed: all {N['rot_windows_poor_match']} of {N['rot_windows']} windows "
    f"are flagged as matched poorly, and the heading traces step only when something bright crosses the frame (Figure {F('fig14_tracker_example_4min_14min.png')}). I do not use the script's "
    f"steady/swinging/rotating labels, its deg/s, or its swing amplitudes. The second method does not depend on tracking: it asks how long until the view "
    f"repeats. It found a repeat in {N['rot_windows_with_repeat']} windows, {N['rot_windows_40_52s']} of them between 40 and 52 s, spread from "
    f"{N['rot_40_52_tmin_range'][0]:.0f} to {N['rot_40_52_tmin_range'][1]:.0f} min after the start of the footage (Figure {F('fig13_rotation_repeat_period.png')}). The three strongest repeats were "
    f"{N['rot_top3'][0][1]} s at +{N['rot_top3'][0][0]:.0f} min, {N['rot_top3'][1][1]} s at +{N['rot_top3'][1][0]:.0f} min, and {N['rot_top3'][2][1]} s at +{N['rot_top3'][2][0]:.0f} min. "
    f"With the heading drifting one way, a 50 s repeat would mean a full turn about every 50 s, about {N['rw_obs_rate_dps']} deg/s or {N['rw_obs_rot_per_min']} "
    f"rotations per minute. I have not verified this by eye, and I present it as suggestive, not as a result. Window times count from camera clock 12:58:04, "
    f"which is within a few seconds of release under the 15 min 7 s clock offset, with the same ±1 min uncertainty as every other camera time."))
A(fig("fig13_rotation_repeat_period.png", "View-repeat period found by the rotation script in each window of the Mobius footage. Windows with no repeat "
      "are shown at zero. Source: Flight_2_rotation.csv, Flight_2_upper_rotation.csv.", width=6.0 * inch))
A(fig("fig14_tracker_example_4min_14min.png", "Two windows from the rotation script (+4 min and +14 min) as examples of the frame-to-frame tracker failing: "
      "the heading trace (top panels) steps only when bright objects cross the frame, so its deg/s and swing values are not used. The lower panels are the "
      "view-repeat similarity curves, which are used. Source: Flight_2__4_min.png, Flight_2__14_min.png.", width=5.8 * inch))
A(P(f"7g. What is unknown, and hypotheses", h2))
A(P(f"In-flight heading control is unknown. Nothing recorded the wheel in flight, and I do not know when it stopped. What I know is this: it launched "
    "running, it worked in a ground test, and its in-flight performance is unknown. My application described the wheel as partially holding heading "
    "during ascent and losing control after burst; the data cannot support either part of that, and this report replaces it. The video suggests, "
    "without confirmation, that the payload may have turned about once every 40 to 50 s at times during the first 95 min of ascent."))
A(P(f"Possible causes, as hypotheses and not findings:"))
A(B(f"A tilt fault at release. The 2.53 g release jolt and the swing that follows could exceed 15 deg; the fault latches until a serial command clears it, which nothing could send in flight."))
A(B(f"A mid-flight reset that left the controller disarmed."))
A(B(f"A power problem, with no battery-voltage log to show it."))
A(B(f"If the estimated rotation rate is correct, the wheel's speed band was about {N['rw_band_shortfall_x']} times too small to stop it, so even a running wheel could have saturated."))
A(P(f"7h. Next-flight changes for the wheel", h2))
A(B(f"Log wheel telemetry to SD with timestamps, so the next report can answer the question this one cannot."))
A(B(f"Make faults clear themselves after a few seconds of normal readings instead of latching until a serial command clears them."))
A(B(f"Require tilt past the limit for about 2 s, not one reading, before faulting."))
A(B(f"Re-arm automatically after a reset."))
A(B(f"Log battery voltage."))
A(B(f"Increase authority with a heavier wheel or a wider speed band, or add a swivel so the balloon's twist does not reach the payload."))
A(B(f"Put a status LED in the camera's view."))

A(P(f"8. Next flight", h1))
A(P(f"Three changes follow directly from sections 5 to 7."))
A(P(f"<b>1. Record the reaction wheel and give it a chance to work.</b> The wheel is the one subsystem I cannot grade, because nothing logged it. I will log "
    f"its telemetry to SD, change the fault handling as listed in 7h, and add a swivel between the flight train and the payload. My hypothesis, which "
    f"needs a flight test, is that a swivel matters more than a bigger wheel: if it decouples the balloon's twist, the remaining disturbances "
    f"may be small enough for ±{N['rw_authority_dps']:.0f} deg/s of authority; without it, the calculation says about {N['rw_band_shortfall_x']} times the "
    f"band is needed."))
A(P(f"<b>2. Close the GPS gap.</b> One receiver lost its fix for {g1['minutes']} min on ascent and 18 min on descent, and the Mega's own GPS columns were "
    f"empty. I will fly two receivers, feed the second one to the Mega's Serial2 so the gps_* columns are populated, and log satellite count and fix quality "
    f"every second. I will also rate the ascent-rate assumption in the prediction honestly: both flights rose slower than the model, 3.42 m/s on Flight 2 "
    f"and about {N['f1_rate_4_10km_mps']} m/s at 4 to 10 km on Flight 1. The slower ascent likely contributed to the {N['pred_landing_vs_spot_km']} km landing "
    f"error, but I cannot separate its effect from errors in the wind forecast or burst altitude."))
A(P(f"<b>3. Fix the sensors and cameras that failed.</b> Shield the exterior temperature sensors and give them forced airflow so the "
    f"stratosphere readings mean something. Replace the humidity sensors with one rated for low pressure, or drop humidity. Keep the BMP390 for the "
    f"troposphere only. Check every mux channel with the flight box closed, so a repeat of the MCP9808 #2 failure shows up on the bench. Insulate and tie "
    f"down all three cameras after a cold-chamber test. Two of three cameras stopping at two minutes and the third missing burst is the single biggest loss of this flight."))
A(photos2(["09_liftoff.jpg", "11_altitude_frame.jpg"],
          [f"Mobius frame shortly after liftoff, camera clock 12:58:02 (about {rel.split()[0]} UTC). The launch site, car, and crew are below.",
           f"Mobius frame at camera clock 14:35:28, about {fmt_t(N['cam_alt_frame_utc'])} UTC and about {N['cam_alt_frame_palt_m']/1000:.1f} km pressure altitude. The lens is wide-angle; the horizon's curve is mostly lens distortion."]))
A(photos2(["10_recovery.jpg", "12_recovery_photo.jpg"],
          [f"Flight 2 payload where it landed, about {N['f2_baker_km']} km ({N['f2_baker_mi']} mi) east of Baker, CA: upright and intact, bridle lines across the lid.",
           "Second view of the Flight 2 landing site, with the parachute in the foreground."]))

A(P(f"Appendix A. Flight 1 notes", h1))
A(P(f"Flight 1 on Aug 7, 2026 carried only an Adafruit Ultimate GPS logging to its internal LOCUS memory, a SPOT Trace, and an action camera. I decoded "
    f"{N['f1_fixes']} fixes at {N['f1_interval_s']:.0f} s intervals in {N['f1_sessions']} logger sessions. The peak was {N['f1_peak_m']:,} m ({N['f1_peak_ft']:,} ft) "
    f"at {fmt_ts(N['f1_peak_t'])} UTC and the last fix is at {N['f1_landing_lat']:.4f} N, {abs(N['f1_landing_lon']):.4f} W, {N['f1_landing_alt_m']} m, at "
    f"{N['f1_last_t'][11:19]} UTC on Aug 8. The launch time cannot be computed from the log: the last ground fix is at {N['f1_last_ground_t'][11:19]} UTC and the "
    f"first airborne fix is {N['f1_launch_gap_min']:.0f} min later at {N['f1_first_air_alt_m']:,} m, and a second gap of 132 min hides the climb from 10 to 29.5 km. "
    f"The gaps line up with the logger's session boundaries. The straight-line distance is {N['f1_straight_km']} km ({N['f1_straight_mi']} mi); the distance along "
    f"the recorded fixes is {N['f1_along_track_km']} km, a lower bound because of the gaps. The ascent was slow, about {N['f1_rate_4_10km_mps']} m/s between 4 and "
    f"10 km and {N['f1_rate_29_32km_mps']} m/s over the last 2 km before burst."))

A(P(f"Appendix B. Parts list", h1))
A(table([
    ["Subsystem", "Parts"],
    ["Structure", "XPS foam box, reflective insulation wrap, black tape; Kaymont TX1200 balloon (Flight 2); parachute (model not recorded); lines (type not recorded)"],
    ["Logger", "Arduino Mega 2560 R3; Arduino Uno with Adafruit Ultimate GPS Logger Shield (SD card); PCA9548 I2C multiplexer; STEMMA QT cables"],
    ["Sensors", "MCP9808 ×3, TMP119, MS8607, BMP390, AHT20, VEML7700, LTR390, LIS3DH"],
    ["Tracking", "SPOT Trace satellite tracker; Adafruit Ultimate GPS (LOCUS logging, Flight 1)"],
    ["Cameras", "Mobius Maxi 4K; RunCam Thumb 2 ×2 (Flight 2); action camera (Flight 1)"],
    ["Reaction wheel", "Arduino Nano; Adafruit BNO085 IMU; Pololu 25D 20.4:1 gearmotor with encoder; Pololu G2 18v17 motor driver; custom 4-spoke wheel"],
    ["Power", "Nitecore USB power banks ×2 (one powers the reaction-wheel Nano); LiPo battery for the wheel motor; two battery holders"],
], [1.2 * inch, 5.0 * inch]))
A(Spacer(1, 6))
A(P(f"Appendix C. File and script index", h1))
A(table([
    ["File", "What it is"],
    ["data/flight2_2026-10-28/SCI_000.CSV", f"Flight 2 science log: {N['s_records_total']:,} S rows, {N['g_rows']:,} raw NMEA G rows, boot/detect/event rows"],
    ["data/flight2_2026-10-28/Second_Successful_Launch.kml", "Google Earth project: observed track split at gaps, 13 prediction runs, SPOT fixes, special-use airspace overlay"],
    ["data/flight1_2026-08-07/Aug_7_2026_GPS_Fixes.csv", f"Flight 1 GPS: {N['f1_fixes']} LOCUS fixes"],
    ["data/flight1_2026-08-07/First_Successful_Launch.kml", "Flight 1 decoded track; red connectors mark gaps"],
    ["firmware/HAB_Mega_Science/, firmware/HAB_Uno_Logger/", "Flight 2 Mega (1.2.0-I2C) and Uno (SCI-UNO-1.3.0-GPS-UNO) sketches with sci_link.h"],
    ["firmware/HAB_RW_Controller/", "Reaction-wheel firmware HAB-RW 1.2.0-ground, its telemetry format, and flight_run_summary.txt (ground test)"],
    ["rotation_results/", "Rotation-script output: Flight_2_rotation.csv, Flight_2_upper_rotation.csv, per-window plots"],
    ["analysis/common.py", "Loaders for every raw file; computes every flight number quoted in this report (compute_numbers)"],
    ["analysis/flight2_numbers.py", "Prints the numbers as JSON"],
    ["analysis/make_figures.py", "The drawn figures (block diagrams, charts, maps) as 300-dpi PNGs in docs/figures/"],
    ["analysis/make_altitude_csv.py", "docs/flight2_gps_altitude_1min.csv: GPS altitude at one-minute intervals from release"],
    ["analysis/build_report.py", "Builds this PDF from the numbers and figures"],
    ["docs/figures/", "Every figure as a separate 300-dpi PNG"],
], [2.6 * inch, 3.6 * inch]))
A(Spacer(1, 6))
A(P(f"Regulatory note: I notified the FAA before each flight. This report makes no claim about FAR Part 101 compliance or exemption.", small))
A(P(f"Not in the repository: the original video clips, the Flight 1 LOCUS binary, the wheel's raw telemetry and EEPROM bytes, CAD and "
    f"bifilar-pendulum records, motor bench logs, the archived flight binaries, and the post-landing files SCI_001 to SCI_006. Claims that "
    f"depend on them are labeled as observations or estimates.", small))

_out, _first = [], True
for _fl in story:
    if isinstance(_fl, Paragraph) and _fl.style is h1:
        if not _first:
            _r = HRFlowable(width="100%", thickness=0.5, color=LINE, spaceBefore=8, spaceAfter=8)
            _r.keepWithNext = 1
            _out.append(_r)
        _first = False
    _out.append(_fl)
story[:] = _out
doc = SimpleDocTemplate(str(OUT), pagesize=letter, leftMargin=0.85 * inch, rightMargin=0.85 * inch,
                        topMargin=0.75 * inch, bottomMargin=0.85 * inch,
                        title="Two high-altitude balloon flights, 2026", author="Charlie Cohen", subject="Technical report")
doc.build(story, onFirstPage=footer, onLaterPages=footer)
print("wrote", OUT, OUT.stat().st_size / 1e6, "MB")
