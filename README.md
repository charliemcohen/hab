# hab

Raw data and flight code from my high-altitude balloon project.

I launched three weather balloons in 2026. The first one failed before it ever left the ground. The other two climbed past 30 km (about 100,000 ft), and I recovered both payloads. Everything I recorded is in this repo, so you can check any flight measurement in my report against the original logs.

## The flights

| | Date | Where | Peak altitude (GPS) | Data here |
|---|---|---|---|---|
| Failed launch | Before Aug 2026 | Mojave Desert, CA | Not measured (a few meters) | No flight data. The underfilled balloon lifted the payload only a few meters before I caught it. After I added helium and released it again, the swivel between the parachute and the payload broke, and the balloon and parachute flew away without the payload. |
| Flight 1 | Aug 7, 2026 | Lucerne Dry Lake, CA | 31,761 m (104,203 ft) | GPS track, 930 fixes |
| Flight 2 | Oct 28, 2026 | Stoddard Valley OHV Area near Barstow, CA. Landed about 20 mi east of Baker, CA. | 30,849.2 m (101,211 ft) | Science log and GPS |

After the failed launch I upgraded to stronger swivels, switched from one line to two, and ran extra lines through the foam payload body, to reduce the risk of a single attachment failure. Both flights since then stayed together.

## What's in here

```
data/
  flight1_2026-08-07/   Aug_7_2026_GPS_Fixes.csv, First_Successful_Launch.kml
  flight2_2026-10-28/   SCI_000.CSV, Second_Successful_Launch.kml
  SHA256SUMS.txt        checksums, so you can tell the data hasn't been edited
firmware/
  HAB_Mega_Science/     Arduino Mega: reads the sensors
  HAB_Uno_Logger/       Arduino Uno: writes everything to the SD card
  HAB_RW_Controller/    Arduino Nano: reaction-wheel controller, plus its telemetry format
analysis/               scripts that turn the raw data into the report's numbers and figures, plus my video tools
docs/                   the technical report (PDF), every figure as a 300-dpi PNG, and the one-minute altitude CSV
rotation_results/       output of my video rotation script (Flight 2 Mobius footage)
photos/                 build, launch, recovery, and ground-test photos used in the report
fonts/                  IBM Plex Sans and Mono (SIL Open Font License), used by the figures and the report
```

## Flight 2 science log (`SCI_000.CSV`)

This is the main dataset. Flight 2 carried two Arduinos. The Mega reads every sensor through a PCA9548 I²C multiplexer and builds one record about once a second. It hands each record to the Uno over I²C, and the Uno writes it to the SD card along with the GPS data.

The first column tells you what kind of line it is:

- `B`: boot line, with the firmware version
- `S`: one science record, about once per second
- `G`: a raw NMEA sentence from the GPS, exactly as it came in

### Sensor columns

| Column(s) | Sensor | Units |
|---|---|---|
| `t_mcp1_c`, `t_mcp2_c`, `t_mcp3_c` | MCP9808 ×3: inside, by the batteries, outside on the bottom | °C |
| `t_tmp119_c` | TMP119 | °C |
| `ms_p_hpa`, `ms_t_c`, `ms_rh_pct` | MS8607 | hPa, °C, %RH |
| `bmp_p_hpa`, `bmp_t_c` | BMP390 | hPa, °C |
| `aht_t_c`, `aht_rh_pct` | AHT20 | °C, %RH |
| `veml_als_raw`, `veml_white_raw`, `veml_lux` | VEML7700 visible light | counts, counts, lux |
| `ltr_uvs_raw` | LTR390 UV | counts |
| `lis_ax_g`, `lis_ay_g`, `lis_az_g` | LIS3DH accelerometer | g |
| `gps_lat`, `gps_lon`, `gps_alt_m`, `gps_utc`, `gps_date`, `gps_sats`, `gps_hdop`, `gps_fix`, `gps_qual`, `gps_age_ms` | GPS | degrees, m, UTC |

If a sensor fails a read during flight, its column is left blank instead of getting a made-up number.

### Status columns

- `seq` and `uno_seq`: record counters from the Mega and the Uno, so you can spot dropped records.
- `mega_ms` and `uno_ms`: milliseconds since each board booted.
- `test`: 1 if the record was made in bench test mode, where missing sensors get placeholder values. It should be 0 for every flight record.
- `valid`: a hex bitmask with one bit per sensor. A bit is set when that sensor read successfully.
- `oor`: bitmask of readings outside the expected range. They're still logged, just flagged.
- `synth`: bitmask of placeholder values from test mode. Throw out any record where this isn't zero.
- `rtc_utc`: GPS time converted with the RTClib library. Despite the name, there's no clock chip on the payload.

### What went wrong

- MCP9808 #2 (by the batteries) never returned a single reading.
- The `gps_*` columns in the S rows are the Mega's own GPS snapshot (a receiver on its Serial2). In every record gps_fix is 0 and the other fields are blank, so no fix ever reached the Mega. All GPS positions in the science log come from the Uno shield's receiver in the G rows; the SPOT Trace positions are in the Flight 2 KML.
- The Uno shield GPS lost its fix for 52 minutes on the way up (20:33:46 to 21:26:04 UTC, 10.5 to 21.0 km) and for 5, 3, and 10 minutes on the way down. It kept sending no-fix sentences the whole time. The SPOT fixes in the KML cover the ascent gap at about 5-minute intervals.
- The Mega-to-Uno I2C link dropped records: 98 `SEQ_GAP` events, 520 missing sequence numbers out of 12,765 generated (about 4%).
- GPS fixes before 19:20 UTC have 2 to 9 satellites and altitudes between 188 and 964 m while the payload was sitting on the ground at about 893 m. I exclude them.
- The BMP390's pressure stops changing around 91 hPa, while the MS8607 keeps going down to about 10 hPa. Above that point I use the MS8607.
- I haven't validated the humidity readings at altitude yet, so don't trust them.
- After landing, the payload rebooted several times and started new files (SCI_001 to SCI_006). Those only have the landing-site GPS and no sensor data, so I left them out.
- Two of my three cameras (the RunCams) shut off about two minutes after launch. The Mobius recorded until about 21:22 UTC (near 20 km), stopped for 66 minutes, and resumed on descent, so the burst was not recorded.

## Flight 1 GPS (`Aug_7_2026_GPS_Fixes.csv`)

Flight 1 didn't carry science sensors, just a GPS that logs to its own internal memory (Adafruit Ultimate GPS, LOCUS mode). These are the 930 fixes I decoded from it. Columns: `session`, `time_utc`, `time_pacific`, `latitude`, `longitude`, `altitude_m`, `fix_byte`, `locus_packet`, `record_slot`.

## KML files

Google Earth projects for each flight. The observed GPS track is split wherever the GPS had a gap, and the gaps are left open instead of being filled in. Flight 2's file also has my pre-flight landing predictions and the special-use airspace overlay I checked before launch, so not every path in it is the real flight. Use the CSVs for anything quantitative.

## Firmware

| Sketch | Board | Version | What it does |
|---|---|---|---|
| `HAB_Mega_Science.ino` | Arduino Mega 2560 | 1.2.0-I2C | Reads the sensors through the multiplexer and sends records to the Uno |
| `HAB_Uno_Logger.ino` | Arduino Uno | SCI-UNO 1.3.0-GPS-UNO | Writes science records and GPS to SD |
| `HAB_RW_Controller.ino` | Arduino Nano | HAB-RW 1.2.0-ground | Reaction-wheel yaw controller: BNO085 IMU, Pololu 25D geared motor with encoder, Pololu G2 18v17 driver |

The Mega and Uno sketches are the versions that flew on Flight 2. Both use `sci_link.h`, which defines the CRC-checked frame format between them; Arduino needs a copy next to each sketch, so it's in both folders. The version string the Uno printed when it booted, `SCI-UNO-1.3.0-GPS-UNO`, is in the first record of `SCI_000.CSV`. I rewrote the comments after the flights; the code itself is unchanged, which I checked by comparing the comment-stripped output of gcc's preprocessor against the originals.

The reaction wheel also flew on Flight 2. Its build flags are set to match the flight board: the closed-loop build (`BUILD_COMMISSIONING 0`) with the sign checks passed (`SIGN_COMMISSIONED 1`, `HEADING_RATE_SIGN_VERIFIED 1`). The exact binary that flew wasn't archived: with current library versions this source compiles to 31,220 bytes, about 500 bytes more than the standard Nano setting allows, so the flown binary was built differently in a way I can't reconstruct, such as with older library versions. Nothing logged the wheel's 10 Hz telemetry in flight, because the Uno doesn't record it and the Nano only streams it over serial. The controller saves a summary of its most recent armed run to EEPROM, but I re-armed the wheel for a ground test after recovery, so the summary I read back (`firmware/HAB_RW_Controller/flight_run_summary.txt`) describes that ground run, not the flight. There's no controller data from Flight 2. Logging the wheel to SD is the first thing I'd change for the next flight.

## Flight 2 payload

- Sensors: MCP9808 ×3, TMP119, MS8607, BMP390, AHT20, VEML7700, LTR390, LIS3DH, GPS
- Tracking: SPOT Trace satellite tracker
- Cameras: Mobius Maxi 4K (continuous from before release until about 21:22 UTC near 20 km, off for 66 minutes, resumed on descent near 8.6 km; the burst was not recorded) and two RunCam Thumb 2 (about the first two minutes)
- Attitude control: single-axis reaction wheel for yaw
- Balloon: Kaymont TX1200

## Analysis

The scripts in `analysis/` turn the raw files into every flight number and figure in the report. They need Python 3 with pandas, numpy, matplotlib, Pillow, and reportlab. Run them from the repo root:

```
python analysis/flight2_numbers.py       # prints the computed flight numbers the report quotes, as JSON
python analysis/make_figures.py          # writes docs/figures/*.png at 300 dpi
python analysis/make_altitude_csv.py     # writes docs/flight2_gps_altitude_1min.csv
python analysis/build_report.py          # builds docs/Cohen_HAB_Report.pdf
```

- `common.py` holds the loaders and the calculations. It parses the S rows and the raw GGA sentences in the G rows of `SCI_000.CSV`, the Flight 1 CSV, and the Flight 2 KML (the 19:44 prediction run and the SPOT fixes), applies the exclusion rules (test = 0, synth = 0, GPS fixes from 19:20 UTC on), and computes release, burst, ascent rate, distances, gaps, sensor statistics, the prediction comparison, and the reaction-wheel authority numbers in `compute_numbers()`. It also has the 1976 U.S. Standard Atmosphere, which I use for the pressure comparison and for a pressure altitude where the GPS had no fix. Pressure altitude is always labelled as an estimate.
- `flight2_numbers.py` prints `compute_numbers()` so you can check any value in the report against the files.
- `make_figures.py` draws the block diagrams and every chart and map. Observed GPS fixes, GPS gaps, SPOT fixes, and predictions use different markers on every map.
- `make_altitude_csv.py` writes the Flight 2 GPS altitude at one-minute intervals from release (19:42:55 UTC), with an empty altitude and a note for minutes inside a GPS gap.
- `build_report.py` lays out the PDF; the text pulls its numbers from `compute_numbers()` rather than from typed-in values. It uses the fonts in `fonts/`.
- `rotation_from_video.py` estimates payload yaw from the Mobius footage two ways: frame-to-frame motion and how long the view takes to repeat. `rotation_results/` holds its Flight 2 output. On this footage the frame-to-frame tracker failed, so the report uses only the repeat periods, and only as unconfirmed evidence.
- `check_clips.py` puts the camera's clips in time order from their file names, flags gaps, and writes a list that `rotation_from_video.py` can read directly.
- `test_rotation_synthetic.py` checks `rotation_from_video.py` against synthetic videos with known rotation. The video tools also need ffmpeg.

Three timing choices are worth knowing. Release is 19:42:55 UTC, from a 2.53 g jolt in the accelerometer; GPS altitude starts climbing three seconds later. The Mobius camera's burned-in clock reads Pacific Daylight Time and runs about 15 min 7 s fast, give or take a minute, so camera-to-UTC conversions are approximate. The SPOT export in the KML is stamped in UTC minus 5 hours.
