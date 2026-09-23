# LiDAR-Lite v3HP replacement: Benewake TF-series evaluation (2026-09-23)

Research report produced with Claude on 2026-09-23 at Andy Wickert's request after a Benewake sales contact. Every figure carries a source key; items marked UNVERIFIED are not sourced. Decision: none yet.

Research is complete; everything below was read from the manufacturer PDFs I downloaded and text-extracted in this session (`pdftotext`), plus the distributor pages that allowed fetching. Nothing is from memory except where marked UNVERIFIED.

# Benewake TF-series as a LIDAR-Lite v3HP replacement for Apis

## 1. Comparison table

Source keys are listed at the end; every figure carries one. "—" = not stated in that source.

| | **LIDAR-Lite v3HP** [G] | **TF02-Pro** [P] | **TF03-100 / -180** [T3] | **TF350** [T350] | **TFmini Plus** [MP][MP2] | **TFmini-S** [MS][MS2] | **TF-Luna** [L][L2] | **TFS20-L** (2024) [S20] | **TF-Nova** (2024, line laser) [N] |
|---|---|---|---|---|---|---|---|---|---|
| Range, 90 % target | 40 m (spec'd at 70 %) | 0.1–40 m; same at 100 klux | 0.1–100/180 m; 80/130 m at 100 klux | 0.2–350 m; 300 m at 100 klux | 0.1–12 m; same at 70 klux | 0.1–12 m | 0.2–8 m | 0.2–20 m; 18 m at 100 klux | ≥14 m; ≥7 m at 100 klux |
| Range, 10 % target (dark) | — | 0.1–13.5 m | 40/70 m; 30/50 m at 100 klux | 110 m; 100 m at 100 klux | 0.1–4 m | 0.1–7 m | 0.2–2.5 m | 18 m; 12 m at 100 klux | ≥13 m; ≥4 m at 100 klux |
| ArduPilot "reliable max" [AP] | — | 13.5 m | 50 m | 150 m | — | — | 8 m | 15 m | 7 m |
| Accuracy / resolution | ±2.5 cm typ ≥2 m, ±5 cm <2 m; 1 cm | ±5 cm (<5 m), ±1 % (5–40 m); 1 cm; 1σ <2 cm | ±10 cm (<10 m), 1 %; 1 cm; 1σ <3 cm | ±10 cm (<10 m), 1 %; 1σ <3 cm | ±5 cm (<5 m), ±1 %; 1 cm | ±6 cm (<6 m), ±1 % | ±6 cm (<3 m), ±2 % | ±6 cm (<6 m), 1 % | ±5 cm (0.1–4 m) |
| Frame rate | >1 kHz | 1–1000 Hz (default 100) | 1–1000 Hz (7 kHz custom) | 1–1000 Hz (10 kHz custom) | 1–1000 Hz | 1–1000 Hz | 1–250 Hz | ≤250 Hz | 1–900 Hz |
| Single-shot / on-demand | Yes: write 0x04→reg 0x00, read 0x0F/0x10 | Yes: frame rate 0, then `5A 04 04 62`; I²C on-demand read `5A 05 00 01 60` → 9 B | Yes: trigger mode `5A 04 04 62` | Yes (trigger mode) [T350 refers to manual] | Yes, same commands as TF02-Pro | Yes, same | Yes: I²C reg 0x24 TRIG_ONE_SHOT; ultra-low-power wake-on-read | — (UART/I²C, 0x10) | I²C regs like TF-Luna |
| Interfaces / address | I²C 0x62 (changeable), PWM, trigger | UART 115200 or I²C 0x10 (0x01–0x7F), I/O | UART/CAN (std), RS485/RS232, 4–20 mA; **no I²C** [T3][AP] | UART/CAN or RS485/RS232; **no I²C** | UART / I²C 0x10 / I/O | UART / I²C 0x10 | UART / I²C 0x10 (pin-5 select) | UART / I²C 0x10 | UART / I²C 0x10 |
| Supply / current | 4.5–5.5 V; 65 mA idle, 85 mA acquiring; power-enable pin | 5–12 V; ≤200 mA avg, 300 mA pk, ≤1 W; low-power mode ≤10 Hz (current not stated) | 5–24 V; ≤150 mA @5 V, ≤1 W; low-power 5 Hz 150 mW **UART only** | 5–24 V; ≤150 mA @5 V, ≤1 W | 5 V ±0.5; ≤110 mA, 140 mA pk, 550 mW; low-power <100 mW ≤10 Hz | 5 V ±0.1; ≤140 mA, 200 mA pk | 3.7–5.2 V; ≤70 mA; ECO 8.85 mA @1 Hz; sleep 1.5 mW | 3.3 V ±9 %; ≤0.35 W, 115 mA pk | 5 V ±5 %; <500 mW; 850 mA start pk |
| Operating temp | −20…60 °C | −20…60 °C | −25…60 °C | −25…60 °C | −20…60 °C | 0…60 °C | −10…60 °C | −20…60 °C | −25…70 °C |
| IP rating / housing | IPX7 | IP65, ABS/PC | IP67, aluminium, IR glass window | IP67, aluminium | IP65, PC/ABS | none ("/") | none stated | none (bare module) | front window IP65, body N.A. |
| Size / mass | 40.2×55×35 mm, 38 g | 69×41.5×26 mm, 50 g, 80 cm cable | 44×43×32 mm, 86–92 g, 70 cm cable, Molex 51021-07 | 78×67×40 mm, 222 g | 35×18.5×21 mm, 12 g | 42×15×16 mm, 5 g | 35×21.25×13.5 mm, <5 g | 21×15×7.9 mm, 1.35 g | 26.5×21×12 mm, <5 g |
| Laser | 905 nm, Class 1 | 850 nm VCSEL, Class 1 | 905 nm LD, Class 1 | 905 nm LD, Class 1 | 850 nm, Class 1 | 850 nm, Class 1 | 850 nm, Class 1 | 905 nm, Class 1 | 905 nm, Class 1 |
| Price (2026‑09‑23) | $169.99, in stock [G2] | $87.90 (9 left) [DF]; $99.90 [SL] | $269 UART/CAN, $299 RS485 [SL] | $599 UART [SL] | $49.90 [SL] | — | $29.90 [SL] | $49.90 [SL] | $39.95 SparkFun [SF] |
| Arduino library | Garmin LIDARLite | none TF02-Pro-specific found; protocol identical to TFmini Plus | none found | none found | budryerson/TFMini-Plus-I2C [LIB1] | same [LIB1] | budryerson/TFLuna-I2C [LIB2] | none found | none found |

## 2. Ranked recommendation

**1. TF02-Pro, I²C variant.** The only Benewake unit that matches the 40 m spec, runs on Apis's switched 5 V, and speaks I²C. Board: no supply change (5–12 V in, so the 5 V switch stays); replace the Garmin connector with the TF02-Pro's 4-wire pigtail (connector type not stated in the manual – UNVERIFIED). Firmware: the ATtiny1634 swaps Garmin register writes for Benewake's `0x5A` command frames – on power-up send `5A 06 03 00 00` (frame rate 0 → trigger mode) once and `5A 04 11 6F` (save), then per reading `5A 04 04 62` followed by `5A 05 00 01 60` and a 9-byte read (dist, strength, temperature, checksum). Address is 0x10, changeable (0x01–0x7F) if it collides on the NW bus. Costs: 10 %-target range is 13.5 m, not 40 m; accuracy ±1 % beyond 5 m (±40 cm at 40 m) vs Garmin ±2.5 cm typ; 1 W while on vs 0.4 W; IP65 plastic vs IPX7; start-up-to-first-valid-frame time not stated (UNVERIFIED, must be measured – it sets the energy per sample).

**2. TF03-100, UART variant.** If dark targets at 30–40 m or a metal IP67 housing matter, this is the only Benewake with the margin (40 m at 10 %, 30 m at 100 klux). Board: 5 V still works and it draws *less* (≤150 mA) than the TF02-Pro; needs the 7-pin Molex 51021. Firmware: ATtiny1634 talks UART 115200 (it has hardware USARTs) using the same command set, and bridges to the host's I²C – a larger firmware change but the host interface is unchanged. Costs: $269, 90 g, ±10 cm/1 %, 0.5° beam.

**3. TFmini Plus, I²C variant – short sites only.** Same protocol and library as TF02-Pro, IP65, −20…60 °C, <100 mW low-power mode, $50. Only for installs within ~10 m over bright targets (4 m at 10 %). Not a general v3HP replacement.

Not recommended: TF-Luna, TFmini-S, TFS20-L (unsealed, −10/0 °C minimum for two of them, ≤8–20 m); TF-Nova (line beam, 7 m outdoors, though Benewake markets it for liquid level [BW]); TF350 (overkill, $599, 222 g); TF02-Pro-W (IP5X wiper unit for bulk solids).

## 3. Concerns

- **Water surfaces.** Neither vendor gives a range figure to water. Benewake's TF02-Pro/TFmini manuals list "transparent substance (such as glass and water)" as a cause of data errors [P][MP]; Garmin's manual says a flat specular surface may return nothing unless viewed from the normal and that ripples help [G]. The TF02-Pro's 3° beam (≈2 m spot at 40 m) versus Garmin's 8 mrad may average ripples differently – untested, UNVERIFIED either way. Bench the candidate over a real pool before committing.
- **Snow.** No Benewake snow-depth reports found. Fresh snow is a bright diffuse target at 850–905 nm (general knowledge, UNVERIFIED here), so the 90 % column applies; wet or dirty snow trends toward the 10 % column.
- **Sunlight.** Benewake specifies 100 klux (70 klux for TFmini Plus); Garmin gives no number, so the comparison is one-sided.
- **Minimum range** 0.1 m vs Garmin 5 cm – irrelevant at 0.5 m mounting.
- **Energy per reading.** TF02-Pro's low-power mode caps at 10 Hz but states no current; TF03's low-power mode is UART-only. Since Apis cuts power between samples, wake-up time matters more than any sleep mode, and it is unstated for TF02-Pro/TF03.
- **Pricing/MOQ.** Single-unit purchase is verified at DFRobot, SensorLiDAR, SparkFun (TF-Nova); Mouser lists all models but its pages timed out, so Mouser pricing and lead times are UNVERIFIED; no MOQ or volume tiers were found anywhere – ask Benewake directly.
- **Garmin status.** The premise "discontinued" is not confirmed: SparkFun shows the v3HP in stock at $169.99 today and no EOL notice was found [G2]. If it remains available, it is still the better sensor for this job on accuracy, IPX7, and power; Benewake's case is price ($88–100) and I²C-compatible fallback, not performance.

**Other alternatives (one paragraph).** LightWare LW20/C: 100 m, I²C or serial, IP67, $299.30 at DigiKey [DK2]; the open-frame SF20/C is $279 but unsealed, −10…50 °C, 0 in stock with 16-week lead [DK1]. LightWare claims first/last-pulse operation in rain/snow and "over water" in marketing copy only (UNVERIFIED figure). Garmin's own LIDAR-Lite v4 LED is a 10 m device and does not cover 40 m (UNVERIFIED, from memory). Industrial units (SICK DT50, Baumer) reach 40 m+ but need 24 V and cost several hundred dollars (UNVERIFIED).

## 4. Unverified list
TF02-Pro connector type and start-up time; TF02-Pro low-power current; Mouser prices/lead times; MOQ/volume tiers; snow reflectance statement; LightWare water claim; Garmin v4 LED range; industrial-sensor pricing; SensorLiDAR's relationship to Benewake.

## Sources
[G] https://static.garmin.com/pumac/LIDAR-Lite_v3HP_Instructions_EN.pdf · [G2] https://www.sparkfun.com/lidar-lite-v3hp.html · [P] https://en.benewake.com/uploadfiles/2024/04/20240426135442695.pdf · [T3] https://en.benewake.com/uploadfiles/2024/04/20240426134845102.pdf · [T350] https://doc.switch-science.com/media/files/fcfd4ded-16ff-4486-acad-a5217d66774f.pdf · [MP] https://en.benewake.com/uploadfiles/2024/04/20240426135807930.pdf · [MP2] https://www.makerguides.com/wp-content/uploads/2024/12/TFmini-Plus-datasheet.pdf · [MS] https://en.benewake.com/uploadfiles/2024/04/20240426140053930.pdf · [MS2] https://www.gotronic.fr/pj2-sj-gu-tfmini-s-01-a00-datasheet-2154.pdf · [L] https://en.benewake.com/uploadfiles/2024/04/20240426135946148.pdf · [L2] https://s3-us-west-2.amazonaws.com/files.seeedstudio.com/products/101990656/res/SJ-GU-TF-luna-A01+Datasheet.pdf · [S20] https://en.benewake.com/uploadfiles/2024/08/20240821162819250.pdf · [N] https://dfimg.dfrobot.com/wiki/18669/SEN0671_tf-nova-line-laser-lidar-sensor_datasheet_V.pdf · [AP] https://ardupilot.org/copter/docs/common-benewake-tf02-lidar.html · [BW] https://en.benewake.com/NewsUpdates/info_itemid_2149.html · [DF] https://www.dfrobot.com/product-1599.html · [SL] https://www.sensorlidar.com/products/benewake-tf02-pro-40m-ip65-lidar-sensor , …/benewake-tf03-100-long-range-ip67-lidar , …/benewake-tf350-long-range-industrial-lidar , …/benewake-tfmini-plus-lidar-ip65-12m-tof-sensor , …/benewake-tf-luna-8m-lidar , …/benewake-tfs20-l-mini-dtof-lidar-20m · [SF] https://www.sparkfun.com/benewake-tf-nova.html · [DK1] https://www.digikey.com/en/products/detail/lightware-lidar-inc/SF20-C/15848650 · [DK2] https://www.digikey.com/en/products/detail/lightware-lidar-inc/LW20-C/15848653 · [LIB1] https://github.com/budryerson/TFMini-Plus-I2C · [LIB2] https://github.com/budryerson/TFLuna-I2C

Extracted datasheet text is in `/tmp/claude-1000/-home-awickert-Dropbox-NorthernWidget-github/7a629d90-8ea9-4f47-b3c2-74b2c03d2dcd/scratchpad/pdf/` if the caller wants to check any figure against the source.
