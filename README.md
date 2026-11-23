# FEO sensor – heel offloading orthosis (XIAO nRF54L15 Sense)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Board](https://img.shields.io/badge/Board-Seeed%20XIAO%20nRF54L15%20Sense-green.svg)](https://wiki.seeedstudio.com/xiao_nrf54l15_sense_getting_started/)
[![Framework](https://img.shields.io/badge/Framework-Zephyr%20%2F%20PlatformIO-blue.svg)](https://platformio.org)

Sensor firmware and browser app for a heel offloading orthosis ("Fersenentlastungsorthese", FEO). Based on [XIAO_IMU_Live](https://github.com/soylentOrange/XIAO_IMU_Live). The **Seeed Studio XIAO nRF54L15 Sense** samples at 100 Hz:

- the 6-axis IMU (LSM6DS3TR-C),
- the output voltage of a force sensor (FSR) amplifier at A0, with automatic ADC range switching,
- the battery voltage (once per second).

The web app shows everything live via **Bluetooth LE**, records CSV, and has a **force–voltage calibration** panel (reference weight per point, averaged voltage, CSV export).

**Web app:** <https://soylentorange.github.io/FEO_Sensor/> (Chrome or Edge on desktop and Android, installable as an app; published by the release workflow, see [Firmware Updates](#firmware-updates-over-bluetooth-ota)). `docs/index.html` also works opened from disk.

<p align="center">
  <img src="docs/assets/screenshot.png" alt="Screenshot of the FEO sensor web app" width="900">
</p>

---

## Table of Contents

1. [Overview](#overview)
2. [Hardware](#hardware) ([details](hardware/README.md))
3. [Firmware](#firmware)
   - [Step Detection and Force per Gait Phase](#step-detection-and-force-per-gait-phase)
   - [Power Saving](#power-saving)
   - [Long-term Measurement](#long-term-measurement)
   - [Switchable Outputs](#switchable-outputs)
   - [Building and Flashing](#building-and-flashing)
   - [Data Formats](#data-formats)
4. [Web App](#web-app)
   - [Features](#features)
   - [Force Calibration](#force-calibration)
   - [Sensor Alignment](#sensor-alignment)
   - [Long-term Measurement Tab](#long-term-measurement-tab)
   - [Browser Support](#browser-support)
   - [Install on Android](#install-on-android)
5. [Repository Layout](#repository-layout)
6. [Acknowledgements & Funding](#acknowledgements--funding)
7. [License](#license)

---

## Overview

- **Sensor:** LSM6DS3TR-C on the XIAO nRF54L15 Sense (I²C, Zephyr driver `lsm6dsl`), ±16 g and ±2000 °/s, internal rate 208 Hz, output 100 Hz.
- **Force sensor:** 30 mm FSR in a transimpedance amplifier (MCP602, V_ref ≈ 0.1 V) with a Sallen-Key low-pass (~23 Hz), output to A0. The SAADC switches its gain automatically (see below).
- **Battery:** LiPo, measured via the on-board 1:2 divider at AIN7. The divider is switched on only for the measurement.
- **Firmware:** Zephyr (PlatformIO, or west with MCUboot for releases). Streams samples over BLE on request (the serial console is off); detects strides and the force per gait phase on its own; sleeps when the leg is still.
- **Web app:** a single HTML file without external dependencies. Uses Web Bluetooth and Web Serial, a Madgwick filter for orientation, and a small 3D model derived from the Seeed STEP model.

## Hardware

Schematic, bill of materials, perfboard layout, wiring, arrangement and housing (STEP): **[hardware/README.md](hardware/README.md)**.

| XIAO pin | nRF54L15 | Use |
|---|---|---|
| D1 | P1.05 | Supply of the MCP602 (driven high by the firmware) |
| A0 / D0 | P1.04 (AIN0) | Amplifier output (U_FSR) |
| – | P1.14 (AIN7) | Battery voltage / 2 (on-board divider) |
| – | P1.15 | Switch for the battery divider (`vbat_pwr`, on-board) |

The pins are set in [`zephyr/app.overlay`](zephyr/app.overlay).

**ADC range switching (FSR channel).** Internal reference 0.9 V, 12 bit, 8× oversampling. The first sample after a switch is discarded (the switch and the repeated sample happen within the same 10 ms period).

| Range | SAADC gain | Full scale | Switch up above | Switch down below |
|---|---|---|---|---|
| 0 | 1 | 0.9 V | 800 mV | – |
| 1 | 1/2 | 1.8 V | 1600 mV | 350 mV |
| 2 | 1/4 | 3.6 V | – | 700 mV |

Voltages are reported in mV at the pin, including the ~0.1 V reference offset of the amplifier. The battery is measured with gain 1/4 and multiplied by 2.

## Firmware

### Firmware Updates over Bluetooth (OTA)

Release builds contain the **MCUboot** bootloader and an **MCUmgr/SMP** service, so new firmware can be installed from the web app without a cable.

1. **Once, via USB:** flash `feo_sensor_vX.Y.Z_factory.hex` (MCUboot + firmware) from a [release](../../releases), e.g.
   ```bash
   pyocd flash -t nrf54l feo_sensor_v1.0.0_factory.hex
   ```
   (pyOCD comes with PlatformIO: `~/.platformio/penv/bin/pyocd`; `pip install pyocd` also works.)
2. **Later updates:** connect the web app via Bluetooth. The *Firmware update* card shows the installed version. Choose `feo_sensor_vX.Y.Z_update.bin` from the release (or press *Install latest version* when the app is served via GitHub Pages, see below). The app uploads the image, the board restarts, MCUboot swaps the images, and the new firmware confirms itself once IMU, ADC and Bluetooth are running. If it does not get that far, MCUboot restores the previous version on the next reset.

**Making a release:** on GitHub, *Releases → Draft a new release*, type a new tag like `v1.0.0` under *Choose a tag* (“Create new tag on publish”), and publish. Or from a clone: `git tag v1.0.0 && git push origin v1.0.0`. The tag must have the form `vX.Y.Z`; it becomes the firmware version. The workflow [`.github/workflows/release.yml`](.github/workflows/release.yml) builds with Zephyr 4.4.2 and sysbuild ([`west.yml`](west.yml), [`zephyr/sysbuild.conf`](zephyr/sysbuild.conf), [`zephyr/ota.conf`](zephyr/ota.conf)) and attaches both files to the release. *Run workflow* without a tag only builds (artifacts).

**Automatic download in the app:** browsers cannot download release assets directly (no CORS). Therefore each release also publishes the web app together with `fw/latest.json` and the update file on GitHub Pages, and the app offers *Install latest version*. The workflow [`pages.yml`](.github/workflows/pages.yml) also redeploys when `docs/` changes on `main`. One-time setup: *Settings → Pages → Source: GitHub Actions*, and *Settings → Secrets and variables → Actions → Variables*: `DEPLOY_PAGES` = `true`.

#### Signing Key

MCUboot only installs update images that are signed with the key its bootloader was built with.

- **Using the official releases** of this repository (`_factory.hex` + `_update.bin`): you do not need a key. Your board then only accepts updates signed by the maintainer of this repository.
- **Building yourself or using a fork:** create your own key and store it as the Actions secret `MCUBOOT_SIGNING_KEY` in your fork (*Settings → Secrets and variables → Actions → Secrets*):
  ```bash
  pip install imgtool
  imgtool keygen -k feo_key.pem -t ecdsa-p256
  ```
  Paste the whole content of `feo_key.pem` into the secret and keep the file safe offline. **Never commit the private key** (`*.pem` is in `.gitignore`).

> [!WARNING]
> Without the secret, the workflow signs with MCUboot's **publicly known test key**: anyone within Bluetooth range could then install firmware on the board. Fine for a bench prototype, not for use on patients.
>
> A board only accepts updates signed with the key of the `_factory.hex` it was flashed with. After changing the key (or switching between official releases and your own builds), flash the matching `_factory.hex` via USB once.

The PlatformIO build below has no bootloader and no OTA. Flashing it overwrites MCUboot; flash a `_factory.hex` again to get OTA back.

### Step Detection and Force per Gait Phase

The firmware detects strides on its own, with or without an app connected ([`src/gait.c`](src/gait.c), host test in [`tools/gait_test.c`](tools/gait_test.c)). The algorithm follows Fadillioglu, Stetter et al. (2020), *Automated gait event detection for a variety of locomotion tasks using a novel gyroscope-based algorithm*, Gait & Posture, doi:[10.1016/j.gaitpost.2020.06.019](https://doi.org/10.1016/j.gaitpost.2020.06.019). It runs in real time on the shank's sagittal angular rate ω (positive = foot swings forward, gyro offset removed while the leg is still):

- **Mid-swing (MS):** maximum of ω (15 Hz low-pass) above a threshold, at least 333 ms after the previous MS. The threshold adapts to 30 % of the recent peaks (at least 50 °/s) instead of the paper's fixed value.
- **Initial contact (IC):** first zero crossing of ω after MS.
- **Toe-off (TO):** from the complementary signal (unfiltered − 15 Hz, then 10 Hz low-pass), searched from the previous MS + 50 % of the cycle to the negative peak before MS (+ 10 % of the cycle for slow gait, cycle > 1 s). The minimum is used for slow gait and the maximum for fast gait.

A stride runs from IC to the next IC. For every stride the firmware reports the stride and stance time, the angular rates at MS/IC/TO, and the force (kg, from the stored calibration): peak, mean and impulse in stance, the time to peak, the mean in swing, the mean over 20 bins of 5 % of the gait cycle, and the mean and maximum per **Rancho Los Amigos** phase. With one sensor only the IC and TO of that leg are known, so the phase limits in between assume a symmetric gait (stride L, stance T):

| Phase | Ends at | Normal gait |
|---|---|---|
| IC Initial contact | 2 % of L | 0–2 % |
| LR Loading response | T − L/2 (toe-off of the other leg) | 2–10 % |
| MSt Mid stance | middle of single support | 10–30 % |
| TSt Terminal stance | L/2 (initial contact of the other leg) | 30–50 % |
| PSw Pre-swing | T (toe-off) | 50–60 % |
| ISw / MSw / TSw | thirds of the swing phase | 60–73 / 73–87 / 87–100 % |

Since firmware v0.8.0 every stride also carries:

- **Heel contact:** share of stance (%) with heel load above the threshold of the long-term measurement (last one started, default 2 kg).
- **Loading rate:** largest rise of the heel force over 30 ms between IC and the force peak (kg/s).
- **Shank excursion:** sagittal range of the shank angle over the stride (°, integrated angular rate). Together with ω at MS (swing peak) a speed-related measure of the swing.

From the mean acceleration over the strides the firmware also learns the shank direction while walking (or takes z of the alignment). At rest it gives the **shank tilt**: above 60° counts as lying or leg up, below as upright (sitting or standing, which a shank sensor cannot tell apart; heel load tells standing with load).

The rotation axis comes from the [sensor alignment](#sensor-alignment). Without one, the firmware uses the principal axis of the angular rate while walking (sign from the third moment: swing peaks are the largest).

### Power Saving

- After 20 s without movement, and while no app requests raw data, the sensor goes to **rest**: gyro off, accelerometer in low-power mode (26 Hz, ±2 g) with its wake-up interrupt on INT1 (P0.02), MCP602 supply (D1) off, no serial output. The nRF sleeps until the IMU detects movement and measures the battery once a minute. On wake-up the amplifier gets 50 ms to settle before the first force sample.
- **Raw data** (IMU + FSR, `7a1e0012`) is only sent while the app subscribes to it. The app does this only on the IMU, Calibration and Alignment tabs and during a recording. Steps are always sent on their own characteristic.
- **Advertising** runs at 100–150 ms for 30 s after power-on or disconnect, then at about 1 s.
- **LED:** off at rest; dimly on while movement is detected; a bright flash at every initial contact. Dim and bright level (% duty of a software PWM, the LED pin has no hardware PWM) and the flash length are set in the app (Firmware tab, stored in flash); defaults 3 %, 100 %, 300 ms; 0 % switches either off.
- The **serial console is off** (`uart20` disabled, no `printk`): the sensor talks only via Bluetooth. Flashing over USB goes through the SWD debugger on the XIAO and still works. For debugging output, remove the console block at the end of `zephyr/prj.conf` and the `uart20` node in `zephyr/app.overlay`; the serial formats below then apply again.
- While a long-term measurement runs, the rest mode switches the amplifier on for 50 ms every 5 s to sample the heel load (standing with load) and reads the accelerometer for the shank tilt.

### Long-term Measurement

The firmware logs a measurement in flash, without the app, for everyday use (e.g. 24 h unsupervised):

- **Every stride** (2 × 16 bytes since v0.8.0): time of initial contact, stride and stance time, heel peak, stance mean and impulse, flags; heel contact, time to peak, swing peak, shank excursion, loading rate, swing heel load.
- **Every minute** (16 bytes): strides, strides above the threshold, seconds with movement, seconds with heel load above the threshold while not moving, seconds upright without heel load and seconds lying / leg up while not moving (since v0.8.0), maximum heel load while not moving, battery.
- **Events:** start, stop, power-up (with reset cause), time set, rest/wake, power-on zero point, markers from the app, battery low, log full.

Storage is the secondary image slot (`slot1_partition`, 712 KB, of which 704 KB are used = 45,056 entries). A typical day of 10,000 steps (5,000 strides) plus 1,440 minute entries takes about 185 KB, so the memory lasts **about 4 days** (6–7 days with firmware before v0.8.0, which logged one entry per stride). Logs written by older firmware (format version 1) can still be read out; a new measurement erases them first. Battery use for a 24 h day is estimated at 7–8 mAh (about 2 h active at ~3 mA, the rest in rest mode).

- The app sets the **clock** every time it connects and again at start. The nRF counts on from there. All entries carry the uptime in ms; after a reset during a measurement the log keeps running without clock time, and the app maps the uptime to the clock again from the next time it connects.
- A measurement **survives resets** and continues after power-up.
- A **firmware update over Bluetooth overwrites the log**: the firmware stops logging as soon as an upload starts, and the app warns if entries have not been read out. Logging only starts once the running image is confirmed; the MCUboot trailer at the end of the slot is never touched.
- Format: see [`src/datalog.h`](src/datalog.h).

### Switchable Outputs

Raw data (BLE) and stride records (BLE) can be switched off individually (stored in flash); the mask also has bits for the serial CSV and `#S` lines, which only matter in a debug build with the console enabled. Battery and the status/activity notification are always on while connected.

### Pairing Code

Without a code, anyone nearby can connect (default). A 6-digit code can be set in the app (Firmware tab); it is stored on the board. From then on, the FEO service and the firmware update (MCUmgr/SMP) need a link that is paired with the code: on connect the board requests pairing, the phone or computer shows its pairing dialog once, and the bond is stored on both sides. Until the link is paired, the board refuses reads and writes and disconnects after 60 s. Changing or removing the code drops all bonds.

Code forgotten: hold the user button on the XIAO for 5 s while switching on (the LED blinks); this clears the code and all bonds. Then remove the board in the Bluetooth settings of the phone or computer. Flashing via USB does not clear the code (the settings partition is kept). Firmware images are still verified with the signing key, independent of the code.

### Building and Flashing

Requirements: [PlatformIO](https://platformio.org) (e.g. the VS Code extension). The Seeed platform is fetched automatically.

```bash
pio run -t upload        # build and flash via the on-board CMSIS-DAP probe
pio device monitor       # optional: show the CSV stream (115200 baud)
```

Settings such as sample rate and ranges are at the top of [`src/main.c`](src/main.c):

| Constant | Default | Meaning |
|---|---|---|
| `SAMPLE_RATE_HZ` | 100 | Output rate |
| `IMU_ODR_HZ` | 208 | Internal sensor rate |
| `ACCEL_RANGE_G` | 16 | ±2, 4, 8 or 16 g |
| `GYRO_RANGE_DPS` | 2000 | ±245, 500, 1000 or 2000 °/s |
| `BLE_BATCH` | 5 | Samples per BLE notification (20 packets/s) |
| `SERIAL_BATCH` | 2 | Lines per serial burst (see below) |
| `BAT_PERIOD` | 100 | Battery measurement every 100 samples (1 s) |
| `BAT_DIVIDER` | 2 | On-board battery divider |

Range thresholds are in `fsr_up_mv` / `fsr_down_mv` in `src/main.c`.

The LED blinks slowly while advertising and fast while a BLE client is connected.

> [!NOTE]
> The USB-serial bridge on the XIAO only forwards data immediately when the line is idle for more than about 10 ms. A steady 100 lines/s would otherwise be buffered in 16 KiB blocks (about 4 s delay). The firmware therefore sends two lines every 20 ms.

### Data Formats

**USB serial** (only in a debug build with the console enabled, 115200 baud), one line per sample; lines starting with `#` are comments:

```text
t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,u_fsr_mv,fsr_range_mv,u_bat_mv
123450,0.012,-0.004,1.001,0.3,-0.1,0.0,412,900,3874
```

`fsr_range_mv` is the full scale the FSR sample was taken with (900, 1800 or 3600). `u_bat_mv` is the last battery reading (0 until the first measurement).

**Bluetooth LE:** device name `XIAO-FEO`

- Service `7a1e0011-5c3b-4f0a-9d2e-3c8f1b6a4d01`
- Characteristic `7a1e0012-5c3b-4f0a-9d2e-3c8f1b6a4d01` (notify): samples
- Characteristic `7a1e0013-5c3b-4f0a-9d2e-3c8f1b6a4d01` (read, notify once per second): battery voltage, uint16 in mV
- Characteristic `7a1e0014-5c3b-4f0a-9d2e-3c8f1b6a4d01` (read, write): force calibration, stored in flash (Zephyr settings). Little endian: float32 `VREF` (mV), `RF` (Ω), `RS` (Ω), `K`, `N`, uint32 calibration time (Unix s, 0 = factory values). Reading returns 4 more bytes: float32 `VREF_0`, the rest voltage measured at power-on (0 = not measured or implausible). Only the first 24 bytes are written.

- Characteristic `7a1e0015-5c3b-4f0a-9d2e-3c8f1b6a4d01` (read, write): sensor alignment, stored in flash. float32 R[3][3] (rows: x forward, y right = sagittal axis, z down, each in sensor coordinates), uint32 time (Unix s; 0 = none, the firmware then estimates the axis itself). Reading returns 28 more bytes: float32 axis[3] currently used, float32 gyro offset[3] (°/s), uint32 number of strides since power-on.
- Characteristic `7a1e0016-5c3b-4f0a-9d2e-3c8f1b6a4d01` (notify, write): one 108-byte record per stride (see below). The last 1024 records stay in RAM; writing a uint32 `seq` sends all records from that number again.
- Characteristic `7a1e0017-5c3b-4f0a-9d2e-3c8f1b6a4d01` (read, write, notify about once per second): control and status. Status (48 bytes): uint8 version, uint8 flags (1 running, 2 log valid, 4 erasing, 8 clock set, 16 locked, 32 full), uint16 output mask, uint32 entries, uint32 capacity, uint32 device time (Unix s), uint16 threshold (0.01 kg), uint8 side, uint8 erase progress (%), char[8] pseudonym, uint32 index of the session header, uint32 uptime (ms), uint32 strides since power-up, uint8 activity (0 rest mode, 1 still, 2 moving), uint8 flags (bit 0 pairing code set, bit 1 this link is paired), uint16 seconds still, uint8 LED dim %, uint8 LED bright %, uint16 LED flash ms. Commands (byte 0): 1 set time (uint32 Unix s), 2 start (uint16 threshold, uint8 side, uint8 options bit 0 = erase first, char[8] pseudonym), 3 stop, 4 erase, 5 read out (uint32 from index, 0xFFFFFFFF = abort), 6 marker (uint16 number), 7 outputs (uint16 mask: 1 raw BLE, 2 strides BLE, 4 serial CSV, 8 serial strides), 8 pairing code (uint32 0..999999, 0xFFFFFFFF = none; drops all bonds), 9 LED (uint8 dim % while moving, uint8 bright % at initial contact, uint16 flash ms; 0 % = off).
- Characteristic `7a1e0018-5c3b-4f0a-9d2e-3c8f1b6a4d01` (notify): log readout, uint32 index + up to 15 entries of 16 bytes; a packet without entries marks the end.

The UUIDs differ from XIAO_IMU_Live, so the two apps do not pick up each other's boards.

Each sample notification (little endian):

| Field | Type | Content |
|---|---|---|
| `t_ms` | uint32 | Timestamp of the first sample in the packet |
| `n` | uint8 | Number of samples, 10 ms apart |
| samples | n × 14 bytes | 6 × int16: ax, ay, az in mg; gx, gy, gz in 0.1 °/s<br>uint16: bits 0–13 U_FSR in mV, bits 14–15 range (0 = 0.9 V, 1 = 1.8 V, 2 = 3.6 V) |

Each stride record (little endian, 108 bytes; 102 before v0.8.0): uint32 `seq`, uint32 `t_ic_ms`, uint16 `stride_ms`, uint16 `stance_ms`, int16 ω at MS, IC, TO (0.1 °/s), uint16 force peak, stance mean (0.01 kg), uint16 stance impulse (0.01 kg·s), uint16 time IC → peak (ms), int16 swing mean (0.01 kg), uint16 flags (1 = overload, 2 = calibrated axis, 4 = automatic axis not settled yet, 8 = simple TO), 20 × uint16 force per 5 % of the cycle, 8 × uint16 mean and 8 × uint16 maximum per Rancho Los Amigos phase (IC, LR, MSt, TSt, PSw, ISw, MSw, TSw; 0.01 kg), uint16 shank excursion (0.1 °), uint16 loading rate (0.1 kg/s), uint8 heel contact (% of stance), uint8 0. Over USB serial each stride appears as a comment line `#S,seq,t_ic_ms,stride_ms,stance_ms,ms_dps,ic_dps,to_dps,f_peak_kg,f_mean_kg,f_imp_kgs,t_fpeak_ms,f_swing_kg,flags,` followed by the 20 bins, 8 phase means and 8 phase maxima in kg, then `rom_deg,f_rate_kgps,heel_pct`.

**CSV recordings** from the web app use the serial columns plus `ref_kg`, the reference weight entered in the calibration panel at that moment (empty if none).

## Web App

The app lives in [`docs/index.html`](docs/index.html). It can be opened directly from disk (USB and BLE work in Chrome/Edge on desktop) or served via GitHub Pages.

### Features

- Connect via **Bluetooth LE** (the USB serial connection is hidden because the firmware no longer uses the console)
- Six tabs: **Steps** (lab measurement: strides and force per gait phase, video), **Long-term** (measurement in the device), **Alignment** (sensor axes on the shank), **Calibration** (force sensor, kg display, calibration points, coefficients), **IMU** (plots and 3D view) and **Firmware update**. Connection, recording, battery and activity badges, output switches and log stay visible on every tab; the last tab is remembered and can be linked directly (`#gait`, `#log`, `#align`, `#cal`, `#imu`, `#fw`)
- **Activity badge** in the header (Bluetooth): moving / still / rest mode and ⏺ while a long-term measurement runs; it flashes orange at every stride
- **Long-term:** start/stop with pseudonym, side and threshold, markers, readout, analysis and CSV export (see [below](#long-term-measurement-tab))
- **Outputs:** switch raw data and stride records on and off
- **Test conditions** (Steps tab): pick or type a condition before each test (presets follow the lab protocol: platform walking, standing, sit to stand, level ground, treadmill, stairs up/down). Each stride gets the condition that was set at its time, also strides loaded later from the device. The evaluation can be shown per condition, a table compares all conditions, and **Download CSV** saves one file per condition plus a summary (`feo_bedingungen_*.csv`). Raw recording and video carry the condition in the file name; in a running long-term measurement each change sets a marker.
- **Video in sync** (Steps tab): records a video with the phone or laptop camera together with the data recording. Device time is mapped to browser time from the incoming samples (minimum delay over a 20 s window), so the data CSV gets a `video_ms` column and a separate CSV lists the strides with their time in the video (`video_s`). The sensor LED flashes at every initial contact, which can be used to check the sync in the video. Needs Chrome on Android or desktop (Web Bluetooth + MediaRecorder); not available on the iPhone
- **Steps:** cadence, stride and stance time, swing time, stance/swing ratio, swing peak and shank excursion, stride-time variability (CV within passes of at least 5 strides, first and last stride left out), heel force, heel contact and loading rate (also per condition and in the summary CSV); bar chart of the force per Rancho Los Amigos phase, mean force curve over the gait cycle with the phase limits, trend per stride, table and CSV export. On connecting, the app loads the strides the device recorded while no app was connected
- Live plots of acceleration and angular rate including the magnitudes |a| and |ω|, window 5 / 10 / 30 s
- **3D orientation** view (Madgwick filter, accelerometer + gyroscope) with a model of the board, drag with mouse or finger to rotate the camera, "Zero" to reset orientation and gyro offset
- **Force sensor** voltage plot with current range, **battery voltage** with a rough charge estimate (LiPo, only meaningful without USB charging)
- **CSV recording** and download (including U_FSR, range, U_bat and the entered reference weight)
- **Force–voltage calibration** points with CSV export
- **Battery** level badge in the header (percentage estimate and voltage). The percentage is interpolated from a typical LiPo open-circuit discharge curve (3.3 V = 0 %, 3.7 V ≈ 25 %, 3.8 V ≈ 50 %, 4.0 V ≈ 78 %, 4.2 V = 100 %), so it reads high while charging and low under load
- **Firmware update** over Bluetooth (MCUmgr/SMP)
- German and English (switch in the top right corner)
- Works offline after the first visit and can be installed as an app (PWA)

Yaw drifts slowly because the board has no magnetometer.

### Force Calibration

**kg display.** The app converts U_FSR to kg with the model

```text
R_FSR = RS + K · kg^(−N)          U = V_ref · (1 + Rf / R_FSR)
kg    = ( K / (V_ref · Rf / (U − V_ref) − RS) )^(1/N)     (0 below V_ref + 2 mV, overload if the denominator ≤ 0)
```

The coefficients live in the device, so each sensor keeps its own calibration. The app reads them on Bluetooth connect (and remembers the last ones for USB). Factory values come from the calibration of 22.11.2026 (Rf = 12k).

**Zero point.** V_ref drifts a little, so the firmware measures it at **power-on**, when the sensor is certainly unloaded: it waits 2 s for the amplifier and filter to settle, averages 1 s and accepts 60–150 mV. The **Zero** button in the app takes the current 1 s mean instead (keep the sensor unloaded; after a load the FSR creeps back for about 10 s). The source in use is shown next to the kg value.

**Fitting new coefficients.** Collect calibration points as below (ideally one at 0 kg and several up to the maximum load), set Rf, press **Fit from points**, check the mean deviation, then **Write to device**. **Factory values** restores the defaults in the app (write them to store them).

#### Calibration points

1. Connect, put a known weight on the sensor, keep it still.
2. Enter the weight in kg under **Reference weight** and press **Add point**. The app stores the mean, standard deviation, min, max and number of FSR samples over the averaging window (0.5–3 s), the range(s) used and the battery voltage.
3. Repeat for all weights, then **Download CSV**:

```text
nr,time_iso,ref_kg,u_fsr_mean_mv,u_fsr_sd_mv,u_fsr_min_mv,u_fsr_max_mv,n,avg_s,fsr_range_mv,u_bat_mv
1,2026-11-22T11:20:54.124Z,0.5,423.0,1.7,420,426,100,1,900,3870
```

Points are kept in the browser (localStorage) until deleted. For loading/unloading curves (hysteresis), use a normal recording instead: every row then carries the currently entered reference weight.

### Sensor Alignment

Step detection needs the shank's angular rate about the medio-lateral axis. Because the sensor is never mounted exactly aligned, the **Alignment** tab builds a shank coordinate system (x forward, y right = rotation axis, z down along the shank):

1. **Keep still** with the shank vertical (2 s): gravity gives the long axis, the mean angular rate gives the gyro offset.
2. **Walk** 10–20 steps or **swing the shank** while seated: the principal axis of the angular rate (PCA) is the rotation axis. The app shows how much of the angular rate it explains and its angle to the vertical (should be close to 90°).
3. **Direction:** when walking, the sign follows from the gait pattern (swing peaks are larger than the negative peaks at toe-off and initial contact). For the pendulum, or to make sure, press *Check direction* and lift the foot forward within 3 s. *Reverse direction* flips it by hand.

The live plot of the sagittal angular rate shows whether the result is right (large positive swing peaks). **Write to device** stores it in flash; **Delete calibration** returns to the automatic axis.

### Long-term Measurement Tab

1. Connect via Bluetooth. The app sets the device clock.
2. Enter a **pseudonym** (1–8 characters, no names), the **side** and the **threshold** (default 2 kg), then **Start measurement**. The device logs on its own until **Stop measurement**, even without the app.
3. **Read new data** loads only the entries added since the last readout (about 4 s per 13,000 entries). The data stays in the browser; **Save raw data (.bin)** keeps everything and can be opened again later without the device.
4. **Analysis** per measurement: duration, strides, share of strides with heel peak above an adjustable threshold (primary endpoint), mean heel peak, minutes with movement, time standing with load; strides per hour as a chart and table; events.
5. **Gait measures in daily life**, per day and in total:
   - *Walking bouts* (Mobilise-D definition: at least 2 strides, pauses below 3 s): number, walking time, bouts < 10 s, 10–30 s, 30–60 s and > 60 s, longest and median duration.
   - *Cadence* (steps/min): median and 95th percentile over bouts of at least 10 s, peak 1-minute and peak 30-minute cadence (mean of the 30 best minutes, Tudor-Locke), minutes at ≥ 100 steps/min.
   - *Stance and swing:* stance %, swing time, stance/swing ratio, swing peak, shank excursion.
   - *Variability:* CV of stride time, stance time and heel peak within bouts of at least 20 strides, without the first and last 2.
   - *Heel load:* total and per-stride impulse (cumulative load), share above the threshold, median peak, heel contact and loading rate.
   - *Posture at rest:* movement, upright without heel load (sitting or standing unloaded), standing with heel load, lying / leg up.
6. **CSV** export of strides, minutes, hours, events, gait measures (one row per day) and walking bouts (local time and Unix ms).

Read out before a firmware update: the update erases the log.

### Browser Support

| | Bluetooth LE | USB serial |
|---|---|---|
| Chrome / Edge on macOS, Windows, Linux | yes | yes |
| Chrome on Android | yes | no |
| Brave | after enabling `brave://flags/#brave-web-bluetooth-api` | yes |
| Safari, Firefox, iOS | no | no |

Web Bluetooth and Web Serial need a secure context: `https://` (e.g. GitHub Pages), `localhost`, or a file opened from disk.

### Install on Android

1. Open the [web app](https://soylentorange.github.io/FEO_Sensor/) in Chrome.
2. Menu ⋮ → **Add to Home screen** → **Install**.
3. Start "FEO live" from the home screen, tap **Connect Bluetooth** and choose `XIAO-FEO`. Android asks for the "Nearby devices" permission the first time.

## Repository Layout

```text
platformio.ini          PlatformIO project (board seeed-xiao-nrf54l15, Zephyr)
src/main.c              Firmware: IMU, FSR and battery readout, serial CSV, BLE GATT service, rest mode
src/gait.c, gait.h      Step detection (Fadillioglu et al. 2020), force per stride and gait phase
tools/gait_test.c       Host test of the step detection (synthetic gait or a CSV recording)
zephyr/prj.conf         Zephyr configuration (I²C, sensor, ADC, BLE)
zephyr/app.overlay      Pins: D1 amplifier supply, A0 FSR, AIN7 battery
zephyr/sysbuild.conf    MCUboot for the release build
zephyr/ota.conf         MCUmgr/SMP over BLE (release build only)
west.yml                Zephyr version for the release build
.github/workflows/      release.yml: firmware release (factory hex + OTA update); pages.yml: web app + latest update on Pages
zephyr/CMakeLists.txt
docs/index.html         Web app (single file)
docs/manifest.webmanifest, docs/sw.js, docs/icons/   PWA files
docs/assets/            Logos and screenshot
hardware/               Schematic, BOM, perfboard layout, wiring, housing (see hardware/README.md)
```

---

## Acknowledgements & Funding

This project has received funding from the European Union's Horizon Europe research and innovation programme under grant agreement No. **101136376** ([**SmILE** – *Smart Implants for Life Enrichment*](https://horizon-smile.eu/)).

<p align="left">
  <a href="https://horizon-smile.eu/">
    <img src="docs/assets/Smile-logo.svg" alt="SmILE Project Logo" height="60">
  </a>
  &nbsp;&nbsp;&nbsp;&nbsp;
  <a href="https://commission.europa.eu">
    <img src="docs/assets/eu-funded.svg" alt="Funded by the European Union" height="60">
  </a>
</p>

* **Project Website:** [https://horizon-smile.eu/](https://horizon-smile.eu/)

*Disclaimer:* Funded by the European Union. Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Health and Digital Executive Agency (HADEA). Neither the European Union nor the granting authority can be held responsible for them.

The simplified 3D board model is derived from the Seeed Studio XIAO nRF54L15 STEP model ([GrabCAD](https://grabcad.com/library/seeed-studio-xiao-nrf54l15-sense-1)).

---

## License

This project is licensed under the **MIT License** ([MIT](LICENSE)).
For details, please refer to the [LICENSE](LICENSE) file.
