# Scope Rotator V13 — Field Reference Card
**BLE_Rotator_V13**  |  Serial: 115,200 baud  |  BLE: Nordic UART (nRF Toolbox / LightBlue)

---

## Quick Commands

| Command | Description |
|---------|-------------|
| `L` | **Enable lock** — arms Target Lock if GPS valid, else Heading Lock |
| `l` | **Disable lock** — manual only until `L` is sent again |
| `A` | Overcurrent trip **+25 mA** |
| `a` | Overcurrent trip **−25 mA** |
| `Z` or `z` | Reset encoder to 0 |
| `?` / `H` / `h` | Print help |
| `VER` | Firmware version |
| `INFO` | Full system snapshot |
| `STATUS ON` | Enable 4 Hz status stream (serial only) |
| `STATUS OFF` | Suppress 4 Hz status stream |

---

## GET / SET — Live Tuning

Changes apply **immediately**. Reset to defaults on reboot — write down good values and hardcode in `config.h`.

```
GET ALL            — show all parameters
GET <param>        — show one parameter
SET <param> <val>  — update live (float or int as appropriate)
```

### Heading Lock

| Parameter | Default | Description |
|-----------|---------|-------------|
| `HL_KP` | `14.0` | Proportional gain — motor cmd = Kp × error°. Raise for faster correction; too high causes oscillation |
| `HL_DEAD` | `2.0` | Deadband (degrees). Below this error, motor is off. Tighten for tighter hold; too small causes hunting |
| `HL_MIN` | `80` | Minimum command sent when outside deadband — overcomes stiction. Lower if motor lurches for small errors |
| `HL_CAP` | `400` | Maximum command (speed cap). Raise to allow faster slewing on large errors |

### Target Lock (GPS)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `TL_DIST` | `23.0` | Projection distance (m) from boat to target at arm time (~75 ft) |
| `TL_HACC` | `2.0` | Max GPS horizontal accuracy (m) required to arm |
| `TL_DRIFT` | `3.0` | Min boat displacement (m) before a new bearing update is sent to HL |
| `TL_ALPHA` | `0.3` | Bearing low-pass filter (0–1). Lower = smoother, slower. Higher = faster, noisier |
| `TL_MAXDIST` | `125.0` | Max distance (m) from stored target before TL cancels (~410 ft) |
| `TL_GPSLOST` | `5000` | How long (ms) TL holds last bearing after GPS lost before dropping to manual |

### Controller

| Parameter | Default | Description |
|-----------|---------|-------------|
| `AUTOLOCK_MS` | `2000` | Idle time (ms) after last pedal input before auto-arm re-engages |
| `TAKEOVER_THR` | `5` | Pedal command magnitude that triggers manual takeover from lock mode |

### Motor / Sensor

| Parameter | Default | Description |
|-----------|---------|-------------|
| `OVERCURRENT` | `1300` | DRV8874 overcurrent trip (mA). Also adjustable with `A`/`a` |
| `MAG_OFFSET` | `0.0` | Heading offset (°) to correct for BNO085 mounting misalignment |

---

## INFO Snapshot Fields

```
INFO
```
Reports: Version, Uptime, Mag/GPS available, Mode, BLE ok, GPS state, hAcc, numSV,
Boat heading, Rotator heading, HL target, TL active, TL dist to target, TL boat delta,
Overcurrent, Motor cmd, Encoder deg, Status stream, Phone connected,
GeoMag status, RotVec heading, RotVec accuracy.

---

## Status Stream (ST) — Serial Only, 4 Hz

Format: `ST, field0, field1, ... field14`

| # | Field | Notes |
|---|-------|-------|
| 0 | `now_ms` | Uptime in milliseconds |
| 1 | `mode` | `MAN` / `HL` / `TL` |
| 2 | `gpsState` | `INIT` / `ACQ` / `VALID` / `LOST` |
| 3 | `gps_ok` | 1 = healthy fix, 0 = no |
| 4 | `hAcc_m` | GPS horizontal accuracy (m) |
| 5 | `boatHead_deg` | Boat compass heading (°) |
| 6 | `rotHead_deg` | Rotator compass heading (°) |
| 7 | `TL_on` | 1 = Target Lock active |
| 8 | `distToTarget_m` | Boat distance from stored target (m) |
| 9 | `HL_target_deg` | Heading Lock target (°) |
| 10 | `motorCmd` | Raw motor command value |
| 11 | `geoMagStatus` | BNO085 geomagnetic calibration (0–3, 3=best) |
| 12 | `rotVecHeading_deg` | Game rotation vector heading (°) |
| 13 | `rotVecAccuracy_deg` | RotVec accuracy estimate — trust heading when < 10° |
| 14 | `rotVecHeadingAvg_deg` | Accuracy-gated 10-sample circular mean heading (°) |

---

## LED Status

### Motor Idle — shows Mode

| Color | Mode |
|-------|------|
| Dim White | Manual |
| Blue | Heading Lock |
| Blue-Violet | Target Lock |

### Motor Active — shows Direction & Speed

| Color | Direction |
|-------|-----------|
| Green | Rotating one way |
| Red | Rotating other way |
| Brightness | Speed — dim = slow, bright = fast |

---

## Boot Notes

- Press `x` within **1.5 s** of power-on to disable BLE (USB serial only)
- All SET changes are **RAM only** — hardcode in `config.h` when home
- `GET ALL` to confirm current live values before tuning
