# GNSS/IMU Vehicle Performance Timer — Technical Specification

**Target:** iPhone 17 Pro Max, iOS 26
**Purpose:** Dragy-equivalent acceleration and distance timing (0–60, 60 ft, 1/8, 1/4 mile, roll races)
**Approach:** Post-processed sensor fusion — forward Kalman filter plus RTS smoother over recorded 100 Hz IMU and 1–25 Hz GNSS
**Version:** 1.0

---

## 0. Read this first: the two design decisions that matter

**Decision 1 — everything is post-processed.** The live display is a convenience readout from the forward filter only. The *reported result* is computed after the run ends, by a backward smoothing pass over the full recorded session. This is the single biggest accuracy lever available to you and most phone timers don't do it. Design the app so that timing results are never produced by the live path.

**Decision 2 — sensors are abstracted behind one protocol.** Internal GNSS at 1 Hz, an external 25 Hz receiver over Wi-Fi, and a CAN wheel-speed channel all feed the same estimator. Build the estimator against an interface, not against `CLLocationManager`. This is what lets you upgrade hardware later without touching the math.

---

## 1. Coordinate frames and notation

| Frame | Definition |
|---|---|
| **D** (device) | CoreMotion body frame. +X right, +Y top, +Z out of screen. |
| **V** (vehicle) | +X forward along vehicle centreline, +Z up perpendicular to the vehicle floor, +Y = Z × X. Fixed relative to the car, *not* to gravity. |
| **L** (local level) | East-North-Up at the session origin. |

`R_DV` is the 3×3 rotation mapping device → vehicle. Solving for it is §5.

Symbols used throughout:

- `a` — specific force along vehicle +X, m/s², gravity removed
- `v` — longitudinal ground speed, m/s
- `s` — distance travelled along the path, m
- `b` — accelerometer bias along vehicle +X, m/s²
- `θ` — road grade angle, radians, positive uphill
- `g` = 9.80665 m/s²

---

## 2. Timebase and synchronisation

This is where most implementations quietly lose 50–100 ms. Get it right before writing any filter code.

### 2.1 The problem

- `CMDeviceMotion.timestamp` is **seconds since device boot**, from the same clock as `ProcessInfo.processInfo.systemUptime` and `CACurrentMediaTime()`. It is monotonic.
- `CLLocation.timestamp` is a `Date` — wall clock.
- Bluetooth and Wi-Fi GNSS packets arrive with their own embedded time (GPS iTOW) plus an arrival time you observe.

You must put all three onto one monotonic session clock.

### 2.2 Session clock

At session start, capture once:

```
sessionEpoch  = CACurrentMediaTime()          // monotonic seconds
bootWallClock = Date(timeIntervalSinceNow: -ProcessInfo.processInfo.systemUptime)
```

Then:

| Source | Conversion to session time |
|---|---|
| `CMDeviceMotion` | `t = dm.timestamp - sessionEpoch` |
| `CLLocation` | `t = loc.timestamp.timeIntervalSince(bootWallClock) - sessionEpoch` |
| External GNSS | see §2.4 |

Recapture `bootWallClock` only at session start. Do not recompute per sample; `systemUptime` and `Date()` drift relative to each other and you will inject jitter.

### 2.3 GNSS delivery latency

`CLLocation.timestamp` is the **fix epoch**, not the delivery time. Delivery typically lags 100–500 ms and the lag is not constant. Because you post-process, this is harmless: buffer the fix, insert it into the measurement stream at its own timestamp, and the smoother handles the out-of-order arrival naturally.

For the live display only, expect the on-screen speed to lag reality by roughly the delivery latency plus half the update interval.

**Never** use the callback arrival time as the measurement time. That alone is worth 0.1–0.3 s of error on a 0–60.

### 2.4 External receiver time alignment

A UBX or BLE packet carries GPS time of week (`iTOW`). You need to map it to the session clock. Do this once per session with a linear fit:

1. For each packet, record `(iTOW_ms, t_arrival_session)`.
2. Collect ≥ 30 pairs at rest.
3. Least-squares fit `t_session = α·iTOW + β`. The slope α should be ≈ 0.001; if it deviates by more than 100 ppm your clocks disagree and something is wrong.
4. Apply the fit to every packet thereafter. The residual scatter is your link jitter — log it.

This removes both the constant transport delay and any clock-rate mismatch. Do **not** just timestamp packets on arrival; Wi-Fi and BLE jitter is 5–40 ms and will show up directly in your results.

---

## 3. Sensor interfaces

### 3.1 CoreMotion (primary IMU)

```
let mm = CMMotionManager()
mm.deviceMotionUpdateInterval = 1.0 / 100.0
mm.startDeviceMotionUpdates(using: .xArbitraryCorrectedZVertical, to: queue) { dm, err in ... }
```

Use a dedicated `OperationQueue` with `maxConcurrentOperationCount = 1`, not the main queue.

Fields consumed per sample:

| Field | Use |
|---|---|
| `userAcceleration` (g) | specific force with gravity removed |
| `gravity` (g) | gravity direction in device frame — for calibration and grade |
| `rotationRate` (rad/s) | gyro, for attitude propagation and turn rejection |
| `attitude` | quaternion; used for the launch-anchored propagation in §3.2 |
| `timestamp` | boot-relative seconds |

**Rate reality check.** 100 Hz is the third-party ceiling on `CMMotionManager`; requesting faster silently clamps. Some intermediate intervals get rounded to whatever the hardware supports, so always compute `dt` from consecutive timestamps rather than assuming `1/100`. The higher-rate `CMBatchedSensorManager` (800 Hz accelerometer, 200 Hz device motion) exists but is watchOS-only, and the IMU rate is not your limiting factor anyway.

### 3.2 The gravity-drift trap

CoreMotion's `gravity` estimate uses the accelerometer as a long-term reference for "down". Under **sustained longitudinal acceleration** — exactly a drag run — the fusion filter cannot distinguish a 0.5 g forward push from a 30° nose-up pitch, and its gravity vector will slowly tilt toward the acceleration. `userAcceleration` inherits that error.

Mitigation:

1. During the pre-launch stationary window, record `q_launch` (the attitude quaternion) and the mean `gravity` vector. At rest, both are trustworthy.
2. From launch onward, **propagate attitude yourself** by integrating `rotationRate` (bias-corrected, see below) forward from `q_launch`.
3. Reconstruct specific force as `f_D = (userAcceleration + gravity) * g` — this recovers the raw accelerometer reading, undoing CoreMotion's gravity removal.
4. Remove gravity using *your* propagated attitude, not CoreMotion's.

Gyro bias: average `rotationRate` over the stationary window and subtract. Residual gyro bias of 0.1 °/s produces ~1.5° of attitude error over a 15 s run, which is ~0.026 g ≈ 0.26 m/s² of false longitudinal acceleration. The filter's bias state `b` absorbs most of this; the point of the propagation is that the error is now a slow ramp the filter can track rather than a signal-correlated distortion it cannot.

Runs longer than about 25 s: reset attitude at a moment of steady-state cruise (near-zero longitudinal accel), where CoreMotion's gravity is trustworthy again.

### 3.3 CoreLocation (internal GNSS)

```
let lm = CLLocationManager()
lm.desiredAccuracy = kCLLocationAccuracyBestForNavigation
lm.distanceFilter  = kCLDistanceFilterNone
lm.pausesLocationUpdatesAutomatically = false
lm.activityType = .otherNavigation      // see warning below
lm.startUpdatingLocation()
```

Fields consumed:

| Field | Use | Validity |
|---|---|---|
| `speed` (m/s) | **primary measurement** — Doppler-derived | negative = invalid |
| `speedAccuracy` (m/s) | measurement noise σ | negative = invalid |
| `course`, `courseAccuracy` | straight-line gating | negative = invalid |
| `coordinate`, `horizontalAccuracy` | secondary position check | — |
| `altitude`, `verticalAccuracy` | grade estimation | — |
| `timestamp` | fix epoch | — |

**`activityType` warning.** Apple DTS has stated that declaring `.automotiveNavigation` causes Core Location to correct coordinate randomness by **pulling positions toward roads**. On a drag strip or an unmapped surface that map-matching will corrupt your position trace and therefore your distance marks. Doppler speed should be unaffected, but until you have verified this on your own device, prefer `.otherNavigation` or `.other`. Make it a debug setting and compare traces.

**Rate.** Expect ~1 Hz. Core Location does not commit to any rate publicly and Apple explicitly advises degrading gracefully based on observed behaviour rather than assuming. Measure the actual interval at runtime and feed it to the filter as `R` scaling.

### 3.4 Barometer (grade)

```
let alt = CMAltimeter()
alt.startRelativeAltitudeUpdates(to: queue) { data, err in ... }
```

Fixed at **1 Hz** — there is no interval property and requests to change it are ignored. Over a 15 s run that's 15 samples, enough to fit a linear elevation profile. Relative altitude resolution is roughly 0.1 m, which is far better than GNSS vertical.

Caveat: cabin pressure changes from HVAC, an open window, or a door seal flexing will show as false altitude. Cross-check against GNSS `altitude`; if they disagree by more than 1 m over the run, fall back to GNSS vertical or disable grade correction and flag the run.

### 3.5 External GNSS over Wi-Fi (recommended upgrade path)

Connect to the receiver's SoftAP, open a TCP socket with `Network.framework`:

```
let params = NWParameters.tcp
params.requiredInterfaceType = .wifi     // MANDATORY
let conn = NWConnection(host: "192.168.4.1", port: 2947, using: params)
```

Two things will bite you:

- **`requiredInterfaceType = .wifi`** is not optional. Without it, iOS sees the SoftAP has no internet route and sends your traffic over cellular, where the ESP32 does not exist.
- **`NSLocalNetworkUsageDescription`** must be in Info.plist or the connection silently fails on iOS 14+. Add `NSBonjourServices` too if you advertise via Bonjour.

Also set `UIApplication.shared.isIdleTimerDisabled = true`. There are field reports of iOS dropping the Wi-Fi association when the screen locks on a network with no internet route.

**Prefer UBX binary over NMEA.** NMEA `RMC` gives you speed in knots with no accuracy estimate. `UBX-NAV-PVT` gives you a real per-fix speed sigma, which is what the filter needs for `R`.

#### UBX-NAV-PVT (class `0x01`, ID `0x07`, payload length 92)

Frame: `B5 62 01 07 5C 00 <92-byte payload> CK_A CK_B`. All multi-byte fields little-endian. Checksum is 8-bit Fletcher over class, ID, length, and payload.

| Offset | Type | Field | Scale / Unit |
|---|---|---|---|
| 0 | U4 | `iTOW` | ms, GPS time of week |
| 4 | U2 | `year` | |
| 6–10 | U1 ×5 | `month, day, hour, min, sec` | |
| 11 | X1 | `valid` | date/time validity flags |
| 12 | U4 | `tAcc` | ns |
| 16 | I4 | `nano` | ns, fraction of second (signed) |
| 20 | U1 | `fixType` | 0 none, 2 2D, 3 3D, 4 GNSS+DR, 5 time-only |
| 21 | X1 | `flags` | bit 0 `gnssFixOK`; bits 6–7 carrSoln (1=float, 2=fixed) |
| 22 | X1 | `flags2` | |
| 23 | U1 | `numSV` | |
| 24 | I4 | `lon` | 1e-7 deg |
| 28 | I4 | `lat` | 1e-7 deg |
| 32 | I4 | `height` | mm above ellipsoid |
| 36 | I4 | `hMSL` | mm above MSL |
| 40 | U4 | `hAcc` | mm |
| 44 | U4 | `vAcc` | mm |
| 48 | I4 | `velN` | mm/s |
| 52 | I4 | `velE` | mm/s |
| 56 | I4 | `velD` | mm/s |
| 60 | I4 | `gSpeed` | mm/s — **2D ground speed, your primary measurement** |
| 64 | I4 | `headMot` | 1e-5 deg |
| 68 | U4 | `sAcc` | mm/s — **speed accuracy, your `R`** |
| 72 | U4 | `headAcc` | 1e-5 deg |
| 76 | U2 | `pDOP` | 0.01 |

Verify this layout against the interface description for your specific module generation before shipping; u-blox has extended the trailing fields across generations, though offsets 0–76 have been stable.

Precise fix epoch = GPS-week-start + `iTOW`/1000 + `nano`/1e9. Use that in the §2.4 fit.

Rate note: 25 Hz needs the TCP path. BLE does not have the bandwidth for it, and NMEA over BLE will not sustain more than 1–2 Hz.

### 3.6 External GNSS over BLE (alternative)

If you'd rather not deal with Wi-Fi, the RaceChrono DIY BLE format is public and you can build a receiver that speaks it, or write your app to consume it from an existing device.

Service UUID `00001ff8-0000-1000-8000-00805f9b34fb`. GPS main characteristic UUID `0x0003`, READ + NOTIFY, 20 bytes, **big-endian**:

| Bytes | Content |
|---|---|
| 0–2 | 3 sync bits, then 21-bit time from hour start = `(min × 30000) + (sec × 500) + (ms / 2)` |
| 3 | fix quality (2 bits), locked satellites (6 bits; `0x3F` invalid) |
| 4–7 | latitude, deg × 1e7, signed |
| 8–11 | longitude, deg × 1e7, signed |
| 12–13 | altitude, `((m + 500) × 10) & 0x7FFF`, or `((m + 500) & 0x7FFF) \| 0x8000` for the coarse form |
| 14–15 | speed, `(km/h × 100) & 0x7FFF`, or `((km/h × 10) & 0x7FFF) \| 0x8000` for the coarse form |
| 16–17 | bearing, deg × 100 |
| 18 | HDOP × 10 |
| 19 | VDOP × 10 |

Characteristic `0x0004` carries the hour and date in a matching 21-bit field. **Match the two by comparing sync bits**; if they differ, wait for one to update.

Note the two-form encoding on altitude and speed — check the high bit before decoding. And note there is no speed accuracy field, so you'll have to derive `R` from HDOP, which is much weaker than `sAcc`.

### 3.7 CAN wheel speed (optional third source)

The cleanest route is to put a CAN transceiver on the same ESP32 that carries the GNSS, and ship wheel speed over the existing link. This avoids the External Accessory framework and MFi entirely.

If you instead use an OBDLink adapter: on iOS only the **MX+** works. The LX, MX and CX will not. Note also that adapter throughput is shared across logged channels, so log wheel speed alone if timing is the goal.

Treat wheel speed as a measurement of `v` with a scale factor:

- Add a fourth state `k` (tyre circumference scale), random walk, initialised to 1.0 with σ ≈ 0.02.
- Measurement: `z = k · v`, so `H = [0, k, 0, v]`.
- **Gate it off during the launch window.** Wheelspin makes wheel speed read high exactly when you care most. Suppress the measurement while `|a| > 0.35 g` or while wheel speed exceeds fused speed by more than 1.5 m/s.

Its real value is the mid-run and high-speed segments, where it eliminates drift, and the calibrated `k` it hands you for free.

---

## 4. Session state machine

```
IDLE
  → ARMED        GNSS lock acquired, calibration valid, stationary detected
  → RECORDING    launch detected (retroactively anchored)
  → COMPLETE     stop condition met
  → ANALYSING    smoother + result extraction
  → RESULT
```

Recording actually begins at **ARMED**, not at RECORDING. You need the pre-launch data for ZUPT and for retroactive launch anchoring. Keep a 10 s ring buffer while ARMED and prepend it on transition.

**Stationary detector** — all four must hold over a 0.5 s window:

- `stdev(|f_D|) < 0.03 g`
- `max(|rotationRate|) < 0.02 rad/s`
- GNSS speed `< 0.3 m/s` (when a valid fix exists)
- window contains ≥ 40 IMU samples

**Stop condition:** target distance exceeded, or speed below 1 m/s for 2 s, or 120 s elapsed, or user stop.

---

## 5. Vehicle frame calibration (`R_DV`)

Run this once per mounting position and persist it, keyed by a mount profile.

**Step 1 — static, vehicle at rest on flat ground.**
Average `gravity` over 2 s → `ĝ_D` (unit vector, points down in device frame).
Vehicle up: `z_V = -ĝ_D`, normalised.

**Step 2 — dynamic, one straight-line acceleration event.**
Take mean `userAcceleration` over the first 1.5 s of a launch → `ā_D`.
Remove the vertical component: `a_h = ā_D - (ā_D · z_V) z_V`.
Vehicle forward: `x_V = normalize(a_h)`.

**Step 3.** `y_V = z_V × x_V`, then re-orthogonalise `x_V = y_V × z_V`.

`R_DV` has rows `[x_V; y_V; z_V]`.

**Sign disambiguation.** Confirm `x_V` points forward, not rearward, by checking that projected acceleration is positive while GNSS speed is increasing. If negative, flip `x_V` and `y_V`.

**Validation gates.** Reject the calibration if:
- `|ā_D · z_V| > 0.15 · |ā_D|` (car not on level ground, or mount moved)
- `|a_h| < 1.5 m/s²` (launch too gentle to resolve the axis)
- GNSS `courseAccuracy` during the event exceeds 5° (not actually straight)

Refine across runs by averaging `x_V` over the last N valid calibration events (use quaternion averaging, not component averaging).

**Longitudinal projection** at each sample:
```
a_raw_V = R_DV · (R_launch_propagated · f_D - g_L)
a       = a_raw_V.x
```

---

## 6. The estimator

### 6.1 State

```
x = [ s, v, b ]ᵀ
```

- `s` — distance along path, m
- `v` — longitudinal ground speed, m/s
- `b` — accelerometer bias along vehicle +X, m/s²

Add `k` (wheel scale) as a fourth state only if you implement §3.7.

### 6.2 Process model

Input `u = a` (measured longitudinal specific force, gravity and grade removed). With timestep `dt`:

```
        ⎡ 1   dt  -dt²/2 ⎤            ⎡ a·dt²/2 ⎤
  F  =  ⎢ 0   1    -dt   ⎥      Bu =  ⎢  a·dt   ⎥
        ⎣ 0   0     1    ⎦            ⎣    0    ⎦
```

Prediction: `x⁻ = F x + Bu`, `P⁻ = F P Fᵀ + Q`.

Process noise, from continuous accelerometer white noise `σ_a` (m/s²/√Hz) and bias random walk `σ_b` (m/s³/√Hz):

```
Q =
⎡ σ_a²·dt⁵/20 + σ_b²·dt⁵/20   σ_a²·dt⁴/8    -σ_b²·dt³/6 ⎤
⎢ σ_a²·dt⁴/8                  σ_a²·dt³/3    -σ_b²·dt²/2 ⎥
⎣ -σ_b²·dt³/6                 -σ_b²·dt²/2    σ_b²·dt    ⎦
```

**Starting values** (tune from logged data, don't trust these blindly):

| Parameter | Start | Notes |
|---|---|---|
| `σ_a` | 0.05 m/s²/√Hz | raise if the mount vibrates |
| `σ_b` | 0.002 m/s³/√Hz | governs how fast bias is allowed to wander |
| `σ_gps` floor | 0.05 m/s | never trust `speedAccuracy` below this |
| `σ_zupt` | 0.01 m/s | ZUPT measurement noise |

### 6.3 Measurement updates

**GNSS speed** — `z = gSpeed` or `location.speed`:
```
H = [0, 1, 0]
R = max(speedAccuracy, 0.05)²
```
Reject the fix if any of: `speed < 0`, `speedAccuracy < 0`, `speedAccuracy > 1.0 m/s`, `fixType < 3`, or `gnssFixOK` clear.

**Innovation gating.** Compute `y = z - H x⁻` and `S = H P⁻ Hᵀ + R`. Reject if `y² / S > 9` (3σ). Log every rejection; a run with more than 2 rejections should be flagged as low confidence.

**ZUPT** — while the stationary detector is true:
```
z = 0, H = [0, 1, 0], R = σ_zupt²
```
Apply at every IMU sample during the stationary window. This is what pins `b` to near-truth immediately before launch, and it is worth more than any hardware upgrade below the 25 Hz tier.

**GNSS position (optional).** Convert lat/lon to local ENU and use path length as a measurement of `s` with `R = hAcc²`. Only worth adding with an RTK receiver; with 3 m consumer accuracy it adds nothing over integrated Doppler.

### 6.4 RTS smoother

Store per sample during the forward pass: `x_k|k`, `P_k|k`, `x_k+1|k`, `P_k+1|k`, `F_k`.

Backward recursion from `k = N-1` down to `0`:

```
C_k    = P_k|k · F_{k+1}ᵀ · (P_{k+1|k})⁻¹
x_k|N  = x_k|k + C_k · (x_{k+1|N} - x_{k+1|k})
P_k|N  = P_k|k + C_k · (P_{k+1|N} - P_{k+1|k}) · C_kᵀ
```

Memory is trivial: a 20 s run at 100 Hz is 2000 samples × (3 + 9 + 9 + 9 + 9) doubles ≈ 620 KB. Store as `Double`, not `Float` — the `P⁻¹` inversion is poorly conditioned in single precision.

Invert the 3×3 `P_{k+1|k}` by cofactor expansion with a determinant guard; if `|det| < 1e-12`, skip the smoothing step for that index and carry `x_k|k` forward.

**Every reported result comes from the smoothed trace.** The forward trace is display-only.

---

## 7. Grade handling

Two separate things, often conflated.

**7.1 Removing grade from the acceleration input.** The vehicle frame is defined relative to the *car*, so on a grade the longitudinal axis includes a component of gravity. Subtract it:

```
a_corrected = a_measured + g · sin(θ)
```

where `θ` is the instantaneous road grade, positive uphill. If you're propagating attitude from a launch anchored on the same grade, this is already partly handled; the residual goes into `b`.

**7.2 Correcting the reported result.** Fit elevation vs. distance over the run using barometric relative altitude (primary) or GNSS altitude (fallback). Report:

- mean grade over the measured interval, as a percentage
- the raw result
- a grade-corrected result

Do not silently apply the correction. Show both, the way a Dragy timeslip charts the slope. A run with more than 1% mean grade should be visually flagged.

Grade correction of elapsed time is an approximation, not a transform — say so in the UI.

---

## 8. Launch detection and anchoring

Because you post-process, you can anchor the launch precisely instead of guessing live.

**Live trigger** (starts recording): smoothed longitudinal `a > 0.15 g` sustained for ≥ 60 ms.

**Retroactive anchor** (defines `t = 0`): after the run, search *backward* from the trigger for the last sample satisfying all of:

- `v_smoothed < 0.02 m/s`
- stationary detector true
- next 100 ms of `a` is monotonically increasing

That sample is `t₀`. Set `s(t₀) = 0`, `v(t₀) = 0`.

This routinely recovers 100–200 ms versus triggering on GNSS, and it removes the arbitrariness of a threshold choice from your reported number.

**Roll-race variant.** For a 60–130 style run there is no stationary anchor and no ZUPT. Anchor on the smoothed speed trace crossing the lower bound (§9) and accept the wider uncertainty. Flag roll results as a distinct confidence class.

---

## 9. Result extraction

### 9.1 Interpolated crossings

Never report the nearest sample. At every sample you have `s`, `v`, and `a` from the smoothed trace, so use a local quadratic.

**Speed target `v_t`:** find `i` where `v[i] ≤ v_t < v[i+1]`. With `j = (a[i+1] - a[i]) / dt`, solve for `τ ∈ [0, dt]`:

```
v_t = v[i] + a[i]·τ + ½·j·τ²
```

Take the root in range. `t_cross = t[i] + τ`.

**Distance target `s_t`:** same structure, one integration higher:

```
s_t = s[i] + v[i]·τ + ½·a[i]·τ² + (1/6)·j·τ³
```

Solve by two or three Newton iterations from `τ₀ = (s_t - s[i]) / v[i]`.

### 9.2 Standard marks

| Mark | Value |
|---|---|
| 60 ft | 18.288 m |
| 330 ft | 100.584 m |
| 1/8 mile | 201.168 m |
| 1000 ft | 304.800 m |
| 1/4 mile | 402.336 m |
| 1/2 mile | 804.672 m |
| 1 mile | 1609.344 m |
| Rollout | 0.3048 m |

Speed marks: 0–30, 0–60, 0–100, 0–130 mph and metric equivalents; roll windows 40–100, 60–130, 100–150, 100–200.

### 9.3 Rollout

Compute `t_rollout` where `s = 0.3048 m`. Report **both**:

- **Rest-anchored:** ET measured from `t₀`
- **1-foot rollout:** ET measured from `t_rollout` — this is what matches a drag strip timeslip and what Dragy displays

Trap speed is `v` at the distance mark, unaffected by which anchor you chose.

### 9.4 Confidence reporting

Every result carries a σ, taken from the smoothed `P` at the crossing index, propagated through the interpolation:

```
σ_t ≈ σ_v(t_cross) / a(t_cross)
```

Display it. A 0–60 of 4.31 ± 0.04 s is a scientifically honest number; 4.31 s alone is not.

Degrade to a coarse confidence badge (High / Medium / Low) based on: number of rejected GNSS fixes, mean `speedAccuracy`, GNSS update rate achieved, calibration age, and grade magnitude.

---

## 10. Data logging

Log every session to disk in `Documents/`, always, regardless of outcome. Failed runs are where you'll learn what's wrong.

**Format:** one JSON header + one CSV per session.

Header: device model, iOS version, session UUID, sensor sources active, `R_DV`, calibration timestamp, filter parameters, clock-fit residuals.

CSV columns, one row per IMU sample:

```
t_session, ax_D, ay_D, az_D, gx_D, gy_D, gz_D,
grav_x, grav_y, grav_z, quat_w, quat_x, quat_y, quat_z,
a_long, v_fwd, s_fwd, b_fwd, v_smooth, s_smooth, b_smooth,
gnss_valid, gnss_speed, gnss_sAcc, gnss_lat, gnss_lon, gnss_alt,
gnss_fixtype, gnss_numsv, baro_alt, zupt_active, gate_reject
```

Set `UIFileSharingEnabled` and `LSSupportsOpeningDocumentsInPlace` to `YES` in Info.plist so you can pull logs off through the Files app without a computer.

Build an offline replay mode that reads a CSV and reruns the whole estimator. You will retune `Q` and `R` a dozen times and you do not want to drive for each iteration.

---

## 11. Info.plist and entitlements

| Key | Why |
|---|---|
| `NSLocationWhenInUseUsageDescription` | required, app will crash without it |
| `NSMotionUsageDescription` | required for CoreMotion, crash without it |
| `NSLocalNetworkUsageDescription` | **required** for the Wi-Fi GNSS socket |
| `NSBluetoothAlwaysUsageDescription` | only if using the BLE path |
| `NSBonjourServices` | only if discovering the receiver via Bonjour |
| `UIFileSharingEnabled` = YES | log retrieval via Files |
| `LSSupportsOpeningDocumentsInPlace` = YES | same |
| `UIBackgroundModes` → `location` | only if you need background recording; adds review friction and isn't needed for a screen-on timer |

Set `UIApplication.shared.isIdleTimerDisabled = true` while ARMED or RECORDING.

---

## 12. Validation plan

Do not trust the app until it has passed these.

**12.1 Bench test.** Phone stationary on a desk for 60 s. Smoothed `v` must stay within ±0.02 m/s of zero and `s` within ±0.5 m. If it drifts, your bias handling or ZUPT is broken.

**12.2 Clock test.** Plot GNSS fix timestamps against session time. The interval histogram should be tight around 1.000 s (or 0.040 s at 25 Hz). Scatter above ±20 ms means your §2 conversion is wrong.

**12.3 Video ground truth.** This is the only test that actually validates the whole chain end to end.

The 17 Pro Max shoots slo-mo at 1080p up to 240 fps — 4.17 ms per frame. Set two cones a tape-measured 60 ft apart, camera on a tripod perpendicular to the run, framing both. Count frames between the nose passing each cone. Compare against your app's 60 ft time.

Repeat five times. Your app should agree within ±0.03 s and, more importantly, the *bias* should be near zero. A consistent offset means a systematic error you can find; scatter means a noise problem.

**12.4 Consistency test.** Six back-to-back runs, same car, same driver, same direction. Report the standard deviation of your 0–60. If your run-to-run σ exceeds your claimed uncertainty, your uncertainty estimate is dishonest.

**12.5 Reciprocal test.** Three runs each direction on the same stretch. Grade correction should collapse the two means together. If it doesn't, your grade handling is wrong.

---

## 13. Expected performance by configuration

| Configuration | Est. 0–60 uncertainty | Notes |
|---|---|---|
| Internal GNSS, forward filter only | ±0.10–0.15 s | what most App Store timers deliver |
| + RTS smoother + ZUPT | ±0.05–0.08 s | software only, no hardware |
| + 25 Hz external GNSS over Wi-Fi | ±0.02–0.03 s | Dragy/RaceBox class |
| + CAN wheel speed | ±0.015 s | drift eliminated mid-run |
| + RTK via NTRIP | ±0.01 s | distance marks become exact |

These are error-budget estimates, not measurements. §12.3 is how you find out where you actually landed.

Be conservative in what the UI claims. An app that reports ±0.06 s and is right is more useful than one that reports ±0.02 s and isn't.

---

## 14. Building and sideloading without a Mac

You cannot compile a native iOS app on the iPhone itself, so you need a build machine somewhere. Options, cheapest first:

**Cloud CI (recommended).**
- **GitHub Actions** provides macOS runners. Public repos get unlimited minutes; private repos consume the free allotment at a 10× multiplier for macOS. Build an unsigned `.ipa`, publish it as a workflow artifact.
- **Codemagic** has a free tier of macOS build minutes and is purpose-built for mobile.

Push code from any machine, let CI produce the `.ipa`, download it.

**Rented Mac.** MacinCloud, MacStadium, or AWS EC2 Mac instances give you full Xcode over VNC. More expensive but you get a debugger, Instruments, and the simulator — worth it for the first few weeks of a project this sensor-heavy.

**Getting the .ipa onto the phone.**
- **Sideloadly** runs on Windows and Linux.
- **AltStore** needs AltServer running on a computer on the same network.
- **SideStore** needs a one-time computer-assisted setup, then refreshes on-device over a local WireGuard tunnel.

**Signing limits.** A free Apple ID gives 7-day certificates, a 3-app limit, and 10 app IDs per week. A paid Developer Program account ($99/yr) gives 1-year certificates and removes the refresh treadmill. For a project you'll iterate on daily, the paid account pays for itself in avoided frustration within a month.

**Cross-platform frameworks are a trap here.** Expo EAS Build will produce an iOS build without a Mac, but pushing 100 Hz CoreMotion samples across a JavaScript bridge introduces jitter in exactly the signal your filter is most sensitive to. If you go that route, put the sensor capture and the estimator in a native module and use JS only for UI. At which point you may as well write it in Swift.

---

## 15. Build order

1. Logging skeleton — CoreMotion + CoreLocation → CSV, plus the §2 clock conversion. Verify with §12.2.
2. Offline replay harness. Read CSV, no UI. Everything after this is developed against recorded data.
3. Forward Kalman filter, GNSS speed updates only. Verify with §12.1.
4. ZUPT and stationary detection.
5. RTS smoother.
6. Vehicle frame calibration.
7. Launch anchoring and crossing interpolation. First real results.
8. Video validation (§12.3). **Stop and do this before building any UI.**
9. Grade correction.
10. UI, results storage, sharing.
11. External GNSS ingestion.
12. CAN, then RTK.

Steps 1–8 get you a working, honest timer. Everything after is refinement.

---

## Appendix A — Constants

```
g                 = 9.80665      m/s²
mph → m/s         = 0.44704
knots → m/s       = 0.514444
km/h → m/s        = 0.277778
foot              = 0.3048       m
mile              = 1609.344     m
WGS84 a           = 6378137.0    m
WGS84 f           = 1/298.257223563
```

## Appendix B — Local ENU conversion

For short sessions, use a tangent-plane approximation anchored at the session origin `(φ₀, λ₀)`:

```
R_N = a / sqrt(1 - e²·sin²φ₀)                  // prime vertical radius
R_M = a·(1 - e²) / (1 - e²·sin²φ₀)^(3/2)       // meridional radius
E   = (λ - λ₀) · R_N · cos φ₀
N   = (φ - φ₀) · R_M
U   =  h - h₀
```

with `e² = 2f - f²`. Error is well under a centimetre over a mile, which is below the noise floor of everything except an RTK fix.
