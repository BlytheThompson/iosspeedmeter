# GNSS/IMU Vehicle Performance Timer — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the Dragy-equivalent acceleration/distance timer described in `ios-performance-timer-spec.md` — post-processed GNSS/IMU sensor fusion (forward Kalman + RTS smoother) delivered as an iOS app plus a headless, fully testable numerical core.

**Architecture:** Two layers, split exactly along the spec's Decision 2 boundary.
`PerformanceTimerCore` is pure Swift with **no Apple framework imports** — all the timing, filtering, smoothing, parsing and result-extraction maths. Because it imports nothing from Apple, it compiles and tests on Windows/Linux as well as macOS, which is what makes the maths verifiable without a device. The iOS layer (`App/`) contains only thin adapters (CoreMotion, CoreLocation, CMAltimeter, Network.framework, CoreBluetooth) that convert vendor callbacks into core value types, plus SwiftUI.

**Tech Stack:** Swift 5.9+ / SwiftPM, XCTest, SwiftUI, XcodeGen (`project.yml`), GitHub Actions macOS runners for the unsigned `.ipa` (spec §14).

## Global Constants and Constraints

- Standard gravity `g = 9.80665` m/s²; foot `0.3048` m; mile `1609.344` m; mph `0.44704`; knots `0.514444`; km/h `0.277778`; WGS84 `a = 6378137.0`, `f = 1/298.257223563`.
- All estimator state and covariance arithmetic is `Double`. Never `Float` — spec §6.4 requires it because the `P⁻¹` inversion is ill-conditioned in single precision.
- IMU target rate 100 Hz; `dt` is **always** computed from consecutive timestamps, never assumed to be `1/100` (spec §3.1).
- Reported results come **only** from the smoothed trace. The forward trace is display-only (spec §0 Decision 1, §6.4).
- Measurement time is the fix epoch, never the callback arrival time (spec §2.3).
- Core target imports only `Foundation`. Any `import CoreMotion`/`CoreLocation`/`Network`/`CoreBluetooth` in `Sources/PerformanceTimerCore` is a build-breaking error.

---

## Spec deviations (deliberate, with reasons)

These are the two places the implementation does **not** follow the spec text literally. Both
are recorded in code comments at the point of deviation and covered by tests.

### D1 — §6.2 process-noise matrix `Q` is dimensionally inconsistent; corrected

The spec defines `σ_a` as accelerometer white noise in **m/s²/√Hz** and `σ_b` as bias random
walk in **m/s³/√Hz**, then gives:

```
Q[1][1] = σ_a²·dt³/3        (velocity variance slot)
```

`σ_a²` has units m²/s³, so `σ_a²·dt³/3` has units m² — a *position* variance sitting in the
velocity slot. Every `σ_a` term in the published table is shifted one integration too high,
and the `σ_b` contributions to `Q[0][1]` and `Q[1][1]` are missing entirely.

Numerically this matters enormously. At `dt = 0.01`, `σ_a = 0.05`:
spec `Q[1][1] = 8.3e-10` vs correct `Q[1][1] = 2.5e-5` — **~30,000× too small**. A filter
using the published table believes its own dead-reckoned velocity to ~0.3 mm/s after a
second and effectively ignores every GNSS update, which defeats the purpose of the fusion.

The exact discretisation of the spec's own stated continuous model
(`ṡ = v`, `v̇ = a − b`, `ḃ = w_b`, with `w_a` entering `v̇`) is:

```
Q = σ_a²·⎡ dt³/3   dt²/2   0 ⎤  +  σ_b²·⎡ dt⁵/20   dt⁴/8   −dt³/6 ⎤
         ⎢ dt²/2   dt      0 ⎥          ⎢ dt⁴/8    dt³/3   −dt²/2 ⎥
         ⎣ 0       0       0 ⎦          ⎣ −dt³/6   −dt²/2   dt     ⎦
```

`ProcessNoise.swift` implements this as `.exact` and makes it the default. The published
table is preserved as `.specLiteral` so the two can be compared on real logged data, and
`ProcessNoiseTests` asserts both the dimensional scaling law and the magnitude gap.

### D2 — §5 longitudinal projection formula mixes frames; corrected

The spec writes `a_raw_V = R_DV · (R_launch_propagated · f_D − g_L)`. `R_DV` maps
device→vehicle, but its operand there has already been rotated into the local-level frame,
so the composition is not well formed. Gravity removal is done in the device frame instead:

```
a_kinematic_D = f_D − R_LD · ĝ_L·g          // R_LD from the propagated attitude
a             = (R_DV · a_kinematic_D).x
```

This is the same physical operation with a consistent frame chain, and it keeps `R_DV`
applied to a device-frame vector as §5 defines it.

Site: `Calibration/LongitudinalResolver.swift`.

### D3 — Appendix B's accuracy claim does not hold; exact transform used instead

Appendix B offers a tangent-plane approximation and states its error is "well under a
centimetre over a mile". Measured against an exact geodetic→ECEF→ENU transform, at
mid-latitude ~1.1 km out on a diagonal, it is off by **8.6 cm**. Re-evaluating the radii at
the midpoint latitude only improves it to 5.5 cm — the residual is the tangent-plane geometry
itself, not the choice of latitude.

8.6 cm is irrelevant beside a 3 m consumer fix, but §6.3 contemplates feeding GNSS position
into the filter once an RTK receiver is fitted, and at 1–2 cm RTK accuracy this becomes the
dominant error. Positions arrive at 1–25 Hz, so the exact transform costs nothing.
`.exact` is the default; `.appendixBLiteral` is retained for comparison.

Site: `Geo/LocalENU.swift`. Verified against two independent constructions (a numerically
integrated meridian arc and closed-form parallel geometry), not only against the ECEF oracle.

### D4 — §4's stationary detector re-arms mid-launch; two conditions added

**The most consequential deviation.** As published, all four conditions hold *in the middle of
a launch*: under steady acceleration the specific-force magnitude is constant, so its standard
deviation collapses back below 0.03 g once the 0.5 s window sits inside the accelerating
phase; the gyro is quiet on a straight run; and at 1 Hz the newest GNSS fix still reports the
pre-launch speed.

Measured on a synthetic 0–60 with a known answer: ZUPT resumed 0.5 s after launch, drove the
accelerometer bias state to **1.26 m/s²** of pure fiction, and the reported speed came out
**32% low**. A variance test structurally cannot separate rest from constant acceleration.

Two conditions close it:

1. **Force magnitude.** At rest `|f_D| = g`; under longitudinal acceleration `a` it is
   `√(a² + g²)`. Compared against the magnitude measured during a previously confirmed
   stationary window, so accelerometer scale error cancels.
2. **Post-motion GNSS confirmation.** Once motion has been seen, the IMU alone cannot prove
   the car stopped — constant velocity is indistinguishable from rest. Re-entry requires a
   valid fix newer than the last moving sample reporting near-zero speed. Applies only when
   the session has GNSS, so an IMU-only configuration degrades to published behaviour.

Site: `Calibration/StationaryDetector.swift`.

### D5 — §8's "monotonically increasing" anchor test never matches; slope used instead

§8 says the retroactive anchor requires "the next 100 ms of `a` is monotonically increasing".
Applied literally to 100 Hz data it never matches — accelerometer noise puts small reversals
throughout — so the backward search falls through the samples closest to the launch and
settles on an earlier, quieter instant. Measured: the anchor landed **80 ms early**, and that
error propagated directly into every reported mark.

A least-squares slope over the same window captures the intent ("this is where acceleration
starts to build") without being hostage to individual noisy samples.

Site: `Session/LaunchAnchor.swift`.

### One more design flaw, found and fixed during integration

Not a spec deviation, but worth recording: the ARMED state initially disarmed on the first
non-stationary sample. Since the launch *is* what makes the vehicle non-stationary, and §8's
trigger needs 60 ms to confirm, RECORDING was unreachable. ARMED now holds through a 1 s grace
period while moving, and disarms after it if no launch confirms.

---

## File Structure

```
Package.swift                                  SwiftPM manifest: core lib, replay CLI, tests
tools/swift.cmd                                Windows: run Swift inside MSVC env
project.yml                                    XcodeGen spec for the iOS app
.github/workflows/ci.yml                       macOS CI: swift test + unsigned .ipa

Sources/PerformanceTimerCore/
  Constants.swift                              §Appendix A
  Math/
    Vector3.swift                              3-vector, cross/dot/normalise
    Matrix3.swift                              3×3, rows/cols, transpose, orthonormalise
    Quaternion.swift                           unit quaternion, integration, Markley averaging
    Matrix.swift                               dense n×n/n×m Double, LU inverse with det guard
  Geo/LocalENU.swift                           §Appendix B tangent-plane ENU
  Time/
    SessionClock.swift                         §2.2 monotonic session clock
    ClockFit.swift                             §2.4 least-squares iTOW→session fit
  Sensors/
    SensorSample.swift                         IMUSample, GNSSFix, BaroSample, WheelSpeedSample
    SensorSource.swift                         §0 Decision 2 protocol
  Parsing/
    UBXParser.swift                            §3.5 framer, Fletcher-8, NAV-PVT
    RaceChronoBLEParser.swift                  §3.6 20-byte big-endian format
  Calibration/
    StationaryDetector.swift                   §4 four-condition detector
    AttitudePropagator.swift                   §3.2 launch-anchored attitude
    VehicleFrameCalibrator.swift               §5 R_DV + gates + refinement
    LongitudinalResolver.swift                 §5/D2 projection to vehicle +X
  Estimation/
    ProcessNoise.swift                         §6.2 Q (see D1)
    KalmanFilter.swift                         §6.1–6.3 predict/update/gating
    RTSSmoother.swift                          §6.4 backward recursion
    Estimator.swift                            drives filter over a sample stream
  Session/
    RingBuffer.swift                           §4 10 s pre-arm buffer
    SessionStateMachine.swift                  §4 IDLE→ARMED→RECORDING→…
    LaunchAnchor.swift                         §8 live trigger + retroactive anchor
  Grade/GradeEstimator.swift                   §7 elevation fit, raw vs corrected
  Results/
    Marks.swift                                §9.2 standard marks
    CrossingSolver.swift                       §9.1 quadratic / cubic interpolation
    ResultExtractor.swift                      §9.3 rollout, trap speed, assembly
    Confidence.swift                           §9.4 σ propagation + badge
  Logging/
    SessionLog.swift                           §10 JSON header + CSV schema
    SessionLogWriter.swift                     §10 writer
    SessionLogReader.swift                     §10 replay reader
  Replay/ReplayHarness.swift                   §15 step 2 offline re-run

Sources/PerformanceTimerReplay/main.swift      CLI front end for the harness

App/                                           iOS-only, not built by SwiftPM
  PerformanceTimerApp.swift, Info.plist
  Sensors/CoreMotionSource.swift               §3.1 + §3.2
  Sensors/CoreLocationSource.swift             §3.3
  Sensors/AltimeterSource.swift                §3.4
  Sensors/ExternalGNSSWiFiSource.swift         §3.5 Network.framework
  Sensors/RaceChronoBLESource.swift            §3.6 CoreBluetooth
  Sensors/WheelSpeedSource.swift               §3.7
  Runtime/SessionController.swift              wires sources → estimator → results
  UI/…                                         SwiftUI: arm, live, timeslip, logs, settings

Tests/PerformanceTimerCoreTests/               one file per core component
```

---

## Tasks

Each task is red→green→commit. Test command throughout:

```bash
cmd.exe /c "tools\swift.cmd test"
```

- [ ] **Task 1 — Constants (§App. A).** Exact literals, zero-tolerance assertions.
- [ ] **Task 2 — Math layer.** `Vector3`, `Matrix3`, `Quaternion` (integration + Markley average), dense `Matrix` with LU inverse and `|det| < 1e-12` guard (§6.4).
- [ ] **Task 3 — Local ENU (§App. B).** Prime-vertical/meridional radii; sub-cm accuracy over a mile asserted against a spherical reference.
- [ ] **Task 4 — Session clock (§2.2).** Monotonic and wall-clock conversions; assert the two agree for a simultaneous event.
- [ ] **Task 5 — Clock fit (§2.4).** Least-squares `t = α·iTOW + β`; require ≥30 pairs; reject `|α/0.001 − 1| > 100 ppm`; expose residual scatter.
- [ ] **Task 6 — Sensor value types + protocol (§3, Decision 2).**
- [ ] **Task 7 — UBX parser (§3.5).** Streaming framer with resync, Fletcher-8 checksum, NAV-PVT field decode at the documented offsets.
- [ ] **Task 8 — RaceChrono BLE parser (§3.6).** 21-bit time, sync-bit matching, dual-form altitude/speed encodings.
- [ ] **Task 9 — Stationary detector (§4).** All four conditions over a 0.5 s window.
- [ ] **Task 10 — Process noise (§6.2 / D1).** `.exact` and `.specLiteral`; scaling-law and magnitude-gap tests.
- [ ] **Task 11 — Kalman filter (§6.1–6.3).** Predict, GNSS-speed update, ZUPT, 3σ innovation gate, optional wheel-scale 4th state.
- [ ] **Task 12 — RTS smoother (§6.4).** Backward recursion; smoothed covariance ≤ forward covariance; determinant guard path.
- [ ] **Task 13 — Bench test (§12.1).** 60 s stationary: smoothed `|v| ≤ 0.02` m/s, `|s| ≤ 0.5` m.
- [ ] **Task 14 — Attitude propagation (§3.2).** Gyro-bias removal, launch-anchored integration, gravity reconstruction.
- [ ] **Task 15 — Vehicle frame calibration (§5).** `R_DV`, sign disambiguation, three rejection gates, quaternion-averaged refinement.
- [ ] **Task 16 — Crossing solver (§9.1).** Quadratic speed root, Newton+bisection cubic distance root, exactness on analytic motion.
- [ ] **Task 17 — Marks and result extraction (§9.2, §9.3).** Rest-anchored and 1-ft-rollout ETs, trap speeds.
- [ ] **Task 18 — Launch anchoring (§8).** Live trigger, backward search, roll-race variant.
- [ ] **Task 19 — Confidence (§9.4).** `σ_t ≈ σ_v/a` (and `σ_s/v` for distance marks); High/Medium/Low badge.
- [ ] **Task 20 — Grade (§7).** Elevation-vs-distance fit, baro/GNSS cross-check, raw + corrected reporting, >1% flag.
- [ ] **Task 21 — Session state machine (§4).** Ring buffer prepend on ARM→RECORD, stop conditions.
- [ ] **Task 22 — Logging (§10).** JSON header + exact CSV column list; round-trip test.
- [ ] **Task 23 — Replay harness + CLI (§15 step 2).** End-to-end synthetic run reproduces known ground truth.
- [ ] **Task 24 — iOS sensor adapters (§3.1, 3.3, 3.4, 3.5, 3.6, 3.7).**
- [ ] **Task 25 — SwiftUI app + Info.plist (§11).**
- [ ] **Task 26 — XcodeGen project + GitHub Actions unsigned .ipa (§14).**

## Spec coverage map

| Spec § | Task |
|---|---|
| 0 Decisions | Architecture (core/app split), 12, 23 |
| 1 Frames | 2, 14, 15 |
| 2 Timebase | 4, 5 |
| 3.1–3.4 Sensors | 6, 14, 24 |
| 3.5 UBX/Wi-Fi | 7, 24 |
| 3.6 BLE | 8, 24 |
| 3.7 CAN | 11 (4th state), 24 |
| 4 State machine | 9, 21 |
| 5 Calibration | 15 |
| 6 Estimator | 10, 11, 12 |
| 7 Grade | 20 |
| 8 Launch | 18 |
| 9 Results | 16, 17, 19 |
| 10 Logging | 22 |
| 11 Info.plist | 25 |
| 12 Validation | 13, 23, and `docs/validation.md` |
| 13 Performance | 19, `docs/validation.md` |
| 14 Build w/o Mac | 26, `docs/building.md` |
| 15 Build order | task order above |
