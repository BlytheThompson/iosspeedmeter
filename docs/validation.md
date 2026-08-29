# Validation

Spec §12: *"Do not trust the app until it has passed these."*

This page records honestly what has and has not been done. The distinction matters, because
four of the five tests below need a car and none of them have been run.

## Status

| Test | What it proves | Status |
|---|---|---|
| §12.1 Bench | Bias handling and ZUPT | **Automated and passing** |
| §12.2 Clock | The §2 time conversion | **Partly automated** — conversions are unit-tested; the interval histogram needs a device |
| §12.3 Video ground truth | The whole chain, end to end | **Not run.** Needs a car, two cones and a tripod |
| §12.4 Consistency | That the claimed uncertainty is honest | **Not run** on real data; a synthetic analogue passes |
| §12.5 Reciprocal | Grade handling | **Not run** |

Everything automated runs on every push, including a macOS job that compiles the iOS layer and
produces an unsigned `.ipa`. Nothing automated can validate a sensor.

## What the automated tests actually cover

242 tests over `PerformanceTimerCore`. The ones that carry the most weight:

**§12.1 bench test** (`testBenchTestSixtySecondsStationary`). Sixty seconds of synthetic
stationary data at 100 Hz with a realistic 0.08 m/s² turn-on bias. The smoothed speed must stay
within ±0.02 m/s and the distance within ±0.5 m. A companion test
(`testBenchTestWithoutZUPTWouldDriftAway`) shows the same data drifting more than 5 m with ZUPT
suppressed, which is what makes the first test meaningful rather than vacuous.

**End-to-end synthetic run** (`testRecoversKnownZeroToSixtyFromASyntheticRun`). A full session
generated from exact ground truth: mounted phone at an arbitrary orientation, 0.12 m/s²
accelerometer bias, gyro bias, CoreMotion's gravity vector tilting under sustained acceleration
exactly as §3.2 describes, and noisy 1 Hz GNSS. The recovered 0–60 lands within about 15 ms of
the exact answer, and every distance mark within about 13 ms.

That number is better than the ±0.05–0.08 s that spec §13 budgets for this configuration —
which should be read as *the maths is not the limiting factor*, not as a claim about real
accuracy. Synthetic noise is generous: it is stationary, Gaussian, and uncorrelated with the
signal. Real GNSS multipath is none of those things.

**Smoother beats forward filter** (`testSmoothedResultsBeatForwardOnlyResults`). Runs six
seeds and checks the backward pass produces less total error than the forward trace over the
same data. This is spec Decision 1 stated as a test.

**Honesty of the reported σ** (`testReportedUncertaintyIsNotOptimisticRelativeToRunToRunSpread`).
Spec §12.4: *"If your run-to-run σ exceeds your claimed uncertainty, your uncertainty estimate
is dishonest."* Checked across eight seeds.

## The one that matters: §12.3 video ground truth

**Do this before building anything else on top of the app.** Spec §15 step 8 puts it before the
UI for a reason: it is the only test that validates the sensors, the clock, the calibration and
the maths together.

1. Two cones, a tape-measured **60 ft** apart.
2. Phone on a tripod, perpendicular to the run, framing both cones.
3. Shoot slo-mo at 1080p **240 fps** — 4.17 ms per frame.
4. Count frames between the nose passing each cone.
5. Compare against the app's 60 ft time.
6. **Repeat five times.**

Agreement should be within ±0.03 s, and — more importantly — the *bias* should be near zero. A
consistent offset is a systematic error you can find and fix. Scatter is a noise problem, which
is a different fix.

One systematic offset is already known and quantified: the retroactive anchor snaps to the
100 Hz sample grid, so it lands up to 10 ms early. On the synthetic run this contributes a
consistent +10 ms to every mark. If the video test shows a bias near that figure, this is where
it comes from; if it shows a much larger one, look elsewhere first.

## §12.2 clock test on device

Plot GNSS fix timestamps against session time. The interval histogram should be tight around
1.000 s, or 0.040 s at 25 Hz. **Scatter above ±20 ms means the §2 conversion is wrong.**

Every logged session already carries what this needs — `t_session` and the GNSS columns — so it
is a matter of plotting a CSV, not of writing more app.

For an external receiver, `ClockFit.Solution.residualRMS` is recorded in the log header. That
figure *is* the link jitter. Spec §2.4 expects 5–40 ms over Wi-Fi or BLE; a much larger number
means the fit is not holding.

## §12.4 consistency, on a real car

Six back-to-back runs. Same car, same driver, same direction. Report the standard deviation of
the 0–60.

If run-to-run σ exceeds the app's claimed uncertainty, the uncertainty estimate is dishonest and
should be widened until it is not. Spec §13 closes with the right instruction: *"Be conservative
in what the UI claims. An app that reports ±0.06 s and is right is more useful than one that
reports ±0.02 s and isn't."*

## §12.5 reciprocal, for grade

Three runs each direction on the same stretch. Grade correction should collapse the two means
together. If it does not, the grade handling is wrong.

Note that the app deliberately shows the raw and grade-corrected results side by side and never
silently substitutes one for the other, so this test reads directly off the timeslip.

## Retuning against recorded data

Once you have real logs, `pt-replay` is the loop spec §6.2 asks for:

```bash
swift run pt-replay session.csv --sweep sigma-a
```

```bash
swift run pt-replay session.csv --sweep q-model
```

The second one is worth running early. It compares the exact process-noise discretisation
against the table printed in spec §6.2 — see deviation D1 in the README — and on synthetic data
the published table reports an uncertainty roughly half the honest figure while producing a
similar answer. On real data, with real GNSS outliers, an over-confident filter behaves much
worse than that, because it starts rejecting the fixes that would have corrected it.
