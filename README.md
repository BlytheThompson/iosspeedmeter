# GNSS/IMU Vehicle Performance Timer

A Dragy-equivalent acceleration and distance timer for iPhone, implementing
[`ios-performance-timer-spec.md`](ios-performance-timer-spec.md): post-processed sensor fusion
with a forward Kalman filter and an RTS smoother over recorded 100 Hz IMU and 1–25 Hz GNSS.

## The two decisions the design turns on

**Everything is post-processed.** The live display is a convenience readout from the forward
filter. Every *reported* number comes from a backward smoothing pass over the whole recorded
session. The two paths are separate types — `LiveEstimator` and `SessionProcessor` — and
`LiveEstimator` has no method that can return a result, so the rule is structural rather than
a convention.

**Sensors sit behind one protocol.** Internal GNSS at 1 Hz, a 25 Hz external receiver over
Wi-Fi, and a CAN wheel-speed channel all feed the same estimator through `SensorSource`. The
practical payoff is that `PerformanceTimerCore` imports nothing from Apple, so the entire
numerical core builds and tests on Linux and Windows — the maths is verifiable without a
device, and CI enforces the purity on every push.

## Layout

```
Sources/PerformanceTimerCore/    Pure Swift. All the maths. No Apple frameworks.
Sources/PerformanceTimerReplay/  pt-replay — the offline replay and tuning harness.
App/                             iOS only: sensor adapters and SwiftUI.
Tests/                           246 tests over the core.
docs/                            Build and validation guides, plus the implementation plan.
```

## Running the tests

Anywhere with a Swift toolchain:

```bash
swift test
```

On Windows, Swift needs the MSVC developer environment; `tools/swift.cmd` sets it up:

```bash
powershell -File tools/test.ps1
```

## The replay harness

Spec §15 step 2: *"Everything after this is developed against recorded data."* `pt-replay`
re-runs the exact estimator that runs on the phone, over a logged CSV or a synthetic session.

```bash
swift run pt-replay --synthetic
```

```bash
swift run pt-replay Documents/session-1700000000-1b4e28ba.csv --sweep sigma-a
```

`--sweep` is the retuning loop from spec §6.2 — it re-runs the whole session across a range of
one parameter and prints the resulting 0–60 and its uncertainty, so `Q` and `R` can be tuned
without driving.

## Building the app without a Mac

See [docs/building.md](docs/building.md). In short: push, and the GitHub Actions workflow
produces an unsigned `.ipa` as a build artifact; install it with Sideloadly, AltStore or
SideStore.

## Build status

Both CI jobs pass on every push to `main`:

- **Core tests (Linux)** — 246 tests over `PerformanceTimerCore`, plus a gate that fails the
  build if an Apple-framework import ever reaches the core, and a replay-harness smoke test.
- **Unsigned .ipa (macOS)** — XcodeGen + `xcodebuild` for device. The iOS layer compiles clean,
  and the `.ipa` is downloadable from the run's artifacts.

## Before you trust a number

See [docs/validation.md](docs/validation.md). The synthetic end-to-end test agrees with exact
ground truth to about 15 ms, and CI compiles the iOS layer — but neither validates a sensor.
Both only cover the maths between the sensors and the timeslip. Spec §12.3's video ground truth is the only test that validates the whole chain, and
it has not been run — the app should not be trusted until it has.

## Deviations from the spec

Five places where the implementation does not follow the spec text literally, each with a test
pinning the behaviour. The reasoning is in
[docs/superpowers/plans/2026-08-29-gnss-imu-performance-timer.md](docs/superpowers/plans/2026-08-29-gnss-imu-performance-timer.md)
and in comments at each site.

| | What | Why |
|---|---|---|
| **D1** | §6.2's `Q` table is replaced with the exact discretisation | The published table is dimensionally inconsistent with its own stated units and is ~30,000× too small at 100 Hz, which makes the filter ignore GNSS |
| **D2** | §5's longitudinal projection removes gravity in the device frame | As written, `R_DV` is applied to a local-level vector; the frame chain does not compose |
| **D3** | Appendix B's tangent plane is replaced with an exact ECEF transform | Its "well under a centimetre over a mile" claim measures at 8.6 cm; the exact transform costs nothing at 1–25 Hz |
| **D4** | §4's stationary detector gains a force-magnitude test and a post-motion GNSS confirmation | As published it re-declares "stationary" *mid-launch*, firing ZUPT during the run and corrupting the bias state |
| **D5** | §8's "monotonically increasing" anchor test uses a least-squares slope | Applied sample-wise to 100 Hz data it never matches, placing the anchor ~80 ms early |

D1 and D4 are the consequential ones: with either left as written, the reported speed came out
32% low on a synthetic run whose answer is known exactly.
