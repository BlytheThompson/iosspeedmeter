import XCTest
@testable import PerformanceTimerCore

/// Generates a complete synthetic session — IMU, GNSS and barometer — from a known ground
/// truth, so the whole chain can be checked against an answer that is known exactly.
///
/// This is the offline equivalent of spec §12.3's video ground truth: it cannot validate the
/// sensors, but it validates every line of maths between the sensors and the timeslip.
struct SyntheticSession {
    /// Longitudinal acceleration profile, m/s², as a function of time since launch.
    var acceleration: (Double) -> Double = { _ in 5.0 }
    var stationaryDuration: Double = 3.0
    var runDuration: Double = 14.0
    var imuRate: Double = 100.0
    var gnssRate: Double = 1.0
    /// Mount orientation: rotation taking vehicle-frame vectors into the device frame.
    var mount = Quaternion(axis: Vector3(0.15, -0.3, 0.94).normalized()!, angle: 0.75)
    /// Constant accelerometer bias along vehicle +X, m/s² — what the `b` state exists to absorb.
    var accelerometerBias: Double = 0.12
    var gyroBias = Vector3(0.0008, -0.0011, 0.0006)
    var accelNoise: Double = 0.02
    var gyroNoise: Double = 0.0008
    var gnssSpeedNoise: Double = 0.04
    var gnssSpeedAccuracy: Double = 0.05
    var gradeFraction: Double = 0.0
    var seed: UInt64 = 0x9E3779B97F4A7C15

    /// Ground truth for one instant.
    struct Truth {
        var t: Double
        var speed: Double
        var distance: Double
        var acceleration: Double
    }

    /// True motion, integrated at IMU rate.
    func truth() -> [Truth] {
        let dt = 1.0 / imuRate
        var out: [Truth] = []
        var speed = 0.0
        var distance = 0.0
        var t = 0.0
        let total = stationaryDuration + runDuration
        while t <= total + 1e-9 {
            let sinceLaunch = t - stationaryDuration
            let a = sinceLaunch < 0 ? 0 : acceleration(sinceLaunch)
            out.append(Truth(t: t, speed: speed, distance: distance, acceleration: a))
            distance += speed * dt + 0.5 * a * dt * dt
            speed += a * dt
            t += dt
        }
        return out
    }

    /// Full event stream as the estimator would see it.
    func events() -> [SensorEvent] {
        var rng = TestRNG(state: seed)
        var events: [SensorEvent] = []
        let truths = truth()
        let gnssStride = max(1, Int((imuRate / gnssRate).rounded()))

        let gravityVehicle = Vector3(0, 0, -PTConstants.g)

        for (index, truth) in truths.enumerated() {
            // Specific force in the vehicle frame: true longitudinal acceleration plus the
            // accelerometer's own bias, plus gravity, plus noise.
            let measuredLongitudinal = truth.acceleration + accelerometerBias
                + rng.nextGaussian() * accelNoise
            let specificForceVehicle = Vector3(measuredLongitudinal, 0, 0) + gravityVehicle
            let specificForceDevice = mount.rotate(specificForceVehicle)

            // CoreMotion's own decomposition, including the §3.2 gravity tilt under sustained
            // acceleration: its gravity vector leans toward the acceleration.
            let tilt = atan2(truth.acceleration, PTConstants.g) * 0.35
            let leanedGravityVehicle = Vector3(-sin(tilt) * PTConstants.g, 0,
                                               -cos(tilt) * PTConstants.g)
            let gravityDevice = mount.rotate(leanedGravityVehicle)
            let userAccelerationDevice = specificForceDevice - gravityDevice

            let rotationRate = gyroBias + Vector3(
                rng.nextGaussian() * gyroNoise,
                rng.nextGaussian() * gyroNoise,
                rng.nextGaussian() * gyroNoise
            )

            events.append(.imu(IMUSample(
                t: truth.t,
                specificForce: specificForceDevice,
                rotationRate: rotationRate,
                gravity: gravityDevice,
                userAcceleration: userAccelerationDevice,
                attitude: mount.conjugate
            )))

            if index % gnssStride == 0 {
                events.append(.gnss(GNSSFix(
                    t: truth.t,
                    speed: max(0, truth.speed + rng.nextGaussian() * gnssSpeedNoise),
                    speedAccuracy: gnssSpeedAccuracy,
                    course: 0,
                    courseAccuracy: 1.0,
                    latitudeDegrees: 37.4275,
                    longitudeDegrees: -122.1697,
                    horizontalAccuracy: 2.0,
                    altitude: truth.distance * gradeFraction,
                    verticalAccuracy: 3.0,
                    fixType: 3,
                    numSV: 14,
                    gnssFixOK: true,
                    source: .replay
                )))
                events.append(.baro(BaroSample(
                    t: truth.t,
                    relativeAltitude: truth.distance * gradeFraction,
                    pressure: 101.3
                )))
            }
        }
        return events
    }

    /// Exact time from launch to a given speed, by interpolating the ground truth.
    func trueTime(toSpeed target: Double) -> Double? {
        let truths = truth()
        for i in 0..<(truths.count - 1) where truths[i].speed <= target && truths[i + 1].speed > target {
            let span = truths[i + 1].speed - truths[i].speed
            let blend = span > 0 ? (target - truths[i].speed) / span : 0
            let t = truths[i].t + blend * (truths[i + 1].t - truths[i].t)
            return t - stationaryDuration
        }
        return nil
    }

    /// Exact time from launch to a given distance.
    func trueTime(toDistance target: Double) -> Double? {
        let truths = truth()
        for i in 0..<(truths.count - 1) where truths[i].distance <= target && truths[i + 1].distance > target {
            let span = truths[i + 1].distance - truths[i].distance
            let blend = span > 0 ? (target - truths[i].distance) / span : 0
            let t = truths[i].t + blend * (truths[i + 1].t - truths[i].t)
            return t - stationaryDuration
        }
        return nil
    }
}

/// Spec §12.1 — bench test, and end-to-end validation of the whole pipeline.
final class SessionProcessorTests: XCTestCase {
    /// Spec §12.1: "Phone stationary on a desk for 60 s. Smoothed `v` must stay within
    /// ±0.02 m/s of zero and `s` within ±0.5 m. If it drifts, your bias handling or ZUPT is
    /// broken."
    func testBenchTestSixtySecondsStationary() {
        var rng = TestRNG(state: 0xDEADBEEFCAFEF00D)
        var events: [SensorEvent] = []
        let bias = Vector3(0.08, -0.05, 0.03)      // a realistic turn-on bias, m/s^2

        for i in 0...6000 {                         // 60 s at 100 Hz
            let t = Double(i) * 0.01
            let noise = Vector3(rng.nextGaussian() * 0.02,
                                rng.nextGaussian() * 0.02,
                                rng.nextGaussian() * 0.02)
            events.append(.imu(IMUSample(
                t: t,
                specificForce: Vector3(0, 0, -PTConstants.g) + bias + noise,
                rotationRate: Vector3(rng.nextGaussian() * 0.0005,
                                      rng.nextGaussian() * 0.0005,
                                      rng.nextGaussian() * 0.0005),
                gravity: Vector3(0, 0, -PTConstants.g),
                userAcceleration: bias + noise,
                attitude: .identity
            )))
            if i % 100 == 0 {
                events.append(.gnss(GNSSFix(
                    t: t, speed: max(0, rng.nextGaussian() * 0.03), speedAccuracy: 0.05,
                    fixType: 3, numSV: 12, gnssFixOK: true, source: .replay)))
            }
        }

        let processor = SessionProcessor()
        for event in events { processor.ingest(event) }
        let analysis = processor.analyse()

        let maxSpeed = analysis.smoothed.map { abs($0.speed) }.max() ?? .infinity
        let maxDistance = analysis.smoothed.map { abs($0.distance) }.max() ?? .infinity
        XCTAssertLessThanOrEqual(maxSpeed, 0.02,
                                 "spec §12.1: smoothed speed must stay within ±0.02 m/s")
        XCTAssertLessThanOrEqual(maxDistance, 0.5,
                                 "spec §12.1: smoothed distance must stay within ±0.5 m")
    }

    func testBenchTestWithoutZUPTWouldDriftAway() {
        // The control for the test above: with ZUPT suppressed and a real bias present, the
        // same data walks away. This is what proves the bench test is actually testing ZUPT.
        var rng = TestRNG(state: 0xDEADBEEFCAFEF00D)
        var filter = KalmanFilter()
        let bias = 0.08
        for _ in 0..<6000 {
            _ = filter.predict(acceleration: bias + rng.nextGaussian() * 0.02, dt: 0.01)
        }
        XCTAssertGreaterThan(abs(filter.distance), 5.0,
                             "unaided integration of an 0.08 m/s^2 bias over 60 s must drift")
    }

    /// The end-to-end test: a full synthetic run with a mounted phone, accelerometer bias,
    /// gyro bias, CoreMotion gravity tilt and noisy 1 Hz GNSS, checked against exact truth.
    func testRecoversKnownZeroToSixtyFromASyntheticRun() {
        let session = SyntheticSession()
        let processor = SessionProcessor()
        for event in session.events() { processor.ingest(event) }
        let analysis = processor.analyse()

        XCTAssertNotNil(analysis.anchor, "the launch must be detected and anchored")
        let results = analysis.results!
        let zeroSixty = results.speedResult(for: .zeroToSixty)!
        let expected = session.trueTime(toSpeed: 60 * PTConstants.mphToMetersPerSecond)!

        // Spec §13 budgets ±0.05–0.08 s for "internal GNSS + RTS smoother + ZUPT". The
        // pipeline comfortably beats that on synthetic data, where the only error left is the
        // 10 ms sample grid the anchor snaps to; the tolerance is set to hold that gain.
        XCTAssertEqual(zeroSixty.elapsed, expected, accuracy: 0.04,
                       "0–60 should beat the §13 software-only error budget")
    }

    func testRecoversKnownQuarterMileFromASyntheticRun() {
        let session = SyntheticSession()
        let processor = SessionProcessor()
        for event in session.events() { processor.ingest(event) }
        let analysis = processor.analyse()

        let quarter = analysis.results!.distanceResult(for: .quarterMile)!
        let expected = session.trueTime(toDistance: DistanceMark.quarterMile.meters)!
        XCTAssertEqual(quarter.elapsedFromRest, expected, accuracy: 0.05)
    }

    func testRetroactiveAnchorLandsNearTheTrueLaunchInstant() {
        // Spec §8: the anchor recovers 100–200 ms versus a threshold trigger.
        let session = SyntheticSession()
        let processor = SessionProcessor()
        for event in session.events() { processor.ingest(event) }
        let analysis = processor.analyse()
        XCTAssertEqual(analysis.anchor!.time, session.stationaryDuration, accuracy: 0.05)
    }

    func testSmoothedResultsBeatForwardOnlyResults() {
        // Spec Decision 1 and §13: the backward pass is the single biggest accuracy lever.
        // Compare both against exact truth over several noise seeds.
        var smoothedError = 0.0
        var forwardError = 0.0
        let target = 60 * PTConstants.mphToMetersPerSecond

        for seed in 0..<6 {
            var session = SyntheticSession()
            session.seed = UInt64(0x1234_5678 + seed * 7919)
            let processor = SessionProcessor()
            for event in session.events() { processor.ingest(event) }
            let analysis = processor.analyse()
            guard let anchor = analysis.anchor,
                  let expected = session.trueTime(toSpeed: target) else { continue }

            if let smoothedCrossing = CrossingSolver.speedCrossing(
                target: target, samples: analysis.smoothed) {
                smoothedError += abs((smoothedCrossing.time - anchor.time) - expected)
            }
            // The same extraction applied to the forward trace instead.
            let forwardTrace = analysis.steps.map { step in
                SmoothedSample(t: step.t, dt: step.dt, acceleration: step.acceleration,
                               state: step.xFiltered, covariance: step.PFiltered,
                               carriedForwardDueToSingularCovariance: false)
            }
            if let forwardCrossing = CrossingSolver.speedCrossing(
                target: target, samples: forwardTrace) {
                forwardError += abs((forwardCrossing.time - anchor.time) - expected)
            }
        }

        XCTAssertLessThan(smoothedError, forwardError,
                          "smoothed total error \(smoothedError) should beat forward \(forwardError)")
    }

    /// Spec §12.4: "If your run-to-run σ exceeds your claimed uncertainty, your uncertainty
    /// estimate is dishonest." Checked here across seeds against the reported sigma.
    func testReportedUncertaintyIsNotOptimisticRelativeToRunToRunSpread() {
        var elapsed: [Double] = []
        var reportedSigma: [Double] = []
        for seed in 0..<8 {
            var session = SyntheticSession()
            session.seed = UInt64(0xABCD_0000 + seed * 104_729)
            let processor = SessionProcessor()
            for event in session.events() { processor.ingest(event) }
            guard let result = processor.analyse().results?.speedResult(for: .zeroToSixty)
            else { continue }
            elapsed.append(result.elapsed)
            reportedSigma.append(result.sigma)
        }
        XCTAssertGreaterThanOrEqual(elapsed.count, 6)

        let mean = elapsed.reduce(0, +) / Double(elapsed.count)
        let spread = (elapsed.reduce(0) { $0 + ($1 - mean) * ($1 - mean) }
                      / Double(elapsed.count - 1)).squareRoot()
        // The run-to-run spread on identical synthetic input isolates noise sensitivity; it
        // must be small in absolute terms for the claimed sigma to mean anything.
        XCTAssertLessThan(spread, 0.03, "run-to-run spread was \(spread) s")
        XCTAssertTrue(reportedSigma.allSatisfy { $0 > 0 && $0 < 0.1 })
    }

    func testDiagnosticsReportAchievedRateAndRejections() {
        let session = SyntheticSession()
        let processor = SessionProcessor()
        for event in session.events() { processor.ingest(event) }
        let analysis = processor.analyse()
        XCTAssertEqual(analysis.diagnostics.achievedGNSSRate, 1.0, accuracy: 0.1)
        XCTAssertGreaterThan(analysis.diagnostics.acceptedFixCount, 10)
        XCTAssertEqual(analysis.diagnostics.meanSpeedAccuracy, 0.05, accuracy: 0.001)
    }

    func testConfidenceIsReportedAndSaysWhySoWhenDegraded() {
        var session = SyntheticSession()
        session.gnssSpeedAccuracy = 0.5          // poor fixes
        let processor = SessionProcessor()
        for event in session.events() { processor.ingest(event) }
        let analysis = processor.analyse()
        XCTAssertNotEqual(analysis.confidence.badge, .high)
        XCTAssertFalse(analysis.confidence.caveats.isEmpty)
    }

    func testUncertaintyIsReportedAlongsideEveryResult() {
        // Spec §9.4: "A 0–60 of 4.31 ± 0.04 s is a scientifically honest number; 4.31 s alone
        // is not."
        let session = SyntheticSession()
        let processor = SessionProcessor()
        for event in session.events() { processor.ingest(event) }
        let analysis = processor.analyse()
        for result in analysis.results!.speedResults {
            XCTAssertGreaterThan(result.sigma, 0)
            XCTAssertLessThan(result.sigma, 0.5)
        }
    }

    func testGradeIsEstimatedAndBothResultsAreOffered() {
        // Spec §7.2: report the raw result and a grade-corrected one; do not silently apply.
        var session = SyntheticSession()
        session.gradeFraction = 0.02
        let processor = SessionProcessor()
        for event in session.events() { processor.ingest(event) }
        let analysis = processor.analyse()

        XCTAssertNotNil(analysis.grade)
        XCTAssertEqual(analysis.grade!.meanGradePercent, 2.0, accuracy: 0.3)
        XCTAssertTrue(analysis.grade!.exceedsReportingThreshold)
        XCTAssertNotNil(analysis.gradeCorrectedResults)
        XCTAssertNotEqual(analysis.results!.distanceResult(for: .quarterMile)!.elapsedFromRest,
                          analysis.gradeCorrectedResults!.distanceResult(for: .quarterMile)!
                            .elapsedFromRest)
    }

    func testProcessorHandlesAnEmptySession() {
        let analysis = SessionProcessor().analyse()
        XCTAssertTrue(analysis.smoothed.isEmpty)
        XCTAssertNil(analysis.results)
        XCTAssertNil(analysis.anchor)
    }

    func testProcessorHandlesASessionThatNeverLaunches() {
        var events: [SensorEvent] = []
        for i in 0..<1000 {
            events.append(.imu(IMUSample(
                t: Double(i) * 0.01, userAccelerationG: .zero, gravityG: Vector3(0, 0, -1),
                rotationRate: .zero, attitude: .identity)))
        }
        let processor = SessionProcessor()
        for event in events { processor.ingest(event) }
        let analysis = processor.analyse()
        XCTAssertNil(analysis.anchor)
        XCTAssertNil(analysis.results)
        XCTAssertFalse(analysis.smoothed.isEmpty, "the trace is still produced for the log")
    }

    func testOutOfOrderGNSSFixIsStillApplied() {
        // Spec §2.3: delivery lags the fix by 100–500 ms and the lag varies. Because results
        // are post-processed, a late fix is inserted at its own timestamp.
        var session = SyntheticSession()
        session.runDuration = 8
        var events = session.events()
        // Delay every GNSS event's position in the stream by 40 samples without changing its
        // timestamp — exactly what a 400 ms delivery lag looks like.
        var imu: [SensorEvent] = []
        var gnss: [SensorEvent] = []
        for event in events {
            if case .gnss = event { gnss.append(event) } else { imu.append(event) }
        }
        events = imu
        for (index, fix) in gnss.enumerated() {
            let insertAt = min(events.count, (index + 1) * 100 + 40)
            events.insert(fix, at: insertAt)
        }

        let processor = SessionProcessor()
        for event in events { processor.ingest(event) }
        let analysis = processor.analyse()
        XCTAssertGreaterThan(analysis.diagnostics.acceptedFixCount, 5,
                             "late-delivered fixes must still be consumed")
    }
}

/// Spec Decision 1 — the live path is a readout, never a result.
final class LiveEstimatorTests: XCTestCase {
    private func run(_ estimator: LiveEstimator, session: SyntheticSession) {
        for event in session.events().sorted(by: { $0.t < $1.t }) {
            estimator.ingest(event)
        }
    }

    func testProgressesIdleToArmedToRecordingToComplete() {
        var session = SyntheticSession()
        session.runDuration = 16
        let estimator = LiveEstimator()
        run(estimator, session: session)
        // The synthetic run passes the quarter mile, which is the default stop distance.
        XCTAssertEqual(estimator.state, .complete)
        XCTAssertEqual(estimator.readout.stopReason, .targetDistanceReached)
    }

    func testLiveReadoutTracksSpeedWhileRecording() {
        var session = SyntheticSession()
        session.runDuration = 6
        let estimator = LiveEstimator()
        run(estimator, session: session)
        // Forward-only, so it lags, but it must be in the right region.
        XCTAssertGreaterThan(estimator.readout.speed, 20)
        XCTAssertGreaterThan(estimator.readout.speedMph, 45)
    }

    /// Spec §4: "Recording actually begins at ARMED, not at RECORDING. You need the pre-launch
    /// data for ZUPT and for retroactive launch anchoring."
    func testCapturedSessionIncludesPreLaunchSamplesFromTheRingBuffer() {
        var session = SyntheticSession()
        session.runDuration = 8
        let estimator = LiveEstimator()
        run(estimator, session: session)

        let firstCaptured = estimator.capturedEvents.map(\.t).min() ?? .infinity
        // The launch is at t = 3; the buffer must have carried samples from before it.
        XCTAssertLessThan(firstCaptured, 2.0,
                          "pre-launch window was not prepended on the ARMED→RECORDING transition")
    }

    func testTheCapturedSessionPostProcessesToAResult() {
        // The live path hands off; the processor produces the number. This is the seam that
        // Decision 1 requires.
        var session = SyntheticSession()
        session.runDuration = 14
        let estimator = LiveEstimator()
        run(estimator, session: session)

        let analysis = estimator.makeProcessor().analyse()
        XCTAssertNotNil(analysis.anchor)
        let zeroSixty = analysis.results!.speedResult(for: .zeroToSixty)!
        let expected = session.trueTime(toSpeed: 60 * PTConstants.mphToMetersPerSecond)!
        XCTAssertEqual(zeroSixty.elapsed, expected, accuracy: 0.05)
    }

    func testUserStopIsHonouredMidRun() {
        var session = SyntheticSession()
        session.runDuration = 14
        let estimator = LiveEstimator()
        var stopped = false
        for event in session.events().sorted(by: { $0.t < $1.t }) {
            estimator.ingest(event)
            if !stopped, estimator.state == .recording, event.t > 6.0 {
                estimator.requestStop()
                stopped = true
            }
        }
        XCTAssertEqual(estimator.state, .complete)
        XCTAssertEqual(estimator.readout.stopReason, .userStopped)
    }

    func testResetReturnsToIdleAndClearsCapture() {
        var session = SyntheticSession()
        session.runDuration = 6
        let estimator = LiveEstimator()
        run(estimator, session: session)
        estimator.reset()
        XCTAssertEqual(estimator.state, .idle)
        XCTAssertTrue(estimator.capturedEvents.isEmpty)
        XCTAssertEqual(estimator.readout.speed, 0)
    }
}

extension LiveEstimatorTests {
    func testPeakSpeedRecordsTheHighestSpeedSeenAndNeverFalls() {
        var session = SyntheticSession()
        session.runDuration = 10
        let estimator = LiveEstimator()
        var previousPeak = 0.0
        for event in session.events().sorted(by: { $0.t < $1.t }) {
            estimator.ingest(event)
            let peak = estimator.readout.peakSpeed
            XCTAssertGreaterThanOrEqual(peak, previousPeak, "peak speed must be monotonic")
            XCTAssertGreaterThanOrEqual(peak, estimator.readout.speed - 1e-9)
            previousPeak = peak
        }
        XCTAssertGreaterThan(estimator.readout.peakSpeed, 20)
    }

    func testResetClearsPeakSpeed() {
        var session = SyntheticSession()
        session.runDuration = 6
        let estimator = LiveEstimator()
        for event in session.events().sorted(by: { $0.t < $1.t }) { estimator.ingest(event) }
        XCTAssertGreaterThan(estimator.readout.peakSpeed, 0)
        estimator.reset()
        XCTAssertEqual(estimator.readout.peakSpeed, 0)
    }
}
