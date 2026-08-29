import XCTest
@testable import PerformanceTimerCore

/// Deterministic noise so every numeric assertion is reproducible.
struct TestRNG {
    var state: UInt64 = 0x2545F4914F6CDD1D
    mutating func nextUnit() -> Double {
        state ^= state << 13
        state ^= state >> 7
        state ^= state << 17
        return Double(state >> 11) / Double(1 << 53)
    }
    /// Approximately standard normal, via the sum of 12 uniforms.
    mutating func nextGaussian() -> Double {
        var sum = 0.0
        for _ in 0..<12 { sum += nextUnit() }
        return sum - 6.0
    }
}

// MARK: - §4 stationary detector

final class StationaryDetectorTests: XCTestCase {
    private func restSample(t: Double, noise: Double = 0, rotation: Double = 0) -> IMUSample {
        IMUSample(
            t: t,
            specificForce: Vector3(noise, 0, -PTConstants.g),
            rotationRate: Vector3(rotation, 0, 0),
            gravity: Vector3(0, 0, -PTConstants.g),
            userAcceleration: Vector3(noise, 0, 0),
            attitude: .identity
        )
    }

    private func fill(_ detector: inout StationaryDetector, count: Int, noise: Double = 0,
                      rotation: Double = 0, dt: Double = 0.01) {
        for i in 0..<count {
            detector.add(restSample(t: Double(i) * dt, noise: noise, rotation: rotation))
        }
    }

    func testAtRestWithQuietSensorsIsStationary() {
        var d = StationaryDetector()
        fill(&d, count: 60)
        XCTAssertTrue(d.isStationary)
    }

    func testFewerThanFortySamplesIsNotStationary() {
        // Spec §4: "window contains >= 40 IMU samples".
        var d = StationaryDetector()
        fill(&d, count: 39)
        XCTAssertFalse(d.isStationary)
        XCTAssertEqual(d.unmetConditions, [.insufficientSamples])
    }

    func testAccelerationStandardDeviationAboveThresholdIsNotStationary() {
        // Threshold is 0.03 g on the standard deviation of |f_D|.
        var d = StationaryDetector()
        var rng = TestRNG()
        for i in 0..<60 {
            let jolt = rng.nextGaussian() * 0.20 * PTConstants.g
            d.add(restSample(t: Double(i) * 0.01, noise: jolt))
        }
        XCTAssertFalse(d.isStationary)
        XCTAssertTrue(d.unmetConditions.contains(.accelerationVariance))
    }

    func testSmallAccelerationNoiseStaysStationary() {
        var d = StationaryDetector()
        var rng = TestRNG()
        for i in 0..<60 {
            d.add(restSample(t: Double(i) * 0.01, noise: rng.nextGaussian() * 0.002 * PTConstants.g))
        }
        XCTAssertTrue(d.isStationary)
    }

    func testRotationRateAboveThresholdIsNotStationary() {
        var d = StationaryDetector()
        fill(&d, count: 60, rotation: 0.05)     // 0.05 > 0.02 rad/s
        XCTAssertFalse(d.isStationary)
        XCTAssertTrue(d.unmetConditions.contains(.rotationRate))
    }

    func testGNSSSpeedAboveThresholdIsNotStationary() {
        var d = StationaryDetector()
        fill(&d, count: 60)
        d.noteGNSS(GNSSFix(t: 0.3, speed: 0.9, speedAccuracy: 0.2))
        XCTAssertFalse(d.isStationary)
        XCTAssertTrue(d.unmetConditions.contains(.gnssSpeed))
    }

    func testGNSSSpeedBelowThresholdKeepsStationary() {
        var d = StationaryDetector()
        fill(&d, count: 60)
        d.noteGNSS(GNSSFix(t: 0.3, speed: 0.1, speedAccuracy: 0.2))
        XCTAssertTrue(d.isStationary)
    }

    func testInvalidGNSSFixIsIgnored() {
        // Spec §4 qualifies the GNSS condition with "when a valid fix exists".
        var d = StationaryDetector()
        fill(&d, count: 60)
        d.noteGNSS(GNSSFix(t: 0.3, speed: -1, speedAccuracy: -1))
        XCTAssertTrue(d.isStationary)
    }

    func testStaleGNSSFixIsIgnored() {
        var d = StationaryDetector()
        d.noteGNSS(GNSSFix(t: 0.0, speed: 5.0, speedAccuracy: 0.2))
        // Advance well past the fix's validity horizon with quiet IMU samples.
        for i in 0..<600 { d.add(restSample(t: 5.0 + Double(i) * 0.01)) }
        XCTAssertTrue(d.isStationary, "a fix many seconds old must not veto forever")
    }

    /// Deviation D4. The published four conditions all hold in the middle of a launch: under
    /// steady acceleration the force magnitude is constant so its variance collapses, the gyro
    /// is quiet on a straight run, and at 1 Hz the newest fix still reports the pre-launch
    /// speed. Left unfixed, ZUPT resumes mid-run and corrupts the bias state.
    func testDoesNotReDeclareStationaryDuringSteadyAcceleration() {
        var d = StationaryDetector()
        var rng = TestRNG()
        let g = PTConstants.g

        // A second at rest, with a fix confirming it.
        d.noteGNSS(GNSSFix(t: 0.0, speed: 0.0, speedAccuracy: 0.05))
        for i in 0..<100 {
            d.add(IMUSample(t: Double(i) * 0.01,
                            specificForce: Vector3(rng.nextGaussian() * 0.01, 0, -g),
                            rotationRate: .zero, gravity: Vector3(0, 0, -g),
                            userAcceleration: .zero, attitude: .identity))
        }
        XCTAssertTrue(d.isStationary)

        // Now a steady 5 m/s^2 launch, straight ahead, with no new fix yet — exactly the
        // window in which the published detector wrongly re-arms.
        for i in 100..<250 {
            d.add(IMUSample(t: Double(i) * 0.01,
                            specificForce: Vector3(5.0 + rng.nextGaussian() * 0.01, 0, -g),
                            rotationRate: .zero, gravity: Vector3(0, 0, -g),
                            userAcceleration: Vector3(5.0, 0, 0), attitude: .identity))
        }
        XCTAssertFalse(d.isStationary,
                       "must not report stationary under sustained acceleration")
        XCTAssertTrue(d.unmetConditions.contains(.forceMagnitude)
                        || d.unmetConditions.contains(.awaitingGNSSConfirmation))
    }

    func testAfterMotionAFreshFixIsRequiredBeforeDeclaringStationaryAgain() {
        // The IMU alone cannot distinguish rest from constant velocity, so re-entry needs
        // GNSS evidence newer than the last moving sample.
        var d = StationaryDetector()
        let g = PTConstants.g
        d.noteGNSS(GNSSFix(t: 0.0, speed: 20.0, speedAccuracy: 0.05))   // moving
        for i in 0..<200 {
            d.add(IMUSample(t: Double(i) * 0.01,
                            specificForce: Vector3(0, 0, -g), rotationRate: .zero,
                            gravity: Vector3(0, 0, -g), userAcceleration: .zero,
                            attitude: .identity))
        }
        XCTAssertFalse(d.isStationary, "cruising at constant speed is not stationary")
        XCTAssertTrue(d.unmetConditions.contains(.awaitingGNSSConfirmation))

        // A fresh fix confirming the stop clears it.
        d.noteGNSS(GNSSFix(t: 2.0, speed: 0.05, speedAccuracy: 0.05))
        for i in 200..<300 {
            d.add(IMUSample(t: Double(i) * 0.01,
                            specificForce: Vector3(0, 0, -g), rotationRate: .zero,
                            gravity: Vector3(0, 0, -g), userAcceleration: .zero,
                            attitude: .identity))
        }
        XCTAssertTrue(d.isStationary)
    }

    func testGNSSFreeConfigurationStillArms() {
        // With no GNSS at all the confirmation latch must not permanently block arming.
        var d = StationaryDetector()
        let g = PTConstants.g
        var rng = TestRNG()
        for i in 0..<60 {
            d.add(IMUSample(t: Double(i) * 0.01,
                            specificForce: Vector3(rng.nextGaussian() * 0.5, 0, -g),
                            rotationRate: Vector3(0.5, 0, 0), gravity: Vector3(0, 0, -g),
                            userAcceleration: .zero, attitude: .identity))
        }
        XCTAssertFalse(d.isStationary)
        for i in 60..<200 {
            d.add(IMUSample(t: Double(i) * 0.01, specificForce: Vector3(0, 0, -g),
                            rotationRate: .zero, gravity: Vector3(0, 0, -g),
                            userAcceleration: .zero, attitude: .identity))
        }
        XCTAssertTrue(d.isStationary)
    }

    func testWindowSlidesSoMotionEventuallyClearsFromHistory() {
        var d = StationaryDetector()
        // Half a second of violent motion...
        var rng = TestRNG()
        for i in 0..<60 {
            d.add(restSample(t: Double(i) * 0.01, noise: rng.nextGaussian() * 0.5 * PTConstants.g))
        }
        XCTAssertFalse(d.isStationary)
        // ...then a full window of quiet.
        for i in 0..<80 { d.add(restSample(t: 0.6 + Double(i) * 0.01)) }
        XCTAssertTrue(d.isStationary)
    }
}

// MARK: - §6.2 process noise (deviation D1)

final class ProcessNoiseTests: XCTestCase {
    let sigmaA = 0.05
    let sigmaB = 0.002

    func testExactModelMatchesAnalyticDiscretisation() {
        let dt = 0.01
        let q = ProcessNoise.matrix(dt: dt, sigmaA: sigmaA, sigmaB: sigmaB,
                                    model: .exact, dimension: 3)
        let a2 = sigmaA * sigmaA
        let b2 = sigmaB * sigmaB
        XCTAssertEqual(q[0, 0], a2 * pow(dt, 3) / 3 + b2 * pow(dt, 5) / 20, accuracy: 1e-24)
        XCTAssertEqual(q[0, 1], a2 * pow(dt, 2) / 2 + b2 * pow(dt, 4) / 8, accuracy: 1e-24)
        XCTAssertEqual(q[0, 2], -b2 * pow(dt, 3) / 6, accuracy: 1e-24)
        XCTAssertEqual(q[1, 1], a2 * dt + b2 * pow(dt, 3) / 3, accuracy: 1e-24)
        XCTAssertEqual(q[1, 2], -b2 * pow(dt, 2) / 2, accuracy: 1e-24)
        XCTAssertEqual(q[2, 2], b2 * dt, accuracy: 1e-24)
        XCTAssertEqual(q[1, 0], q[0, 1], accuracy: 0)
        XCTAssertEqual(q[2, 0], q[0, 2], accuracy: 0)
        XCTAssertEqual(q[2, 1], q[1, 2], accuracy: 0)
    }

    func testSpecLiteralModelReproducesThePublishedTable() {
        let dt = 0.01
        let q = ProcessNoise.matrix(dt: dt, sigmaA: sigmaA, sigmaB: sigmaB,
                                    model: .specLiteral, dimension: 3)
        let a2 = sigmaA * sigmaA
        let b2 = sigmaB * sigmaB
        XCTAssertEqual(q[0, 0], a2 * pow(dt, 5) / 20 + b2 * pow(dt, 5) / 20, accuracy: 1e-26)
        XCTAssertEqual(q[0, 1], a2 * pow(dt, 4) / 8, accuracy: 1e-26)
        XCTAssertEqual(q[0, 2], -b2 * pow(dt, 3) / 6, accuracy: 1e-26)
        XCTAssertEqual(q[1, 1], a2 * pow(dt, 3) / 3, accuracy: 1e-26)
        XCTAssertEqual(q[1, 2], -b2 * pow(dt, 2) / 2, accuracy: 1e-26)
        XCTAssertEqual(q[2, 2], b2 * dt, accuracy: 1e-26)
    }

    /// Deviation D1. `σ_a` is specified in m/s²/√Hz, so `σ_a²` has units m²/s³ and the
    /// velocity-variance entry must come out in m²/s². The published `σ_a²·dt³/3` yields m²,
    /// which is a *position* variance in the velocity slot — one integration too high.
    func testVelocityVarianceHasVelocityUnitsOnlyInTheExactModel() {
        // Scaling test: the exact velocity entry must be linear in dt.
        let base = ProcessNoise.matrix(dt: 0.01, sigmaA: sigmaA, sigmaB: 0,
                                       model: .exact, dimension: 3)[1, 1]
        let doubled = ProcessNoise.matrix(dt: 0.02, sigmaA: sigmaA, sigmaB: 0,
                                          model: .exact, dimension: 3)[1, 1]
        XCTAssertEqual(doubled / base, 2.0, accuracy: 1e-9)

        // The published table scales as dt^3 instead, which is the signature of the error.
        let literalBase = ProcessNoise.matrix(dt: 0.01, sigmaA: sigmaA, sigmaB: 0,
                                              model: .specLiteral, dimension: 3)[1, 1]
        let literalDoubled = ProcessNoise.matrix(dt: 0.02, sigmaA: sigmaA, sigmaB: 0,
                                                 model: .specLiteral, dimension: 3)[1, 1]
        XCTAssertEqual(literalDoubled / literalBase, 8.0, accuracy: 1e-9)
    }

    func testPublishedTableIsDramaticallyOverconfidentAtOneHundredHertz() {
        // At dt = 0.01 the ratio is 3/dt^2 = 30000. A filter using the published table
        // believes its dead-reckoned velocity far too much and effectively ignores GNSS.
        let dt = 0.01
        let exact = ProcessNoise.matrix(dt: dt, sigmaA: sigmaA, sigmaB: 0,
                                        model: .exact, dimension: 3)[1, 1]
        let literal = ProcessNoise.matrix(dt: dt, sigmaA: sigmaA, sigmaB: 0,
                                          model: .specLiteral, dimension: 3)[1, 1]
        XCTAssertEqual(exact / literal, 3.0 / (dt * dt), accuracy: 1.0)
        XCTAssertGreaterThan(exact / literal, 10_000)
    }

    func testExactModelIsPositiveSemiDefinite() {
        let q = ProcessNoise.matrix(dt: 0.01, sigmaA: sigmaA, sigmaB: sigmaB,
                                    model: .exact, dimension: 3)
        let (values, _) = q.symmetricEigenDecomposition()!
        for v in values {
            XCTAssertGreaterThan(v, -1e-20, "Q must be positive semi-definite; got \(values)")
        }
    }

    func testFourthStateAddsWheelScaleRandomWalkOnly() {
        let q = ProcessNoise.matrix(dt: 0.01, sigmaA: sigmaA, sigmaB: sigmaB,
                                    model: .exact, dimension: 4,
                                    sigmaWheelScale: 1e-4)
        XCTAssertEqual(q.rows, 4)
        XCTAssertEqual(q[3, 3], 1e-4 * 1e-4 * 0.01, accuracy: 1e-30)
        XCTAssertEqual(q[0, 3], 0, accuracy: 0)
        XCTAssertEqual(q[3, 1], 0, accuracy: 0)
    }

    func testDefaultModelIsExact() {
        XCTAssertEqual(FilterConfiguration().processNoiseModel, .exact)
    }
}

// MARK: - §6.1–6.3 forward filter

final class KalmanFilterTests: XCTestCase {
    private func makeFilter(_ configure: (inout FilterConfiguration) -> Void = { _ in })
        -> KalmanFilter
    {
        var config = FilterConfiguration()
        configure(&config)
        return KalmanFilter(configuration: config)
    }

    func testInitialStateIsZeroWithFiniteCovariance() {
        let f = makeFilter()
        XCTAssertEqual(f.distance, 0, accuracy: 0)
        XCTAssertEqual(f.speed, 0, accuracy: 0)
        XCTAssertEqual(f.accelerometerBias, 0, accuracy: 0)
        XCTAssertGreaterThan(f.covariance[1, 1], 0)
    }

    func testPredictionWithConstantAccelerationMatchesClosedFormKinematics() {
        var f = makeFilter()
        let a = 4.0, dt = 0.01
        let steps = 500                       // 5 s
        for _ in 0..<steps { _ = f.predict(acceleration: a, dt: dt) }
        let t = Double(steps) * dt
        XCTAssertEqual(f.speed, a * t, accuracy: 1e-9)
        XCTAssertEqual(f.distance, 0.5 * a * t * t, accuracy: 1e-9)
    }

    func testPredictionSubtractsTheBiasState() {
        var f = makeFilter()
        f.setState(distance: 0, speed: 0, accelerometerBias: 0.5)
        let a = 2.0, dt = 0.1
        _ = f.predict(acceleration: a, dt: dt)
        // v = a*dt - b*dt, s = a*dt^2/2 - b*dt^2/2
        XCTAssertEqual(f.speed, (a - 0.5) * dt, accuracy: 1e-12)
        XCTAssertEqual(f.distance, (a - 0.5) * dt * dt / 2, accuracy: 1e-12)
    }

    func testPredictionGrowsVelocityUncertainty() {
        var f = makeFilter()
        let before = f.covariance[1, 1]
        _ = f.predict(acceleration: 0, dt: 0.01)
        XCTAssertGreaterThan(f.covariance[1, 1], before)
    }

    func testGNSSSpeedUpdatePullsStateTowardTheMeasurement() {
        var f = makeFilter()
        for _ in 0..<200 { _ = f.predict(acceleration: 0, dt: 0.01) }
        let before = f.speed
        // Innovation must sit inside the 3-sigma gate for this to be an update test rather
        // than a gate test; a wild innovation is covered separately below.
        let outcome = f.updateGNSSSpeed(GNSSFix(t: 2.0, speed: 0.3, speedAccuracy: 0.1))
        XCTAssertEqual(outcome, .applied)
        XCTAssertGreaterThan(f.speed, before)
        XCTAssertLessThanOrEqual(f.speed, 0.3)
    }

    /// A filter that has been tracking properly should follow a genuine acceleration through
    /// a sequence of consistent fixes, without the gate fighting it.
    func testFilterTracksASequenceOfConsistentFixes() {
        var f = makeFilter()
        let trueAcceleration = 4.0
        var t = 0.0
        for step in 0..<500 {
            _ = f.predict(acceleration: trueAcceleration, dt: 0.01)
            t += 0.01
            if step % 25 == 24 {
                _ = f.updateGNSSSpeed(GNSSFix(t: t, speed: trueAcceleration * t,
                                              speedAccuracy: 0.05))
            }
        }
        XCTAssertEqual(f.speed, trueAcceleration * t, accuracy: 0.05)
        XCTAssertEqual(f.gateRejectionCount, 0)
    }

    func testGNSSUpdateReducesVelocityUncertainty() {
        var f = makeFilter()
        for _ in 0..<200 { _ = f.predict(acceleration: 0, dt: 0.01) }
        let before = f.covariance[1, 1]
        _ = f.updateGNSSSpeed(GNSSFix(t: 2.0, speed: 1.0, speedAccuracy: 0.1))
        XCTAssertLessThan(f.covariance[1, 1], before)
    }

    func testSpeedAccuracyIsFlooredAtTheConfiguredSigma() {
        // Spec §6.2: "never trust speedAccuracy below this".
        let f = makeFilter()
        let optimistic = GNSSFix(t: 0, speed: 0, speedAccuracy: 0.0001)
        XCTAssertEqual(f.measurementVariance(for: optimistic), 0.05 * 0.05, accuracy: 1e-15)
    }

    func testInvalidFixesAreRejectedWithAReason() {
        var f = makeFilter()
        XCTAssertEqual(f.updateGNSSSpeed(GNSSFix(t: 0, speed: -1, speedAccuracy: 0.1)),
                       .rejectedInvalid(.negativeSpeed))
        XCTAssertEqual(f.updateGNSSSpeed(GNSSFix(t: 0, speed: 5, speedAccuracy: -1)),
                       .rejectedInvalid(.negativeSpeedAccuracy))
        XCTAssertEqual(f.updateGNSSSpeed(GNSSFix(t: 0, speed: 5, speedAccuracy: 2.0)),
                       .rejectedInvalid(.speedAccuracyTooLarge))
        XCTAssertEqual(f.updateGNSSSpeed(GNSSFix(t: 0, speed: 5, speedAccuracy: 0.1, fixType: 2)),
                       .rejectedInvalid(.fixTypeTooLow))
        XCTAssertEqual(
            f.updateGNSSSpeed(GNSSFix(t: 0, speed: 5, speedAccuracy: 0.1, gnssFixOK: false)),
            .rejectedInvalid(.fixNotOK)
        )
    }

    func testInnovationGateRejectsAnOutlierAndLeavesStateUntouched() {
        // Spec §6.3: reject if y^2 / S > 9.
        var f = makeFilter()
        f.setState(distance: 0, speed: 0, accelerometerBias: 0)
        f.setCovariance(Matrix.diagonal([1e-6, 1e-6, 1e-6]))
        let speedBefore = f.speed
        let outcome = f.updateGNSSSpeed(GNSSFix(t: 1.0, speed: 30.0, speedAccuracy: 0.05))
        guard case .rejectedByGate = outcome else {
            return XCTFail("a 30 m/s innovation against 1e-3 sigma must be gated, got \(outcome)")
        }
        XCTAssertEqual(f.speed, speedBefore, accuracy: 0)
        XCTAssertEqual(f.gateRejectionCount, 1)
    }

    func testInnovationJustInsideTheGateIsAccepted() {
        var f = makeFilter()
        f.setState(distance: 0, speed: 0, accelerometerBias: 0)
        f.setCovariance(Matrix.diagonal([1.0, 1.0, 1.0]))
        // S = P11 + R = 1 + 0.01 -> 3 sigma is about 3.015 m/s.
        let outcome = f.updateGNSSSpeed(GNSSFix(t: 1.0, speed: 2.9, speedAccuracy: 0.1))
        XCTAssertEqual(outcome, .applied)
        XCTAssertEqual(f.gateRejectionCount, 0)
    }

    /// Spec §6.3: ZUPT "is what pins `b` to near-truth immediately before launch, and it is
    /// worth more than any hardware upgrade below the 25 Hz tier."
    func testZUPTDrivesTheBiasStateTowardAnUnmodelledOffset() {
        var f = makeFilter()
        let falseBias = 0.3           // m/s^2 the accelerometer reports while truly at rest
        for _ in 0..<1000 {           // 10 s at 100 Hz
            _ = f.predict(acceleration: falseBias, dt: 0.01)
            _ = f.updateZeroVelocity()
        }
        XCTAssertEqual(f.accelerometerBias, falseBias, accuracy: 0.02)
        XCTAssertEqual(f.speed, 0, accuracy: 0.005)
    }

    func testZUPTKeepsDistanceFromWalkingAway() {
        var f = makeFilter()
        for _ in 0..<6000 {           // 60 s at 100 Hz
            _ = f.predict(acceleration: 0.15, dt: 0.01)
            _ = f.updateZeroVelocity()
        }
        XCTAssertEqual(f.speed, 0, accuracy: 0.005)
    }

    func testCovarianceStaysSymmetricAcrossManyUpdates() {
        var f = makeFilter()
        var rng = TestRNG()
        for i in 0..<3000 {
            _ = f.predict(acceleration: rng.nextGaussian() * 0.5, dt: 0.01)
            if i % 100 == 0 {
                _ = f.updateGNSSSpeed(GNSSFix(t: Double(i) * 0.01, speed: max(0, f.speed),
                                              speedAccuracy: 0.1))
            }
        }
        let p = f.covariance
        for i in 0..<3 {
            for j in 0..<3 {
                XCTAssertEqual(p[i, j], p[j, i], accuracy: 1e-18)
            }
        }
    }

    func testCovarianceStaysPositiveSemiDefinite() {
        var f = makeFilter()
        var rng = TestRNG()
        for i in 0..<2000 {
            _ = f.predict(acceleration: rng.nextGaussian(), dt: 0.01)
            if i % 25 == 0 {
                _ = f.updateGNSSSpeed(GNSSFix(t: Double(i) * 0.01, speed: max(0, f.speed),
                                              speedAccuracy: 0.08))
            }
        }
        let (values, _) = f.covariance.symmetricEigenDecomposition()!
        for v in values { XCTAssertGreaterThan(v, -1e-15) }
    }

    // MARK: §3.7 wheel speed

    func testWheelSpeedStateStartsAtUnityScale() {
        let f = makeFilter { $0.wheelSpeedEnabled = true }
        XCTAssertEqual(f.stateDimension, 4)
        XCTAssertEqual(f.wheelScale, 1.0, accuracy: 0)
    }

    func testWheelSpeedUpdateUsesTheProductMeasurementModel() {
        // Spec §3.7: z = k * v, H = [0, k, 0, v].
        var f = makeFilter { $0.wheelSpeedEnabled = true }
        f.setState(distance: 0, speed: 20.0, accelerometerBias: 0, wheelScale: 1.0)
        f.setCovariance(Matrix.diagonal([1.0, 1.0, 0.01, 0.0004]))
        let outcome = f.updateWheelSpeed(WheelSpeedSample(t: 1.0, speed: 21.0),
                                         measurementSigma: 0.1)
        XCTAssertEqual(outcome, .applied)
        // The estimate must move toward reconciling k*v with 21.
        XCTAssertGreaterThan(f.wheelScale * f.speed, 20.0)
    }

    func testWheelSpeedIsGatedOffDuringHardAcceleration() {
        // Spec §3.7: suppress while |a| > 0.35 g — wheelspin reads high exactly when it matters.
        var f = makeFilter { $0.wheelSpeedEnabled = true }
        f.setState(distance: 0, speed: 5.0, accelerometerBias: 0, wheelScale: 1.0)
        let outcome = f.updateWheelSpeed(WheelSpeedSample(t: 1.0, speed: 7.0),
                                         measurementSigma: 0.1,
                                         currentAcceleration: 0.5 * PTConstants.g)
        XCTAssertEqual(outcome, .rejectedByWheelSpeedGate(.launchAcceleration))
    }

    func testWheelSpeedIsGatedOffWhenItDisagreesWithFusedSpeed() {
        // Spec §3.7: suppress while wheel speed exceeds fused speed by more than 1.5 m/s.
        var f = makeFilter { $0.wheelSpeedEnabled = true }
        f.setState(distance: 0, speed: 20.0, accelerometerBias: 0, wheelScale: 1.0)
        let outcome = f.updateWheelSpeed(WheelSpeedSample(t: 1.0, speed: 22.0),
                                         measurementSigma: 0.1,
                                         currentAcceleration: 0.0)
        XCTAssertEqual(outcome, .rejectedByWheelSpeedGate(.disagreement))
    }

    func testWheelSpeedWithinToleranceIsAccepted() {
        var f = makeFilter { $0.wheelSpeedEnabled = true }
        f.setState(distance: 0, speed: 20.0, accelerometerBias: 0, wheelScale: 1.0)
        let outcome = f.updateWheelSpeed(WheelSpeedSample(t: 1.0, speed: 20.8),
                                         measurementSigma: 0.1,
                                         currentAcceleration: 1.0)
        XCTAssertEqual(outcome, .applied)
    }

    func testWheelSpeedUpdateIsIgnoredWhenTheStateIsDisabled() {
        var f = makeFilter { $0.wheelSpeedEnabled = false }
        XCTAssertEqual(f.updateWheelSpeed(WheelSpeedSample(t: 1.0, speed: 5.0),
                                          measurementSigma: 0.1),
                       .notConfigured)
    }
}

// MARK: - §6.4 RTS smoother

final class RTSSmootherTests: XCTestCase {
    /// Build a short forward run so the smoother has something to work on.
    private func forwardRun(
        sampleCount: Int = 400,
        acceleration: @escaping (Int) -> Double = { _ in 0 },
        gnssAt: Set<Int> = [],
        gnssSpeed: @escaping (Int) -> Double = { _ in 0 },
        configure: (inout FilterConfiguration) -> Void = { _ in }
    ) -> [FilterStep] {
        var config = FilterConfiguration()
        configure(&config)
        var filter = KalmanFilter(configuration: config)
        var steps: [FilterStep] = []
        let dt = 0.01
        for i in 0..<sampleCount {
            let t = Double(i) * dt
            var step = filter.predict(acceleration: acceleration(i), dt: dt)
            step.t = t
            if gnssAt.contains(i) {
                _ = filter.updateGNSSSpeed(GNSSFix(t: t, speed: gnssSpeed(i), speedAccuracy: 0.05))
            }
            filter.finish(step: &step)
            steps.append(step)
        }
        return steps
    }

    func testSmootherReturnsOneSamplePerForwardStep() {
        let steps = forwardRun()
        let smoothed = RTSSmoother.smooth(steps)
        XCTAssertEqual(smoothed.count, steps.count)
    }

    func testFinalSampleEqualsTheForwardEstimate() {
        // Boundary condition of the backward recursion: x_{N-1|N} == x_{N-1|N-1}.
        let steps = forwardRun(gnssAt: [100, 200, 300], gnssSpeed: { Double($0) * 0.01 })
        let smoothed = RTSSmoother.smooth(steps)
        let last = steps.count - 1
        XCTAssertEqual(smoothed[last].speed, steps[last].xFiltered[1], accuracy: 1e-15)
        XCTAssertEqual(smoothed[last].distance, steps[last].xFiltered[0], accuracy: 1e-15)
    }

    func testSmoothedCovarianceIsNeverWorseThanForward() {
        // The whole justification for post-processing (spec Decision 1).
        let steps = forwardRun(gnssAt: [50, 150, 250, 350], gnssSpeed: { _ in 5.0 })
        let smoothed = RTSSmoother.smooth(steps)
        for k in 0..<steps.count {
            XCTAssertLessThanOrEqual(
                smoothed[k].covariance[1, 1], steps[k].PFiltered[1, 1] + 1e-15,
                "smoothed velocity variance grew at index \(k)"
            )
        }
    }

    func testLaterMeasurementImprovesEarlierEstimates() {
        // A GNSS fix arriving at t = 3 s must sharpen the estimate at t = 1 s. The forward
        // filter cannot do this; it is precisely what the backward pass buys.
        let steps = forwardRun(sampleCount: 400, gnssAt: [399], gnssSpeed: { _ in 0.0 })
        let smoothed = RTSSmoother.smooth(steps)
        let mid = 100
        XCTAssertLessThan(smoothed[mid].covariance[1, 1], steps[mid].PFiltered[1, 1])
    }

    func testSmootherRecoversAConstantBiasFromAQuietRun() {
        // Truly stationary, but the accelerometer reports a steady 0.2 m/s^2 and GNSS keeps
        // saying zero. The smoothed bias should converge on 0.2 across the whole trace.
        let steps = forwardRun(
            sampleCount: 1000,
            acceleration: { _ in 0.2 },
            gnssAt: Set(stride(from: 0, to: 1000, by: 100)),
            gnssSpeed: { _ in 0.0 }
        )
        let smoothed = RTSSmoother.smooth(steps)
        XCTAssertEqual(smoothed[500].accelerometerBias, 0.2, accuracy: 0.05)
    }

    func testSingularPredictedCovarianceIsSkippedRatherThanInverted() {
        // Spec §6.4: "if |det| < 1e-12, skip the smoothing step for that index and carry
        // x_k|k forward."
        var steps = forwardRun(sampleCount: 50)
        steps[25].PPredicted = Matrix(rows: 3, columns: 3, repeating: 0)
        let smoothed = RTSSmoother.smooth(steps)
        XCTAssertEqual(smoothed.count, 50)
        XCTAssertEqual(smoothed[24].speed, steps[24].xFiltered[1], accuracy: 1e-15)
        XCTAssertTrue(smoothed[24].carriedForwardDueToSingularCovariance)
        XCTAssertTrue(smoothed.allSatisfy { $0.speed.isFinite && $0.distance.isFinite })
    }

    func testEmptyInputProducesEmptyOutput() {
        XCTAssertTrue(RTSSmoother.smooth([]).isEmpty)
    }

    func testSingleSampleInputIsReturnedUnchanged() {
        let steps = forwardRun(sampleCount: 1)
        let smoothed = RTSSmoother.smooth(steps)
        XCTAssertEqual(smoothed.count, 1)
        XCTAssertEqual(smoothed[0].speed, steps[0].xFiltered[1], accuracy: 1e-15)
    }
}
