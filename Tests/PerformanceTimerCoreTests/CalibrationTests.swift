import XCTest
@testable import PerformanceTimerCore

/// Spec §3.2 — launch-anchored attitude propagation, the mitigation for the gravity-drift trap.
final class AttitudePropagatorTests: XCTestCase {
    private let g = PTConstants.g

    /// A phone lying flat, screen up: CoreMotion reports gravity along device −Z.
    private func restSample(t: Double, rotationRate: Vector3 = .zero) -> IMUSample {
        IMUSample(
            t: t,
            userAccelerationG: .zero,
            gravityG: Vector3(0, 0, -1),
            rotationRate: rotationRate,
            attitude: .identity
        )
    }

    func testSpecificForceReconstructionRecoversTheRawReading() {
        // Spec §3.2 step 3: f_D = (userAcceleration + gravity) * g.
        let sample = IMUSample(
            t: 0,
            userAccelerationG: Vector3(0.4, 0, 0),
            gravityG: Vector3(0, 0, -1),
            rotationRate: .zero,
            attitude: .identity
        )
        XCTAssertEqual(sample.specificForce.x, 0.4 * g, accuracy: 1e-12)
        XCTAssertEqual(sample.specificForce.z, -g, accuracy: 1e-12)
    }

    func testStationaryCalibrationAveragesGyroBiasAndGravity() {
        var calibration = StationaryCalibration()
        let bias = Vector3(0.004, -0.002, 0.001)
        for i in 0..<200 {
            calibration.add(restSample(t: Double(i) * 0.01, rotationRate: bias))
        }
        let result = calibration.result()!
        XCTAssertEqual(result.gyroBias.x, bias.x, accuracy: 1e-12)
        XCTAssertEqual(result.gyroBias.y, bias.y, accuracy: 1e-12)
        XCTAssertEqual(result.gyroBias.z, bias.z, accuracy: 1e-12)
        XCTAssertEqual(result.referenceSpecificForce.z, -g, accuracy: 1e-9)
    }

    func testStationaryCalibrationNeedsSamples() {
        XCTAssertNil(StationaryCalibration().result())
    }

    func testAtRestTheKinematicAccelerationIsZero() {
        var calibration = StationaryCalibration()
        for i in 0..<100 { calibration.add(restSample(t: Double(i) * 0.01)) }
        var propagator = AttitudePropagator(calibration: calibration.result()!)
        for i in 100..<200 {
            let sample = restSample(t: Double(i) * 0.01)
            propagator.advance(to: sample)
            let a = propagator.kinematicAcceleration(of: sample)
            XCTAssertEqual(a.length, 0, accuracy: 1e-9)
        }
    }

    func testGyroBiasIsRemovedSoAttitudeDoesNotWander() {
        // A 0.1 deg/s bias would give ~1.5 deg of attitude error over a 15 s run if left in.
        let bias = Vector3(0, 0, 0.1 * .pi / 180)
        var calibration = StationaryCalibration()
        for i in 0..<200 { calibration.add(restSample(t: Double(i) * 0.01, rotationRate: bias)) }
        var propagator = AttitudePropagator(calibration: calibration.result()!)

        for i in 0..<1500 {                     // 15 s
            propagator.advance(to: restSample(t: 2.0 + Double(i) * 0.01, rotationRate: bias))
        }
        let drifted = propagator.attitude.rotate(Vector3(1, 0, 0))
        XCTAssertEqual(drifted.x, 1.0, accuracy: 1e-6, "bias-corrected attitude must not drift")
    }

    func testUncorrectedGyroBiasWouldHaveDriftedMeasurably() {
        // Control for the test above: without bias removal the same input drifts ~1.5 deg.
        let bias = Vector3(0, 0, 0.1 * .pi / 180)
        var q = Quaternion.identity
        for _ in 0..<1500 { q = q.integrating(angularVelocity: bias, dt: 0.01) }
        let drifted = q.rotate(Vector3(1, 0, 0))
        let angle = atan2(drifted.y, drifted.x) * 180 / .pi
        XCTAssertEqual(angle, 1.5, accuracy: 0.05)
    }

    /// The point of §3.2. Under a sustained 0.5 g forward push CoreMotion's `gravity` estimate
    /// tilts toward the acceleration and `userAcceleration` inherits the error. Propagating
    /// attitude from the stationary anchor must be immune to that.
    func testPropagatedAttitudeIsImmuneToCoreMotionGravityTilt() {
        var calibration = StationaryCalibration()
        for i in 0..<200 { calibration.add(restSample(t: Double(i) * 0.01)) }
        var propagator = AttitudePropagator(calibration: calibration.result()!)

        let trueForward = 0.5 * g            // m/s^2 along device +X
        var worstCoreMotionError = 0.0
        var worstPropagatedError = 0.0

        for i in 0..<1500 {
            let t = 2.0 + Double(i) * 0.01
            // CoreMotion's gravity vector leans into the acceleration, reaching 15 degrees
            // by the end of the run, and its userAcceleration is reduced to match.
            let tilt = (Double(i) / 1500.0) * 15.0 * .pi / 180
            let leanedGravityG = Vector3(sin(tilt), 0, -cos(tilt))
            // The true raw reading is unchanged: forward accel plus true gravity.
            let trueSpecificForce = Vector3(trueForward, 0, -g)
            let coreMotionUserAccelG = (trueSpecificForce - leanedGravityG * g) / g

            let sample = IMUSample(
                t: t,
                userAccelerationG: coreMotionUserAccelG,
                gravityG: leanedGravityG,
                rotationRate: .zero,           // the phone is not actually rotating
                attitude: .identity
            )

            propagator.advance(to: sample)
            let propagated = propagator.kinematicAcceleration(of: sample).x
            worstPropagatedError = max(worstPropagatedError, abs(propagated - trueForward))
            worstCoreMotionError = max(worstCoreMotionError,
                                       abs(sample.userAcceleration.x - trueForward))
        }

        XCTAssertLessThan(worstPropagatedError, 1e-9,
                          "propagated attitude must recover the true forward acceleration")
        XCTAssertGreaterThan(worstCoreMotionError, 1.0,
                             "the control: CoreMotion's own decomposition should be badly wrong")
    }

    func testResetReanchorsAttitudeForLongRuns() {
        // Spec §3.2: for runs beyond ~25 s, re-anchor at steady-state cruise.
        var calibration = StationaryCalibration()
        for i in 0..<200 { calibration.add(restSample(t: Double(i) * 0.01)) }
        var propagator = AttitudePropagator(calibration: calibration.result()!)
        let rotated = Quaternion(axis: Vector3(0, 0, 1), angle: 0.3)
        propagator.reanchor(attitude: rotated)
        XCTAssertEqual(propagator.attitude.dot(rotated), 1.0, accuracy: 1e-12)
    }

    func testAdvanceUsesMeasuredIntervalsNotAnAssumedRate() {
        // Spec §3.1: always compute dt from consecutive timestamps.
        var calibration = StationaryCalibration()
        for i in 0..<100 { calibration.add(restSample(t: Double(i) * 0.01)) }
        var propagator = AttitudePropagator(calibration: calibration.result()!)
        let rate = Vector3(0, 0, 1.0)          // 1 rad/s
        // The anchor sits at the last stationary sample, t = 0.99. Advancing to that same
        // instant is a zero-length step; the next sample is 0.5 s later, so the rotation must
        // be 0.5 rad — not the 0.01 rad an assumed 100 Hz interval would give.
        propagator.advance(to: restSample(t: 0.99, rotationRate: rate))
        propagator.advance(to: restSample(t: 1.49, rotationRate: rate))
        let v = propagator.attitude.rotate(Vector3(1, 0, 0))
        XCTAssertEqual(atan2(v.y, v.x), 0.5, accuracy: 1e-6)
    }
}

/// Spec §5 — vehicle frame calibration.
final class VehicleFrameCalibratorTests: XCTestCase {
    private let g = PTConstants.g

    /// Rotation taking vehicle-frame vectors into the device frame.
    private func mountRotation() -> Quaternion {
        Quaternion(axis: Vector3(0.3, -0.5, 0.81).normalized()!, angle: 1.1)
    }

    private func syntheticInputs(
        mount: Quaternion,
        forwardAcceleration: Double = 4.0,
        verticalContamination: Double = 0,
        sampleCount: Int = 150
    ) -> (gravity: [Vector3], launch: [Vector3]) {
        let gravityVehicle = Vector3(0, 0, -g)
        let launchVehicle = Vector3(forwardAcceleration, 0, verticalContamination)
        return (
            (0..<sampleCount).map { _ in mount.rotate(gravityVehicle) },
            (0..<sampleCount).map { _ in mount.rotate(launchVehicle) }
        )
    }

    func testRecoversTheMountRotationFromGravityAndOneLaunch() {
        let mount = mountRotation()
        let inputs = syntheticInputs(mount: mount)
        let calibration = try! VehicleFrameCalibrator.calibrate(
            staticGravity: inputs.gravity,
            launchAcceleration: inputs.launch,
            courseAccuracyDegrees: 1.0,
            speedIncreasing: true
        )

        // R_DV's rows are the vehicle axes expressed in device coordinates (spec §5).
        let expectedX = mount.rotate(Vector3(1, 0, 0))
        let expectedZ = mount.rotate(Vector3(0, 0, 1))
        let r = calibration.rotation
        XCTAssertEqual(r.row(0).dot(expectedX), 1.0, accuracy: 1e-9)
        XCTAssertEqual(r.row(2).dot(expectedZ), 1.0, accuracy: 1e-9)
    }

    func testProjectionOfDeviceAccelerationYieldsForwardComponent() {
        let mount = mountRotation()
        let inputs = syntheticInputs(mount: mount)
        let calibration = try! VehicleFrameCalibrator.calibrate(
            staticGravity: inputs.gravity,
            launchAcceleration: inputs.launch,
            courseAccuracyDegrees: 1.0,
            speedIncreasing: true
        )
        // A 7 m/s^2 forward push in the vehicle frame, seen in the device frame...
        let deviceVector = mount.rotate(Vector3(7.0, 0, 0))
        // ...must project back to 7 m/s^2 along vehicle +X.
        let vehicle = calibration.rotation * deviceVector
        XCTAssertEqual(vehicle.x, 7.0, accuracy: 1e-9)
        XCTAssertEqual(vehicle.y, 0, accuracy: 1e-9)
        XCTAssertEqual(vehicle.z, 0, accuracy: 1e-9)
    }

    func testResultingBasisIsOrthonormalAndRightHanded() {
        let mount = mountRotation()
        let inputs = syntheticInputs(mount: mount)
        let r = try! VehicleFrameCalibrator.calibrate(
            staticGravity: inputs.gravity,
            launchAcceleration: inputs.launch,
            courseAccuracyDegrees: 1.0,
            speedIncreasing: true
        ).rotation
        let x = r.row(0), y = r.row(1), z = r.row(2)
        XCTAssertEqual(x.length, 1, accuracy: 1e-12)
        XCTAssertEqual(y.length, 1, accuracy: 1e-12)
        XCTAssertEqual(z.length, 1, accuracy: 1e-12)
        XCTAssertEqual(x.dot(y), 0, accuracy: 1e-12)
        XCTAssertEqual(y.dot(z), 0, accuracy: 1e-12)
        XCTAssertEqual(x.cross(y).dot(z), 1, accuracy: 1e-12)
    }

    func testSignIsFlippedWhenTheEventWasADecelerationNotALaunch() {
        // Spec §5: confirm x_V points forward by checking projected acceleration is positive
        // while GNSS speed is increasing; if negative, flip x_V and y_V.
        //
        // x_V is derived as normalize(a_h), so the projection onto it is positive by
        // construction — the observable that actually resolves the ambiguity is whether the
        // car was speeding up. Here the measured acceleration points along vehicle −X and
        // speed was falling, i.e. a braking event: forward is the opposite direction.
        let mount = mountRotation()
        let inputs = syntheticInputs(mount: mount, forwardAcceleration: -4.0)
        let calibration = try! VehicleFrameCalibrator.calibrate(
            staticGravity: inputs.gravity,
            launchAcceleration: inputs.launch,
            courseAccuracyDegrees: 1.0,
            speedIncreasing: false
        )
        // Even though the observed launch acceleration pointed along vehicle −X, the recovered
        // forward axis must still point forward.
        let expectedX = mount.rotate(Vector3(1, 0, 0))
        XCTAssertEqual(calibration.rotation.row(0).dot(expectedX), 1.0, accuracy: 1e-9)
        XCTAssertTrue(calibration.wasSignFlipped)
        XCTAssertEqual(calibration.rotation.row(0).cross(calibration.rotation.row(1))
            .dot(calibration.rotation.row(2)), 1, accuracy: 1e-12)
    }

    func testRejectsWhenAccelerationHasTooLargeAVerticalComponent() {
        // Spec §5 gate: |a_D . z_V| > 0.15 |a_D| means the car is not on level ground,
        // or the mount moved.
        let mount = mountRotation()
        let inputs = syntheticInputs(mount: mount, forwardAcceleration: 4.0,
                                     verticalContamination: 2.0)   // 2 / |a| = 0.45
        XCTAssertThrowsError(try VehicleFrameCalibrator.calibrate(
            staticGravity: inputs.gravity,
            launchAcceleration: inputs.launch,
            courseAccuracyDegrees: 1.0,
            speedIncreasing: true
        )) { error in
            guard case .notLevel = error as? VehicleFrameCalibrator.Error else {
                return XCTFail("expected .notLevel, got \(error)")
            }
        }
    }

    func testAcceptsASmallVerticalComponentInsideTheGate() {
        let mount = mountRotation()
        // 0.4 / sqrt(4^2 + 0.4^2) = 0.0995, inside the 0.15 limit.
        let inputs = syntheticInputs(mount: mount, forwardAcceleration: 4.0,
                                     verticalContamination: 0.4)
        XCTAssertNoThrow(try VehicleFrameCalibrator.calibrate(
            staticGravity: inputs.gravity,
            launchAcceleration: inputs.launch,
            courseAccuracyDegrees: 1.0,
            speedIncreasing: true
        ))
    }

    func testRejectsALaunchTooGentleToResolveTheAxis() {
        // Spec §5 gate: |a_h| < 1.5 m/s^2.
        let mount = mountRotation()
        let inputs = syntheticInputs(mount: mount, forwardAcceleration: 1.0)
        XCTAssertThrowsError(try VehicleFrameCalibrator.calibrate(
            staticGravity: inputs.gravity,
            launchAcceleration: inputs.launch,
            courseAccuracyDegrees: 1.0,
            speedIncreasing: true
        )) { error in
            guard case .launchTooGentle = error as? VehicleFrameCalibrator.Error else {
                return XCTFail("expected .launchTooGentle, got \(error)")
            }
        }
    }

    func testRejectsWhenCourseAccuracyShowsTheRunWasNotStraight() {
        // Spec §5 gate: courseAccuracy during the event above 5 degrees.
        let mount = mountRotation()
        let inputs = syntheticInputs(mount: mount)
        XCTAssertThrowsError(try VehicleFrameCalibrator.calibrate(
            staticGravity: inputs.gravity,
            launchAcceleration: inputs.launch,
            courseAccuracyDegrees: 8.0,
            speedIncreasing: true
        )) { error in
            guard case .courseAccuracyTooPoor = error as? VehicleFrameCalibrator.Error else {
                return XCTFail("expected .courseAccuracyTooPoor, got \(error)")
            }
        }
    }

    func testMissingCourseAccuracyDoesNotBlockCalibration() {
        // CoreLocation reports a negative courseAccuracy when it has none; that is an absence
        // of evidence, not evidence of a curve.
        let mount = mountRotation()
        let inputs = syntheticInputs(mount: mount)
        XCTAssertNoThrow(try VehicleFrameCalibrator.calibrate(
            staticGravity: inputs.gravity,
            launchAcceleration: inputs.launch,
            courseAccuracyDegrees: nil,
            speedIncreasing: true
        ))
    }

    func testRejectsDegenerateGravity() {
        XCTAssertThrowsError(try VehicleFrameCalibrator.calibrate(
            staticGravity: [Vector3.zero],
            launchAcceleration: [Vector3(4, 0, 0)],
            courseAccuracyDegrees: 1.0,
            speedIncreasing: true
        ))
    }

    func testRejectsEmptyInput() {
        XCTAssertThrowsError(try VehicleFrameCalibrator.calibrate(
            staticGravity: [],
            launchAcceleration: [],
            courseAccuracyDegrees: nil,
            speedIncreasing: true
        ))
    }

    /// Spec §5: "Refine across runs by averaging x_V over the last N valid calibration events
    /// (use quaternion averaging, not component averaging)."
    func testRefinementAveragesCalibrationsAsRotations() {
        let base = mountRotation()
        var calibrations: [VehicleFrameCalibration] = []
        for offset in [-0.02, 0.0, 0.02] {
            let perturbed = base * Quaternion(axis: Vector3(0, 0, 1), angle: offset)
            let inputs = syntheticInputs(mount: perturbed)
            calibrations.append(try! VehicleFrameCalibrator.calibrate(
                staticGravity: inputs.gravity,
                launchAcceleration: inputs.launch,
                courseAccuracyDegrees: 1.0,
                speedIncreasing: true
            ))
        }
        let refined = VehicleFrameCalibrator.refine(calibrations)!
        let expectedX = base.rotate(Vector3(1, 0, 0))
        XCTAssertEqual(refined.rotation.row(0).dot(expectedX), 1.0, accuracy: 1e-6)
        XCTAssertEqual(refined.sourceEventCount, 3)
    }

    func testRefinementOfNothingIsNil() {
        XCTAssertNil(VehicleFrameCalibrator.refine([]))
    }

    func testRefinementKeepsOnlyTheMostRecentEvents() {
        let base = mountRotation()
        let inputs = syntheticInputs(mount: base)
        let one = try! VehicleFrameCalibrator.calibrate(
            staticGravity: inputs.gravity, launchAcceleration: inputs.launch,
            courseAccuracyDegrees: 1.0, speedIncreasing: true
        )
        let refined = VehicleFrameCalibrator.refine([one, one, one, one, one], limit: 3)!
        XCTAssertEqual(refined.sourceEventCount, 3)
    }
}

/// Deviation D2 — longitudinal projection with a consistent frame chain.
final class LongitudinalResolverTests: XCTestCase {
    private let g = PTConstants.g

    func testResolvesForwardAccelerationThroughAttitudeAndMount() {
        let mount = Quaternion(axis: Vector3(0.2, 0.4, 0.89).normalized()!, angle: 0.9)

        // Calibrate the mount from synthetic gravity plus one launch.
        let gravityDevice = mount.rotate(Vector3(0, 0, -g))
        let launchDevice = mount.rotate(Vector3(5, 0, 0))
        let calibration = try! VehicleFrameCalibrator.calibrate(
            staticGravity: [Vector3](repeating: gravityDevice, count: 100),
            launchAcceleration: [Vector3](repeating: launchDevice, count: 100),
            courseAccuracyDegrees: 1.0,
            speedIncreasing: true
        )

        // Stationary window: the device reports gravity only.
        var stationary = StationaryCalibration()
        for i in 0..<200 {
            stationary.add(IMUSample(
                t: Double(i) * 0.01,
                userAccelerationG: .zero,
                gravityG: gravityDevice / g,
                rotationRate: .zero,
                attitude: .identity
            ))
        }

        var resolver = LongitudinalResolver(
            calibration: calibration,
            stationary: stationary.result()!
        )

        // Now a genuine 3 m/s^2 forward push.
        let trueForward = 3.0
        let specificForce = mount.rotate(Vector3(trueForward, 0, 0)) + gravityDevice
        let sample = IMUSample(
            t: 2.0,
            specificForce: specificForce,
            rotationRate: .zero,
            gravity: gravityDevice,
            userAcceleration: mount.rotate(Vector3(trueForward, 0, 0)),
            attitude: .identity
        )
        XCTAssertEqual(resolver.longitudinalAcceleration(of: sample), trueForward, accuracy: 1e-9)
    }

    func testAtRestTheLongitudinalAccelerationIsZero() {
        let mount = Quaternion(axis: Vector3(0, 1, 0), angle: 0.4)
        let gravityDevice = mount.rotate(Vector3(0, 0, -g))
        let calibration = try! VehicleFrameCalibrator.calibrate(
            staticGravity: [Vector3](repeating: gravityDevice, count: 50),
            launchAcceleration: [Vector3](repeating: mount.rotate(Vector3(4, 0, 0)), count: 50),
            courseAccuracyDegrees: 1.0,
            speedIncreasing: true
        )
        var stationary = StationaryCalibration()
        for i in 0..<100 {
            stationary.add(IMUSample(t: Double(i) * 0.01, userAccelerationG: .zero,
                                     gravityG: gravityDevice / g, rotationRate: .zero,
                                     attitude: .identity))
        }
        var resolver = LongitudinalResolver(calibration: calibration,
                                            stationary: stationary.result()!)
        let sample = IMUSample(t: 2.0, specificForce: gravityDevice, rotationRate: .zero,
                               gravity: gravityDevice, userAcceleration: .zero,
                               attitude: .identity)
        XCTAssertEqual(resolver.longitudinalAcceleration(of: sample), 0, accuracy: 1e-9)
    }

    /// Spec §7.1: on a grade the vehicle longitudinal axis includes a component of gravity.
    /// Because attitude is propagated from a launch anchored on the same grade, removing the
    /// full gravity vector already yields the true along-track acceleration, so the optional
    /// correction is off by default.
    func testGradeCorrectionIsOptionalAndSignedUphillPositive() {
        let theta = 0.05                                   // ~5% uphill, radians
        XCTAssertEqual(
            GradeCorrection.correct(acceleration: 2.0, gradeRadians: theta),
            2.0 + PTConstants.g * sin(theta),
            accuracy: 1e-12
        )
        XCTAssertEqual(GradeCorrection.correct(acceleration: 2.0, gradeRadians: 0), 2.0,
                       accuracy: 1e-12)
    }
}
