import XCTest
@testable import PerformanceTimerCore

// MARK: - §4 ring buffer

final class RingBufferTests: XCTestCase {
    func testKeepsOnlyTheMostRecentWindow() {
        // Spec §4: "Keep a 10 s ring buffer while ARMED and prepend it on transition."
        var buffer = TimeWindowBuffer<IMUSample>(duration: 10.0) { $0.t }
        for i in 0..<2000 {                      // 20 s at 100 Hz
            buffer.append(makeSample(t: Double(i) * 0.01))
        }
        XCTAssertEqual(buffer.elements.last!.t, 19.99, accuracy: 1e-9)
        // The retained window must cover the full 10 s the spec asks for, and not much more.
        let span = buffer.elements.last!.t - buffer.elements.first!.t
        XCTAssertGreaterThanOrEqual(span, 10.0 - 1e-9)
        XCTAssertLessThan(span, 10.02)
    }

    func testDrainReturnsContentsAndEmptiesTheBuffer() {
        var buffer = TimeWindowBuffer<IMUSample>(duration: 10.0) { $0.t }
        for i in 0..<100 { buffer.append(makeSample(t: Double(i) * 0.01)) }
        let drained = buffer.drain()
        XCTAssertEqual(drained.count, 100)
        XCTAssertEqual(buffer.count, 0)
    }

    func testOutOfOrderSamplesDoNotShrinkTheWindow() {
        // A late GNSS fix legitimately arrives out of order (spec §2.3).
        var buffer = TimeWindowBuffer<IMUSample>(duration: 10.0) { $0.t }
        buffer.append(makeSample(t: 100.0))
        buffer.append(makeSample(t: 99.5))
        XCTAssertEqual(buffer.count, 2)
    }

    private func makeSample(t: Double) -> IMUSample {
        IMUSample(t: t, userAccelerationG: .zero, gravityG: Vector3(0, 0, -1),
                  rotationRate: .zero, attitude: .identity)
    }
}

// MARK: - §8 launch detection and anchoring

final class LaunchDetectionTests: XCTestCase {
    private let g = PTConstants.g

    func testTriggersAfterSustainedAccelerationAboveThreshold() {
        // Spec §8: a > 0.15 g sustained for >= 60 ms.
        var detector = LaunchDetector()
        var triggered = false
        for i in 0..<100 {
            let t = Double(i) * 0.01
            triggered = detector.update(t: t, acceleration: 0.3 * g)
            if triggered { XCTAssertGreaterThanOrEqual(t, 0.06); break }
        }
        XCTAssertTrue(triggered)
    }

    func testDoesNotTriggerOnABriefSpike() {
        var detector = LaunchDetector()
        // 40 ms above threshold, then back down — a pothole, not a launch.
        for i in 0..<4 {
            XCTAssertFalse(detector.update(t: Double(i) * 0.01, acceleration: 0.4 * g))
        }
        for i in 4..<40 {
            XCTAssertFalse(detector.update(t: Double(i) * 0.01, acceleration: 0.01 * g))
        }
    }

    func testDoesNotTriggerBelowThreshold() {
        var detector = LaunchDetector()
        for i in 0..<200 {
            XCTAssertFalse(detector.update(t: Double(i) * 0.01, acceleration: 0.10 * g))
        }
    }

    func testResetClearsProgress() {
        var detector = LaunchDetector()
        _ = detector.update(t: 0.00, acceleration: 0.4 * g)
        _ = detector.update(t: 0.01, acceleration: 0.4 * g)
        detector.reset()
        XCTAssertFalse(detector.update(t: 0.02, acceleration: 0.4 * g))
    }

    /// Spec §8: search backward from the trigger for the last sample where the car was
    /// genuinely still, and call that `t₀`. "This routinely recovers 100–200 ms versus
    /// triggering on GNSS."
    func testRetroactiveAnchorFindsTheLastTrulyStationarySample() {
        var samples: [SmoothedSample] = []
        var stationary: [Bool] = []

        // 2 s at rest.
        for i in 0..<200 {
            samples.append(SyntheticTrace.sample(t: Double(i) * 0.01, s: 0, v: 0, a: 0))
            stationary.append(true)
        }
        // Then a launch at 5 m/s^2 from t = 2.0.
        for i in 0..<300 {
            let dt = Double(i) * 0.01
            samples.append(SyntheticTrace.sample(
                t: 2.0 + dt, s: 0.5 * 5 * dt * dt, v: 5 * dt, a: 5))
            stationary.append(false)
        }

        // The live trigger fires late, once 0.15 g has been sustained.
        let triggerIndex = 215
        let anchor = LaunchAnchor.retroactiveAnchor(
            samples: samples, stationaryFlags: stationary, triggerIndex: triggerIndex
        )!
        XCTAssertEqual(samples[anchor.index].t, 1.99, accuracy: 0.02)
        XCTAssertLessThan(anchor.index, triggerIndex)
        // The recovered time must be meaningfully earlier than the trigger.
        XCTAssertGreaterThan(samples[triggerIndex].t - anchor.time, 0.1)
    }

    func testRetroactiveAnchorRequiresNearZeroSmoothedSpeed() {
        // Spec §8 condition: v_smoothed < 0.02 m/s.
        var samples: [SmoothedSample] = []
        var stationary: [Bool] = []
        for i in 0..<300 {
            let t = Double(i) * 0.01
            // Creeping forward at 0.5 m/s the whole time — never truly stopped.
            samples.append(SyntheticTrace.sample(t: t, s: 0.5 * t, v: 0.5, a: 0))
            stationary.append(true)
        }
        XCTAssertNil(LaunchAnchor.retroactiveAnchor(
            samples: samples, stationaryFlags: stationary, triggerIndex: 250))
    }

    func testRetroactiveAnchorRequiresRisingAccelerationAfterIt() {
        // Spec §8 condition: the next 100 ms of a must be monotonically increasing.
        var samples: [SmoothedSample] = []
        var stationary: [Bool] = []
        for i in 0..<300 {
            let t = Double(i) * 0.01
            // At rest, but the acceleration signal is falling, not rising.
            samples.append(SyntheticTrace.sample(t: t, s: 0, v: 0, a: -Double(i) * 0.01))
            stationary.append(true)
        }
        XCTAssertNil(LaunchAnchor.retroactiveAnchor(
            samples: samples, stationaryFlags: stationary, triggerIndex: 250))
    }

    /// Spec §8 roll-race variant: no stationary anchor, no ZUPT; anchor on the smoothed speed
    /// trace crossing the lower bound, and accept the wider uncertainty.
    func testRollAnchorUsesTheLowerSpeedBoundCrossing() {
        let trace = SyntheticTrace.constantAcceleration(4.0, duration: 25, dt: 0.01)
        let window = RollWindow(fromMph: 60, toMph: 130)
        let anchor = LaunchAnchor.rollAnchor(samples: trace, window: window)!
        XCTAssertEqual(anchor.time, window.fromMetersPerSecond / 4.0, accuracy: 1e-9)
    }

    func testRollAnchorIsNilWhenTheWindowIsNeverEntered() {
        let trace = SyntheticTrace.constantAcceleration(4.0, duration: 3, dt: 0.01)
        XCTAssertNil(LaunchAnchor.rollAnchor(samples: trace,
                                             window: RollWindow(fromMph: 60, toMph: 130)))
    }
}

// MARK: - §4 session state machine

final class SessionStateMachineTests: XCTestCase {
    func testStartsIdle() {
        XCTAssertEqual(SessionStateMachine().state, .idle)
    }

    func testArmsOnlyWhenLockCalibrationAndStillnessAllHold() {
        var machine = SessionStateMachine()
        machine.update(SessionStateMachine.Inputs(
            t: 0, hasGNSSLock: false, calibrationValid: true, isStationary: true,
            longitudinalAcceleration: 0, speed: 0, distance: 0, launchTriggered: false,
            userRequestedStop: false))
        XCTAssertEqual(machine.state, .idle)

        machine.update(SessionStateMachine.Inputs(
            t: 1, hasGNSSLock: true, calibrationValid: false, isStationary: true,
            longitudinalAcceleration: 0, speed: 0, distance: 0, launchTriggered: false,
            userRequestedStop: false))
        XCTAssertEqual(machine.state, .idle)

        machine.update(SessionStateMachine.Inputs(
            t: 2, hasGNSSLock: true, calibrationValid: true, isStationary: true,
            longitudinalAcceleration: 0, speed: 0, distance: 0, launchTriggered: false,
            userRequestedStop: false))
        XCTAssertEqual(machine.state, .armed)
    }

    func testRecordingBeginsOnLaunchTrigger() {
        var machine = armedMachine()
        machine.update(SessionStateMachine.Inputs(
            t: 3, hasGNSSLock: true, calibrationValid: true, isStationary: false,
            longitudinalAcceleration: 4.0, speed: 1.0, distance: 0.2, launchTriggered: true,
            userRequestedStop: false))
        XCTAssertEqual(machine.state, .recording)
    }

    func testStopsWhenTargetDistanceIsExceeded() {
        var machine = recordingMachine(targetDistance: 402.336)
        machine.update(SessionStateMachine.Inputs(
            t: 15, hasGNSSLock: true, calibrationValid: true, isStationary: false,
            longitudinalAcceleration: 4.0, speed: 50, distance: 410, launchTriggered: false,
            userRequestedStop: false))
        XCTAssertEqual(machine.state, .complete)
        XCTAssertEqual(machine.stopReason, .targetDistanceReached)
    }

    func testStopsAfterTwoSecondsBelowOneMetrePerSecond() {
        var machine = recordingMachine()
        for i in 0..<300 {
            let t = 10.0 + Double(i) * 0.01
            machine.update(SessionStateMachine.Inputs(
                t: t, hasGNSSLock: true, calibrationValid: true, isStationary: false,
                longitudinalAcceleration: 0, speed: 0.4, distance: 100,
                launchTriggered: false, userRequestedStop: false))
        }
        XCTAssertEqual(machine.state, .complete)
        XCTAssertEqual(machine.stopReason, .vehicleStopped)
    }

    func testBriefDipBelowOneMetrePerSecondDoesNotStop() {
        var machine = recordingMachine()
        for i in 0..<100 {                       // 1 s only
            machine.update(SessionStateMachine.Inputs(
                t: 10.0 + Double(i) * 0.01, hasGNSSLock: true, calibrationValid: true,
                isStationary: false, longitudinalAcceleration: 0, speed: 0.4, distance: 100,
                launchTriggered: false, userRequestedStop: false))
        }
        XCTAssertEqual(machine.state, .recording)
    }

    func testStopsAfterTheDurationLimit() {
        var machine = recordingMachine()
        machine.update(SessionStateMachine.Inputs(
            t: 200, hasGNSSLock: true, calibrationValid: true, isStationary: false,
            longitudinalAcceleration: 2, speed: 40, distance: 300, launchTriggered: false,
            userRequestedStop: false))
        XCTAssertEqual(machine.state, .complete)
        XCTAssertEqual(machine.stopReason, .durationLimit)
    }

    func testUserStopIsHonoured() {
        var machine = recordingMachine()
        machine.update(SessionStateMachine.Inputs(
            t: 12, hasGNSSLock: true, calibrationValid: true, isStationary: false,
            longitudinalAcceleration: 2, speed: 40, distance: 100, launchTriggered: false,
            userRequestedStop: true))
        XCTAssertEqual(machine.state, .complete)
        XCTAssertEqual(machine.stopReason, .userStopped)
    }

    func testAnalysingThenResultCompletesTheCycle() {
        var machine = recordingMachine()
        machine.update(SessionStateMachine.Inputs(
            t: 12, hasGNSSLock: true, calibrationValid: true, isStationary: false,
            longitudinalAcceleration: 0, speed: 0, distance: 100, launchTriggered: false,
            userRequestedStop: true))
        XCTAssertEqual(machine.state, .complete)
        machine.beginAnalysis()
        XCTAssertEqual(machine.state, .analysing)
        machine.finishAnalysis()
        XCTAssertEqual(machine.state, .result)
    }

    func testDisarmsImmediatelyWhenGNSSLockIsLost() {
        var machine = armedMachine()
        machine.update(SessionStateMachine.Inputs(
            t: 3, hasGNSSLock: false, calibrationValid: true, isStationary: true,
            longitudinalAcceleration: 0, speed: 0, distance: 0, launchTriggered: false,
            userRequestedStop: false))
        XCTAssertEqual(machine.state, .idle)
    }

    func testDisarmsImmediatelyWhenCalibrationBecomesInvalid() {
        var machine = armedMachine()
        machine.update(SessionStateMachine.Inputs(
            t: 3, hasGNSSLock: true, calibrationValid: false, isStationary: true,
            longitudinalAcceleration: 0, speed: 0, distance: 0, launchTriggered: false,
            userRequestedStop: false))
        XCTAssertEqual(machine.state, .idle)
    }

    /// The launch makes the vehicle non-stationary before the §8 trigger can confirm, so
    /// ARMED must survive briefly losing stillness or RECORDING is unreachable.
    func testStaysArmedThroughTheMomentBetweenMotionAndLaunchConfirmation() {
        var machine = armedMachine()
        for i in 0..<6 {                          // 60 ms of motion, no trigger yet
            machine.update(SessionStateMachine.Inputs(
                t: 3.0 + Double(i) * 0.01, hasGNSSLock: true, calibrationValid: true,
                isStationary: false, longitudinalAcceleration: 4.0, speed: 0.2, distance: 0.01,
                launchTriggered: false, userRequestedStop: false))
        }
        XCTAssertEqual(machine.state, .armed)

        machine.update(SessionStateMachine.Inputs(
            t: 3.07, hasGNSSLock: true, calibrationValid: true, isStationary: false,
            longitudinalAcceleration: 4.0, speed: 0.3, distance: 0.02,
            launchTriggered: true, userRequestedStop: false))
        XCTAssertEqual(machine.state, .recording)
    }

    func testDisarmsWhenTheCarSimplyDrivesAwayWithoutLaunching() {
        var machine = armedMachine()
        for i in 0..<200 {                        // 2 s of gentle motion, never triggering
            machine.update(SessionStateMachine.Inputs(
                t: 3.0 + Double(i) * 0.01, hasGNSSLock: true, calibrationValid: true,
                isStationary: false, longitudinalAcceleration: 0.3, speed: 2.0, distance: 3,
                launchTriggered: false, userRequestedStop: false))
        }
        XCTAssertEqual(machine.state, .idle)
    }

    /// Spec §4: "Recording actually begins at ARMED, not at RECORDING."
    func testRecordingFlagIsSetFromArmedNotFromRecording() {
        var machine = SessionStateMachine()
        XCTAssertFalse(machine.shouldCaptureSamples)
        machine = armedMachine()
        XCTAssertTrue(machine.shouldCaptureSamples)
    }

    private func armedMachine() -> SessionStateMachine {
        var machine = SessionStateMachine()
        machine.update(SessionStateMachine.Inputs(
            t: 2, hasGNSSLock: true, calibrationValid: true, isStationary: true,
            longitudinalAcceleration: 0, speed: 0, distance: 0, launchTriggered: false,
            userRequestedStop: false))
        return machine
    }

    private func recordingMachine(targetDistance: Double = 402.336) -> SessionStateMachine {
        var machine = SessionStateMachine(targetDistance: targetDistance)
        machine.update(SessionStateMachine.Inputs(
            t: 2, hasGNSSLock: true, calibrationValid: true, isStationary: true,
            longitudinalAcceleration: 0, speed: 0, distance: 0, launchTriggered: false,
            userRequestedStop: false))
        machine.update(SessionStateMachine.Inputs(
            t: 3, hasGNSSLock: true, calibrationValid: true, isStationary: false,
            longitudinalAcceleration: 4.0, speed: 1.0, distance: 0.2, launchTriggered: true,
            userRequestedStop: false))
        return machine
    }
}
