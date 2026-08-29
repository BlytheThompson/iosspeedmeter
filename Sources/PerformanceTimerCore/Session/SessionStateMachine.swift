import Foundation

/// Spec §4 — session state machine.
///
/// ```
/// IDLE
///   → ARMED        GNSS lock acquired, calibration valid, stationary detected
///   → RECORDING    launch detected (retroactively anchored)
///   → COMPLETE     stop condition met
///   → ANALYSING    smoother + result extraction
///   → RESULT
/// ```
public enum SessionState: String, Equatable, Codable, Sendable, CaseIterable {
    case idle
    case armed
    case recording
    case complete
    case analysing
    case result
}

public struct SessionStateMachine: Sendable {
    /// Everything the machine needs to decide a transition, gathered per IMU sample.
    public struct Inputs: Sendable {
        public var t: Double
        public var hasGNSSLock: Bool
        public var calibrationValid: Bool
        public var isStationary: Bool
        public var longitudinalAcceleration: Double
        public var speed: Double
        public var distance: Double
        public var launchTriggered: Bool
        public var userRequestedStop: Bool

        public init(
            t: Double, hasGNSSLock: Bool, calibrationValid: Bool, isStationary: Bool,
            longitudinalAcceleration: Double, speed: Double, distance: Double,
            launchTriggered: Bool, userRequestedStop: Bool
        ) {
            self.t = t
            self.hasGNSSLock = hasGNSSLock
            self.calibrationValid = calibrationValid
            self.isStationary = isStationary
            self.longitudinalAcceleration = longitudinalAcceleration
            self.speed = speed
            self.distance = distance
            self.launchTriggered = launchTriggered
            self.userRequestedStop = userRequestedStop
        }
    }

    /// Spec §4: "target distance exceeded, or speed below 1 m/s for 2 s, or 120 s elapsed, or
    /// user stop."
    public enum StopReason: String, Equatable, Codable, Sendable {
        case targetDistanceReached
        case vehicleStopped
        case durationLimit
        case userStopped
    }

    public private(set) var state: SessionState = .idle
    public private(set) var stopReason: StopReason?
    /// Session time at which RECORDING began. The reported `t₀` comes from the retroactive
    /// anchor of §8, not from this.
    public private(set) var recordingStartTime: Double?

    public var targetDistance: Double
    /// Spec §4 stop condition: speed below this for `stoppedDuration`.
    public var stoppedSpeedThreshold: Double
    public var stoppedDuration: Double
    /// Spec §4 stop condition: 120 s elapsed.
    public var maximumDuration: Double
    /// How long the vehicle may be non-stationary while ARMED before the machine gives up and
    /// returns to IDLE.
    ///
    /// This grace period is load-bearing. The launch itself makes the vehicle non-stationary
    /// **before** the launch trigger can confirm — spec §8 needs 0.15 g sustained for 60 ms —
    /// so disarming on the first moving sample makes the ARMED→RECORDING transition
    /// unreachable. One second comfortably covers the 60 ms confirmation while still dropping
    /// out of ARMED if the car simply drives away.
    public var disarmGracePeriod: Double

    private var slowSince: Double?
    private var movingSince: Double?

    public init(
        targetDistance: Double = DistanceMark.quarterMile.meters,
        stoppedSpeedThreshold: Double = 1.0,
        stoppedDuration: Double = 2.0,
        maximumDuration: Double = 120.0,
        disarmGracePeriod: Double = 1.0
    ) {
        self.targetDistance = targetDistance
        self.stoppedSpeedThreshold = stoppedSpeedThreshold
        self.stoppedDuration = stoppedDuration
        self.maximumDuration = maximumDuration
        self.disarmGracePeriod = disarmGracePeriod
    }

    /// Spec §4: "Recording actually begins at ARMED, not at RECORDING" — the pre-launch window
    /// is needed for ZUPT and for retroactive anchoring.
    public var shouldCaptureSamples: Bool {
        state == .armed || state == .recording
    }

    public mutating func update(_ inputs: Inputs) {
        switch state {
        case .idle:
            if inputs.hasGNSSLock, inputs.calibrationValid, inputs.isStationary {
                state = .armed
            }

        case .armed:
            if inputs.launchTriggered {
                state = .recording
                recordingStartTime = inputs.t
                slowSince = nil
                movingSince = nil
            } else if !inputs.hasGNSSLock || !inputs.calibrationValid {
                // Losing lock or calibration is immediately disqualifying.
                state = .idle
                movingSince = nil
            } else if !inputs.isStationary {
                // Moving, but the launch trigger has not confirmed yet. Hold ARMED for the
                // grace period so a real launch can complete its 60 ms confirmation, then
                // fall back to IDLE if this was just the car driving off.
                if let since = movingSince {
                    if inputs.t - since >= disarmGracePeriod {
                        state = .idle
                        movingSince = nil
                    }
                } else {
                    movingSince = inputs.t
                }
            } else {
                movingSince = nil
            }

        case .recording:
            if inputs.userRequestedStop {
                finish(.userStopped)
            } else if inputs.distance >= targetDistance {
                finish(.targetDistanceReached)
            } else if let start = recordingStartTime, inputs.t - start >= maximumDuration {
                finish(.durationLimit)
            } else if inputs.speed < stoppedSpeedThreshold {
                if let since = slowSince {
                    if inputs.t - since >= stoppedDuration { finish(.vehicleStopped) }
                } else {
                    slowSince = inputs.t
                }
            } else {
                slowSince = nil
            }

        case .complete, .analysing, .result:
            break
        }
    }

    private mutating func finish(_ reason: StopReason) {
        state = .complete
        stopReason = reason
    }

    public mutating func beginAnalysis() {
        guard state == .complete else { return }
        state = .analysing
    }

    public mutating func finishAnalysis() {
        guard state == .analysing else { return }
        state = .result
    }

    /// Return to IDLE for another run, keeping configuration.
    public mutating func reset() {
        state = .idle
        stopReason = nil
        recordingStartTime = nil
        slowSince = nil
        movingSince = nil
    }
}
