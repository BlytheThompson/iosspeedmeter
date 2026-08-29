import Foundation

/// What the pre-launch stationary window yields. Spec §3.2 step 1: "During the pre-launch
/// stationary window, record `q_launch` … and the mean `gravity` vector. At rest, both are
/// trustworthy."
public struct StationaryCalibrationResult: Equatable, Codable, Sendable {
    /// Mean gyro reading at rest — this *is* the bias (spec §3.2).
    public var gyroBias: Vector3
    /// Mean specific force at rest, device frame, m/s². Points down, magnitude ≈ g.
    public var referenceSpecificForce: Vector3
    /// CoreMotion's attitude at the anchor, `q_launch`.
    public var launchAttitude: Quaternion
    public var sampleCount: Int
    /// Session time of the last sample in the window.
    public var t: Double

    public init(
        gyroBias: Vector3,
        referenceSpecificForce: Vector3,
        launchAttitude: Quaternion,
        sampleCount: Int,
        t: Double
    ) {
        self.gyroBias = gyroBias
        self.referenceSpecificForce = referenceSpecificForce
        self.launchAttitude = launchAttitude
        self.sampleCount = sampleCount
        self.t = t
    }
}

/// Accumulates the pre-launch stationary window (spec §3.2 step 1, §5 step 1).
public struct StationaryCalibration: Sendable {
    private var gyroSum = Vector3.zero
    private var forceSum = Vector3.zero
    private var gravitySum = Vector3.zero
    private var count = 0
    private var lastAttitude = Quaternion.identity
    private var lastTime = 0.0

    /// Spec §4 requires at least 40 samples in a 0.5 s window; requiring the same here keeps
    /// a too-short window from producing a confidently wrong bias.
    public var minimumSamples: Int

    public init(minimumSamples: Int = 40) {
        self.minimumSamples = minimumSamples
    }

    public mutating func add(_ sample: IMUSample) {
        gyroSum += sample.rotationRate
        forceSum += sample.specificForce
        gravitySum += sample.gravity
        lastAttitude = sample.attitude
        lastTime = sample.t
        count += 1
    }

    public mutating func reset() {
        gyroSum = .zero
        forceSum = .zero
        gravitySum = .zero
        count = 0
        lastAttitude = .identity
        lastTime = 0
    }

    public var sampleCount: Int { count }

    /// Mean gravity over the window, device frame, m/s². Used by §5 step 1 to define `z_V`.
    public var meanGravity: Vector3? {
        count > 0 ? gravitySum / Double(count) : nil
    }

    public func result() -> StationaryCalibrationResult? {
        guard count >= minimumSamples else { return nil }
        let n = Double(count)
        return StationaryCalibrationResult(
            gyroBias: gyroSum / n,
            referenceSpecificForce: forceSum / n,
            launchAttitude: lastAttitude,
            sampleCount: count,
            t: lastTime
        )
    }
}

/// Spec §3.2 — launch-anchored attitude propagation.
///
/// **The trap this exists to avoid.** CoreMotion's `gravity` uses the accelerometer as a
/// long-term reference for "down". Under sustained longitudinal acceleration — exactly a drag
/// run — its fusion filter cannot distinguish a 0.5 g forward push from a 30° nose-up pitch,
/// so its gravity vector slowly tilts toward the acceleration and `userAcceleration` inherits
/// the error. Reading `userAcceleration` straight out during a run is therefore wrong in a way
/// that correlates with the very signal being measured.
///
/// The mitigation is to stop trusting CoreMotion's decomposition once the run starts:
/// reconstruct the raw specific force, propagate attitude forward from the stationary anchor
/// by integrating the bias-corrected gyro, and remove gravity using *that* attitude.
///
/// Residual gyro bias still produces a slow attitude ramp — 0.1 °/s gives ~1.5° over 15 s,
/// worth ~0.26 m/s² of false longitudinal acceleration. The filter's bias state `b` absorbs
/// most of it. The value of the propagation is that the remaining error is a slow ramp the
/// filter can track, rather than a signal-correlated distortion it cannot.
public struct AttitudePropagator: Sendable {
    /// Current attitude, device → reference, propagated from the anchor.
    public private(set) var attitude: Quaternion
    /// Gyro bias removed from every sample, rad/s.
    public let gyroBias: Vector3
    /// Unit gravity direction in the reference frame, established at rest.
    public let referenceGravityDirection: Vector3
    /// Gravity magnitude measured at rest, m/s². Using the measured value rather than the
    /// standard constant absorbs the accelerometer's scale-factor error.
    public let gravityMagnitude: Double

    private var lastTime: Double?

    public init(calibration: StationaryCalibrationResult) {
        attitude = calibration.launchAttitude
        gyroBias = calibration.gyroBias
        gravityMagnitude = calibration.referenceSpecificForce.length

        // At rest the specific force is purely gravity, so rotating it into the reference
        // frame with the trustworthy resting attitude fixes "down" once and for all.
        let inReference = calibration.launchAttitude.rotate(calibration.referenceSpecificForce)
        referenceGravityDirection = inReference.normalized() ?? Vector3(0, 0, -1)
        lastTime = calibration.t
    }

    /// Integrate attitude forward to this sample's timestamp.
    ///
    /// `dt` comes from consecutive timestamps, never an assumed 1/100 (spec §3.1).
    public mutating func advance(to sample: IMUSample) {
        defer { lastTime = sample.t }
        guard let previous = lastTime else { return }
        let dt = sample.t - previous
        guard dt > 0, dt.isFinite else { return }
        attitude = attitude.integrating(angularVelocity: sample.rotationRate - gyroBias, dt: dt)
    }

    /// Gravity expressed in the device frame using the *propagated* attitude — not
    /// CoreMotion's.
    public var gravityInDeviceFrame: Vector3 {
        attitude.rotateInverse(referenceGravityDirection * gravityMagnitude)
    }

    /// True kinematic acceleration in the device frame: raw specific force minus the gravity
    /// implied by the propagated attitude (spec §3.2 step 4).
    public func kinematicAcceleration(of sample: IMUSample) -> Vector3 {
        sample.specificForce - gravityInDeviceFrame
    }

    /// Re-anchor attitude, for runs beyond about 25 s.
    ///
    /// Spec §3.2: "reset attitude at a moment of steady-state cruise (near-zero longitudinal
    /// accel), where CoreMotion's gravity is trustworthy again."
    public mutating func reanchor(attitude newAttitude: Quaternion) {
        attitude = newAttitude.normalized()
    }
}

/// Spec §7.1 — removing grade from the acceleration input.
public enum GradeCorrection {
    /// `a_corrected = a_measured + g·sin(θ)`, θ positive uphill.
    ///
    /// This is **off by default** in the pipeline. When attitude is propagated from a launch
    /// anchored on the same grade, removing the full gravity vector already yields the true
    /// along-track acceleration, and §7.1 itself notes the residual goes into `b`. Applying
    /// the correction as well would double-count. It is exposed for the case where attitude is
    /// not trusted, and so the replay harness can measure the difference on real data.
    public static func correct(acceleration: Double, gradeRadians theta: Double) -> Double {
        acceleration + PTConstants.g * sin(theta)
    }
}
