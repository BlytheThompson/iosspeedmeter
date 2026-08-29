import Foundation

/// Produces the scalar longitudinal acceleration `a` that the estimator's process model
/// consumes (spec §6.2 input `u = a`).
///
/// **Deviation D2 from the spec.** §5 writes the projection as
/// `a_raw_V = R_DV · (R_launch_propagated · f_D − g_L)`. `R_DV` maps device→vehicle, but its
/// operand there has already been rotated into the local-level frame, so the composition is
/// not well formed as written.
///
/// The same physical operation with a consistent frame chain is to remove gravity in the
/// **device** frame and only then project:
///
/// ```
/// a_kinematic_D = f_D − R_LD · ĝ_L·g        // R_LD from the propagated attitude
/// a             = (R_DV · a_kinematic_D).x
/// ```
///
/// This keeps `R_DV` applied to a device-frame vector, which is what §5 defines it to do, and
/// leaves the physics unchanged.
public struct LongitudinalResolver: Sendable {
    public let calibration: VehicleFrameCalibration
    public private(set) var propagator: AttitudePropagator

    /// Spec §7.1 grade correction. **Off by default** — when attitude is propagated from a
    /// launch anchored on the same grade, removing the full gravity vector already yields the
    /// true along-track acceleration, and applying `g·sin θ` as well would double-count.
    public var gradeCorrectionEnabled: Bool
    /// Current road grade, radians, positive uphill. Only consulted when the correction is on.
    public var gradeRadians: Double

    public init(
        calibration: VehicleFrameCalibration,
        stationary: StationaryCalibrationResult,
        gradeCorrectionEnabled: Bool = false,
        gradeRadians: Double = 0
    ) {
        self.calibration = calibration
        propagator = AttitudePropagator(calibration: stationary)
        self.gradeCorrectionEnabled = gradeCorrectionEnabled
        self.gradeRadians = gradeRadians
    }

    /// Advance attitude and return the full kinematic acceleration in the vehicle frame.
    public mutating func vehicleAcceleration(of sample: IMUSample) -> Vector3 {
        propagator.advance(to: sample)
        return calibration.rotation * propagator.kinematicAcceleration(of: sample)
    }

    /// The scalar the filter consumes: acceleration along vehicle +X, m/s².
    public mutating func longitudinalAcceleration(of sample: IMUSample) -> Double {
        let a = vehicleAcceleration(of: sample).x
        return gradeCorrectionEnabled
            ? GradeCorrection.correct(acceleration: a, gradeRadians: gradeRadians)
            : a
    }

    /// Re-anchor attitude mid-run (spec §3.2, runs beyond ~25 s).
    public mutating func reanchor(attitude: Quaternion) {
        propagator.reanchor(attitude: attitude)
    }
}
