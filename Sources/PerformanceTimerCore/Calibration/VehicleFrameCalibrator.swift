import Foundation

/// A solved device→vehicle rotation, `R_DV` (spec §5).
///
/// Persist this keyed by a mount profile — it only changes when the phone is remounted.
public struct VehicleFrameCalibration: Equatable, Codable, Sendable {
    /// `R_DV`. Rows are `[x_V; y_V; z_V]`, the vehicle axes in device coordinates, so
    /// `rotation * v_D` gives the components of a device-frame vector along the vehicle axes.
    public var rotation: Matrix3
    /// Session or wall-clock time the calibration was solved, for the "calibration age" input
    /// to the confidence badge (spec §9.4).
    public var timestamp: Double
    /// Magnitude of the horizontal launch acceleration used, m/s². Larger is better resolved.
    public var launchAccelerationMagnitude: Double
    /// Fraction of the launch acceleration that lay along `z_V`. The §5 gate caps this at 0.15.
    public var verticalFraction: Double
    /// True when the observed launch pointed along vehicle −X and the axes were flipped.
    public var wasSignFlipped: Bool
    /// How many calibration events were averaged into this one.
    public var sourceEventCount: Int

    public init(
        rotation: Matrix3,
        timestamp: Double,
        launchAccelerationMagnitude: Double,
        verticalFraction: Double,
        wasSignFlipped: Bool,
        sourceEventCount: Int = 1
    ) {
        self.rotation = rotation
        self.timestamp = timestamp
        self.launchAccelerationMagnitude = launchAccelerationMagnitude
        self.verticalFraction = verticalFraction
        self.wasSignFlipped = wasSignFlipped
        self.sourceEventCount = sourceEventCount
    }

    /// Quaternion form, used for cross-run averaging.
    public var quaternion: Quaternion {
        Quaternion(rotationMatrix: rotation)
    }
}

/// Spec §5 — solve `R_DV`, the device→vehicle rotation.
///
/// Run once per mounting position and persist the result. The vehicle frame is fixed relative
/// to the *car*, not to gravity, which is what makes the grade handling of §7 well defined.
public enum VehicleFrameCalibrator {
    public enum Error: Swift.Error, Equatable {
        /// `|ā_D · z_V| > 0.15 · |ā_D|` — car not on level ground, or the mount moved.
        case notLevel(verticalFraction: Double)
        /// `|a_h| < 1.5 m/s²` — launch too gentle to resolve the axis.
        case launchTooGentle(magnitude: Double)
        /// GNSS `courseAccuracy` during the event exceeded 5° — not actually straight.
        case courseAccuracyTooPoor(degrees: Double)
        /// Gravity or acceleration averaged to something with no usable direction.
        case degenerate
    }

    /// Spec §5 validation gates.
    public static let maximumVerticalFraction = 0.15
    public static let minimumHorizontalAcceleration = 1.5
    public static let maximumCourseAccuracyDegrees = 5.0

    /// Solve from a static window and one straight-line acceleration event.
    ///
    /// - Parameters:
    ///   - staticGravity: `gravity` samples with the vehicle at rest on flat ground (§5 step 1,
    ///     2 s worth).
    ///   - launchAcceleration: `userAcceleration` samples over the first 1.5 s of a launch
    ///     (§5 step 2).
    ///   - courseAccuracyDegrees: GNSS course accuracy during the event, or `nil` when the
    ///     receiver did not report one. A missing value is an absence of evidence, not
    ///     evidence that the run curved, so it does not fail the gate.
    ///   - speedIncreasing: whether GNSS speed was rising through the event, used for the sign
    ///     disambiguation of §5.
    public static func calibrate(
        staticGravity: [Vector3],
        launchAcceleration: [Vector3],
        courseAccuracyDegrees: Double?,
        speedIncreasing: Bool,
        timestamp: Double = 0
    ) throws -> VehicleFrameCalibration {
        // Gate on straightness first: it is the cheapest check and it invalidates everything
        // downstream.
        if let course = courseAccuracyDegrees, course >= 0,
           course > maximumCourseAccuracyDegrees {
            throw Error.courseAccuracyTooPoor(degrees: course)
        }

        // Step 1 — vehicle up is opposite the mean gravity direction.
        guard let meanGravity = staticGravity.mean(),
              let gravityDirection = meanGravity.normalized()
        else { throw Error.degenerate }
        let zV = -gravityDirection

        // Step 2 — forward from the horizontal part of the launch acceleration.
        guard let meanAcceleration = launchAcceleration.mean() else { throw Error.degenerate }
        let magnitude = meanAcceleration.length
        guard magnitude > 1e-9 else { throw Error.degenerate }

        let verticalComponent = meanAcceleration.dot(zV)
        let verticalFraction = abs(verticalComponent) / magnitude
        guard verticalFraction <= maximumVerticalFraction else {
            throw Error.notLevel(verticalFraction: verticalFraction)
        }

        let horizontal = meanAcceleration.removingComponent(along: zV)
        let horizontalMagnitude = horizontal.length
        guard horizontalMagnitude >= minimumHorizontalAcceleration else {
            throw Error.launchTooGentle(magnitude: horizontalMagnitude)
        }
        guard var xV = horizontal.normalized() else { throw Error.degenerate }

        // Sign disambiguation — spec §5. The projected acceleration must be positive while
        // GNSS speed is increasing; if it is not, the axis is pointing rearward.
        //
        // `horizontal` is by construction parallel to `xV`, so the projection is positive
        // whenever they agree in sign. The observable that decides it is whether the car was
        // actually speeding up during the event.
        var wasSignFlipped = false
        if !speedIncreasing {
            xV = -xV
            wasSignFlipped = true
        }

        // Step 3 — complete the basis and re-orthogonalise, exactly as §5 prescribes:
        // `y_V = z_V × x_V`, then `x_V = y_V × z_V`. This is right-handed, since
        // x × y = x × (z × x) = z(x·x) − x(x·z) = z, so `rotation` is a genuine rotation
        // with determinant +1 and `R·v_D = [v·x_V, v·y_V, v·z_V]` holds.
        guard let yV = zV.cross(xV).normalized() else { throw Error.degenerate }
        guard let xReorthogonalised = yV.cross(zV).normalized() else { throw Error.degenerate }
        xV = xReorthogonalised

        let basis = Matrix3(rows: xV, zV.cross(xV), zV)
        guard let orthonormal = basis.orthonormalised() else { throw Error.degenerate }

        return VehicleFrameCalibration(
            rotation: orthonormal,
            timestamp: timestamp,
            launchAccelerationMagnitude: horizontalMagnitude,
            verticalFraction: verticalFraction,
            wasSignFlipped: wasSignFlipped
        )
    }

    /// Spec §5: "Refine across runs by averaging `x_V` over the last N valid calibration
    /// events (use quaternion averaging, not component averaging)."
    ///
    /// Component averaging of rotation matrices does not produce a rotation, and component
    /// averaging of quaternions cancels when some are stored as `−q`. Markley's method
    /// (`Quaternion.average`) is immune to both.
    public static func refine(
        _ calibrations: [VehicleFrameCalibration],
        limit: Int = 8
    ) -> VehicleFrameCalibration? {
        guard !calibrations.isEmpty else { return nil }
        let recent = Array(calibrations.suffix(limit))
        guard let mean = Quaternion.average(recent.map(\.quaternion)) else { return nil }
        guard let rotation = mean.rotationMatrix.orthonormalised() else { return nil }

        return VehicleFrameCalibration(
            rotation: rotation,
            timestamp: recent.map(\.timestamp).max() ?? 0,
            launchAccelerationMagnitude: recent.map(\.launchAccelerationMagnitude)
                .reduce(0, +) / Double(recent.count),
            verticalFraction: recent.map(\.verticalFraction).max() ?? 0,
            wasSignFlipped: recent.last?.wasSignFlipped ?? false,
            sourceEventCount: recent.count
        )
    }
}

extension Quaternion {
    /// Recover a unit quaternion from a rotation matrix, using Shepperd's method: pick the
    /// largest of the four possible divisors so the square root is never taken of a
    /// near-zero, which is where the naive formula loses precision.
    public init(rotationMatrix m: Matrix3) {
        let trace = m[0, 0] + m[1, 1] + m[2, 2]
        if trace > 0 {
            let s = (trace + 1).squareRoot() * 2
            self.init(
                w: 0.25 * s,
                x: (m[2, 1] - m[1, 2]) / s,
                y: (m[0, 2] - m[2, 0]) / s,
                z: (m[1, 0] - m[0, 1]) / s
            )
        } else if m[0, 0] > m[1, 1], m[0, 0] > m[2, 2] {
            let s = (1 + m[0, 0] - m[1, 1] - m[2, 2]).squareRoot() * 2
            self.init(
                w: (m[2, 1] - m[1, 2]) / s,
                x: 0.25 * s,
                y: (m[0, 1] + m[1, 0]) / s,
                z: (m[0, 2] + m[2, 0]) / s
            )
        } else if m[1, 1] > m[2, 2] {
            let s = (1 + m[1, 1] - m[0, 0] - m[2, 2]).squareRoot() * 2
            self.init(
                w: (m[0, 2] - m[2, 0]) / s,
                x: (m[0, 1] + m[1, 0]) / s,
                y: 0.25 * s,
                z: (m[1, 2] + m[2, 1]) / s
            )
        } else {
            let s = (1 + m[2, 2] - m[0, 0] - m[1, 1]).squareRoot() * 2
            self.init(
                w: (m[1, 0] - m[0, 1]) / s,
                x: (m[0, 2] + m[2, 0]) / s,
                y: (m[1, 2] + m[2, 1]) / s,
                z: 0.25 * s
            )
        }
        self = normalized()
    }
}
