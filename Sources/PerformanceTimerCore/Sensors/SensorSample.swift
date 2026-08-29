import Foundation

/// One inertial sample, already converted to SI and stamped on the session clock (spec §2.2).
///
/// `specificForce` is the reconstructed raw accelerometer reading — spec §3.2 step 3,
/// `f_D = (userAcceleration + gravity) · g`. Keeping the raw reading rather than CoreMotion's
/// `userAcceleration` is the whole point of §3.2: under sustained longitudinal acceleration
/// CoreMotion's gravity estimate tilts toward the acceleration, and `userAcceleration`
/// inherits that error. The pipeline removes gravity using its own propagated attitude
/// instead, so it needs the un-decomposed reading.
///
/// `gravity` and `userAcceleration` are retained anyway because calibration (§5) uses gravity
/// at rest, where CoreMotion is trustworthy, and the log format (§10) records both.
public struct IMUSample: Equatable, Codable, Sendable {
    /// Session time, seconds (spec §2.2).
    public var t: Double
    /// Raw specific force in the device frame, m/s². Points "down" at rest.
    public var specificForce: Vector3
    /// Bias-uncorrected angular rate in the device frame, rad/s.
    public var rotationRate: Vector3
    /// CoreMotion's gravity estimate in the device frame, m/s².
    public var gravity: Vector3
    /// CoreMotion's user acceleration in the device frame, m/s².
    public var userAcceleration: Vector3
    /// CoreMotion's attitude quaternion (device → reference).
    public var attitude: Quaternion

    public init(
        t: Double,
        specificForce: Vector3,
        rotationRate: Vector3,
        gravity: Vector3,
        userAcceleration: Vector3,
        attitude: Quaternion
    ) {
        self.t = t
        self.specificForce = specificForce
        self.rotationRate = rotationRate
        self.gravity = gravity
        self.userAcceleration = userAcceleration
        self.attitude = attitude
    }

    /// Build from CoreMotion's g-unit outputs, applying spec §3.2 step 3.
    public init(
        t: Double,
        userAccelerationG: Vector3,
        gravityG: Vector3,
        rotationRate: Vector3,
        attitude: Quaternion
    ) {
        let g = PTConstants.g
        self.init(
            t: t,
            specificForce: (userAccelerationG + gravityG) * g,
            rotationRate: rotationRate,
            gravity: gravityG * g,
            userAcceleration: userAccelerationG * g,
            attitude: attitude
        )
    }
}

/// Where a GNSS fix came from. Recorded in the log header (spec §10) and used to pick the
/// right validity rules and noise model.
public enum GNSSSource: String, Equatable, Codable, Sendable, CaseIterable {
    case coreLocation
    case externalUBX
    case externalBLE
    case replay
}

/// One GNSS fix on the session clock.
///
/// `t` is the **fix epoch**, never the callback arrival time (spec §2.3). Delivery lags the
/// fix by 100–500 ms and the lag varies; because results are post-processed, a late fix is
/// simply inserted at its own timestamp and the smoother absorbs the out-of-order arrival.
public struct GNSSFix: Equatable, Codable, Sendable {
    public var t: Double
    /// Doppler-derived ground speed, m/s. Negative means invalid (CoreLocation convention).
    public var speed: Double
    /// 1σ speed accuracy, m/s. Negative means invalid.
    public var speedAccuracy: Double
    /// Course over ground, degrees. Negative means invalid.
    public var course: Double
    /// Course accuracy, degrees. Negative means invalid.
    public var courseAccuracy: Double
    public var latitudeDegrees: Double
    public var longitudeDegrees: Double
    /// 1σ horizontal accuracy, m. Negative means invalid.
    public var horizontalAccuracy: Double
    public var altitude: Double
    /// 1σ vertical accuracy, m. Negative means invalid.
    public var verticalAccuracy: Double
    /// 0 none, 2 2D, 3 3D, 4 GNSS+DR, 5 time-only (spec §3.5). CoreLocation fixes report 3.
    public var fixType: Int
    public var numSV: Int
    public var gnssFixOK: Bool
    public var source: GNSSSource

    public init(
        t: Double,
        speed: Double,
        speedAccuracy: Double,
        course: Double = -1,
        courseAccuracy: Double = -1,
        latitudeDegrees: Double = 0,
        longitudeDegrees: Double = 0,
        horizontalAccuracy: Double = -1,
        altitude: Double = 0,
        verticalAccuracy: Double = -1,
        fixType: Int = 3,
        numSV: Int = 0,
        gnssFixOK: Bool = true,
        source: GNSSSource = .coreLocation
    ) {
        self.t = t
        self.speed = speed
        self.speedAccuracy = speedAccuracy
        self.course = course
        self.courseAccuracy = courseAccuracy
        self.latitudeDegrees = latitudeDegrees
        self.longitudeDegrees = longitudeDegrees
        self.horizontalAccuracy = horizontalAccuracy
        self.altitude = altitude
        self.verticalAccuracy = verticalAccuracy
        self.fixType = fixType
        self.numSV = numSV
        self.gnssFixOK = gnssFixOK
        self.source = source
    }

    /// Why a fix was refused, for the rejection log of spec §6.3.
    public enum Rejection: String, Equatable, Codable, Sendable {
        case negativeSpeed
        case negativeSpeedAccuracy
        case speedAccuracyTooLarge
        case fixTypeTooLow
        case fixNotOK
    }

    /// Spec §6.3: "Reject the fix if any of: speed < 0, speedAccuracy < 0,
    /// speedAccuracy > 1.0 m/s, fixType < 3, or gnssFixOK clear."
    public func rejectionReason(maximumSpeedAccuracy: Double = 1.0) -> Rejection? {
        if speed < 0 { return .negativeSpeed }
        if speedAccuracy < 0 { return .negativeSpeedAccuracy }
        if speedAccuracy > maximumSpeedAccuracy { return .speedAccuracyTooLarge }
        if fixType < 3 { return .fixTypeTooLow }
        if !gnssFixOK { return .fixNotOK }
        return nil
    }

    public func isUsableForSpeedUpdate(maximumSpeedAccuracy: Double = 1.0) -> Bool {
        rejectionReason(maximumSpeedAccuracy: maximumSpeedAccuracy) == nil
    }
}

/// Barometric relative altitude (spec §3.4). Fixed at 1 Hz; there is no interval property.
public struct BaroSample: Equatable, Codable, Sendable {
    public var t: Double
    /// Relative altitude since `startRelativeAltitudeUpdates`, m. Resolution ~0.1 m.
    public var relativeAltitude: Double
    /// Pressure, kPa.
    public var pressure: Double

    public init(t: Double, relativeAltitude: Double, pressure: Double) {
        self.t = t
        self.relativeAltitude = relativeAltitude
        self.pressure = pressure
    }
}

/// Wheel speed from CAN (spec §3.7). Measured as `k · v`, where `k` is the tyre
/// circumference scale carried as a fourth filter state.
public struct WheelSpeedSample: Equatable, Codable, Sendable {
    public var t: Double
    /// Reported wheel speed, m/s.
    public var speed: Double

    public init(t: Double, speed: Double) {
        self.t = t
        self.speed = speed
    }
}
