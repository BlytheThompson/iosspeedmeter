import Foundation

/// Spec Appendix A — Constants.
///
/// These are exact definitions, not measurements, so they are written as literals and
/// compared with zero tolerance in the tests. `g` is standard gravity as used by CoreMotion
/// when converting its g-unit outputs to m/s².
public enum PTConstants {
    /// Standard gravity, m/s².
    public static let g = 9.80665

    // MARK: - Unit conversions

    public static let mphToMetersPerSecond = 0.44704
    public static let knotsToMetersPerSecond = 0.514444
    public static let kmhToMetersPerSecond = 0.277778

    /// International foot, m.
    public static let foot = 0.3048
    /// International mile, m.
    public static let mile = 1609.344

    // MARK: - WGS84

    public static let wgs84SemiMajorAxis = 6378137.0
    public static let wgs84Flattening = 1.0 / 298.257223563
    /// e² = 2f − f²
    public static let wgs84EccentricitySquared =
        2.0 * wgs84Flattening - wgs84Flattening * wgs84Flattening

    // MARK: - Derived helpers

    public static func mph(fromMetersPerSecond v: Double) -> Double {
        v / mphToMetersPerSecond
    }

    public static func metersPerSecond(fromMph v: Double) -> Double {
        v * mphToMetersPerSecond
    }

    public static func kmh(fromMetersPerSecond v: Double) -> Double {
        v / kmhToMetersPerSecond
    }

    public static func metersPerSecond(fromKmh v: Double) -> Double {
        v * kmhToMetersPerSecond
    }
}
