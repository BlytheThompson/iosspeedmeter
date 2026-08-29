import Foundation

/// Spec §9.2 — standard distance marks.
///
/// Every value is derived from the Appendix A constants rather than typed as a decimal, so
/// there is exactly one definition of a foot and a mile in the project.
public struct DistanceMark: Equatable, Hashable, Codable, Sendable, Identifiable {
    public var meters: Double
    public var name: String
    public var shortName: String

    public var id: String { name }

    public init(meters: Double, name: String, shortName: String) {
        self.meters = meters
        self.name = name
        self.shortName = shortName
    }

    public static let rollout = DistanceMark(
        meters: PTConstants.foot, name: "Rollout", shortName: "1 ft")
    public static let sixtyFoot = DistanceMark(
        meters: 60 * PTConstants.foot, name: "60 ft", shortName: "60 ft")
    public static let threeThirtyFoot = DistanceMark(
        meters: 330 * PTConstants.foot, name: "330 ft", shortName: "330 ft")
    public static let eighthMile = DistanceMark(
        meters: PTConstants.mile / 8, name: "1/8 mile", shortName: "1/8")
    public static let thousandFoot = DistanceMark(
        meters: 1000 * PTConstants.foot, name: "1000 ft", shortName: "1000 ft")
    public static let quarterMile = DistanceMark(
        meters: PTConstants.mile / 4, name: "1/4 mile", shortName: "1/4")
    public static let halfMile = DistanceMark(
        meters: PTConstants.mile / 2, name: "1/2 mile", shortName: "1/2")
    public static let mile = DistanceMark(
        meters: PTConstants.mile, name: "1 mile", shortName: "1 mi")

    /// The marks reported on a timeslip, in order.
    public static let standard: [DistanceMark] = [
        .sixtyFoot, .threeThirtyFoot, .eighthMile, .thousandFoot, .quarterMile,
        .halfMile, .mile,
    ]
}

/// Spec §9.2 — standing-start speed marks.
public struct SpeedMark: Equatable, Hashable, Codable, Sendable, Identifiable {
    /// Target in mph. Metric marks carry their km/h value here converted, so a single
    /// representation covers both.
    public var targetMetersPerSecond: Double
    public var name: String

    public var id: String { name }

    public init(targetMetersPerSecond: Double, name: String) {
        self.targetMetersPerSecond = targetMetersPerSecond
        self.name = name
    }

    public init(targetMph: Double) {
        self.init(targetMetersPerSecond: targetMph * PTConstants.mphToMetersPerSecond,
                  name: "0–\(Int(targetMph)) mph")
    }

    public init(targetKmh: Double) {
        self.init(targetMetersPerSecond: targetKmh * PTConstants.kmhToMetersPerSecond,
                  name: "0–\(Int(targetKmh)) km/h")
    }

    public var targetMph: Double { targetMetersPerSecond / PTConstants.mphToMetersPerSecond }
    public var targetKmh: Double { targetMetersPerSecond / PTConstants.kmhToMetersPerSecond }

    public static let zeroToThirty = SpeedMark(targetMph: 30)
    public static let zeroToSixty = SpeedMark(targetMph: 60)
    public static let zeroToHundred = SpeedMark(targetMph: 100)
    public static let zeroToOneThirty = SpeedMark(targetMph: 130)

    public static let standardImperial: [SpeedMark] = [
        .zeroToThirty, .zeroToSixty, .zeroToHundred, .zeroToOneThirty,
    ]

    public static let standardMetric: [SpeedMark] = [
        SpeedMark(targetKmh: 50), SpeedMark(targetKmh: 100),
        SpeedMark(targetKmh: 160), SpeedMark(targetKmh: 200),
    ]
}

/// Spec §9.2 — roll windows, measured between two speeds rather than from rest.
///
/// Spec §8 notes these have no stationary anchor and no ZUPT, so they carry wider uncertainty
/// and are reported as a distinct confidence class.
public struct RollWindow: Equatable, Hashable, Codable, Sendable, Identifiable {
    public var fromMph: Double
    public var toMph: Double

    public var id: String { name }
    public var name: String { "\(Int(fromMph))–\(Int(toMph)) mph" }

    public init(fromMph: Double, toMph: Double) {
        self.fromMph = fromMph
        self.toMph = toMph
    }

    public var fromMetersPerSecond: Double { fromMph * PTConstants.mphToMetersPerSecond }
    public var toMetersPerSecond: Double { toMph * PTConstants.mphToMetersPerSecond }

    public static let standard: [RollWindow] = [
        RollWindow(fromMph: 40, toMph: 100),
        RollWindow(fromMph: 60, toMph: 130),
        RollWindow(fromMph: 100, toMph: 150),
        RollWindow(fromMph: 100, toMph: 200),
    ]
}
