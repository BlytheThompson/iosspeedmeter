import Foundation

/// Spec §9.4 — the coarse confidence badge.
public enum ConfidenceBadge: String, Equatable, Codable, Sendable, CaseIterable {
    case high = "High"
    case medium = "Medium"
    case low = "Low"
}

/// Spec §9.4 — "Degrade to a coarse confidence badge (High / Medium / Low) based on: number
/// of rejected GNSS fixes, mean `speedAccuracy`, GNSS update rate achieved, calibration age,
/// and grade magnitude."
///
/// The thresholds map onto the configuration table in spec §13. Note the closing instruction
/// there: "Be conservative in what the UI claims. An app that reports ±0.06 s and is right is
/// more useful than one that reports ±0.02 s and isn't." Every rule below therefore degrades
/// rather than promotes, and a run has to clear all of them to earn High.
public struct ConfidenceAssessment: Equatable, Sendable {
    /// GNSS fixes thrown out by validity rules or the 3σ gate. Spec §6.3: more than 2 means
    /// low confidence.
    public var rejectedFixCount: Int
    /// Mean reported speed accuracy across accepted fixes, m/s.
    public var meanSpeedAccuracy: Double
    /// Fixes per second actually achieved. Spec §3.3 warns Core Location commits to no rate,
    /// so this is measured, not assumed.
    public var achievedGNSSRate: Double
    /// Age of the vehicle-frame calibration when the run was recorded, seconds.
    public var calibrationAgeSeconds: Double
    /// Mean grade over the measured interval, percent.
    public var meanGradePercent: Double
    /// False when results came from the forward filter only. Spec Decision 1 means a result
    /// like that can never be a top-confidence number.
    public var usedSmoother: Bool
    /// Spec §8: a roll result has no stationary anchor and no ZUPT, so it is a distinct and
    /// wider confidence class.
    public var isRollResult: Bool

    public init(
        rejectedFixCount: Int,
        meanSpeedAccuracy: Double,
        achievedGNSSRate: Double,
        calibrationAgeSeconds: Double,
        meanGradePercent: Double,
        usedSmoother: Bool,
        isRollResult: Bool = false
    ) {
        self.rejectedFixCount = rejectedFixCount
        self.meanSpeedAccuracy = meanSpeedAccuracy
        self.achievedGNSSRate = achievedGNSSRate
        self.calibrationAgeSeconds = calibrationAgeSeconds
        self.meanGradePercent = meanGradePercent
        self.usedSmoother = usedSmoother
        self.isRollResult = isRollResult
    }

    /// Human-readable reasons the badge is not High. Shown next to the badge so the number is
    /// never mysterious.
    public var caveats: [String] {
        var reasons: [String] = []
        if !usedSmoother {
            reasons.append("Forward filter only — no backward smoothing pass")
        }
        if rejectedFixCount > 2 {
            reasons.append("\(rejectedFixCount) GNSS fixes rejected")
        } else if rejectedFixCount > 0 {
            reasons.append("\(rejectedFixCount) GNSS fix\(rejectedFixCount == 1 ? "" : "es") rejected")
        }
        if meanSpeedAccuracy > 0.3 {
            reasons.append(String(format: "Mean GNSS speed accuracy %.2f m/s", meanSpeedAccuracy))
        }
        if achievedGNSSRate < 0.9 {
            reasons.append(String(format: "GNSS rate only %.1f Hz", achievedGNSSRate))
        }
        if calibrationAgeSeconds > 30 * 24 * 3600 {
            reasons.append("Vehicle-frame calibration is over a month old")
        }
        if abs(meanGradePercent) > 1.0 {
            reasons.append(String(format: "Mean grade %.1f%%", meanGradePercent))
        }
        if isRollResult {
            reasons.append("Rolling start — no stationary anchor or ZUPT")
        }
        return reasons
    }

    public var badge: ConfidenceBadge {
        // Hard demotions to Low.
        if rejectedFixCount > 2 { return .low }
        if meanSpeedAccuracy > 0.6 { return .low }
        if achievedGNSSRate < 0.5 { return .low }

        // Anything that stops a run being pristine caps it at Medium.
        if !usedSmoother { return .medium }
        if rejectedFixCount > 0 { return .medium }
        if meanSpeedAccuracy > 0.3 { return .medium }
        if achievedGNSSRate < 0.9 { return .medium }
        if calibrationAgeSeconds > 30 * 24 * 3600 { return .medium }
        if abs(meanGradePercent) > 1.0 { return .medium }
        if isRollResult { return .medium }

        return .high
    }

    /// The error budget from spec §13, for the configuration actually in use. Presented as a
    /// range because these are budget estimates, not measurements — §12.3 is how you find out
    /// where you really landed.
    public static func expectedUncertaintyRange(
        hasSmootherAndZUPT: Bool,
        gnssRate: Double,
        hasWheelSpeed: Bool,
        hasRTK: Bool
    ) -> ClosedRange<Double> {
        if hasRTK { return 0.008...0.012 }
        if hasWheelSpeed { return 0.012...0.018 }
        if gnssRate >= 10 { return 0.02...0.03 }
        if hasSmootherAndZUPT { return 0.05...0.08 }
        return 0.10...0.15
    }
}
