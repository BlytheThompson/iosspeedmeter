import Foundation

/// Spec §2.2 — the single monotonic session clock.
///
/// Three sources stamp their samples three different ways: CoreMotion uses seconds since
/// boot, CoreLocation uses wall-clock `Date`, and an external receiver carries GPS time of
/// week. All three must land on one timeline before any filtering happens, or the fusion
/// silently loses 50–100 ms (spec §2).
///
/// Both reference values are captured **once** at session start. `systemUptime` and `Date()`
/// drift relative to one another, so recomputing `bootWallClockUnixTime` per sample would
/// inject jitter directly into the measurement times — that is the trap §2.2 warns about.
///
/// This type is deliberately free of CoreFoundation: the caller passes in
/// `CACurrentMediaTime()` and `ProcessInfo.processInfo.systemUptime`, which keeps the whole
/// numerical core testable off-device.
public struct SessionClock: Equatable, Codable, Sendable {
    /// Value of the monotonic clock (`CACurrentMediaTime()`) at session start.
    public let sessionEpoch: Double
    /// Unix time corresponding to device boot, i.e. `now - systemUptime`.
    public let bootWallClockUnixTime: Double

    public init(sessionEpoch: Double, bootWallClockUnixTime: Double) {
        self.sessionEpoch = sessionEpoch
        self.bootWallClockUnixTime = bootWallClockUnixTime
    }

    /// Convenience matching the spec's capture snippet:
    /// ```
    /// sessionEpoch  = CACurrentMediaTime()
    /// bootWallClock = Date(timeIntervalSinceNow: -systemUptime)
    /// ```
    public init(sessionEpoch: Double, currentWallClockUnixTime: Double, currentUptime: Double) {
        self.sessionEpoch = sessionEpoch
        self.bootWallClockUnixTime = currentWallClockUnixTime - currentUptime
    }

    /// For a boot-relative monotonic stamp — `CMDeviceMotion.timestamp`,
    /// `CMAltitudeData.timestamp`, `CACurrentMediaTime()`.
    public func sessionTime(monotonicTimestamp: Double) -> Double {
        monotonicTimestamp - sessionEpoch
    }

    /// For a wall-clock stamp — `CLLocation.timestamp` as seconds since 1970.
    ///
    /// This is the **fix epoch**, not the delivery time. Never pass the callback's arrival
    /// time here (spec §2.3): doing so is worth 0.1–0.3 s of error on a 0–60.
    public func sessionTime(wallClockUnixTime: Double) -> Double {
        (wallClockUnixTime - bootWallClockUnixTime) - sessionEpoch
    }

    /// Inverse of `sessionTime(monotonicTimestamp:)`, for writing timestamps back out.
    public func monotonicTimestamp(sessionTime: Double) -> Double {
        sessionTime + sessionEpoch
    }

    /// Inverse of `sessionTime(wallClockUnixTime:)`, for log headers and export.
    public func wallClockUnixTime(sessionTime: Double) -> Double {
        sessionTime + sessionEpoch + bootWallClockUnixTime
    }
}
