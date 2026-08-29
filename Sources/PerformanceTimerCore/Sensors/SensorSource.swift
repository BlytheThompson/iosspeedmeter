import Foundation

/// Anything a source can emit.
public enum SensorEvent: Equatable, Sendable {
    case imu(IMUSample)
    case gnss(GNSSFix)
    case baro(BaroSample)
    case wheelSpeed(WheelSpeedSample)

    /// Session time of the event, used to merge streams into one ordered timeline.
    public var t: Double {
        switch self {
        case .imu(let s): return s.t
        case .gnss(let s): return s.t
        case .baro(let s): return s.t
        case .wheelSpeed(let s): return s.t
        }
    }
}

public struct SensorSourceDescriptor: Equatable, Codable, Sendable {
    /// Stable identifier recorded in the log header (spec §10).
    public var identifier: String
    /// Human-readable name for the UI.
    public var displayName: String
    /// Nominal update rate, Hz. Spec §3.3 warns that Core Location does not commit to a rate,
    /// so this is only a hint — the estimator scales `R` from the *observed* interval.
    public var nominalRate: Double
    public var kind: Kind

    public enum Kind: String, Equatable, Codable, Sendable {
        case imu, gnss, barometer, wheelSpeed
    }

    public init(identifier: String, displayName: String, nominalRate: Double, kind: Kind) {
        self.identifier = identifier
        self.displayName = displayName
        self.nominalRate = nominalRate
        self.kind = kind
    }
}

/// Spec Decision 2 — "sensors are abstracted behind one protocol".
///
/// Internal GNSS at 1 Hz, an external 25 Hz receiver over Wi-Fi, and a CAN wheel-speed channel
/// all feed the same estimator through this interface. The estimator is built against this,
/// never against `CLLocationManager`, which is what makes a hardware upgrade a matter of
/// adding a conformance rather than touching the maths — and what lets the entire numerical
/// core be tested off-device against a replayed or synthetic source.
public protocol SensorSource: AnyObject {
    var descriptor: SensorSourceDescriptor { get }
    var isRunning: Bool { get }

    /// Begin delivering events. The handler may be called on any queue; implementations must
    /// document theirs. Events must carry session-clock timestamps (spec §2.2).
    func start(handler: @escaping (SensorEvent) -> Void) throws
    func stop()
}

/// Errors a source may raise while starting.
public enum SensorSourceError: Error, Equatable {
    case unavailable(String)
    case permissionDenied(String)
    case alreadyRunning
    case connectionFailed(String)
}

/// An in-memory source that replays a fixed list of events.
///
/// This is what makes spec §15 step 2 ("offline replay harness … everything after this is
/// developed against recorded data") and the whole test suite possible: the estimator cannot
/// tell it apart from a real device.
public final class ArraySensorSource: SensorSource, @unchecked Sendable {
    public let descriptor: SensorSourceDescriptor
    public private(set) var isRunning = false
    private let events: [SensorEvent]

    public init(events: [SensorEvent], descriptor: SensorSourceDescriptor? = nil) {
        // Merge-sorted on session time so the estimator sees one coherent timeline.
        self.events = events.sorted { $0.t < $1.t }
        self.descriptor = descriptor ?? SensorSourceDescriptor(
            identifier: "replay.array",
            displayName: "Recorded session",
            nominalRate: 0,
            kind: .imu
        )
    }

    public func start(handler: @escaping (SensorEvent) -> Void) throws {
        guard !isRunning else { throw SensorSourceError.alreadyRunning }
        isRunning = true
        for event in events {
            guard isRunning else { break }
            handler(event)
        }
        isRunning = false
    }

    public func stop() { isRunning = false }
}
