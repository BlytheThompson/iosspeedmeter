import Foundation

/// Spec §10 — the JSON header written alongside every session's CSV.
///
/// "Header: device model, iOS version, session UUID, sensor sources active, `R_DV`,
/// calibration timestamp, filter parameters, clock-fit residuals."
///
/// Everything here exists so a log can be fully re-analysed months later without the device
/// that produced it: the rotation matrix lets the projection be redone, the filter parameters
/// say what produced the recorded forward columns, and the clock fit records how good the
/// external receiver's time alignment was.
public struct SessionLogHeader: Codable, Equatable, Sendable {
    public var sessionID: UUID
    public var startedAtUnixTime: Double
    public var deviceModel: String
    public var systemVersion: String
    public var appVersion: String
    public var activeSources: [SensorSourceDescriptor]
    /// `R_DV` and its provenance (spec §5).
    public var vehicleCalibration: VehicleFrameCalibration?
    public var filterConfiguration: FilterConfiguration
    public var sessionClock: SessionClock
    /// Spec §2.4 — the external receiver clock fit and, crucially, its residual scatter.
    public var clockFit: ClockFit.Solution?
    public var notes: String?

    public init(
        sessionID: UUID,
        startedAtUnixTime: Double,
        deviceModel: String,
        systemVersion: String,
        appVersion: String,
        activeSources: [SensorSourceDescriptor],
        vehicleCalibration: VehicleFrameCalibration?,
        filterConfiguration: FilterConfiguration,
        sessionClock: SessionClock,
        clockFit: ClockFit.Solution?,
        notes: String?
    ) {
        self.sessionID = sessionID
        self.startedAtUnixTime = startedAtUnixTime
        self.deviceModel = deviceModel
        self.systemVersion = systemVersion
        self.appVersion = appVersion
        self.activeSources = activeSources
        self.vehicleCalibration = vehicleCalibration
        self.filterConfiguration = filterConfiguration
        self.sessionClock = sessionClock
        self.clockFit = clockFit
        self.notes = notes
    }
}

/// One CSV row — one IMU sample, plus whatever else landed at that instant.
///
/// The column list is verbatim from spec §10 and its order is asserted by the tests, because
/// any offline analysis script written against these logs depends on it.
public struct SessionLogRow: Equatable, Sendable {
    public struct GNSS: Equatable, Sendable {
        public var valid: Bool
        public var speed: Double
        public var speedAccuracy: Double
        public var latitude: Double
        public var longitude: Double
        public var altitude: Double
        public var fixType: Int
        public var satelliteCount: Int

        public init(
            valid: Bool, speed: Double, speedAccuracy: Double, latitude: Double,
            longitude: Double, altitude: Double, fixType: Int, satelliteCount: Int
        ) {
            self.valid = valid
            self.speed = speed
            self.speedAccuracy = speedAccuracy
            self.latitude = latitude
            self.longitude = longitude
            self.altitude = altitude
            self.fixType = fixType
            self.satelliteCount = satelliteCount
        }
    }

    public var t: Double
    /// `ax_D, ay_D, az_D` — user acceleration in the device frame, m/s².
    public var acceleration: Vector3 = .zero
    /// `gx_D, gy_D, gz_D` — gyro in the device frame, rad/s.
    public var rotationRate: Vector3 = .zero
    public var gravity: Vector3 = .zero
    public var attitude: Quaternion = .identity
    /// `a_long` — resolved longitudinal specific force, m/s².
    public var longitudinalAcceleration: Double = 0
    public var forwardSpeed: Double = 0
    public var forwardDistance: Double = 0
    public var forwardBias: Double = 0
    public var smoothedSpeed: Double = 0
    public var smoothedDistance: Double = 0
    public var smoothedBias: Double = 0
    public var gnss: GNSS?
    public var barometricAltitude: Double?
    public var zuptActive: Bool = false
    public var gateRejected: Bool = false

    public init(t: Double) {
        self.t = t
    }

    /// Spec §10 column list, in order.
    public static let columns: [String] = [
        "t_session", "ax_D", "ay_D", "az_D", "gx_D", "gy_D", "gz_D",
        "grav_x", "grav_y", "grav_z", "quat_w", "quat_x", "quat_y", "quat_z",
        "a_long", "v_fwd", "s_fwd", "b_fwd", "v_smooth", "s_smooth", "b_smooth",
        "gnss_valid", "gnss_speed", "gnss_sAcc", "gnss_lat", "gnss_lon", "gnss_alt",
        "gnss_fixtype", "gnss_numsv", "baro_alt", "zupt_active", "gate_reject",
    ]

    /// Nine significant figures: enough that a replay reproduces results to well under a
    /// millisecond, without doubling the file size for digits that are pure sensor noise.
    private static func format(_ value: Double) -> String {
        value.isFinite ? String(format: "%.9g", value) : ""
    }

    public func csvLine() -> String {
        var fields: [String] = []
        fields.reserveCapacity(Self.columns.count)

        fields.append(Self.format(t))
        for v in [acceleration.x, acceleration.y, acceleration.z] { fields.append(Self.format(v)) }
        for v in [rotationRate.x, rotationRate.y, rotationRate.z] { fields.append(Self.format(v)) }
        for v in [gravity.x, gravity.y, gravity.z] { fields.append(Self.format(v)) }
        for v in [attitude.w, attitude.x, attitude.y, attitude.z] { fields.append(Self.format(v)) }
        for v in [longitudinalAcceleration, forwardSpeed, forwardDistance, forwardBias,
                  smoothedSpeed, smoothedDistance, smoothedBias] {
            fields.append(Self.format(v))
        }

        if let gnss {
            fields.append(gnss.valid ? "1" : "0")
            for v in [gnss.speed, gnss.speedAccuracy, gnss.latitude, gnss.longitude,
                      gnss.altitude] {
                fields.append(Self.format(v))
            }
            fields.append(String(gnss.fixType))
            fields.append(String(gnss.satelliteCount))
        } else {
            fields.append("0")
            fields.append(contentsOf: [String](repeating: "", count: 7))
        }

        fields.append(barometricAltitude.map(Self.format) ?? "")
        fields.append(zuptActive ? "1" : "0")
        fields.append(gateRejected ? "1" : "0")

        return fields.joined(separator: ",")
    }

    /// Parse one line. Returns `nil` for anything that is not a complete row, so a truncated
    /// final line — the normal result of a crash mid-write — is skipped rather than
    /// half-decoded into plausible-looking numbers.
    public init?(csvLine line: String) {
        let fields = line.split(separator: ",", omittingEmptySubsequences: false).map(String.init)
        guard fields.count == Self.columns.count else { return nil }

        func number(_ index: Int) -> Double? {
            let text = fields[index].trimmingCharacters(in: .whitespaces)
            return text.isEmpty ? nil : Double(text)
        }
        guard let t = number(0) else { return nil }

        self.init(t: t)
        acceleration = Vector3(number(1) ?? 0, number(2) ?? 0, number(3) ?? 0)
        rotationRate = Vector3(number(4) ?? 0, number(5) ?? 0, number(6) ?? 0)
        gravity = Vector3(number(7) ?? 0, number(8) ?? 0, number(9) ?? 0)
        attitude = Quaternion(w: number(10) ?? 1, x: number(11) ?? 0,
                              y: number(12) ?? 0, z: number(13) ?? 0)
        longitudinalAcceleration = number(14) ?? 0
        forwardSpeed = number(15) ?? 0
        forwardDistance = number(16) ?? 0
        forwardBias = number(17) ?? 0
        smoothedSpeed = number(18) ?? 0
        smoothedDistance = number(19) ?? 0
        smoothedBias = number(20) ?? 0

        let gnssValid = (number(21) ?? 0) != 0
        if let speed = number(22) {
            gnss = GNSS(
                valid: gnssValid,
                speed: speed,
                speedAccuracy: number(23) ?? -1,
                latitude: number(24) ?? 0,
                longitude: number(25) ?? 0,
                altitude: number(26) ?? 0,
                fixType: Int(number(27) ?? 0),
                satelliteCount: Int(number(28) ?? 0)
            )
        }
        barometricAltitude = number(29)
        zuptActive = (number(30) ?? 0) != 0
        gateRejected = (number(31) ?? 0) != 0
    }
}

/// Accumulates CSV text, header line first.
public struct SessionLogCSVWriter: Sendable {
    private var lines: [String]

    public init() {
        lines = [SessionLogRow.columns.joined(separator: ",")]
    }

    public mutating func append(_ row: SessionLogRow) {
        lines.append(row.csvLine())
    }

    public mutating func append(contentsOf rows: [SessionLogRow]) {
        for row in rows { append(row) }
    }

    public var rowCount: Int { lines.count - 1 }

    public func text() -> String {
        lines.joined(separator: "\n") + "\n"
    }

    public func data() -> Data {
        Data(text().utf8)
    }
}

public enum SessionLogCSVReader {
    /// Parse CSV text into rows, skipping the header line and any malformed trailing line.
    public static func rows(from text: String) -> [SessionLogRow] {
        text
            .split(separator: "\n", omittingEmptySubsequences: true)
            .dropFirst()
            .compactMap { SessionLogRow(csvLine: String($0)) }
    }

    public static func rows(from data: Data) -> [SessionLogRow] {
        rows(from: String(decoding: data, as: UTF8.self))
    }
}
