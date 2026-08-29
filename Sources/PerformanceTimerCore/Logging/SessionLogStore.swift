import Foundation

/// Builds CSV rows from an analysis, and rebuilds an event stream from CSV rows.
///
/// The second direction is what makes spec §15 step 2 possible: "Offline replay harness. Read
/// CSV, no UI. Everything after this is developed against recorded data."
public enum SessionLogBuilder {
    /// One row per IMU sample, with GNSS and barometer readings attached to the sample whose
    /// interval they fall in.
    public static func rows(analysis: SessionAnalysis, events: [SensorEvent]) -> [SessionLogRow] {
        var imu: [IMUSample] = []
        var fixes: [GNSSFix] = []
        var baro: [BaroSample] = []
        for event in events {
            switch event {
            case .imu(let s): imu.append(s)
            case .gnss(let f): fixes.append(f)
            case .baro(let b): baro.append(b)
            case .wheelSpeed: break
            }
        }
        imu.sort { $0.t < $1.t }
        fixes.sort { $0.t < $1.t }
        baro.sort { $0.t < $1.t }

        var rows: [SessionLogRow] = []
        rows.reserveCapacity(analysis.steps.count)

        var fixIndex = 0
        var baroIndex = 0

        for (index, step) in analysis.steps.enumerated() {
            var row = SessionLogRow(t: step.t)

            if index < imu.count {
                let sample = imu[index]
                row.acceleration = sample.userAcceleration
                row.rotationRate = sample.rotationRate
                row.gravity = sample.gravity
                row.attitude = sample.attitude
            }

            row.longitudinalAcceleration = step.acceleration
            row.forwardDistance = step.xFiltered[StateIndex.distance, 0]
            row.forwardSpeed = step.xFiltered[StateIndex.speed, 0]
            row.forwardBias = step.xFiltered[StateIndex.bias, 0]

            if index < analysis.smoothed.count {
                let smoothed = analysis.smoothed[index]
                row.smoothedDistance = smoothed.distance
                row.smoothedSpeed = smoothed.speed
                row.smoothedBias = smoothed.accelerometerBias
            }

            while fixIndex < fixes.count, fixes[fixIndex].t <= step.t {
                let fix = fixes[fixIndex]
                fixIndex += 1
                row.gnss = SessionLogRow.GNSS(
                    valid: fix.isUsableForSpeedUpdate(),
                    speed: fix.speed,
                    speedAccuracy: fix.speedAccuracy,
                    latitude: fix.latitudeDegrees,
                    longitude: fix.longitudeDegrees,
                    altitude: fix.altitude,
                    fixType: fix.fixType,
                    satelliteCount: fix.numSV
                )
            }

            while baroIndex < baro.count, baro[baroIndex].t <= step.t {
                row.barometricAltitude = baro[baroIndex].relativeAltitude
                baroIndex += 1
            }

            row.zuptActive = step.zuptApplied
            row.gateRejected = step.gateRejected
            rows.append(row)
        }
        return rows
    }

    /// Rebuild the sensor event stream from logged rows so the estimator can be re-run.
    ///
    /// The IMU specific force is reconstructed the same way spec §3.2 step 3 does it from a
    /// live sample — `f_D = (userAcceleration + gravity)` — because that is exactly what the
    /// columns store.
    public static func events(from rows: [SessionLogRow]) -> [SensorEvent] {
        var events: [SensorEvent] = []
        events.reserveCapacity(rows.count * 2)

        for row in rows {
            events.append(.imu(IMUSample(
                t: row.t,
                specificForce: row.acceleration + row.gravity,
                rotationRate: row.rotationRate,
                gravity: row.gravity,
                userAcceleration: row.acceleration,
                attitude: row.attitude
            )))

            if let gnss = row.gnss {
                events.append(.gnss(GNSSFix(
                    t: row.t,
                    speed: gnss.speed,
                    speedAccuracy: gnss.speedAccuracy,
                    latitudeDegrees: gnss.latitude,
                    longitudeDegrees: gnss.longitude,
                    altitude: gnss.altitude,
                    fixType: gnss.fixType,
                    numSV: gnss.satelliteCount,
                    gnssFixOK: gnss.valid,
                    source: .replay
                )))
            }

            if let altitude = row.barometricAltitude {
                events.append(.baro(BaroSample(t: row.t, relativeAltitude: altitude,
                                               pressure: 0)))
            }
        }
        return events
    }
}

/// Reads and writes session logs on disk.
///
/// Spec §10: "Log every session to disk in `Documents/`, always, regardless of outcome."
/// On iOS the caller passes the Documents directory; with `UIFileSharingEnabled` and
/// `LSSupportsOpeningDocumentsInPlace` set (spec §11) the files can then be pulled off through
/// the Files app with no computer involved.
public struct SessionLogStore {
    public struct SessionFiles: Equatable, Sendable {
        public var headerURL: URL
        public var csvURL: URL
    }

    public struct LoadedSession: Sendable {
        public var header: SessionLogHeader
        public var rows: [SessionLogRow]
        public var files: SessionFiles
    }

    public let directory: URL

    public init(directory: URL) {
        self.directory = directory
    }

    /// Documents directory on iOS; the current directory elsewhere.
    public static func defaultDirectory() -> URL {
        let documents = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)
        return documents.first ?? URL(fileURLWithPath: FileManager.default.currentDirectoryPath)
    }

    private func filenameStem(for header: SessionLogHeader) -> String {
        // Sortable, and unique even if two sessions start in the same second.
        let seconds = Int(header.startedAtUnixTime)
        let shortID = header.sessionID.uuidString.prefix(8)
        return "session-\(seconds)-\(shortID)"
    }

    @discardableResult
    public func write(header: SessionLogHeader, rows: [SessionLogRow]) throws -> SessionFiles {
        try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)

        let stem = filenameStem(for: header)
        let headerURL = directory.appendingPathComponent("\(stem).json")
        let csvURL = directory.appendingPathComponent("\(stem).csv")

        let encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        try encoder.encode(header).write(to: headerURL, options: .atomic)

        var writer = SessionLogCSVWriter()
        writer.append(contentsOf: rows)
        try writer.data().write(to: csvURL, options: .atomic)

        return SessionFiles(headerURL: headerURL, csvURL: csvURL)
    }

    public func read(headerURL: URL, csvURL: URL) throws -> LoadedSession {
        let header = try JSONDecoder().decode(
            SessionLogHeader.self, from: Data(contentsOf: headerURL))
        let rows = SessionLogCSVReader.rows(from: try Data(contentsOf: csvURL))
        return LoadedSession(header: header, rows: rows,
                             files: SessionFiles(headerURL: headerURL, csvURL: csvURL))
    }

    /// All sessions in the directory, newest first.
    ///
    /// A session whose header will not decode is skipped rather than throwing: one corrupt
    /// file must not make the whole log list unreadable, which is precisely when you need it.
    public func listSessions() throws -> [LoadedSession] {
        guard FileManager.default.fileExists(atPath: directory.path) else { return [] }
        let contents = try FileManager.default.contentsOfDirectory(
            at: directory, includingPropertiesForKeys: nil)

        var sessions: [LoadedSession] = []
        for headerURL in contents where headerURL.pathExtension == "json" {
            let csvURL = headerURL.deletingPathExtension().appendingPathExtension("csv")
            guard let session = try? read(headerURL: headerURL, csvURL: csvURL) else { continue }
            sessions.append(session)
        }
        return sessions.sorted { $0.header.startedAtUnixTime > $1.header.startedAtUnixTime }
    }

    public func delete(_ files: SessionFiles) throws {
        try? FileManager.default.removeItem(at: files.headerURL)
        try? FileManager.default.removeItem(at: files.csvURL)
    }
}
