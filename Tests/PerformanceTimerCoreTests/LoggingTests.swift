import XCTest
@testable import PerformanceTimerCore

/// Spec §10 — data logging. "Log every session to disk in `Documents/`, always, regardless of
/// outcome. Failed runs are where you'll learn what's wrong."
final class SessionLogTests: XCTestCase {
    func testCSVColumnOrderMatchesTheSpecExactly() {
        let expected = [
            "t_session", "ax_D", "ay_D", "az_D", "gx_D", "gy_D", "gz_D",
            "grav_x", "grav_y", "grav_z", "quat_w", "quat_x", "quat_y", "quat_z",
            "a_long", "v_fwd", "s_fwd", "b_fwd", "v_smooth", "s_smooth", "b_smooth",
            "gnss_valid", "gnss_speed", "gnss_sAcc", "gnss_lat", "gnss_lon", "gnss_alt",
            "gnss_fixtype", "gnss_numsv", "baro_alt", "zupt_active", "gate_reject",
        ]
        XCTAssertEqual(SessionLogRow.columns, expected)
    }

    func testHeaderRoundTripsThroughJSON() throws {
        let header = SessionLogHeader(
            sessionID: UUID(uuidString: "1B4E28BA-2FA1-11D2-883F-0016D3CCA427")!,
            startedAtUnixTime: 1_700_000_000,
            deviceModel: "iPhone17,2",
            systemVersion: "26.0",
            appVersion: "1.0",
            activeSources: [
                SensorSourceDescriptor(identifier: "coremotion", displayName: "CoreMotion",
                                       nominalRate: 100, kind: .imu),
                SensorSourceDescriptor(identifier: "corelocation", displayName: "CoreLocation",
                                       nominalRate: 1, kind: .gnss),
            ],
            vehicleCalibration: VehicleFrameCalibration(
                rotation: .identity, timestamp: 12.5, launchAccelerationMagnitude: 4.2,
                verticalFraction: 0.02, wasSignFlipped: false),
            filterConfiguration: FilterConfiguration(),
            sessionClock: SessionClock(sessionEpoch: 100, bootWallClockUnixTime: 1_699_999_900),
            clockFit: ClockFit.Solution(slope: 0.001, intercept: -3600, residualRMS: 0.004,
                                        sampleCount: 42),
            notes: "bench"
        )

        let data = try JSONEncoder().encode(header)
        let decoded = try JSONDecoder().decode(SessionLogHeader.self, from: data)

        XCTAssertEqual(decoded.sessionID, header.sessionID)
        XCTAssertEqual(decoded.deviceModel, "iPhone17,2")
        XCTAssertEqual(decoded.systemVersion, "26.0")
        XCTAssertEqual(decoded.activeSources.count, 2)
        XCTAssertEqual(decoded.vehicleCalibration?.launchAccelerationMagnitude, 4.2)
        XCTAssertEqual(decoded.filterConfiguration.sigmaA, 0.05)
        XCTAssertEqual(decoded.filterConfiguration.processNoiseModel, .exact)
        XCTAssertEqual(decoded.clockFit?.residualRMS, 0.004)
        XCTAssertEqual(decoded.sessionClock.bootWallClockUnixTime, 1_699_999_900)
    }

    func testHeaderRecordsTheRotationMatrixSoARunCanBeReanalysed() {
        // Spec §10 lists R_DV among the header fields — without it a logged run cannot be
        // re-projected offline.
        let rotation = Matrix3(rows: Vector3(0, 1, 0), Vector3(-1, 0, 0), Vector3(0, 0, 1))
        let header = SessionLogHeader(
            sessionID: UUID(), startedAtUnixTime: 0, deviceModel: "x", systemVersion: "y",
            appVersion: "z", activeSources: [],
            vehicleCalibration: VehicleFrameCalibration(
                rotation: rotation, timestamp: 0, launchAccelerationMagnitude: 5,
                verticalFraction: 0, wasSignFlipped: false),
            filterConfiguration: FilterConfiguration(),
            sessionClock: SessionClock(sessionEpoch: 0, bootWallClockUnixTime: 0),
            clockFit: nil, notes: nil)
        let decoded = try! JSONDecoder().decode(
            SessionLogHeader.self, from: try! JSONEncoder().encode(header))
        XCTAssertEqual(decoded.vehicleCalibration!.rotation.row(0).y, 1, accuracy: 1e-12)
        XCTAssertEqual(decoded.vehicleCalibration!.rotation.row(1).x, -1, accuracy: 1e-12)
    }

    func testRowRoundTripsThroughCSV() {
        var row = SessionLogRow(t: 1.23)
        row.acceleration = Vector3(0.1, -0.2, 0.3)
        row.rotationRate = Vector3(0.01, 0.02, -0.03)
        row.gravity = Vector3(0, 0, -9.80665)
        row.attitude = Quaternion(w: 0.5, x: 0.5, y: 0.5, z: 0.5)
        row.longitudinalAcceleration = 4.5
        row.forwardSpeed = 12.5
        row.forwardDistance = 30.25
        row.forwardBias = -0.03
        row.smoothedSpeed = 12.48
        row.smoothedDistance = 30.2
        row.smoothedBias = -0.031
        row.gnss = SessionLogRow.GNSS(valid: true, speed: 12.4, speedAccuracy: 0.05,
                                      latitude: 37.4275, longitude: -122.1697, altitude: 30.5,
                                      fixType: 3, satelliteCount: 14)
        row.barometricAltitude = 1.75
        row.zuptActive = true
        row.gateRejected = false

        let line = row.csvLine()
        let parsed = SessionLogRow(csvLine: line)!

        XCTAssertEqual(parsed.t, 1.23, accuracy: 1e-9)
        XCTAssertEqual(parsed.acceleration.x, 0.1, accuracy: 1e-9)
        XCTAssertEqual(parsed.rotationRate.z, -0.03, accuracy: 1e-9)
        XCTAssertEqual(parsed.gravity.z, -9.80665, accuracy: 1e-9)
        XCTAssertEqual(parsed.attitude.w, 0.5, accuracy: 1e-9)
        XCTAssertEqual(parsed.longitudinalAcceleration, 4.5, accuracy: 1e-9)
        XCTAssertEqual(parsed.smoothedDistance, 30.2, accuracy: 1e-9)
        XCTAssertEqual(parsed.gnss?.speed, 12.4)
        XCTAssertEqual(parsed.gnss?.satelliteCount, 14)
        XCTAssertEqual(parsed.barometricAltitude, 1.75)
        XCTAssertTrue(parsed.zuptActive)
        XCTAssertFalse(parsed.gateRejected)
    }

    func testRowWithoutGNSSMarksTheFixInvalidAndLeavesFieldsEmpty() {
        let row = SessionLogRow(t: 0.5)
        let fields = row.csvLine().components(separatedBy: ",")
        let validIndex = SessionLogRow.columns.firstIndex(of: "gnss_valid")!
        XCTAssertEqual(fields[validIndex], "0")
        let speedIndex = SessionLogRow.columns.firstIndex(of: "gnss_speed")!
        XCTAssertEqual(fields[speedIndex], "")

        let parsed = SessionLogRow(csvLine: row.csvLine())!
        XCTAssertNil(parsed.gnss)
    }

    func testMalformedLineIsRejectedRatherThanPartiallyParsed() {
        XCTAssertNil(SessionLogRow(csvLine: "1.0,2.0,3.0"))
        XCTAssertNil(SessionLogRow(csvLine: ""))
    }

    func testWriterEmitsHeaderLineThenRows() {
        var writer = SessionLogCSVWriter()
        writer.append(SessionLogRow(t: 0.0))
        writer.append(SessionLogRow(t: 0.01))
        let text = writer.text()
        let lines = text.split(separator: "\n", omittingEmptySubsequences: false)
        XCTAssertEqual(String(lines[0]), SessionLogRow.columns.joined(separator: ","))
        XCTAssertEqual(lines.count, 4)          // header + 2 rows + trailing newline
    }

    func testRowsAreBuiltFromAnAnalysisWithForwardAndSmoothedColumnsPopulated() {
        var session = SyntheticSession()
        session.runDuration = 6
        let processor = SessionProcessor()
        let events = session.events()
        processor.ingest(events)
        let analysis = processor.analyse()

        let rows = SessionLogBuilder.rows(analysis: analysis, events: events)
        XCTAssertEqual(rows.count, analysis.steps.count)

        // Forward and smoothed columns must both be present — spec §10 lists both, and the
        // whole point of the log is being able to compare them offline.
        let mid = rows[rows.count / 2]
        XCTAssertNotEqual(mid.forwardSpeed, 0)
        XCTAssertNotEqual(mid.smoothedSpeed, 0)
        XCTAssertTrue(rows.contains { $0.gnss != nil })
        XCTAssertTrue(rows.contains { $0.zuptActive })
        XCTAssertTrue(rows.contains { $0.barometricAltitude != nil })
    }

    /// Spec §10: "Build an offline replay mode that reads a CSV and reruns the whole
    /// estimator. You will retune `Q` and `R` a dozen times and you do not want to drive for
    /// each iteration."
    func testReplayingALoggedSessionReproducesTheSameResult() {
        var session = SyntheticSession()
        session.runDuration = 12
        let events = session.events()

        let original = SessionProcessor()
        original.ingest(events)
        let originalAnalysis = original.analyse()
        let originalZeroSixty = originalAnalysis.results!.speedResult(for: .zeroToSixty)!.elapsed

        // Round-trip through the CSV.
        let rows = SessionLogBuilder.rows(analysis: originalAnalysis, events: events)
        var writer = SessionLogCSVWriter()
        for row in rows { writer.append(row) }
        let csv = writer.text()

        let replayedRows = SessionLogCSVReader.rows(from: csv)
        XCTAssertEqual(replayedRows.count, rows.count)

        let replayedEvents = SessionLogBuilder.events(from: replayedRows)
        let replayed = SessionProcessor()
        replayed.ingest(replayedEvents)
        let replayedAnalysis = replayed.analyse()
        let replayedZeroSixty = replayedAnalysis.results!.speedResult(for: .zeroToSixty)!.elapsed

        // Six significant figures of CSV keep the replay well inside a millisecond.
        XCTAssertEqual(replayedZeroSixty, originalZeroSixty, accuracy: 0.001)
    }

    func testReplayHonoursADifferentFilterConfigurationSoTuningIsPossible() {
        var session = SyntheticSession()
        session.runDuration = 10
        let events = session.events()

        let baseline = SessionProcessor()
        baseline.ingest(events)
        let baselineAnalysis = baseline.analyse()

        // Re-run the same recorded data with the published (broken) Q, which is exactly the
        // comparison the replay harness exists to make cheap.
        var tuned = FilterConfiguration()
        tuned.processNoiseModel = .specLiteral
        let retuned = SessionProcessor(
            configuration: SessionProcessor.Configuration(filter: tuned))
        retuned.ingest(events)
        let retunedAnalysis = retuned.analyse()

        let baselineSpeed = baselineAnalysis.smoothed.last!.speed
        let retunedSpeed = retunedAnalysis.smoothed.last!.speed
        XCTAssertNotEqual(baselineSpeed, retunedSpeed, accuracy: 1e-9)
    }

    func testWritingAndReadingRealFilesRoundTrips() throws {
        let directory = FileManager.default.temporaryDirectory
            .appendingPathComponent("pt-log-tests-\(UUID().uuidString)")
        defer { try? FileManager.default.removeItem(at: directory) }

        let header = SessionLogHeader(
            sessionID: UUID(), startedAtUnixTime: 1_700_000_000, deviceModel: "iPhone17,2",
            systemVersion: "26.0", appVersion: "1.0", activeSources: [],
            vehicleCalibration: nil, filterConfiguration: FilterConfiguration(),
            sessionClock: SessionClock(sessionEpoch: 0, bootWallClockUnixTime: 0),
            clockFit: nil, notes: nil)

        let store = SessionLogStore(directory: directory)
        let rows = (0..<50).map { SessionLogRow(t: Double($0) * 0.01) }
        let urls = try store.write(header: header, rows: rows)

        XCTAssertTrue(FileManager.default.fileExists(atPath: urls.headerURL.path))
        XCTAssertTrue(FileManager.default.fileExists(atPath: urls.csvURL.path))

        let loaded = try store.read(headerURL: urls.headerURL, csvURL: urls.csvURL)
        XCTAssertEqual(loaded.header.sessionID, header.sessionID)
        XCTAssertEqual(loaded.rows.count, 50)
        XCTAssertEqual(loaded.rows[10].t, 0.10, accuracy: 1e-9)
    }

    func testStoreListsSessionsNewestFirst() throws {
        let directory = FileManager.default.temporaryDirectory
            .appendingPathComponent("pt-log-list-\(UUID().uuidString)")
        defer { try? FileManager.default.removeItem(at: directory) }
        let store = SessionLogStore(directory: directory)

        for i in 0..<3 {
            let header = SessionLogHeader(
                sessionID: UUID(), startedAtUnixTime: Double(1_700_000_000 + i * 60),
                deviceModel: "d", systemVersion: "v", appVersion: "a", activeSources: [],
                vehicleCalibration: nil, filterConfiguration: FilterConfiguration(),
                sessionClock: SessionClock(sessionEpoch: 0, bootWallClockUnixTime: 0),
                clockFit: nil, notes: nil)
            _ = try store.write(header: header, rows: [SessionLogRow(t: 0)])
        }

        let sessions = try store.listSessions()
        XCTAssertEqual(sessions.count, 3)
        XCTAssertGreaterThan(sessions[0].header.startedAtUnixTime,
                             sessions[2].header.startedAtUnixTime)
    }
}
