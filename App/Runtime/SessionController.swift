#if canImport(SwiftUI)
import Foundation
import QuartzCore
import SwiftUI
import PerformanceTimerCore

#if canImport(UIKit)
import UIKit
#endif

/// Wires sensor sources to the live estimator, and the finished session to the post-processor.
///
/// The division of labour follows spec Decision 1 exactly: `LiveEstimator` drives what is on
/// screen while driving, and `SessionProcessor` — running after the run ends, over the full
/// recorded session — produces every number that is ever called a result.
@MainActor
public final class SessionController: ObservableObject {
    @Published public private(set) var readout = LiveEstimator.Readout(
        state: .idle, speed: 0, distance: 0, accelerometerBias: 0,
        longitudinalAcceleration: 0, isStationary: false, hasGNSSLock: false,
        calibrationValid: false, stopReason: nil, t: 0
    )
    @Published public private(set) var analysis: SessionAnalysis?
    @Published public private(set) var isAnalysing = false
    @Published public private(set) var lastError: String?
    @Published public private(set) var externalGNSSState: String?
    @Published public var settings = AppSettings.load()

    /// Sessions already written to disk (spec §10).
    @Published public private(set) var savedSessions: [SessionLogStore.LoadedSession] = []

    private var estimator: LiveEstimator?
    private var sources: [SensorSource] = []
    private var clock: SessionClock?
    private var sessionID = UUID()
    private var clockFitSolution: ClockFit.Solution?

    private let store: SessionLogStore

    public init(store: SessionLogStore = SessionLogStore(directory: SessionLogStore.defaultDirectory())) {
        self.store = store
        refreshSessions()
    }

    public var state: SessionState { readout.state }

    /// True once `startSession()` has run, so the start screen knows to step aside.
    public private(set) var hasStartedSession = false

    /// Which of the three ARMED preconditions are met (spec §4). Drives the staging tree.
    public var armingConditions: (lock: Bool, calibration: Bool, still: Bool) {
        (readout.hasGNSSLock, readout.calibrationValid, readout.isStationary)
    }

    // MARK: - Session lifecycle

    public func startSession() {
        stopSession()
        hasStartedSession = true
        lastError = nil
        analysis = nil
        sessionID = UUID()

        // Spec §2.2 — capture both clock references once, at session start.
        let clock = SessionClock(
            sessionEpoch: CACurrentMediaTime(),
            currentWallClockUnixTime: Date().timeIntervalSince1970,
            currentUptime: ProcessInfo.processInfo.systemUptime
        )
        self.clock = clock

        var configuration = SessionProcessor.Configuration(
            filter: settings.filterConfiguration,
            vehicleCalibration: settings.storedCalibration
        )
        configuration.filter.wheelSpeedEnabled = settings.wheelSpeedEnabled
        let estimator = LiveEstimator(configuration: configuration)
        self.estimator = estimator

        // Spec §11: keep the screen alive while ARMED or RECORDING. There are field reports of
        // iOS dropping the Wi-Fi association to a network with no internet route when the
        // screen locks, which would silently kill an external receiver mid-run.
        #if canImport(UIKit)
        UIApplication.shared.isIdleTimerDisabled = true
        #endif

        var started: [SensorSource] = []

        #if canImport(CoreMotion)
        let motion = CoreMotionSource(clock: clock)
        let altimeter = AltimeterSource(clock: clock)
        started.append(motion)
        started.append(altimeter)
        #endif

        #if canImport(CoreLocation)
        if settings.gnssSource == .internalGNSS {
            started.append(CoreLocationSource(clock: clock,
                                              activityMode: settings.activityMode))
        }
        #endif

        #if canImport(Network)
        if settings.gnssSource == .externalWiFi {
            let external = ExternalGNSSWiFiSource(host: settings.externalHost,
                                                  port: settings.externalPort,
                                                  clock: clock)
            external.onStateChange = { [weak self] state in
                Task { @MainActor in
                    self?.externalGNSSState = String(describing: state)
                    self?.clockFitSolution = external.clockFitSolution
                }
            }
            started.append(external)
        }
        #endif

        #if canImport(CoreBluetooth)
        if settings.gnssSource == .externalBLE {
            started.append(RaceChronoBLESource(clock: clock))
        }
        #endif

        sources = started
        for source in started {
            do {
                try source.start { [weak self] event in
                    Task { @MainActor in self?.handle(event) }
                }
            } catch {
                lastError = "\(source.descriptor.displayName): \(error)"
            }
        }
    }

    public func stopSession() {
        for source in sources { source.stop() }
        sources.removeAll()
        #if canImport(UIKit)
        UIApplication.shared.isIdleTimerDisabled = false
        #endif
    }

    /// Ask the run to end now. The analysis still runs over everything recorded.
    public func requestStop() {
        estimator?.requestStop()
    }

    /// Discard the current run and go back to arming.
    public func rearm() {
        analysis = nil
        estimator?.reset()
        readout = LiveEstimator.Readout(
            state: .idle, speed: 0, distance: 0, accelerometerBias: 0,
            longitudinalAcceleration: 0, isStationary: false, hasGNSSLock: false,
            calibrationValid: false, stopReason: nil, t: 0)
    }

    private func handle(_ event: SensorEvent) {
        guard let estimator else { return }
        let previousState = estimator.state
        estimator.ingest(event)
        readout = estimator.readout

        if previousState != .complete, estimator.state == .complete {
            finishRun()
        }
    }

    // MARK: - Post-processing

    private func finishRun() {
        guard let estimator, let clock else { return }
        isAnalysing = true
        estimator.beginAnalysis()

        let processor = estimator.makeProcessor()
        let events = estimator.capturedEvents
        let header = makeHeader(clock: clock)
        let store = self.store

        // The smoother is O(n) over a few thousand samples — milliseconds — but it runs off
        // the main actor anyway so a long session never stutters the UI.
        Task.detached(priority: .userInitiated) {
            let result = processor.analyse()
            let rows = SessionLogBuilder.rows(analysis: result, events: events)

            // Spec §10: log every session, always, regardless of outcome. A failure here must
            // be visible — a silently missing log is exactly the run you would later want to
            // read, and swallowing the error would make the app quietly lossy.
            var writeError: String?
            do {
                _ = try store.write(header: header, rows: rows)
            } catch {
                writeError = "Could not save the session log: \(error.localizedDescription)"
            }

            await MainActor.run {
                self.analysis = result
                self.isAnalysing = false
                if let writeError { self.lastError = writeError }
                estimator.finishAnalysis()
                self.readout = estimator.readout
                self.refreshSessions()
                if let solved = result.vehicleCalibration, !result.diagnostics.usedProvisionalVehicleFrame {
                    self.settings.storeCalibration(solved)
                }
            }
        }
    }

    private func makeHeader(clock: SessionClock) -> SessionLogHeader {
        SessionLogHeader(
            sessionID: sessionID,
            startedAtUnixTime: clock.wallClockUnixTime(sessionTime: 0),
            deviceModel: DeviceInfo.model,
            systemVersion: DeviceInfo.systemVersion,
            appVersion: DeviceInfo.appVersion,
            activeSources: sources.map(\.descriptor),
            vehicleCalibration: settings.storedCalibration,
            filterConfiguration: settings.filterConfiguration,
            sessionClock: clock,
            clockFit: clockFitSolution,
            notes: settings.notes.isEmpty ? nil : settings.notes
        )
    }

    // MARK: - Log management

    public func refreshSessions() {
        savedSessions = (try? store.listSessions()) ?? []
    }

    public func delete(_ session: SessionLogStore.LoadedSession) {
        try? store.delete(session.files)
        refreshSessions()
    }

    /// Re-analyse a stored session — the on-device equivalent of the replay harness.
    public func reanalyse(_ session: SessionLogStore.LoadedSession) {
        let processor = SessionProcessor(configuration: SessionProcessor.Configuration(
            filter: settings.filterConfiguration,
            vehicleCalibration: session.header.vehicleCalibration))
        processor.ingest(SessionLogBuilder.events(from: session.rows))
        analysis = processor.analyse()
    }
}

public enum DeviceInfo {
    public static var model: String {
        #if canImport(UIKit)
        // `utsname.machine` is a fixed-size C char array bridged to a large tuple. Reading it
        // via `MemoryLayout.size(ofValue:)` on the *pointer* is a common and wrong idiom — that
        // yields the pointer's size, not the buffer's. Walk the raw bytes instead and stop at
        // the NUL.
        var systemInfo = utsname()
        uname(&systemInfo)
        let identifier = withUnsafeBytes(of: &systemInfo.machine) { buffer -> String in
            let bytes = buffer.prefix { $0 != 0 }
            return String(decoding: bytes, as: UTF8.self)
        }
        return identifier.isEmpty ? "unknown" : identifier
        #else
        return "unknown"
        #endif
    }

    public static var systemVersion: String {
        #if canImport(UIKit)
        return UIDevice.current.systemVersion
        #else
        return "unknown"
        #endif
    }

    public static var appVersion: String {
        let bundle = Bundle.main.infoDictionary
        let short = bundle?["CFBundleShortVersionString"] as? String ?? "0"
        let build = bundle?["CFBundleVersion"] as? String ?? "0"
        return "\(short) (\(build))"
    }
}
#endif
