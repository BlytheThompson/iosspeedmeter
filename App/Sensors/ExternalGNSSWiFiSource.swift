#if canImport(Network)
import Foundation
import Network
import PerformanceTimerCore

/// Spec §3.5 — external GNSS over Wi-Fi, the recommended upgrade path.
///
/// Connects to the receiver's SoftAP over TCP and decodes UBX binary. Two things in §3.5 will
/// bite anyone who skips them, and both are handled here:
///
/// - **`requiredInterfaceType = .wifi` is not optional.** Without it iOS notices the SoftAP has
///   no internet route and sends the traffic over cellular, where the ESP32 does not exist.
/// - **`NSLocalNetworkUsageDescription` must be in Info.plist** or the connection silently
///   fails on iOS 14+.
///
/// UBX is preferred over NMEA because `UBX-NAV-PVT` carries `sAcc`, a real per-fix speed sigma,
/// which is what the filter needs for `R`. NMEA `RMC` gives knots and no accuracy estimate at
/// all.
public final class ExternalGNSSWiFiSource: SensorSource {
    public enum State: Equatable, Sendable {
        case idle
        case connecting
        case connected
        /// Collecting `(iTOW, arrival)` pairs for the §2.4 clock fit.
        case aligningClock(samplesCollected: Int, required: Int)
        case streaming
        case failed(String)
    }

    public let descriptor: SensorSourceDescriptor
    public private(set) var isRunning = false
    public private(set) var state: State = .idle

    /// Called whenever `state` changes, for the UI.
    public var onStateChange: ((State) -> Void)?

    private let host: NWEndpoint.Host
    private let port: NWEndpoint.Port
    private let clock: SessionClock
    private let queue = DispatchQueue(label: "com.performancetimer.externalgnss")

    private var connection: NWConnection?
    private var parser = UBXParser()
    private var clockFit = ClockFit()
    private var clockSolution: ClockFit.Solution?
    private var handler: ((SensorEvent) -> Void)?

    /// Spec §2.4 requires at least 30 pairs at rest before the fit is trusted.
    private let requiredClockSamples = ClockFit.minimumSamples

    /// Residual scatter of the clock fit, seconds — this is the link jitter, and §2.4 says to
    /// log it. Surfaced so the UI and the log header can both record it.
    public private(set) var clockResidualRMS: Double?

    public init(
        host: String = "192.168.4.1",
        port: UInt16 = 2947,
        clock: SessionClock,
        nominalRate: Double = 25
    ) {
        self.host = NWEndpoint.Host(host)
        self.port = NWEndpoint.Port(rawValue: port) ?? 2947
        self.clock = clock
        descriptor = SensorSourceDescriptor(
            identifier: "external.ubx.wifi",
            displayName: "External GNSS (Wi-Fi, UBX)",
            nominalRate: nominalRate,
            kind: .gnss
        )
    }

    public func start(handler: @escaping (SensorEvent) -> Void) throws {
        guard !isRunning else { throw SensorSourceError.alreadyRunning }
        self.handler = handler
        isRunning = true
        parser.reset()
        clockFit.reset()
        clockSolution = nil

        let parameters = NWParameters.tcp
        // MANDATORY — see the type documentation.
        parameters.requiredInterfaceType = .wifi
        // The SoftAP has no internet route; without this iOS may decline to use it.
        parameters.prohibitExpensivePaths = false
        if let ip = parameters.defaultProtocolStack.internetProtocol as? NWProtocolIP.Options {
            ip.version = .v4
        }

        let connection = NWConnection(host: host, port: port, using: parameters)
        self.connection = connection

        connection.stateUpdateHandler = { [weak self] newState in
            guard let self else { return }
            switch newState {
            case .ready:
                self.setState(.aligningClock(samplesCollected: 0,
                                             required: self.requiredClockSamples))
                self.receive()
            case .waiting(let error):
                self.setState(.failed("Waiting: \(error.localizedDescription)"))
            case .failed(let error):
                self.setState(.failed(error.localizedDescription))
            case .cancelled:
                self.setState(.idle)
            default:
                break
            }
        }

        setState(.connecting)
        connection.start(queue: queue)
    }

    public func stop() {
        guard isRunning else { return }
        connection?.cancel()
        connection = nil
        handler = nil
        isRunning = false
        setState(.idle)
    }

    private func setState(_ newState: State) {
        state = newState
        onStateChange?(newState)
    }

    private func receive() {
        connection?.receive(minimumIncompleteLength: 1, maximumLength: 8192) {
            [weak self] data, _, isComplete, error in
            guard let self else { return }
            if let data, !data.isEmpty {
                self.consume([UInt8](data))
            }
            if isComplete || error != nil {
                self.setState(.failed(error?.localizedDescription ?? "Connection closed"))
                return
            }
            self.receive()
        }
    }

    private func consume(_ bytes: [UInt8]) {
        // Arrival time on the monotonic session clock. This is used ONLY to build the §2.4
        // fit — never as the measurement time.
        let arrival = clock.sessionTime(monotonicTimestamp: ProcessInfo.processInfo.systemUptime)

        for message in parser.consume(bytes) {
            // Spec §3.7 — wheel speed shipped over the same link as a UBX private message, so
            // it inherits the framing, the checksum and the §2.4 clock alignment.
            if case .other(let cls, let id, let payload) = message,
               cls == WheelSpeedMessage.messageClass, id == WheelSpeedMessage.messageID,
               let wheel = WheelSpeedMessage.decode(payload),
               let solution = clockSolution {
                handler?(.wheelSpeed(wheel.sample(
                    sessionTime: solution.sessionTime(itowMilliseconds: Double(wheel.iTOW)))))
                continue
            }

            guard case .navPVT(let pvt) = message else { continue }
            guard pvt.gnssFixOK, pvt.fixType >= 3 else { continue }

            if let solution = clockSolution {
                // Spec §2.4: map the packet's own iTOW onto the session clock. Wi-Fi jitter of
                // 5–40 ms would otherwise land directly in the results.
                let t = solution.sessionTime(itowMilliseconds: Double(pvt.iTOW))
                    + Double(pvt.nano) / 1e9
                handler?(.gnss(pvt.gnssFix(sessionTime: t)))
            } else {
                clockFit.add(itowMilliseconds: Double(pvt.iTOW), arrivalSessionTime: arrival)
                if let solved = try? clockFit.solve() {
                    clockSolution = solved
                    clockResidualRMS = solved.residualRMS
                    setState(.streaming)
                } else {
                    setState(.aligningClock(samplesCollected: clockFit.sampleCount,
                                            required: requiredClockSamples))
                }
            }
        }
    }

    /// Exposed for the log header (spec §10).
    public var clockFitSolution: ClockFit.Solution? { clockSolution }
}
#endif
