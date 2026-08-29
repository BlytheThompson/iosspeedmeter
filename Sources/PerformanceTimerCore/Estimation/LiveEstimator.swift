import Foundation

/// The forward-only path that drives the on-screen readout.
///
/// **This type can never produce a result.** Spec Decision 1: "The live display is a
/// convenience readout from the forward filter only. The *reported result* is computed after
/// the run ends… Design the app so that timing results are never produced by the live path."
///
/// Keeping it as a separate type from `SessionProcessor`, with no method that returns a
/// `RunResults`, is how that rule is enforced structurally rather than by convention.
///
/// It also owns the session state machine and the ring buffer of spec §4, because those are
/// live concerns: what the app records, and when, is decided here; what the numbers *mean* is
/// decided afterwards by the processor.
public final class LiveEstimator {
    public struct Readout: Equatable, Sendable {
        public var state: SessionState
        /// Forward-filter speed, m/s. Lags reality by roughly the GNSS delivery latency plus
        /// half the update interval (spec §2.3) — it is a readout, not a measurement.
        public var speed: Double
        public var distance: Double
        public var accelerometerBias: Double
        /// Longitudinal specific force from the most recent sample, m/s².
        public var longitudinalAcceleration: Double
        public var isStationary: Bool
        public var hasGNSSLock: Bool
        public var calibrationValid: Bool
        public var stopReason: SessionStateMachine.StopReason?
        /// Session time of the most recent sample.
        public var t: Double

        public var speedMph: Double { speed / PTConstants.mphToMetersPerSecond }
        public var speedKmh: Double { speed / PTConstants.kmhToMetersPerSecond }

        // Swift only synthesises an *internal* memberwise initialiser for a public struct, so
        // this has to be spelled out for the app module to construct a Readout at all.
        public init(
            state: SessionState, speed: Double, distance: Double, accelerometerBias: Double,
            longitudinalAcceleration: Double, isStationary: Bool, hasGNSSLock: Bool,
            calibrationValid: Bool, stopReason: SessionStateMachine.StopReason?, t: Double
        ) {
            self.state = state
            self.speed = speed
            self.distance = distance
            self.accelerometerBias = accelerometerBias
            self.longitudinalAcceleration = longitudinalAcceleration
            self.isStationary = isStationary
            self.hasGNSSLock = hasGNSSLock
            self.calibrationValid = calibrationValid
            self.stopReason = stopReason
            self.t = t
        }
    }

    public private(set) var readout = Readout(
        state: .idle, speed: 0, distance: 0, accelerometerBias: 0,
        longitudinalAcceleration: 0, isStationary: false, hasGNSSLock: false,
        calibrationValid: false, stopReason: nil, t: 0
    )

    /// Every event captured since ARMED, ready to hand to `SessionProcessor` when the run ends.
    /// Spec §4: "Recording actually begins at ARMED, not at RECORDING."
    public private(set) var capturedEvents: [SensorEvent] = []

    private let configuration: SessionProcessor.Configuration
    private var filter: KalmanFilter
    private var detector = StationaryDetector()
    private var launchDetector = LaunchDetector()
    private var machine: SessionStateMachine
    private var stationaryCalibration = StationaryCalibration()
    private var resolver: LongitudinalResolver?

    /// Spec §4: a 10 s ring buffer while ARMED, prepended on transition to RECORDING.
    private var preArmBuffer: TimeWindowBuffer<SensorEvent>
    private var lastSampleTime: Double?
    private var lastFixTime: Double?
    private var userRequestedStop = false

    /// How stale a fix may be before GNSS counts as lost, seconds.
    public var gnssLockTimeout: Double = 3.0

    public init(
        configuration: SessionProcessor.Configuration = SessionProcessor.Configuration(),
        preArmBufferDuration: Double = 10.0
    ) {
        self.configuration = configuration
        filter = KalmanFilter(configuration: configuration.filter)
        machine = SessionStateMachine()
        preArmBuffer = TimeWindowBuffer(duration: preArmBufferDuration) { $0.t }
    }

    public var state: SessionState { machine.state }

    /// Vehicle-frame calibration currently in use, if any.
    public var vehicleCalibration: VehicleFrameCalibration? {
        configuration.vehicleCalibration
    }

    public func requestStop() {
        userRequestedStop = true
    }

    /// Begin another run, keeping the calibration.
    public func reset() {
        filter = KalmanFilter(configuration: configuration.filter)
        detector.reset()
        launchDetector.reset()
        machine.reset()
        stationaryCalibration.reset()
        resolver = nil
        preArmBuffer.removeAll()
        capturedEvents.removeAll()
        lastSampleTime = nil
        lastFixTime = nil
        userRequestedStop = false
        readout = Readout(state: .idle, speed: 0, distance: 0, accelerometerBias: 0,
                          longitudinalAcceleration: 0, isStationary: false,
                          hasGNSSLock: false, calibrationValid: false, stopReason: nil, t: 0)
    }

    /// Feed one event from any `SensorSource`.
    public func ingest(_ event: SensorEvent) {
        switch event {
        case .gnss(let fix):
            detector.noteGNSS(fix)
            if fix.isUsableForSpeedUpdate(maximumSpeedAccuracy: configuration.filter.maximumSpeedAccuracy) {
                lastFixTime = fix.t
                _ = filter.updateGNSSSpeed(fix)
            }
            capture(event)

        case .imu(let sample):
            ingestIMU(sample)
            capture(event)

        case .baro, .wheelSpeed:
            capture(event)
        }
    }

    private func capture(_ event: SensorEvent) {
        // While ARMED the ring buffer rolls; on RECORDING everything goes to the session.
        switch machine.state {
        case .recording, .complete, .analysing, .result:
            capturedEvents.append(event)
        default:
            preArmBuffer.append(event)
        }
    }

    private func ingestIMU(_ sample: IMUSample) {
        detector.add(sample)

        let dt = lastSampleTime.map { sample.t - $0 } ?? 0
        lastSampleTime = sample.t

        // Spec §3.1: dt from consecutive timestamps, never assumed.
        let effectiveDt = dt > 0 ? dt : 0.01

        // Accumulate the pre-launch stationary window until we can anchor attitude.
        if detector.isStationary, resolver == nil {
            stationaryCalibration.add(sample)
            if let result = stationaryCalibration.result(),
               let calibration = configuration.vehicleCalibration
                ?? VehicleFrameCalibration.provisional(gravity: result.referenceSpecificForce,
                                                       timestamp: sample.t) {
                resolver = LongitudinalResolver(calibration: calibration, stationary: result)
            }
        }

        let acceleration = resolver?.longitudinalAcceleration(of: sample) ?? 0
        _ = filter.predict(acceleration: acceleration, dt: effectiveDt)
        if detector.isStationary {
            _ = filter.updateZeroVelocity()
        }

        let hasLock = lastFixTime.map { sample.t - $0 <= gnssLockTimeout } ?? false
        let triggered = launchDetector.update(t: sample.t, acceleration: acceleration)

        let previousState = machine.state
        machine.update(SessionStateMachine.Inputs(
            t: sample.t,
            hasGNSSLock: hasLock,
            calibrationValid: resolver != nil,
            isStationary: detector.isStationary,
            longitudinalAcceleration: acceleration,
            speed: filter.speed,
            distance: filter.distance,
            launchTriggered: triggered,
            userRequestedStop: userRequestedStop
        ))

        // Spec §4: prepend the pre-launch ring buffer the moment recording starts.
        if previousState != .recording, machine.state == .recording {
            capturedEvents = preArmBuffer.drain()
        }

        readout = Readout(
            state: machine.state,
            speed: filter.speed,
            distance: filter.distance,
            accelerometerBias: filter.accelerometerBias,
            longitudinalAcceleration: acceleration,
            isStationary: detector.isStationary,
            hasGNSSLock: hasLock,
            calibrationValid: resolver != nil,
            stopReason: machine.stopReason,
            t: sample.t
        )
    }

    /// Hand the captured session to the post-processor. This is the only route to a result.
    public func makeProcessor() -> SessionProcessor {
        let processor = SessionProcessor(configuration: configuration)
        processor.ingest(capturedEvents)
        return processor
    }

    public func beginAnalysis() { machine.beginAnalysis() }
    public func finishAnalysis() { machine.finishAnalysis() }
}
