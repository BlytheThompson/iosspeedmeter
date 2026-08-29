import Foundation

/// Counts and rates gathered while processing, for the log header and the confidence badge.
public struct SessionDiagnostics: Equatable, Sendable {
    public var imuSampleCount = 0
    public var acceptedFixCount = 0
    public var rejectedFixCount = 0
    public var gateRejectionCount = 0
    public var zuptSampleCount = 0
    /// Mean reported speed accuracy across accepted fixes, m/s.
    public var meanSpeedAccuracy = 0.0
    /// Fixes per second actually achieved. Spec §3.3: measure it, never assume it.
    public var achievedGNSSRate = 0.0
    /// Mean IMU interval actually delivered, s. Spec §3.1 warns the requested rate is a
    /// request, not a promise.
    public var meanIMUInterval = 0.0
    /// True when no vehicle-frame calibration could be solved and a gravity-only provisional
    /// frame was used instead.
    public var usedProvisionalVehicleFrame = false

    public init() {}
}

/// The complete post-processed outcome of one session.
public struct SessionAnalysis: Sendable {
    /// Forward pass, retained for the log and for the display-only trace.
    public var steps: [FilterStep]
    /// Smoothed trace. **Every reported result comes from here** (spec Decision 1, §6.4).
    public var smoothed: [SmoothedSample]
    /// Stationary-detector verdict per IMU sample.
    public var stationaryFlags: [Bool]
    /// The retroactive launch anchor (spec §8), if a launch was found.
    public var anchor: LaunchAnchor.Anchor?
    /// Raw results. `nil` when no launch was detected.
    public var results: RunResults?
    public var grade: GradeEstimate?
    /// Grade-corrected results, offered alongside the raw ones — never instead of them
    /// (spec §7.2).
    public var gradeCorrectedResults: RunResults?
    public var confidence: ConfidenceAssessment
    public var diagnostics: SessionDiagnostics
    public var vehicleCalibration: VehicleFrameCalibration?

    public init(
        steps: [FilterStep], smoothed: [SmoothedSample], stationaryFlags: [Bool],
        anchor: LaunchAnchor.Anchor?, results: RunResults?, grade: GradeEstimate?,
        gradeCorrectedResults: RunResults?, confidence: ConfidenceAssessment,
        diagnostics: SessionDiagnostics, vehicleCalibration: VehicleFrameCalibration?
    ) {
        self.steps = steps
        self.smoothed = smoothed
        self.stationaryFlags = stationaryFlags
        self.anchor = anchor
        self.results = results
        self.grade = grade
        self.gradeCorrectedResults = gradeCorrectedResults
        self.confidence = confidence
        self.diagnostics = diagnostics
        self.vehicleCalibration = vehicleCalibration
    }
}

/// Spec §15 — the whole estimation pipeline, from a recorded event stream to a timeslip.
///
/// **This is deliberately an offline, two-pass design.** Spec Decision 1: "everything is
/// post-processed … the reported result is computed after the run ends, by a backward
/// smoothing pass over the full recorded session. Design the app so that timing results are
/// never produced by the live path."
///
/// Accumulating first also makes spec §2.3 fall out for free: a GNSS fix delivered 400 ms
/// late is simply sorted back to its own fix epoch before the forward pass runs, rather than
/// being applied at the wrong instant. `LiveEstimator` handles the on-screen readout; it is a
/// separate type precisely so a live number can never be mistaken for a result.
public final class SessionProcessor {
    public struct Configuration: Sendable {
        public var filter: FilterConfiguration
        /// A persisted `R_DV` from a previous calibration (spec §5). When `nil`, the processor
        /// solves one from this session's own launch.
        public var vehicleCalibration: VehicleFrameCalibration?
        public var distanceMarks: [DistanceMark]
        public var speedMarks: [SpeedMark]
        public var rollWindows: [RollWindow]
        /// Spec §7.1 correction applied to the acceleration input. Off by default; see
        /// `GradeCorrection`.
        public var applyGradeToAcceleration: Bool

        public init(
            filter: FilterConfiguration = FilterConfiguration(),
            vehicleCalibration: VehicleFrameCalibration? = nil,
            distanceMarks: [DistanceMark] = DistanceMark.standard,
            speedMarks: [SpeedMark] = SpeedMark.standardImperial,
            rollWindows: [RollWindow] = RollWindow.standard,
            applyGradeToAcceleration: Bool = false
        ) {
            self.filter = filter
            self.vehicleCalibration = vehicleCalibration
            self.distanceMarks = distanceMarks
            self.speedMarks = speedMarks
            self.rollWindows = rollWindows
            self.applyGradeToAcceleration = applyGradeToAcceleration
        }
    }

    public let configuration: Configuration

    private var imuSamples: [IMUSample] = []
    private var gnssFixes: [GNSSFix] = []
    private var baroSamples: [BaroSample] = []
    private var wheelSpeeds: [WheelSpeedSample] = []

    public init(configuration: Configuration = Configuration()) {
        self.configuration = configuration
    }

    /// Accumulate one event. Order does not matter — everything is sorted by fix epoch before
    /// the forward pass.
    public func ingest(_ event: SensorEvent) {
        switch event {
        case .imu(let sample): imuSamples.append(sample)
        case .gnss(let fix): gnssFixes.append(fix)
        case .baro(let sample): baroSamples.append(sample)
        case .wheelSpeed(let sample): wheelSpeeds.append(sample)
        }
    }

    public func ingest(_ events: [SensorEvent]) {
        for event in events { ingest(event) }
    }

    public var sampleCount: Int { imuSamples.count }

    public func reset() {
        imuSamples.removeAll()
        gnssFixes.removeAll()
        baroSamples.removeAll()
        wheelSpeeds.removeAll()
    }

    // MARK: - Analysis

    public func analyse() -> SessionAnalysis {
        var diagnostics = SessionDiagnostics()

        let imu = imuSamples.sorted { $0.t < $1.t }
        let fixes = gnssFixes.sorted { $0.t < $1.t }
        let baro = baroSamples.sorted { $0.t < $1.t }
        let wheels = wheelSpeeds.sorted { $0.t < $1.t }

        diagnostics.imuSampleCount = imu.count
        if imu.count > 1 {
            diagnostics.meanIMUInterval = (imu.last!.t - imu.first!.t) / Double(imu.count - 1)
        }
        if fixes.count > 1 {
            let span = fixes.last!.t - fixes.first!.t
            diagnostics.achievedGNSSRate = span > 0 ? Double(fixes.count - 1) / span : 0
        }

        guard !imu.isEmpty else {
            return SessionAnalysis(
                steps: [], smoothed: [], stationaryFlags: [], anchor: nil, results: nil,
                grade: nil, gradeCorrectedResults: nil,
                confidence: makeConfidence(diagnostics: diagnostics, grade: nil,
                                           calibration: nil),
                diagnostics: diagnostics, vehicleCalibration: nil
            )
        }

        // Pass 1 — stationary flags, the pre-launch window, and the vehicle frame.
        let stationaryFlags = computeStationaryFlags(imu: imu, fixes: fixes)
        let preparation = prepare(imu: imu, fixes: fixes, stationaryFlags: stationaryFlags)
        diagnostics.usedProvisionalVehicleFrame = preparation.isProvisional

        // Pass 2 — the forward filter over the whole session, in fix-epoch order.
        let forward = runForwardPass(
            imu: imu, fixes: fixes, wheels: wheels,
            stationaryFlags: stationaryFlags,
            resolver: preparation.resolver,
            diagnostics: &diagnostics
        )

        // Spec §6.4 — the backward pass. This is what produces every reported number.
        let smoothed = RTSSmoother.smooth(forward.steps)

        // Spec §8 — retroactive anchoring on the smoothed trace.
        var anchor: LaunchAnchor.Anchor?
        if let triggerIndex = forward.launchTriggerIndex {
            anchor = LaunchAnchor.retroactiveAnchor(
                samples: smoothed, stationaryFlags: stationaryFlags, triggerIndex: triggerIndex
            )
        }

        // Spec §7 — grade, then §9 — results.
        let grade = GradeEstimator.estimate(samples: smoothed, barometric: baro, gnss: fixes)

        var results: RunResults?
        var gradeCorrected: RunResults?
        if let anchor {
            results = ResultExtractor.extract(
                from: smoothed, anchorTime: anchor.time,
                distanceMarks: configuration.distanceMarks,
                speedMarks: configuration.speedMarks,
                rollWindows: configuration.rollWindows
            )
            if let grade, grade.exceedsReportingThreshold {
                gradeCorrected = ResultExtractor.extract(
                    from: grade.correctedTrace(smoothed), anchorTime: anchor.time,
                    distanceMarks: configuration.distanceMarks,
                    speedMarks: configuration.speedMarks,
                    rollWindows: configuration.rollWindows
                )
            }
        }

        return SessionAnalysis(
            steps: forward.steps,
            smoothed: smoothed,
            stationaryFlags: stationaryFlags,
            anchor: anchor,
            results: results,
            grade: grade,
            gradeCorrectedResults: gradeCorrected,
            confidence: makeConfidence(diagnostics: diagnostics, grade: grade,
                                       calibration: preparation.calibration),
            diagnostics: diagnostics,
            vehicleCalibration: preparation.calibration
        )
    }

    // MARK: - Pass 1

    private func computeStationaryFlags(imu: [IMUSample], fixes: [GNSSFix]) -> [Bool] {
        var detector = StationaryDetector()
        var flags = [Bool]()
        flags.reserveCapacity(imu.count)
        var fixIndex = 0

        for sample in imu {
            while fixIndex < fixes.count, fixes[fixIndex].t <= sample.t {
                detector.noteGNSS(fixes[fixIndex])
                fixIndex += 1
            }
            detector.add(sample)
            flags.append(detector.isStationary)
        }
        return flags
    }

    private struct Preparation {
        var resolver: LongitudinalResolver?
        var calibration: VehicleFrameCalibration?
        var isProvisional: Bool
    }

    /// Establish the stationary anchor (spec §3.2 step 1) and the vehicle frame (spec §5).
    ///
    /// The vehicle frame needs the mean acceleration over the first 1.5 s of a launch, which
    /// is only knowable once the launch has been found — hence the separate pass. Doing it
    /// this way also means a session that never launches still yields a usable trace, which is
    /// what the §12.1 bench test needs.
    private func prepare(
        imu: [IMUSample], fixes: [GNSSFix], stationaryFlags: [Bool]
    ) -> Preparation {
        // The last contiguous stationary run before motion starts is the anchor window.
        guard let launchIndex = firstMotionIndex(imu: imu, stationaryFlags: stationaryFlags) else {
            return prepareWithoutLaunch(imu: imu, stationaryFlags: stationaryFlags)
        }

        var stationary = StationaryCalibration()
        var index = launchIndex - 1
        while index >= 0, stationaryFlags[index] {
            stationary.add(imu[index])
            index -= 1
        }
        guard let stationaryResult = stationary.result() else {
            return prepareWithoutLaunch(imu: imu, stationaryFlags: stationaryFlags)
        }

        // Spec §5 step 2: mean userAcceleration over the first 1.5 s of the launch.
        let windowEnd = imu[launchIndex].t + 1.5
        let launchWindow = imu[launchIndex...].prefix { $0.t <= windowEnd }
        let gravityWindow = imu[..<launchIndex].suffix(200).map(\.gravity)

        let courseAccuracy = fixes
            .filter { $0.t >= imu[launchIndex].t && $0.t <= windowEnd && $0.courseAccuracy >= 0 }
            .map(\.courseAccuracy).max()
        let speedIncreasing = isSpeedIncreasing(fixes: fixes,
                                                from: imu[launchIndex].t, to: windowEnd)

        if let supplied = configuration.vehicleCalibration {
            return Preparation(
                resolver: LongitudinalResolver(calibration: supplied, stationary: stationaryResult),
                calibration: supplied,
                isProvisional: false
            )
        }

        if let solved = try? VehicleFrameCalibrator.calibrate(
            staticGravity: gravityWindow,
            launchAcceleration: launchWindow.map(\.userAcceleration),
            courseAccuracyDegrees: courseAccuracy,
            speedIncreasing: speedIncreasing,
            timestamp: imu[launchIndex].t
        ) {
            return Preparation(
                resolver: LongitudinalResolver(calibration: solved, stationary: stationaryResult),
                calibration: solved,
                isProvisional: false
            )
        }

        // The gates of §5 refused this launch. Fall back to a gravity-only frame so the run is
        // still recorded and analysable, and mark it so the confidence badge can say why.
        let provisional = VehicleFrameCalibration.provisional(
            gravity: stationaryResult.referenceSpecificForce, timestamp: imu[launchIndex].t
        )
        return Preparation(
            resolver: provisional.map {
                LongitudinalResolver(calibration: $0, stationary: stationaryResult)
            },
            calibration: provisional,
            isProvisional: true
        )
    }

    private func prepareWithoutLaunch(imu: [IMUSample], stationaryFlags: [Bool]) -> Preparation {
        var stationary = StationaryCalibration()
        for (index, sample) in imu.enumerated() where stationaryFlags[index] {
            stationary.add(sample)
            if stationary.sampleCount >= 200 { break }
        }
        guard let result = stationary.result() else {
            return Preparation(resolver: nil, calibration: nil, isProvisional: true)
        }
        let calibration = configuration.vehicleCalibration
            ?? VehicleFrameCalibration.provisional(gravity: result.referenceSpecificForce,
                                                   timestamp: result.t)
        return Preparation(
            resolver: calibration.map {
                LongitudinalResolver(calibration: $0, stationary: result)
            },
            calibration: calibration,
            isProvisional: configuration.vehicleCalibration == nil
        )
    }

    /// First index where the vehicle stopped being stationary, having been stationary long
    /// enough beforehand to anchor on.
    private func firstMotionIndex(imu: [IMUSample], stationaryFlags: [Bool]) -> Int? {
        var stationaryRun = 0
        for index in 0..<imu.count {
            if stationaryFlags[index] {
                stationaryRun += 1
            } else if stationaryRun >= 40 {
                return index
            } else {
                stationaryRun = 0
            }
        }
        return nil
    }

    private func isSpeedIncreasing(fixes: [GNSSFix], from: Double, to: Double) -> Bool {
        let window = fixes.filter { $0.t >= from && $0.t <= to && $0.speed >= 0 }
        guard let first = window.first, let last = window.last, window.count >= 2 else {
            // Without GNSS evidence, assume a launch rather than a braking event: that is what
            // the user pressed the button for, and §5's gates already reject a gentle event.
            return true
        }
        return last.speed > first.speed
    }

    // MARK: - Pass 2

    private struct ForwardPass {
        var steps: [FilterStep]
        var launchTriggerIndex: Int?
    }

    private func runForwardPass(
        imu: [IMUSample],
        fixes: [GNSSFix],
        wheels: [WheelSpeedSample],
        stationaryFlags: [Bool],
        resolver: LongitudinalResolver?,
        diagnostics: inout SessionDiagnostics
    ) -> ForwardPass {
        var filter = KalmanFilter(configuration: configuration.filter)
        var resolver = resolver
        var launchDetector = LaunchDetector()

        var steps: [FilterStep] = []
        steps.reserveCapacity(imu.count)

        var fixIndex = 0
        var wheelIndex = 0
        var launchTriggerIndex: Int?
        var speedAccuracySum = 0.0
        var previousTime = imu[0].t

        for (index, sample) in imu.enumerated() {
            let dt = index == 0 ? 0 : sample.t - previousTime
            previousTime = sample.t

            // Spec §3.1: dt always comes from consecutive timestamps. A duplicated or
            // out-of-order stamp would otherwise produce a zero or negative step.
            let effectiveDt = dt > 0 ? dt : (diagnostics.meanIMUInterval > 0
                                             ? diagnostics.meanIMUInterval : 0.01)

            let acceleration = resolver?.longitudinalAcceleration(of: sample) ?? 0

            var step = filter.predict(acceleration: acceleration, dt: effectiveDt)
            step.t = sample.t

            // ZUPT while the stationary detector holds (spec §6.3).
            if stationaryFlags[index] {
                _ = filter.updateZeroVelocity()
                step.zuptApplied = true
                diagnostics.zuptSampleCount += 1
            }

            // Every fix whose epoch falls in this sample's interval.
            while fixIndex < fixes.count, fixes[fixIndex].t <= sample.t {
                let fix = fixes[fixIndex]
                fixIndex += 1
                switch filter.updateGNSSSpeed(fix) {
                case .applied:
                    diagnostics.acceptedFixCount += 1
                    speedAccuracySum += fix.speedAccuracy
                    step.gnssApplied = true
                case .rejectedByGate:
                    diagnostics.rejectedFixCount += 1
                    diagnostics.gateRejectionCount += 1
                    step.gateRejected = true
                default:
                    diagnostics.rejectedFixCount += 1
                }
            }

            // Spec §3.7 wheel speed, gated on the acceleration fed to this same step.
            while wheelIndex < wheels.count, wheels[wheelIndex].t <= sample.t {
                let wheel = wheels[wheelIndex]
                wheelIndex += 1
                if case .applied = filter.updateWheelSpeed(
                    wheel, measurementSigma: 0.1, currentAcceleration: acceleration
                ) {
                    step.wheelSpeedApplied = true
                }
            }

            filter.finish(step: &step)
            steps.append(step)

            if launchTriggerIndex == nil,
               launchDetector.update(t: sample.t, acceleration: acceleration) {
                launchTriggerIndex = index
            }
        }

        if diagnostics.acceptedFixCount > 0 {
            diagnostics.meanSpeedAccuracy = speedAccuracySum / Double(diagnostics.acceptedFixCount)
        }
        return ForwardPass(steps: steps, launchTriggerIndex: launchTriggerIndex)
    }

    private func makeConfidence(
        diagnostics: SessionDiagnostics,
        grade: GradeEstimate?,
        calibration: VehicleFrameCalibration?
    ) -> ConfidenceAssessment {
        ConfidenceAssessment(
            rejectedFixCount: diagnostics.rejectedFixCount,
            meanSpeedAccuracy: diagnostics.acceptedFixCount > 0
                ? diagnostics.meanSpeedAccuracy : 1.0,
            achievedGNSSRate: diagnostics.achievedGNSSRate,
            calibrationAgeSeconds: 0,
            meanGradePercent: grade?.meanGradePercent ?? 0,
            usedSmoother: true
        )
    }
}

extension VehicleFrameCalibration {
    /// A vehicle frame derived from gravity alone, with an arbitrary but stable forward axis.
    ///
    /// This is **not** a substitute for the §5 calibration — the forward direction is a guess.
    /// It exists so a session that never launched (the §12.1 bench test) or one whose launch
    /// failed the §5 gates still produces a complete, loggable trace instead of nothing.
    /// Anything using it is flagged via `SessionDiagnostics.usedProvisionalVehicleFrame`.
    public static func provisional(gravity: Vector3, timestamp: Double) -> VehicleFrameCalibration? {
        guard let direction = gravity.normalized() else { return nil }
        let zV = -direction
        // Any vector not parallel to z_V gives a stable horizontal seed.
        let seed = abs(zV.x) < 0.9 ? Vector3(1, 0, 0) : Vector3(0, 1, 0)
        guard let xV = seed.removingComponent(along: zV).normalized() else { return nil }
        guard let rotation = Matrix3(rows: xV, zV.cross(xV), zV).orthonormalised() else {
            return nil
        }
        return VehicleFrameCalibration(
            rotation: rotation,
            timestamp: timestamp,
            launchAccelerationMagnitude: 0,
            verticalFraction: 0,
            wasSignFlipped: false,
            sourceEventCount: 0
        )
    }
}
