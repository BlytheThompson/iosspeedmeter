import Foundation

/// Spec §4 — stationary detection.
///
/// The four conditions the spec lists, all of which must hold over a 0.5 s window:
/// - `stdev(|f_D|) < 0.03 g`
/// - `max(|rotationRate|) < 0.02 rad/s`
/// - GNSS speed `< 0.3 m/s` (when a valid fix exists)
/// - the window contains ≥ 40 IMU samples
///
/// This gates ZUPT, which spec §6.3 calls worth more than any hardware upgrade below the
/// 25 Hz tier. Getting it wrong in the permissive direction is far worse than in the strict
/// direction: a false "stationary" applies a zero-speed measurement to a moving vehicle.
///
/// **Deviation D4 from the spec: two extra conditions.** As published, the detector
/// re-declares "stationary" *in the middle of a launch*. Under steady acceleration the
/// specific-force magnitude is constant, so its standard deviation collapses back below the
/// 0.03 g threshold once the whole 0.5 s window sits inside the accelerating phase; the gyro
/// is quiet because the car is going straight; and at 1 Hz the most recent GNSS fix still
/// reports the speed from before the launch. All four published conditions hold while the car
/// pulls 0.5 g.
///
/// Observed on a synthetic 0–60: ZUPT resumed 0.5 s after launch, drove the accelerometer bias
/// state to 1.26 m/s² of pure fiction, and the reported speed came out 32% low. This is not a
/// tuning problem — a variance test structurally cannot separate rest from constant
/// acceleration.
///
/// Two conditions close it:
///
/// 1. **Force magnitude.** At rest `|f_D| = g`; under longitudinal acceleration `a` it is
///    `√(a² + g²)`. Comparing the window mean against the magnitude measured during a
///    previously confirmed stationary window makes the test self-referential, so accelerometer
///    scale error cancels instead of eating the margin.
/// 2. **Post-motion GNSS confirmation.** Once genuine motion has been seen, the IMU alone can
///    never prove the car stopped — constant velocity is indistinguishable from rest. Re-entry
///    to "stationary" therefore requires a valid fix, newer than the last moving sample,
///    reporting near-zero speed. This only applies when the session has GNSS at all, so an
///    IMU-only configuration degrades to the published behaviour rather than never arming.
public struct StationaryDetector: Sendable {
    /// Which conditions are currently unmet. Empty means stationary.
    public struct Conditions: OptionSet, Sendable, Hashable {
        public let rawValue: Int
        public init(rawValue: Int) { self.rawValue = rawValue }

        public static let accelerationVariance = Conditions(rawValue: 1 << 0)
        public static let rotationRate = Conditions(rawValue: 1 << 1)
        public static let gnssSpeed = Conditions(rawValue: 1 << 2)
        public static let insufficientSamples = Conditions(rawValue: 1 << 3)
        /// Deviation D4 (1): the window's mean `|f_D|` does not match the at-rest magnitude.
        public static let forceMagnitude = Conditions(rawValue: 1 << 4)
        /// Deviation D4 (2): motion was seen and no fresh fix has confirmed a stop since.
        public static let awaitingGNSSConfirmation = Conditions(rawValue: 1 << 5)

        /// The conditions that indicate real motion, as opposed to simply not having enough
        /// data yet. Only these arm the confirmation latch.
        public static let motionIndicators: Conditions = [
            .accelerationVariance, .rotationRate, .gnssSpeed, .forceMagnitude,
        ]
    }

    public var windowDuration: Double
    /// Threshold on the standard deviation of `|f_D|`, m/s². Spec: 0.03 g.
    public var accelerationStdDevThreshold: Double
    /// Threshold on the largest component magnitude of the gyro, rad/s. Spec: 0.02.
    public var rotationRateThreshold: Double
    /// Threshold on GNSS speed, m/s. Spec: 0.3.
    public var gnssSpeedThreshold: Double
    /// Spec: at least 40 IMU samples in the window.
    public var minimumSamples: Int
    /// How long a GNSS fix keeps vetoing. Without this a single stale fix would veto forever;
    /// spec §4 only qualifies the condition with "when a valid fix exists", and a fix from
    /// several seconds ago is no longer evidence about now.
    public var gnssValidityHorizon: Double
    /// Deviation D4 (1): tolerance on `| mean(|f_D|) − restingMagnitude |`, m/s².
    ///
    /// 0.15 m/s² rejects sustained longitudinal acceleration above roughly 1.7 m/s² — well
    /// under the 0.15 g launch trigger of spec §8 — while staying clear of the ~0.1 m/s² a 1%
    /// accelerometer scale error contributes. Once a resting magnitude has been measured the
    /// comparison is self-referential, so scale error largely cancels.
    public var forceMagnitudeTolerance: Double
    /// Deviation D4 (2): require a fresh fix confirming a stop before re-entering "stationary"
    /// after motion. Disable only for a genuinely GNSS-free configuration.
    public var requiresGNSSConfirmationAfterMotion: Bool

    private var times: [Double] = []
    private var forceMagnitudes: [Double] = []
    private var rotationMagnitudes: [Double] = []
    private var lastValidGNSS: GNSSFix?
    private var latestSampleTime: Double = -.infinity

    /// Mean `|f_D|` measured during a confirmed stationary window, m/s².
    private var restingForceMagnitude: Double?
    /// Session time of the most recent sample that showed real motion.
    private var lastMotionTime: Double?
    /// Whether any valid fix has been seen this session.
    private var hasSeenGNSS = false

    public init(
        windowDuration: Double = 0.5,
        accelerationStdDevThreshold: Double = 0.03 * PTConstants.g,
        rotationRateThreshold: Double = 0.02,
        gnssSpeedThreshold: Double = 0.3,
        minimumSamples: Int = 40,
        gnssValidityHorizon: Double = 2.0,
        forceMagnitudeTolerance: Double = 0.15,
        requiresGNSSConfirmationAfterMotion: Bool = true
    ) {
        self.windowDuration = windowDuration
        self.accelerationStdDevThreshold = accelerationStdDevThreshold
        self.rotationRateThreshold = rotationRateThreshold
        self.gnssSpeedThreshold = gnssSpeedThreshold
        self.minimumSamples = minimumSamples
        self.gnssValidityHorizon = gnssValidityHorizon
        self.forceMagnitudeTolerance = forceMagnitudeTolerance
        self.requiresGNSSConfirmationAfterMotion = requiresGNSSConfirmationAfterMotion
    }

    public mutating func add(_ sample: IMUSample) {
        latestSampleTime = sample.t
        times.append(sample.t)
        forceMagnitudes.append(sample.specificForce.length)
        rotationMagnitudes.append(
            max(abs(sample.rotationRate.x),
                max(abs(sample.rotationRate.y), abs(sample.rotationRate.z)))
        )
        trim()

        // Evaluate after the window is updated so the latch reflects this sample.
        let unmet = evaluate()
        if !unmet.isDisjoint(with: .motionIndicators) {
            lastMotionTime = sample.t
        } else if unmet.isEmpty {
            // A confirmed stationary window is the reference for the magnitude test.
            restingForceMagnitude = meanForceMagnitude
        }
    }

    /// Feed the most recent fix. Invalid fixes are ignored, matching spec §4's "when a valid
    /// fix exists".
    public mutating func noteGNSS(_ fix: GNSSFix) {
        guard fix.speed >= 0, fix.speedAccuracy >= 0 else { return }
        hasSeenGNSS = true
        lastValidGNSS = fix
    }

    public mutating func reset() {
        times.removeAll(keepingCapacity: true)
        forceMagnitudes.removeAll(keepingCapacity: true)
        rotationMagnitudes.removeAll(keepingCapacity: true)
        lastValidGNSS = nil
        latestSampleTime = -.infinity
        restingForceMagnitude = nil
        lastMotionTime = nil
        hasSeenGNSS = false
    }

    private mutating func trim() {
        let cutoff = latestSampleTime - windowDuration
        var drop = 0
        while drop < times.count, times[drop] < cutoff { drop += 1 }
        if drop > 0 {
            times.removeFirst(drop)
            forceMagnitudes.removeFirst(drop)
            rotationMagnitudes.removeFirst(drop)
        }
    }

    public var sampleCount: Int { times.count }

    /// Standard deviation of `|f_D|` over the window, m/s².
    public var accelerationStdDev: Double {
        let n = forceMagnitudes.count
        guard n > 1 else { return 0 }
        let mean = forceMagnitudes.reduce(0, +) / Double(n)
        let variance = forceMagnitudes.reduce(0) { $0 + ($1 - mean) * ($1 - mean) } / Double(n - 1)
        return variance.squareRoot()
    }

    public var maximumRotationRate: Double {
        rotationMagnitudes.max() ?? 0
    }

    /// The fix that is currently eligible to veto, if any.
    private var activeGNSS: GNSSFix? {
        guard let fix = lastValidGNSS else { return nil }
        guard latestSampleTime.isFinite else { return fix }
        guard latestSampleTime - fix.t <= gnssValidityHorizon else { return nil }
        return fix
    }

    /// Mean `|f_D|` over the window, m/s².
    public var meanForceMagnitude: Double {
        guard !forceMagnitudes.isEmpty else { return 0 }
        return forceMagnitudes.reduce(0, +) / Double(forceMagnitudes.count)
    }

    /// The at-rest `|f_D|` this detector is comparing against, m/s². Falls back to standard
    /// gravity until a stationary window has been confirmed.
    public var referenceForceMagnitude: Double { restingForceMagnitude ?? PTConstants.g }

    private func evaluate() -> Conditions {
        var unmet: Conditions = []
        if times.count < minimumSamples { unmet.insert(.insufficientSamples) }
        if accelerationStdDev >= accelerationStdDevThreshold { unmet.insert(.accelerationVariance) }
        if maximumRotationRate >= rotationRateThreshold { unmet.insert(.rotationRate) }
        if let fix = activeGNSS, fix.speed >= gnssSpeedThreshold { unmet.insert(.gnssSpeed) }

        // Deviation D4 (1) — force magnitude.
        //
        // Before any stationary window has been confirmed the reference is standard gravity,
        // and the tolerance is loosened so an unknown accelerometer scale error cannot stop
        // the detector ever arming. Once a resting magnitude is known the tight tolerance
        // applies, because the comparison is then self-referential.
        if !forceMagnitudes.isEmpty {
            let tolerance = restingForceMagnitude == nil
                ? max(forceMagnitudeTolerance, 0.5)
                : forceMagnitudeTolerance
            if abs(meanForceMagnitude - referenceForceMagnitude) > tolerance {
                unmet.insert(.forceMagnitude)
            }
        }

        // Deviation D4 (2) — post-motion GNSS confirmation.
        if requiresGNSSConfirmationAfterMotion, hasSeenGNSS, let motionTime = lastMotionTime {
            let confirmed = activeGNSS.map { $0.t > motionTime && $0.speed < gnssSpeedThreshold }
                ?? false
            if !confirmed { unmet.insert(.awaitingGNSSConfirmation) }
        }

        return unmet
    }

    public var unmetConditions: Conditions { evaluate() }

    public var isStationary: Bool { unmetConditions.isEmpty }
}
