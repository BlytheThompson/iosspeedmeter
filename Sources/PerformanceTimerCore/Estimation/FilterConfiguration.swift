import Foundation

/// Tuning for the estimator (spec §6.2 "Starting values").
///
/// The spec is explicit that these are starting points: "tune from logged data, don't trust
/// these blindly". They are all surfaced here rather than scattered through the filter so the
/// offline replay harness (§15 step 2) can sweep them without touching estimator code, and so
/// the exact set used for a run can be written into the log header (§10).
public struct FilterConfiguration: Equatable, Codable, Sendable {
    /// Accelerometer white noise, m/s²/√Hz. Spec §6.2 start: 0.05. Raise if the mount vibrates.
    public var sigmaA: Double
    /// Bias random walk, m/s³/√Hz. Spec §6.2 start: 0.002. Governs how fast bias may wander.
    public var sigmaB: Double
    /// Floor on GNSS speed sigma, m/s. Spec §6.2: "never trust speedAccuracy below this".
    public var sigmaGPSFloor: Double
    /// ZUPT measurement noise, m/s. Spec §6.2 start: 0.01.
    public var sigmaZUPT: Double

    /// Innovation gate in sigmas. Spec §6.3 rejects when `y²/S > 9`, i.e. 3σ.
    public var innovationGateSigma: Double
    /// Fixes with a worse speed sigma than this are refused outright (spec §6.3).
    public var maximumSpeedAccuracy: Double

    /// Which `Q` discretisation to use. See `ProcessNoise` for why `.exact` is the default.
    public var processNoiseModel: ProcessNoiseModel

    // MARK: Initial covariance

    /// Initial 1σ on distance, m. Zero by definition at the anchor, but a small positive value
    /// keeps `P` non-singular.
    public var initialSigmaDistance: Double
    /// Initial 1σ on speed, m/s.
    public var initialSigmaSpeed: Double
    /// Initial 1σ on accelerometer bias, m/s². A phone accelerometer's turn-on bias is a few
    /// hundredths of a g; ZUPT collapses this within the pre-launch window anyway.
    public var initialSigmaBias: Double

    // MARK: §3.7 wheel speed

    /// Adds the tyre circumference scale `k` as a fourth state.
    public var wheelSpeedEnabled: Bool
    /// Initial 1σ on `k`. Spec §3.7: about 0.02.
    public var initialSigmaWheelScale: Double
    /// Random walk on `k`, 1/√Hz. It should drift only with tyre temperature and pressure.
    public var sigmaWheelScaleRandomWalk: Double
    /// Spec §3.7: gate wheel speed off while `|a| > 0.35 g` — wheelspin reads high exactly
    /// when the measurement matters most.
    public var wheelSpeedAccelerationGate: Double
    /// Spec §3.7: gate off while wheel speed exceeds fused speed by more than 1.5 m/s.
    public var wheelSpeedDisagreementGate: Double

    public init(
        sigmaA: Double = 0.05,
        sigmaB: Double = 0.002,
        sigmaGPSFloor: Double = 0.05,
        sigmaZUPT: Double = 0.01,
        innovationGateSigma: Double = 3.0,
        maximumSpeedAccuracy: Double = 1.0,
        processNoiseModel: ProcessNoiseModel = .exact,
        initialSigmaDistance: Double = 0.01,
        initialSigmaSpeed: Double = 0.1,
        initialSigmaBias: Double = 0.2,
        wheelSpeedEnabled: Bool = false,
        initialSigmaWheelScale: Double = 0.02,
        sigmaWheelScaleRandomWalk: Double = 1e-5,
        wheelSpeedAccelerationGate: Double = 0.35 * PTConstants.g,
        wheelSpeedDisagreementGate: Double = 1.5
    ) {
        self.sigmaA = sigmaA
        self.sigmaB = sigmaB
        self.sigmaGPSFloor = sigmaGPSFloor
        self.sigmaZUPT = sigmaZUPT
        self.innovationGateSigma = innovationGateSigma
        self.maximumSpeedAccuracy = maximumSpeedAccuracy
        self.processNoiseModel = processNoiseModel
        self.initialSigmaDistance = initialSigmaDistance
        self.initialSigmaSpeed = initialSigmaSpeed
        self.initialSigmaBias = initialSigmaBias
        self.wheelSpeedEnabled = wheelSpeedEnabled
        self.initialSigmaWheelScale = initialSigmaWheelScale
        self.sigmaWheelScaleRandomWalk = sigmaWheelScaleRandomWalk
        self.wheelSpeedAccelerationGate = wheelSpeedAccelerationGate
        self.wheelSpeedDisagreementGate = wheelSpeedDisagreementGate
    }

    /// 3, or 4 when the wheel-scale state is enabled.
    public var stateDimension: Int { wheelSpeedEnabled ? 4 : 3 }

    /// `y²/S` threshold. Spec §6.3 uses 9.
    public var innovationGateThreshold: Double { innovationGateSigma * innovationGateSigma }
}

/// Index of each state in the filter vector.
public enum StateIndex {
    public static let distance = 0
    public static let speed = 1
    public static let bias = 2
    public static let wheelScale = 3
}
