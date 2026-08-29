import Foundation

/// What happened to a measurement.
public enum MeasurementOutcome: Equatable, Sendable {
    case applied
    /// Failed the validity rules of spec §6.3 before the gate was even considered.
    case rejectedInvalid(GNSSFix.Rejection)
    /// Failed the 3σ innovation gate of spec §6.3. Carries `y²/S`.
    case rejectedByGate(normalisedInnovationSquared: Double)
    /// Failed one of the spec §3.7 wheel-speed gates.
    case rejectedByWheelSpeedGate(WheelSpeedGate)
    /// A wheel-speed update arrived but the fourth state is not configured.
    case notConfigured

    public enum WheelSpeedGate: String, Equatable, Sendable {
        /// `|a| > 0.35 g` — wheelspin.
        case launchAcceleration
        /// Wheel speed exceeds fused speed by more than the configured tolerance.
        case disagreement
    }
}

/// One recorded forward step, carrying everything the RTS smoother needs (spec §6.4:
/// "Store per sample during the forward pass: `x_k|k`, `P_k|k`, `x_k+1|k`, `P_k+1|k`, `F_k`").
///
/// Memory is trivial — a 20 s run at 100 Hz is a few hundred kilobytes — and `Double`
/// throughout, because the `P⁻¹` inversion in the backward pass is poorly conditioned in
/// single precision.
public struct FilterStep: Sendable {
    public var t: Double
    public var dt: Double
    /// Longitudinal specific force fed to this step, m/s².
    public var acceleration: Double
    /// Transition that carried the previous state to this one.
    public var F: Matrix
    /// `x_{k|k-1}` — the prediction for this index.
    public var xPredicted: Matrix
    /// `P_{k|k-1}`.
    public var PPredicted: Matrix
    /// `x_{k|k}` — after any measurement updates at this index.
    public var xFiltered: Matrix
    /// `P_{k|k}`.
    public var PFiltered: Matrix

    public var zuptApplied: Bool = false
    public var gnssApplied: Bool = false
    public var wheelSpeedApplied: Bool = false
    public var gateRejected: Bool = false

    public init(
        t: Double, dt: Double, acceleration: Double,
        F: Matrix, xPredicted: Matrix, PPredicted: Matrix,
        xFiltered: Matrix, PFiltered: Matrix
    ) {
        self.t = t
        self.dt = dt
        self.acceleration = acceleration
        self.F = F
        self.xPredicted = xPredicted
        self.PPredicted = PPredicted
        self.xFiltered = xFiltered
        self.PFiltered = PFiltered
    }
}

/// Spec §6 — the forward Kalman filter.
///
/// State is `x = [s, v, b]ᵀ` (distance, longitudinal speed, accelerometer bias along vehicle
/// +X), extended with the tyre circumference scale `k` when spec §3.7 wheel speed is enabled.
///
/// Note the division of labour set out in spec Decision 1: this filter drives the **live
/// display only**. Reported results always come from the RTS smoother running over the
/// recorded forward trace.
public struct KalmanFilter: Sendable {
    public let configuration: FilterConfiguration
    public let stateDimension: Int

    private var x: Matrix
    private var P: Matrix

    /// Count of fixes thrown out by the 3σ gate. Spec §6.3: "Log every rejection; a run with
    /// more than 2 rejections should be flagged as low confidence."
    public private(set) var gateRejectionCount = 0
    /// Count of fixes refused by the validity rules.
    public private(set) var invalidFixCount = 0
    /// Every rejection, for the log and the confidence badge.
    public private(set) var rejections: [(t: Double, outcome: MeasurementOutcome)] = []

    public init(configuration: FilterConfiguration = FilterConfiguration()) {
        self.configuration = configuration
        stateDimension = configuration.stateDimension
        x = Matrix(rows: stateDimension, columns: 1)
        var initial = [
            configuration.initialSigmaDistance * configuration.initialSigmaDistance,
            configuration.initialSigmaSpeed * configuration.initialSigmaSpeed,
            configuration.initialSigmaBias * configuration.initialSigmaBias,
        ]
        if configuration.wheelSpeedEnabled {
            x[StateIndex.wheelScale, 0] = 1.0
            initial.append(configuration.initialSigmaWheelScale * configuration.initialSigmaWheelScale)
        }
        P = Matrix.diagonal(initial)
    }

    // MARK: - State access

    public var state: Matrix { x }
    public var covariance: Matrix { P }

    public var distance: Double { x[StateIndex.distance, 0] }
    public var speed: Double { x[StateIndex.speed, 0] }
    public var accelerometerBias: Double { x[StateIndex.bias, 0] }
    public var wheelScale: Double {
        configuration.wheelSpeedEnabled ? x[StateIndex.wheelScale, 0] : 1.0
    }

    public mutating func setState(
        distance: Double, speed: Double, accelerometerBias: Double, wheelScale: Double = 1.0
    ) {
        x[StateIndex.distance, 0] = distance
        x[StateIndex.speed, 0] = speed
        x[StateIndex.bias, 0] = accelerometerBias
        if configuration.wheelSpeedEnabled { x[StateIndex.wheelScale, 0] = wheelScale }
    }

    public mutating func setCovariance(_ newValue: Matrix) {
        precondition(newValue.rows == stateDimension && newValue.columns == stateDimension,
                     "covariance dimension mismatch")
        P = newValue
    }

    // MARK: - Prediction

    /// Advance one step. Returns a `FilterStep` whose filtered fields still hold the
    /// prediction; call `finish(step:)` after any measurement updates to capture `x_{k|k}`.
    ///
    /// `dt` must be the measured interval between consecutive timestamps. Spec §3.1: some
    /// intervals get rounded to whatever the hardware supports, so never assume `1/100`.
    @discardableResult
    public mutating func predict(acceleration a: Double, dt: Double) -> FilterStep {
        let f = ProcessNoise.transition(dt: dt, dimension: stateDimension)
        let bu = ProcessNoise.controlInput(acceleration: a, dt: dt, dimension: stateDimension)
        let q = ProcessNoise.matrix(
            dt: dt,
            sigmaA: configuration.sigmaA,
            sigmaB: configuration.sigmaB,
            model: configuration.processNoiseModel,
            dimension: stateDimension,
            sigmaWheelScale: configuration.sigmaWheelScaleRandomWalk
        )

        x = f * x + bu
        P = (f * P * f.transposed + q).symmetrised()

        return FilterStep(
            t: 0, dt: dt, acceleration: a,
            F: f, xPredicted: x, PPredicted: P,
            xFiltered: x, PFiltered: P
        )
    }

    /// Capture the post-update state into a step produced by `predict`.
    public func finish(step: inout FilterStep) {
        step.xFiltered = x
        step.PFiltered = P
    }

    // MARK: - Measurement updates

    /// `R` for a GNSS speed update, with the §6.2 floor applied.
    public func measurementVariance(for fix: GNSSFix) -> Double {
        let sigma = max(fix.speedAccuracy, configuration.sigmaGPSFloor)
        return sigma * sigma
    }

    /// Spec §6.3 GNSS speed update. `H = [0, 1, 0]`, `R = max(speedAccuracy, floor)²`.
    @discardableResult
    public mutating func updateGNSSSpeed(_ fix: GNSSFix) -> MeasurementOutcome {
        if let reason = fix.rejectionReason(maximumSpeedAccuracy: configuration.maximumSpeedAccuracy) {
            invalidFixCount += 1
            let outcome = MeasurementOutcome.rejectedInvalid(reason)
            rejections.append((fix.t, outcome))
            return outcome
        }
        var h = Matrix(rows: 1, columns: stateDimension)
        h[0, StateIndex.speed] = 1
        return applyScalarUpdate(z: fix.speed, h: h, r: measurementVariance(for: fix),
                                 t: fix.t, gate: true)
    }

    /// Spec §6.3 ZUPT. Applied at every IMU sample while the stationary detector holds.
    ///
    /// This is what pins `b` to near-truth immediately before launch. The gate is deliberately
    /// **not** applied: a ZUPT is asserted by the stationary detector, not measured, and
    /// gating it would defeat its purpose exactly when a large bias makes it most valuable.
    @discardableResult
    public mutating func updateZeroVelocity() -> MeasurementOutcome {
        var h = Matrix(rows: 1, columns: stateDimension)
        h[0, StateIndex.speed] = 1
        return applyScalarUpdate(z: 0, h: h,
                                 r: configuration.sigmaZUPT * configuration.sigmaZUPT,
                                 t: 0, gate: false)
    }

    /// Spec §3.7 wheel-speed update. `z = k·v`, so `H = [0, k, 0, v]`.
    ///
    /// - Parameter currentAcceleration: longitudinal specific force, used for the wheelspin
    ///   gate. Pass the value fed to the matching `predict`.
    @discardableResult
    public mutating func updateWheelSpeed(
        _ sample: WheelSpeedSample,
        measurementSigma: Double,
        currentAcceleration: Double = 0
    ) -> MeasurementOutcome {
        guard configuration.wheelSpeedEnabled else { return .notConfigured }

        if abs(currentAcceleration) > configuration.wheelSpeedAccelerationGate {
            let outcome = MeasurementOutcome.rejectedByWheelSpeedGate(.launchAcceleration)
            rejections.append((sample.t, outcome))
            return outcome
        }
        if sample.speed - speed > configuration.wheelSpeedDisagreementGate {
            let outcome = MeasurementOutcome.rejectedByWheelSpeedGate(.disagreement)
            rejections.append((sample.t, outcome))
            return outcome
        }

        var h = Matrix(rows: 1, columns: stateDimension)
        h[0, StateIndex.speed] = wheelScale
        h[0, StateIndex.wheelScale] = speed
        // Non-linear measurement: innovation uses h(x) = k*v, not H*x.
        let predicted = wheelScale * speed
        return applyScalarUpdate(z: sample.speed, h: h, r: measurementSigma * measurementSigma,
                                 t: sample.t, gate: true, predictedMeasurement: predicted)
    }

    /// Spec §6.3 optional GNSS position update: path length as a measurement of `s`.
    /// Only worth enabling with an RTK receiver; with 3 m consumer accuracy it adds nothing
    /// over integrated Doppler.
    @discardableResult
    public mutating func updatePathLength(_ pathLength: Double, sigma: Double, t: Double)
        -> MeasurementOutcome
    {
        var h = Matrix(rows: 1, columns: stateDimension)
        h[0, StateIndex.distance] = 1
        return applyScalarUpdate(z: pathLength, h: h, r: sigma * sigma, t: t, gate: true)
    }

    // MARK: - Core scalar update

    /// Scalar Kalman update with optional 3σ innovation gating.
    ///
    /// Uses the Joseph form for the covariance. It costs one extra matrix product over the
    /// short form and guarantees the result stays symmetric positive-semi-definite even when
    /// the gain is imperfectly conditioned — which matters here because the smoother must
    /// later invert these matrices.
    private mutating func applyScalarUpdate(
        z: Double,
        h: Matrix,
        r: Double,
        t: Double,
        gate: Bool,
        predictedMeasurement: Double? = nil
    ) -> MeasurementOutcome {
        let predicted = predictedMeasurement ?? (h * x)[0, 0]
        let y = z - predicted
        let s = (h * P * h.transposed)[0, 0] + r

        guard s > 0, s.isFinite else { return .rejectedByGate(normalisedInnovationSquared: .infinity) }

        if gate {
            let normalised = y * y / s
            if normalised > configuration.innovationGateThreshold {
                gateRejectionCount += 1
                let outcome = MeasurementOutcome.rejectedByGate(normalisedInnovationSquared: normalised)
                rejections.append((t, outcome))
                return outcome
            }
        }

        let k = P * h.transposed * (1.0 / s)          // n×1
        x = x + k * y

        let identity = Matrix.identity(stateDimension)
        let a = identity - k * h
        P = (a * P * a.transposed + (k * k.transposed) * r).symmetrised()

        return .applied
    }
}

extension MeasurementOutcome {
    /// Convenience for tests and call sites that only care whether the gate fired.
    public static func == (lhs: MeasurementOutcome, rhs: MeasurementOutcome) -> Bool {
        switch (lhs, rhs) {
        case (.applied, .applied), (.notConfigured, .notConfigured):
            return true
        case (.rejectedInvalid(let a), .rejectedInvalid(let b)):
            return a == b
        case (.rejectedByGate, .rejectedByGate):
            return true
        case (.rejectedByWheelSpeedGate(let a), .rejectedByWheelSpeedGate(let b)):
            return a == b
        default:
            return false
        }
    }
}
