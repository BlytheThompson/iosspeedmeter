import Foundation

/// One smoothed sample. **Every reported result comes from these** (spec §6.4); the forward
/// trace is display-only.
public struct SmoothedSample: Sendable {
    public var t: Double
    public var dt: Double
    /// Longitudinal specific force at this sample, m/s². Carried through because the crossing
    /// interpolation of spec §9.1 needs `a` alongside `s` and `v`.
    public var acceleration: Double
    public var state: Matrix
    public var covariance: Matrix
    /// True when `P_{k+1|k}` was too ill-conditioned to invert and the forward estimate was
    /// carried through instead (spec §6.4's determinant guard).
    public var carriedForwardDueToSingularCovariance: Bool

    public var distance: Double { state[StateIndex.distance, 0] }
    public var speed: Double { state[StateIndex.speed, 0] }
    public var accelerometerBias: Double { state[StateIndex.bias, 0] }
    public var wheelScale: Double {
        state.rows > StateIndex.wheelScale ? state[StateIndex.wheelScale, 0] : 1.0
    }

    /// 1σ on speed, m/s — the input to the confidence figure of spec §9.4.
    public var speedSigma: Double { max(0, covariance[StateIndex.speed, StateIndex.speed]).squareRoot() }
    /// 1σ on distance, m.
    public var distanceSigma: Double {
        max(0, covariance[StateIndex.distance, StateIndex.distance]).squareRoot()
    }

    /// True longitudinal acceleration implied by this sample: measured minus estimated bias.
    public var correctedAcceleration: Double { acceleration - accelerometerBias }
}

/// Spec §6.4 — Rauch–Tung–Striebel fixed-interval smoother.
///
/// ```
/// C_k    = P_k|k · F_{k+1}ᵀ · (P_{k+1|k})⁻¹
/// x_k|N  = x_k|k + C_k · (x_{k+1|N} − x_{k+1|k})
/// P_k|N  = P_k|k + C_k · (P_{k+1|N} − P_{k+1|k}) · C_kᵀ
/// ```
///
/// This is the single biggest accuracy lever in the whole design (spec Decision 1): it lets a
/// GNSS fix arriving at t = 3 s sharpen the state estimate at t = 1 s, which a forward filter
/// structurally cannot do. Spec §13 puts it at roughly half the 0–60 uncertainty of a
/// forward-only filter, for no hardware at all.
public enum RTSSmoother {
    /// Run the backward recursion over a recorded forward pass.
    public static func smooth(_ steps: [FilterStep]) -> [SmoothedSample] {
        guard !steps.isEmpty else { return [] }

        let n = steps.count
        var samples = [SmoothedSample]()
        samples.reserveCapacity(n)

        // Seed with the forward estimates; the last index is already correct by construction
        // (x_{N-1|N} == x_{N-1|N-1}).
        for step in steps {
            samples.append(SmoothedSample(
                t: step.t,
                dt: step.dt,
                acceleration: step.acceleration,
                state: step.xFiltered,
                covariance: step.PFiltered,
                carriedForwardDueToSingularCovariance: false
            ))
        }

        guard n > 1 else { return samples }

        for k in stride(from: n - 2, through: 0, by: -1) {
            let next = steps[k + 1]

            // Spec §6.4: invert P_{k+1|k} with a determinant guard; if it is too close to
            // singular, skip this index and carry x_k|k forward rather than producing noise.
            guard let inversePredicted = next.PPredicted.inverted(determinantEpsilon: 1e-12) else {
                samples[k].carriedForwardDueToSingularCovariance = true
                continue
            }

            let c = steps[k].PFiltered * next.F.transposed * inversePredicted

            let stateCorrection = c * (samples[k + 1].state - next.xPredicted)
            samples[k].state = steps[k].xFiltered + stateCorrection

            let covarianceCorrection =
                c * (samples[k + 1].covariance - next.PPredicted) * c.transposed
            samples[k].covariance = (steps[k].PFiltered + covarianceCorrection).symmetrised()
        }

        return samples
    }
}
