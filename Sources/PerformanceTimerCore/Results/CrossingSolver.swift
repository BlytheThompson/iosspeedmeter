import Foundation

/// Spec §9.1 — interpolated crossings.
///
/// "Never report the nearest sample." At 100 Hz the nearest sample can be 5 ms from the true
/// crossing, which is a sixth of the entire error budget for a Dragy-class result — and it is
/// a bias, not noise, so averaging runs will not remove it.
///
/// Both solvers assume the state is locally a constant-jerk motion, using the jerk implied by
/// two consecutive smoothed accelerations. That makes them *exact* for constant jerk rather
/// than merely better than linear.
public enum CrossingSolver {
    public struct Crossing: Equatable, Sendable {
        /// Session time of the crossing.
        public var time: Double
        /// Index of the sample bracketing the crossing from below.
        public var index: Int
        /// Offset from `samples[index].t` to the crossing, seconds.
        public var tau: Double
        public var speed: Double
        public var distance: Double
        /// True longitudinal acceleration at the crossing (bias removed).
        public var acceleration: Double
        /// 1σ on speed at the crossing, m/s.
        public var speedSigma: Double
        /// 1σ on distance at the crossing, m.
        public var distanceSigma: Double
    }

    /// Solve for the first time the speed trace reaches `target`.
    ///
    /// Spec §9.1: find `i` where `v[i] ≤ v_t < v[i+1]`, take `j = (a[i+1] − a[i]) / dt`, and
    /// solve `v_t = v[i] + a[i]·τ + ½·j·τ²` for the root in `[0, dt]`.
    public static func speedCrossing(target: Double, samples: [SmoothedSample]) -> Crossing? {
        guard samples.count >= 2 else { return nil }

        for i in 0..<(samples.count - 1) {
            let low = samples[i]
            let high = samples[i + 1]
            guard low.speed <= target, high.speed > target else { continue }

            let dt = high.t - low.t
            guard dt > 0 else { continue }

            let a0 = low.correctedAcceleration
            let jerk = (high.correctedAcceleration - a0) / dt
            let c = low.speed - target

            guard let tau = solveQuadratic(halfJerk: jerk / 2, linear: a0, constant: c,
                                           upperBound: dt) else { continue }
            return crossing(at: i, tau: tau, samples: samples)
        }
        return nil
    }

    /// Solve for the first time the distance trace reaches `target`.
    ///
    /// Spec §9.1: `s_t = s[i] + v[i]·τ + ½·a[i]·τ² + (1/6)·j·τ³`, solved by Newton iteration
    /// from `τ₀ = (s_t − s[i]) / v[i]`.
    ///
    /// That seed divides by zero at the launch sample, where `v = 0` — and the rollout mark
    /// (0.3048 m) is always crossed in that first fraction of a second, so this is the normal
    /// path rather than an edge case. The seed is clamped into the bracket and Newton is
    /// backed by bisection, which also guarantees convergence when the cubic is nearly flat.
    public static func distanceCrossing(target: Double, samples: [SmoothedSample]) -> Crossing? {
        guard samples.count >= 2 else { return nil }

        for i in 0..<(samples.count - 1) {
            let low = samples[i]
            let high = samples[i + 1]
            guard low.distance <= target, high.distance > target else { continue }

            let dt = high.t - low.t
            guard dt > 0 else { continue }

            let v0 = low.speed
            let a0 = low.correctedAcceleration
            let jerk = (high.correctedAcceleration - a0) / dt
            let offset = low.distance - target

            func f(_ tau: Double) -> Double {
                offset + v0 * tau + 0.5 * a0 * tau * tau + jerk * tau * tau * tau / 6
            }
            func df(_ tau: Double) -> Double {
                v0 + a0 * tau + 0.5 * jerk * tau * tau
            }

            // Spec's seed, clamped into the bracket; falls back to the midpoint when v0 is
            // zero or the seed lands outside.
            var tau = v0 > 1e-9 ? (target - low.distance) / v0 : dt / 2
            if !(tau.isFinite) || tau <= 0 || tau >= dt { tau = dt / 2 }

            var lower = 0.0
            var upper = dt
            for _ in 0..<60 {
                let value = f(tau)
                if abs(value) < 1e-14 { break }
                if value > 0 { upper = tau } else { lower = tau }

                let slope = df(tau)
                var next = slope != 0 ? tau - value / slope : .nan
                if !next.isFinite || next <= lower || next >= upper {
                    next = (lower + upper) / 2      // bisection fallback
                }
                if abs(next - tau) < 1e-15 { tau = next; break }
                tau = next
            }
            return crossing(at: i, tau: tau, samples: samples)
        }
        return nil
    }

    /// Smallest non-negative root of `A·τ² + B·τ + C = 0` within `[0, upperBound]`.
    private static func solveQuadratic(
        halfJerk a: Double, linear b: Double, constant c: Double, upperBound: Double
    ) -> Double? {
        // Degenerate to linear when the jerk term is negligible.
        if abs(a) < 1e-12 {
            guard abs(b) > 1e-12 else { return nil }
            let tau = -c / b
            return (tau >= 0 && tau <= upperBound * (1 + 1e-9)) ? min(tau, upperBound) : nil
        }

        let discriminant = b * b - 4 * a * c
        guard discriminant >= 0 else { return nil }
        let root = discriminant.squareRoot()

        // Numerically stable pair of roots.
        let q = -0.5 * (b + (b >= 0 ? root : -root))
        let candidates = [q / a, c == 0 ? 0 : c / q].filter { $0.isFinite }

        let inRange = candidates
            .filter { $0 >= -1e-12 && $0 <= upperBound * (1 + 1e-9) }
            .map { min(max($0, 0), upperBound) }
            .sorted()
        return inRange.first
    }

    /// Evaluate the interpolated state at `samples[index].t + tau`.
    private static func crossing(at index: Int, tau: Double, samples: [SmoothedSample]) -> Crossing {
        let low = samples[index]
        let high = samples[index + 1]
        let dt = high.t - low.t
        let a0 = low.correctedAcceleration
        let jerk = dt > 0 ? (high.correctedAcceleration - a0) / dt : 0

        let speed = low.speed + a0 * tau + 0.5 * jerk * tau * tau
        let distance = low.distance + low.speed * tau + 0.5 * a0 * tau * tau
            + jerk * tau * tau * tau / 6
        let acceleration = a0 + jerk * tau

        // Linearly blend the covariance across the interval — it changes far more slowly than
        // the state, so anything more elaborate would be false precision.
        let blend = dt > 0 ? tau / dt : 0
        let speedSigma = low.speedSigma + (high.speedSigma - low.speedSigma) * blend
        let distanceSigma = low.distanceSigma + (high.distanceSigma - low.distanceSigma) * blend

        return Crossing(
            time: low.t + tau,
            index: index,
            tau: tau,
            speed: speed,
            distance: distance,
            acceleration: acceleration,
            speedSigma: speedSigma,
            distanceSigma: distanceSigma
        )
    }
}
