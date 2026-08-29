import Foundation

/// Which discretisation to use for the process-noise matrix `Q`.
public enum ProcessNoiseModel: String, Equatable, Codable, Sendable, CaseIterable {
    /// Exact discretisation of the continuous model the spec describes. **Default.**
    case exact
    /// The table printed in spec §6.2, verbatim. Kept only for comparison — see the
    /// discussion on `ProcessNoise`.
    case specLiteral
}

/// Spec §6.2 — process noise.
///
/// **Deviation D1 from the spec.** The published `Q` table is dimensionally inconsistent with
/// the units the same section defines, and the discrepancy is large enough to break the
/// filter.
///
/// §6.2 states `σ_a` is accelerometer white noise in **m/s²/√Hz** and `σ_b` is bias random
/// walk in **m/s³/√Hz**, then gives `Q[1][1] = σ_a²·dt³/3`. Since `σ_a²` has units m²/s³,
/// that expression evaluates to m² — a *position* variance sitting in the velocity slot.
/// Every `σ_a` term in the table is shifted one integration too high, and the `σ_b`
/// contributions to `Q[0][1]` and `Q[1][1]` are missing.
///
/// The consequence is not cosmetic. At `dt = 0.01`, `σ_a = 0.05`, the published entry is
/// ~30,000× smaller than the correct one (the ratio is exactly `3/dt²`). A filter using it
/// believes its own dead-reckoned velocity to a fraction of a millimetre per second after a
/// second of integration and therefore ignores essentially every GNSS update — which defeats
/// the entire point of the fusion.
///
/// The exact discretisation of the spec's own continuous model
/// (`ṡ = v`, `v̇ = a − b`, `ḃ = w_b`, with `w_a` entering `v̇`) is
/// `Q = ∫₀^dt Φ(τ)·(G_a σ_a² G_aᵀ + G_b σ_b² G_bᵀ)·Φ(τ)ᵀ dτ`, giving
///
/// ```
/// Q = σ_a²·⎡ dt³/3  dt²/2  0 ⎤  +  σ_b²·⎡ dt⁵/20  dt⁴/8  −dt³/6 ⎤
///          ⎢ dt²/2  dt     0 ⎥          ⎢ dt⁴/8   dt³/3  −dt²/2 ⎥
///          ⎣ 0      0      0 ⎦          ⎣ −dt³/6  −dt²/2  dt     ⎦
/// ```
///
/// which is what `.exact` implements and what the estimator uses by default. `.specLiteral`
/// reproduces the published table so the two can be compared on recorded data — spec §6.2
/// does say "tune from logged data, don't trust these blindly".
public enum ProcessNoise {
    /// Build `Q` for one step.
    ///
    /// - Parameters:
    ///   - dt: timestep, s. Always the measured interval between consecutive samples, never
    ///     an assumed `1/100` (spec §3.1).
    ///   - sigmaA: accelerometer white noise, m/s²/√Hz.
    ///   - sigmaB: bias random walk, m/s³/√Hz.
    ///   - dimension: 3, or 4 when the §3.7 wheel-scale state is enabled.
    ///   - sigmaWheelScale: random walk on the tyre circumference scale, 1/√Hz.
    public static func matrix(
        dt: Double,
        sigmaA: Double,
        sigmaB: Double,
        model: ProcessNoiseModel = .exact,
        dimension: Int = 3,
        sigmaWheelScale: Double = 0
    ) -> Matrix {
        precondition(dimension == 3 || dimension == 4, "state must be 3 or 4 dimensional")

        var q = Matrix(rows: dimension, columns: dimension)
        let a2 = sigmaA * sigmaA
        let b2 = sigmaB * sigmaB
        let dt2 = dt * dt
        let dt3 = dt2 * dt
        let dt4 = dt3 * dt
        let dt5 = dt4 * dt

        switch model {
        case .exact:
            q[0, 0] = a2 * dt3 / 3 + b2 * dt5 / 20
            q[0, 1] = a2 * dt2 / 2 + b2 * dt4 / 8
            q[0, 2] = -b2 * dt3 / 6
            q[1, 1] = a2 * dt + b2 * dt3 / 3
            q[1, 2] = -b2 * dt2 / 2
            q[2, 2] = b2 * dt

        case .specLiteral:
            q[0, 0] = a2 * dt5 / 20 + b2 * dt5 / 20
            q[0, 1] = a2 * dt4 / 8
            q[0, 2] = -b2 * dt3 / 6
            q[1, 1] = a2 * dt3 / 3
            q[1, 2] = -b2 * dt2 / 2
            q[2, 2] = b2 * dt
        }

        // Mirror the upper triangle.
        q[1, 0] = q[0, 1]
        q[2, 0] = q[0, 2]
        q[2, 1] = q[1, 2]

        if dimension == 4 {
            // The tyre circumference scale is an independent random walk; it does not couple
            // to the inertial states through the process model.
            q[3, 3] = sigmaWheelScale * sigmaWheelScale * dt
        }

        return q
    }

    /// State transition for one step (spec §6.2).
    ///
    /// ```
    ///      ⎡ 1  dt  −dt²/2 ⎤
    /// F =  ⎢ 0  1   −dt    ⎥
    ///      ⎣ 0  0    1     ⎦
    /// ```
    /// The wheel-scale state, when present, is constant across a step.
    public static func transition(dt: Double, dimension: Int = 3) -> Matrix {
        var f = Matrix.identity(dimension)
        f[0, 1] = dt
        f[0, 2] = -dt * dt / 2
        f[1, 2] = -dt
        return f
    }

    /// Control input for one step: `Bu = [a·dt²/2, a·dt, 0]ᵀ` (spec §6.2).
    public static func controlInput(acceleration a: Double, dt: Double, dimension: Int = 3) -> Matrix {
        var bu = Matrix(rows: dimension, columns: 1)
        bu[0, 0] = a * dt * dt / 2
        bu[1, 0] = a * dt
        return bu
    }
}
