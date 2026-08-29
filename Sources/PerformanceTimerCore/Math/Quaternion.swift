import Foundation

/// Unit quaternion representing a rotation from the body (device) frame to a reference frame.
///
/// `rotate(v)` computes `q ⊗ v ⊗ q*`, i.e. it takes a **body-frame** vector and returns it in
/// the **reference** frame. This is the convention CoreMotion's `CMAttitude.quaternion` uses,
/// and it is what spec §3.2's launch-anchored propagation needs: integrate the device-frame
/// gyro forward from `q_launch`, then use the result to express the reference-frame gravity
/// vector back in the device frame.
public struct Quaternion: Equatable, Codable, Sendable {
    public var w: Double
    public var x: Double
    public var y: Double
    public var z: Double

    public init(w: Double, x: Double, y: Double, z: Double) {
        self.w = w
        self.x = x
        self.y = y
        self.z = z
    }

    public static let identity = Quaternion(w: 1, x: 0, y: 0, z: 0)

    /// Rotation of `angle` radians about `axis` (need not be unit length).
    public init(axis: Vector3, angle: Double) {
        let unit = axis.normalized() ?? Vector3(0, 0, 1)
        let half = angle / 2
        let s = sin(half)
        w = cos(half)
        x = unit.x * s
        y = unit.y * s
        z = unit.z * s
    }

    public var length: Double {
        (w * w + x * x + y * y + z * z).squareRoot()
    }

    public func normalized() -> Quaternion {
        let l = length
        guard l > 1e-15 else { return .identity }
        return Quaternion(w: w / l, x: x / l, y: y / l, z: z / l)
    }

    public var conjugate: Quaternion {
        Quaternion(w: w, x: -x, y: -y, z: -z)
    }

    /// Inverse rotation. For a unit quaternion this is the conjugate.
    public var inverse: Quaternion { normalized().conjugate }

    public var vectorPart: Vector3 { Vector3(x, y, z) }

    public func dot(_ other: Quaternion) -> Double {
        w * other.w + x * other.x + y * other.y + z * other.z
    }

    public static prefix func - (q: Quaternion) -> Quaternion {
        Quaternion(w: -q.w, x: -q.x, y: -q.y, z: -q.z)
    }

    /// Hamilton product. `a ⊗ b` applies `b` first, then `a`.
    public static func * (a: Quaternion, b: Quaternion) -> Quaternion {
        Quaternion(
            w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
        )
    }

    /// Rotate a body-frame vector into the reference frame.
    public func rotate(_ v: Vector3) -> Vector3 {
        // v' = v + 2 * q_vec × (q_vec × v + w * v) — algebraically identical to q v q*,
        // and cheaper and better conditioned than forming the full matrix.
        let u = vectorPart
        let t = u.cross(v) * 2.0
        return v + t * w + u.cross(t)
    }

    /// Rotate a reference-frame vector into the body frame.
    public func rotateInverse(_ v: Vector3) -> Vector3 {
        conjugate.rotate(v)
    }

    /// Rotation matrix `R` with `v_reference = R * v_body`.
    public var rotationMatrix: Matrix3 {
        let n = normalized()
        let (w, x, y, z) = (n.w, n.x, n.y, n.z)
        return Matrix3(m: [
            1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
            2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
            2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
        ])
    }

    /// Propagate attitude forward by a body-frame angular velocity over `dt`.
    ///
    /// This is the core of spec §3.2: rather than trusting CoreMotion's `gravity` (which tilts
    /// toward sustained longitudinal acceleration), integrate the bias-corrected gyro yourself
    /// from the stationary-window anchor.
    ///
    /// The exact incremental rotation is used rather than the first-order `q + ½ q ω dt`
    /// approximation, so a 100 Hz stream accumulates no systematic magnitude error, and the
    /// result is renormalised every step.
    public func integrating(angularVelocity omega: Vector3, dt: Double) -> Quaternion {
        let rate = omega.length
        guard rate > 1e-12, dt != 0 else { return self }
        let angle = rate * dt
        let delta = Quaternion(axis: omega / rate, angle: angle)
        return (self * delta).normalized()
    }

    /// Markley's quaternion average: the eigenvector of `Σ qᵢqᵢᵀ` with the largest eigenvalue.
    ///
    /// Spec §5 requires this ("use quaternion averaging, not component averaging") when
    /// refining `x_V` across runs. Component-wise averaging is wrong because `q` and `−q` are
    /// the same rotation but cancel when added; this formulation is sign-invariant because
    /// the outer product `qqᵀ` is unchanged by a sign flip.
    ///
    /// Returns `nil` for an empty input.
    public static func average(_ quaternions: [Quaternion], weights: [Double]? = nil) -> Quaternion? {
        guard !quaternions.isEmpty else { return nil }
        if let weights { precondition(weights.count == quaternions.count, "weight count mismatch") }

        var accumulator = Matrix(rows: 4, columns: 4, repeating: 0)
        for (index, raw) in quaternions.enumerated() {
            let q = raw.normalized()
            let weight = weights?[index] ?? 1.0
            let components = [q.w, q.x, q.y, q.z]
            for i in 0..<4 {
                for j in 0..<4 {
                    accumulator[i, j] += weight * components[i] * components[j]
                }
            }
        }

        guard let (_, vectors) = accumulator.symmetricEigenDecomposition() else { return nil }
        // symmetricEigenDecomposition returns eigenvalues in descending order, eigenvectors
        // as columns, so column 0 is the dominant one.
        let v = vectors.column(0)
        let q = Quaternion(w: v[0], x: v[1], y: v[2], z: v[3]).normalized()
        // Canonicalise the sign so repeated averaging is stable.
        return q.w < 0 ? -q : q
    }
}
