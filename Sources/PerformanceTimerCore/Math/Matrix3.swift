import Foundation

/// A 3×3 matrix stored row-major.
///
/// The primary use is `R_DV`, the device→vehicle rotation of spec §5, whose **rows** are the
/// vehicle basis axes expressed in device coordinates. With that layout `R_DV * v_D` yields
/// the components of `v_D` along the vehicle axes, which is exactly the projection §5 asks for.
public struct Matrix3: Equatable, Codable, Sendable {
    /// Row-major, 9 entries.
    public private(set) var m: [Double]

    public init(m: [Double]) {
        precondition(m.count == 9, "Matrix3 requires exactly 9 values")
        self.m = m
    }

    /// Build from three row vectors. Spec §5: `R_DV` has rows `[x_V; y_V; z_V]`.
    public init(rows r0: Vector3, _ r1: Vector3, _ r2: Vector3) {
        m = [r0.x, r0.y, r0.z, r1.x, r1.y, r1.z, r2.x, r2.y, r2.z]
    }

    public static let identity = Matrix3(m: [1, 0, 0, 0, 1, 0, 0, 0, 1])

    public subscript(row: Int, column: Int) -> Double {
        get { m[row * 3 + column] }
        set { m[row * 3 + column] = newValue }
    }

    public func row(_ i: Int) -> Vector3 {
        Vector3(m[i * 3], m[i * 3 + 1], m[i * 3 + 2])
    }

    public func column(_ j: Int) -> Vector3 {
        Vector3(m[j], m[3 + j], m[6 + j])
    }

    public var transposed: Matrix3 {
        Matrix3(m: [
            m[0], m[3], m[6],
            m[1], m[4], m[7],
            m[2], m[5], m[8],
        ])
    }

    public var determinant: Double {
        m[0] * (m[4] * m[8] - m[5] * m[7])
            - m[1] * (m[3] * m[8] - m[5] * m[6])
            + m[2] * (m[3] * m[7] - m[4] * m[6])
    }

    public static func * (a: Matrix3, v: Vector3) -> Vector3 {
        Vector3(
            a.m[0] * v.x + a.m[1] * v.y + a.m[2] * v.z,
            a.m[3] * v.x + a.m[4] * v.y + a.m[5] * v.z,
            a.m[6] * v.x + a.m[7] * v.y + a.m[8] * v.z
        )
    }

    public static func * (a: Matrix3, b: Matrix3) -> Matrix3 {
        var out = [Double](repeating: 0, count: 9)
        for i in 0..<3 {
            for j in 0..<3 {
                var s = 0.0
                for k in 0..<3 { s += a.m[i * 3 + k] * b.m[k * 3 + j] }
                out[i * 3 + j] = s
            }
        }
        return Matrix3(m: out)
    }

    /// Gram–Schmidt the rows into a right-handed orthonormal basis.
    ///
    /// Spec §5 step 3 does exactly this by hand (`y_V = z_V × x_V`, then re-orthogonalise
    /// `x_V = y_V × z_V`). Doing it generically here means any accumulated numerical drift in
    /// a stored or averaged `R_DV` is cleaned up before use.
    ///
    /// Returns `nil` if the rows are degenerate (parallel or zero-length).
    public func orthonormalised(epsilon: Double = 1e-12) -> Matrix3? {
        guard let x = row(0).normalized(epsilon: epsilon) else { return nil }
        let y1 = row(1).removingComponent(along: x)
        guard let y = y1.normalized(epsilon: epsilon) else { return nil }
        let z = x.cross(y)
        guard z.length > epsilon else { return nil }
        return Matrix3(rows: x, y, z)
    }
}
