import Foundation

/// Dense row-major matrix of `Double`.
///
/// The Kalman filter of spec §6 is 3-state, or 4-state when the CAN wheel-speed channel of
/// §3.7 adds the tyre-circumference scale `k`. Rather than writing two hand-unrolled filters,
/// the estimator is dimension-generic over this type. At 100 Hz over a 20 s run the cost is
/// irrelevant (a few thousand 4×4 operations) and the correctness win is large.
///
/// `Double` only — spec §6.4 is explicit that the `P⁻¹` inversion in the RTS smoother is
/// poorly conditioned in single precision.
public struct Matrix: Equatable, Codable, Sendable {
    public let rows: Int
    public let columns: Int
    public private(set) var values: [Double]

    public init(rows: Int, columns: Int, values: [Double]) {
        precondition(rows > 0 && columns > 0, "matrix dimensions must be positive")
        precondition(values.count == rows * columns, "value count does not match dimensions")
        self.rows = rows
        self.columns = columns
        self.values = values
    }

    public init(rows: Int, columns: Int, repeating value: Double = 0) {
        self.init(rows: rows, columns: columns,
                  values: [Double](repeating: value, count: rows * columns))
    }

    /// Column vector from a flat array.
    public init(column: [Double]) {
        self.init(rows: column.count, columns: 1, values: column)
    }

    /// Row vector from a flat array.
    public init(row: [Double]) {
        self.init(rows: 1, columns: row.count, values: row)
    }

    public static func identity(_ n: Int) -> Matrix {
        var m = Matrix(rows: n, columns: n)
        for i in 0..<n { m[i, i] = 1 }
        return m
    }

    /// Square diagonal matrix from the given diagonal entries.
    public static func diagonal(_ d: [Double]) -> Matrix {
        var m = Matrix(rows: d.count, columns: d.count)
        for i in 0..<d.count { m[i, i] = d[i] }
        return m
    }

    public subscript(row: Int, column: Int) -> Double {
        get { values[row * columns + column] }
        set { values[row * columns + column] = newValue }
    }

    /// Element of a vector-shaped matrix (either `n×1` or `1×n`).
    public subscript(index: Int) -> Double {
        get {
            precondition(rows == 1 || columns == 1, "single-index access requires a vector")
            return values[index]
        }
        set {
            precondition(rows == 1 || columns == 1, "single-index access requires a vector")
            values[index] = newValue
        }
    }

    public var isSquare: Bool { rows == columns }

    public func column(_ j: Int) -> [Double] {
        (0..<rows).map { self[$0, j] }
    }

    public func rowValues(_ i: Int) -> [Double] {
        Array(values[(i * columns)..<((i + 1) * columns)])
    }

    public var transposed: Matrix {
        var out = Matrix(rows: columns, columns: rows)
        for i in 0..<rows {
            for j in 0..<columns {
                out[j, i] = self[i, j]
            }
        }
        return out
    }

    /// `(A + Aᵀ)/2`. Covariance matrices drift out of symmetry over thousands of Joseph-form
    /// updates; forcing symmetry each step keeps the smoother's inversions well behaved.
    public func symmetrised() -> Matrix {
        precondition(isSquare, "symmetrised requires a square matrix")
        var out = self
        for i in 0..<rows {
            for j in (i + 1)..<columns {
                let v = (self[i, j] + self[j, i]) / 2
                out[i, j] = v
                out[j, i] = v
            }
        }
        return out
    }

    public var trace: Double {
        precondition(isSquare, "trace requires a square matrix")
        return (0..<rows).reduce(0) { $0 + self[$1, $1] }
    }

    // MARK: - Arithmetic

    public static func + (a: Matrix, b: Matrix) -> Matrix {
        precondition(a.rows == b.rows && a.columns == b.columns, "dimension mismatch in +")
        var out = a
        for i in 0..<a.values.count { out.values[i] += b.values[i] }
        return out
    }

    public static func - (a: Matrix, b: Matrix) -> Matrix {
        precondition(a.rows == b.rows && a.columns == b.columns, "dimension mismatch in -")
        var out = a
        for i in 0..<a.values.count { out.values[i] -= b.values[i] }
        return out
    }

    public static func * (a: Matrix, b: Matrix) -> Matrix {
        precondition(a.columns == b.rows, "dimension mismatch in *")
        var out = Matrix(rows: a.rows, columns: b.columns)
        for i in 0..<a.rows {
            for k in 0..<a.columns {
                let aik = a[i, k]
                if aik == 0 { continue }
                for j in 0..<b.columns {
                    out[i, j] += aik * b[k, j]
                }
            }
        }
        return out
    }

    public static func * (a: Matrix, s: Double) -> Matrix {
        var out = a
        for i in 0..<out.values.count { out.values[i] *= s }
        return out
    }

    public static func * (s: Double, a: Matrix) -> Matrix { a * s }

    // MARK: - Inversion

    /// Gauss–Jordan inverse with partial pivoting.
    ///
    /// Returns `nil` when `|det| < determinantEpsilon`. Spec §6.4 requires exactly this guard
    /// on the `P_{k+1|k}` inversion inside the RTS recursion: when it trips, the smoother
    /// skips that index and carries the forward estimate through instead of producing
    /// garbage from a near-singular solve.
    public func inverted(determinantEpsilon: Double = 1e-12) -> Matrix? {
        precondition(isSquare, "inverted requires a square matrix")
        let n = rows
        var a = self
        var inv = Matrix.identity(n)
        var determinant = 1.0

        for col in 0..<n {
            // Partial pivot: largest magnitude in this column at or below the diagonal.
            var pivotRow = col
            var pivotMagnitude = abs(a[col, col])
            for r in (col + 1)..<n where abs(a[r, col]) > pivotMagnitude {
                pivotMagnitude = abs(a[r, col])
                pivotRow = r
            }
            guard pivotMagnitude > 0 else { return nil }

            if pivotRow != col {
                for j in 0..<n {
                    a.values.swapAt(col * n + j, pivotRow * n + j)
                    inv.values.swapAt(col * n + j, pivotRow * n + j)
                }
                determinant = -determinant
            }

            let pivot = a[col, col]
            determinant *= pivot
            let invPivot = 1.0 / pivot
            for j in 0..<n {
                a[col, j] *= invPivot
                inv[col, j] *= invPivot
            }

            for r in 0..<n where r != col {
                let factor = a[r, col]
                if factor == 0 { continue }
                for j in 0..<n {
                    a[r, j] -= factor * a[col, j]
                    inv[r, j] -= factor * inv[col, j]
                }
            }
        }

        guard abs(determinant) >= determinantEpsilon, determinant.isFinite else { return nil }
        guard inv.values.allSatisfy({ $0.isFinite }) else { return nil }
        return inv
    }

    /// Determinant via the same elimination used by `inverted()`.
    public func determinant() -> Double {
        precondition(isSquare, "determinant requires a square matrix")
        let n = rows
        var a = self
        var det = 1.0
        for col in 0..<n {
            var pivotRow = col
            var pivotMagnitude = abs(a[col, col])
            for r in (col + 1)..<n where abs(a[r, col]) > pivotMagnitude {
                pivotMagnitude = abs(a[r, col])
                pivotRow = r
            }
            guard pivotMagnitude > 0 else { return 0 }
            if pivotRow != col {
                for j in 0..<n { a.values.swapAt(col * n + j, pivotRow * n + j) }
                det = -det
            }
            let pivot = a[col, col]
            det *= pivot
            for r in (col + 1)..<n {
                let factor = a[r, col] / pivot
                if factor == 0 { continue }
                for j in col..<n { a[r, j] -= factor * a[col, j] }
            }
        }
        return det
    }

    // MARK: - Symmetric eigen-decomposition

    /// Cyclic Jacobi eigen-decomposition for symmetric matrices.
    ///
    /// Returns eigenvalues in **descending** order and the matching eigenvectors as the
    /// **columns** of the second result. Used by `Quaternion.average` (spec §5's cross-run
    /// calibration refinement). Jacobi is chosen over power iteration because it is
    /// unconditionally convergent for symmetric input and does not degrade when the two
    /// largest eigenvalues are close — which is the normal case when averaging a handful of
    /// near-identical calibration quaternions.
    public func symmetricEigenDecomposition(
        maxSweeps: Int = 100,
        tolerance: Double = 1e-14
    ) -> (values: [Double], vectors: Matrix)? {
        precondition(isSquare, "eigen-decomposition requires a square matrix")
        let n = rows
        var a = symmetrised()
        var v = Matrix.identity(n)

        for _ in 0..<maxSweeps {
            // Sum of squares of the strictly upper triangle.
            var off = 0.0
            for i in 0..<n {
                for j in (i + 1)..<n { off += a[i, j] * a[i, j] }
            }
            if off <= tolerance { break }

            for p in 0..<(n - 1) {
                for q in (p + 1)..<n {
                    let apq = a[p, q]
                    if abs(apq) <= tolerance { continue }
                    let app = a[p, p]
                    let aqq = a[q, q]
                    let theta = (aqq - app) / (2 * apq)
                    let t: Double = {
                        let sign: Double = theta >= 0 ? 1 : -1
                        return sign / (abs(theta) + (theta * theta + 1).squareRoot())
                    }()
                    let c = 1 / (t * t + 1).squareRoot()
                    let s = t * c

                    for k in 0..<n {
                        let akp = a[k, p]
                        let akq = a[k, q]
                        a[k, p] = c * akp - s * akq
                        a[k, q] = s * akp + c * akq
                    }
                    for k in 0..<n {
                        let apk = a[p, k]
                        let aqk = a[q, k]
                        a[p, k] = c * apk - s * aqk
                        a[q, k] = s * apk + c * aqk
                    }
                    for k in 0..<n {
                        let vkp = v[k, p]
                        let vkq = v[k, q]
                        v[k, p] = c * vkp - s * vkq
                        v[k, q] = s * vkp + c * vkq
                    }
                }
            }
        }

        let eigenvalues = (0..<n).map { a[$0, $0] }
        let order = (0..<n).sorted { eigenvalues[$0] > eigenvalues[$1] }
        var sortedVectors = Matrix(rows: n, columns: n)
        for (newIndex, oldIndex) in order.enumerated() {
            for k in 0..<n { sortedVectors[k, newIndex] = v[k, oldIndex] }
        }
        return (order.map { eigenvalues[$0] }, sortedVectors)
    }
}

extension Matrix {
    /// Convenience for reading a column as a `[Double]`, used by the eigen path.
    public func columnVector(_ j: Int) -> [Double] { column(j) }
}
