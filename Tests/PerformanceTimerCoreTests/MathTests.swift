import XCTest
@testable import PerformanceTimerCore

final class Vector3Tests: XCTestCase {
    func testDotAndCrossFollowRightHandRule() {
        let x = Vector3(1, 0, 0)
        let y = Vector3(0, 1, 0)
        XCTAssertEqual(x.dot(y), 0, accuracy: 1e-15)
        let z = x.cross(y)
        XCTAssertEqual(z.x, 0, accuracy: 1e-15)
        XCTAssertEqual(z.y, 0, accuracy: 1e-15)
        XCTAssertEqual(z.z, 1, accuracy: 1e-15)
    }

    func testNormalizedHasUnitLength() {
        let v = Vector3(3, 4, 12)
        XCTAssertEqual(v.length, 13, accuracy: 1e-12)
        XCTAssertEqual(v.normalized()!.length, 1, accuracy: 1e-15)
    }

    func testNormalizedReturnsNilForZeroVector() {
        XCTAssertNil(Vector3(0, 0, 0).normalized())
    }

    func testProjectionRemovesComponentAlongAxis() {
        // Spec §5 step 2: a_h = a - (a . z_V) z_V must be orthogonal to z_V.
        let axis = Vector3(0, 0, 1)
        let a = Vector3(2, -1, 7)
        let horizontal = a - axis * a.dot(axis)
        XCTAssertEqual(horizontal.dot(axis), 0, accuracy: 1e-15)
        XCTAssertEqual(horizontal.z, 0, accuracy: 1e-15)
    }
}

final class Matrix3Tests: XCTestCase {
    func testRowsConstructorAppliesRowsAsBasisVectors() {
        // Spec §5: R_DV has rows [x_V; y_V; z_V], so R * v yields v's components
        // along each vehicle axis.
        let xV = Vector3(0, 1, 0)
        let yV = Vector3(0, 0, 1)
        let zV = Vector3(1, 0, 0)
        let r = Matrix3(rows: xV, yV, zV)
        let deviceVector = Vector3(7, 5, 3)
        let vehicle = r * deviceVector
        XCTAssertEqual(vehicle.x, 5, accuracy: 1e-15)   // along xV = device +Y
        XCTAssertEqual(vehicle.y, 3, accuracy: 1e-15)   // along yV = device +Z
        XCTAssertEqual(vehicle.z, 7, accuracy: 1e-15)   // along zV = device +X
    }

    func testTransposeOfRotationIsItsInverse() {
        let q = Quaternion(axis: Vector3(1, 2, 3).normalized()!, angle: 0.7)
        let r = q.rotationMatrix
        let identity = r * r.transposed
        for i in 0..<3 {
            for j in 0..<3 {
                XCTAssertEqual(identity[i, j], i == j ? 1 : 0, accuracy: 1e-12)
            }
        }
    }

    func testOrthonormalisedProducesRightHandedOrthonormalBasis() {
        // Feed deliberately non-orthogonal, non-unit axes.
        let m = Matrix3(rows: Vector3(2, 0.1, 0), Vector3(0.05, 3, 0), Vector3(0, 0, 0.5))
        let o = m.orthonormalised()!
        let x = o.row(0), y = o.row(1), z = o.row(2)
        XCTAssertEqual(x.length, 1, accuracy: 1e-12)
        XCTAssertEqual(y.length, 1, accuracy: 1e-12)
        XCTAssertEqual(z.length, 1, accuracy: 1e-12)
        XCTAssertEqual(x.dot(y), 0, accuracy: 1e-12)
        XCTAssertEqual(y.dot(z), 0, accuracy: 1e-12)
        XCTAssertEqual(x.dot(z), 0, accuracy: 1e-12)
        // Right-handed: x cross y == z
        let c = x.cross(y)
        XCTAssertEqual(c.dot(z), 1, accuracy: 1e-12)
    }
}

final class QuaternionTests: XCTestCase {
    func testIdentityRotationLeavesVectorUnchanged() {
        let v = Vector3(1, 2, 3)
        let r = Quaternion.identity.rotate(v)
        XCTAssertEqual(r.x, 1, accuracy: 1e-15)
        XCTAssertEqual(r.y, 2, accuracy: 1e-15)
        XCTAssertEqual(r.z, 3, accuracy: 1e-15)
    }

    func testNinetyDegreeRotationAboutZMapsXToY() {
        let q = Quaternion(axis: Vector3(0, 0, 1), angle: .pi / 2)
        let r = q.rotate(Vector3(1, 0, 0))
        XCTAssertEqual(r.x, 0, accuracy: 1e-12)
        XCTAssertEqual(r.y, 1, accuracy: 1e-12)
        XCTAssertEqual(r.z, 0, accuracy: 1e-12)
    }

    func testIntegratingConstantRateReproducesClosedFormRotation() {
        // Spec §3.2: attitude is propagated by integrating rotationRate.
        // 1 rad/s about Z for 1 s, in 1000 steps, must match a 1 rad rotation.
        let rate = Vector3(0, 0, 1.0)
        var q = Quaternion.identity
        let dt = 0.001
        for _ in 0..<1000 {
            q = q.integrating(angularVelocity: rate, dt: dt)
        }
        let expected = Quaternion(axis: Vector3(0, 0, 1), angle: 1.0)
        let v = Vector3(1, 0, 0)
        let a = q.rotate(v), b = expected.rotate(v)
        XCTAssertEqual(a.x, b.x, accuracy: 1e-9)
        XCTAssertEqual(a.y, b.y, accuracy: 1e-9)
        XCTAssertEqual(q.length, 1, accuracy: 1e-12, "integration must renormalise")
    }

    func testAverageOfIdenticalQuaternionsIsThatQuaternion() {
        let q = Quaternion(axis: Vector3(1, 1, 0).normalized()!, angle: 0.4)
        let avg = Quaternion.average([q, q, q])!
        XCTAssertEqual(abs(avg.dot(q)), 1, accuracy: 1e-12)
    }

    func testAverageIsInsensitiveToSignFlips() {
        // q and -q are the same rotation; naive component averaging cancels them out.
        // Markley averaging must not (spec §5 "use quaternion averaging").
        let q = Quaternion(axis: Vector3(0, 0, 1), angle: 0.6)
        let avg = Quaternion.average([q, -q, q, -q])!
        XCTAssertEqual(abs(avg.dot(q)), 1, accuracy: 1e-9)
    }

    func testAverageOfTwoRotationsLiesBetweenThem() {
        let a = Quaternion(axis: Vector3(0, 0, 1), angle: 0.0)
        let b = Quaternion(axis: Vector3(0, 0, 1), angle: 1.0)
        let avg = Quaternion.average([a, b])!
        let v = avg.rotate(Vector3(1, 0, 0))
        XCTAssertEqual(atan2(v.y, v.x), 0.5, accuracy: 1e-9)
    }
}

final class MatrixTests: XCTestCase {
    func testIdentityInverseIsIdentity() {
        let inv = Matrix.identity(3).inverted()!
        for i in 0..<3 {
            for j in 0..<3 {
                XCTAssertEqual(inv[i, j], i == j ? 1 : 0, accuracy: 1e-15)
            }
        }
    }

    func testInverseTimesOriginalIsIdentity() {
        let m = Matrix(rows: 3, columns: 3, values: [
            4, 2, 0.5,
            2, 5, 1,
            0.5, 1, 3,
        ])
        let product = m * m.inverted()!
        for i in 0..<3 {
            for j in 0..<3 {
                XCTAssertEqual(product[i, j], i == j ? 1 : 0, accuracy: 1e-12)
            }
        }
    }

    func testSingularMatrixInverseReturnsNil() {
        // Spec §6.4: |det| < 1e-12 must be detected so the smoother can skip the step.
        let singular = Matrix(rows: 3, columns: 3, values: [
            1, 2, 3,
            2, 4, 6,
            1, 1, 1,
        ])
        XCTAssertNil(singular.inverted())
    }

    func testMultiplicationMatchesHandComputation() {
        let a = Matrix(rows: 2, columns: 3, values: [1, 2, 3, 4, 5, 6])
        let b = Matrix(rows: 3, columns: 2, values: [7, 8, 9, 10, 11, 12])
        let c = a * b
        XCTAssertEqual(c.rows, 2)
        XCTAssertEqual(c.columns, 2)
        XCTAssertEqual(c[0, 0], 58, accuracy: 1e-12)
        XCTAssertEqual(c[0, 1], 64, accuracy: 1e-12)
        XCTAssertEqual(c[1, 0], 139, accuracy: 1e-12)
        XCTAssertEqual(c[1, 1], 154, accuracy: 1e-12)
    }

    func testTransposeSwapsShapeAndEntries() {
        let a = Matrix(rows: 2, columns: 3, values: [1, 2, 3, 4, 5, 6])
        let t = a.transposed
        XCTAssertEqual(t.rows, 3)
        XCTAssertEqual(t.columns, 2)
        XCTAssertEqual(t[2, 0], 3, accuracy: 1e-15)
        XCTAssertEqual(t[0, 1], 4, accuracy: 1e-15)
    }

    func testSymmetrisedRemovesAsymmetryDrift() {
        // Covariance matrices must stay symmetric across many filter steps.
        let m = Matrix(rows: 2, columns: 2, values: [1, 2, 4, 3])
        let s = m.symmetrised()
        XCTAssertEqual(s[0, 1], 3, accuracy: 1e-15)
        XCTAssertEqual(s[1, 0], 3, accuracy: 1e-15)
    }
}
