import Foundation

/// A 3-component vector in `Double`.
///
/// Used for every physical 3-vector in the system: specific force, gravity, rotation rate,
/// and the vehicle basis axes of spec §5. `Double` throughout — see spec §6.4 on precision.
public struct Vector3: Equatable, Hashable, Codable, Sendable {
    public var x: Double
    public var y: Double
    public var z: Double

    public init(_ x: Double, _ y: Double, _ z: Double) {
        self.x = x
        self.y = y
        self.z = z
    }

    public static let zero = Vector3(0, 0, 0)

    public var lengthSquared: Double { x * x + y * y + z * z }
    public var length: Double { lengthSquared.squareRoot() }

    public func dot(_ other: Vector3) -> Double {
        x * other.x + y * other.y + z * other.z
    }

    public func cross(_ other: Vector3) -> Vector3 {
        Vector3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        )
    }

    /// Unit vector in the same direction, or `nil` if the vector is too short to have a
    /// meaningful direction. Callers must handle `nil` rather than dividing by zero — a
    /// degenerate gravity or acceleration average is a calibration failure (spec §5 gates),
    /// not something to paper over.
    public func normalized(epsilon: Double = 1e-12) -> Vector3? {
        let l = length
        guard l > epsilon else { return nil }
        return self / l
    }

    /// The component of `self` perpendicular to `axis`. `axis` must be a unit vector.
    /// This is spec §5 step 2: `a_h = ā_D - (ā_D · z_V) z_V`.
    public func removingComponent(along axis: Vector3) -> Vector3 {
        self - axis * dot(axis)
    }

    public static func + (a: Vector3, b: Vector3) -> Vector3 {
        Vector3(a.x + b.x, a.y + b.y, a.z + b.z)
    }

    public static func - (a: Vector3, b: Vector3) -> Vector3 {
        Vector3(a.x - b.x, a.y - b.y, a.z - b.z)
    }

    public static prefix func - (v: Vector3) -> Vector3 {
        Vector3(-v.x, -v.y, -v.z)
    }

    public static func * (v: Vector3, s: Double) -> Vector3 {
        Vector3(v.x * s, v.y * s, v.z * s)
    }

    public static func * (s: Double, v: Vector3) -> Vector3 { v * s }

    public static func / (v: Vector3, s: Double) -> Vector3 {
        Vector3(v.x / s, v.y / s, v.z / s)
    }

    public static func += (a: inout Vector3, b: Vector3) { a = a + b }
}

extension Array where Element == Vector3 {
    /// Component-wise mean. Returns `nil` for an empty array.
    public func mean() -> Vector3? {
        guard !isEmpty else { return nil }
        var sum = Vector3.zero
        for v in self { sum += v }
        return sum / Double(count)
    }
}
