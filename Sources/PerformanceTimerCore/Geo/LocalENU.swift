import Foundation

/// Local East-North-Up frame anchored at the session origin (spec Appendix B).
///
/// **Deviation D3 from the spec.** Appendix B offers a tangent-plane approximation and states
/// its error is "well under a centimetre over a mile". Measured against an exact
/// geodetic→ECEF→ENU transform that is not true: at mid-latitude, ~1.1 km out on a diagonal,
/// the published formula is off by ~8.6 cm.
///
/// The error comes from evaluating `R_N`, `R_M` and `cos φ` at the origin latitude and
/// treating the result as a plane displacement. Re-evaluating the radii at the midpoint
/// latitude only improves it to ~5.5 cm, because the residual is the tangent-plane geometry
/// itself, not the choice of latitude.
///
/// 8.6 cm is irrelevant beside a 3 m consumer fix, but spec §6.3 contemplates feeding GNSS
/// position into the filter once an RTK receiver is fitted, and at 1–2 cm RTK accuracy this
/// approximation would become the dominant error. The exact transform costs a handful of trig
/// calls per fix — nothing at 1–25 Hz, which is the only rate positions ever arrive at — so
/// there is no reason to approximate. `.exact` is the default; `.appendixBLiteral` is kept so
/// the two can be compared on recorded data.
public struct LocalENU: Equatable, Codable, Sendable {
    public enum Mode: String, Equatable, Codable, Sendable {
        /// The published Appendix B tangent-plane formula. ~8.6 cm error over a mile.
        case appendixBLiteral
        /// Full geodetic→ECEF→ENU. Exact to numerical precision. The default.
        case exact
    }

    public struct Point: Equatable, Codable, Sendable {
        public var east: Double
        public var north: Double
        public var up: Double

        public init(east: Double, north: Double, up: Double) {
            self.east = east
            self.north = north
            self.up = up
        }

        /// Horizontal distance from the origin.
        public var horizontalRange: Double { (east * east + north * north).squareRoot() }
    }

    public struct Geodetic: Equatable, Codable, Sendable {
        public var latitudeDegrees: Double
        public var longitudeDegrees: Double
        public var altitude: Double

        public init(latitudeDegrees: Double, longitudeDegrees: Double, altitude: Double) {
            self.latitudeDegrees = latitudeDegrees
            self.longitudeDegrees = longitudeDegrees
            self.altitude = altitude
        }
    }

    public let originLatitudeDegrees: Double
    public let originLongitudeDegrees: Double
    public let originAltitude: Double
    public let mode: Mode

    /// `R_N` — prime vertical radius of curvature at the origin latitude, m (Appendix B).
    public let primeVerticalRadius: Double
    /// `R_M` — meridional radius of curvature at the origin latitude, m (Appendix B).
    public let meridionalRadius: Double

    // Derived once at construction; the origin never moves during a session.
    private let sinOriginLatitude: Double
    private let cosOriginLatitude: Double
    private let sinOriginLongitude: Double
    private let cosOriginLongitude: Double
    private let originX: Double
    private let originY: Double
    private let originZ: Double

    public init(
        originLatitudeDegrees: Double,
        originLongitudeDegrees: Double,
        originAltitude: Double,
        mode: Mode = .exact
    ) {
        self.originLatitudeDegrees = originLatitudeDegrees
        self.originLongitudeDegrees = originLongitudeDegrees
        self.originAltitude = originAltitude
        self.mode = mode

        let phi0 = originLatitudeDegrees * .pi / 180
        let lambda0 = originLongitudeDegrees * .pi / 180
        let radii = Self.radii(atLatitudeRadians: phi0)
        primeVerticalRadius = radii.primeVertical
        meridionalRadius = radii.meridional

        sinOriginLatitude = sin(phi0)
        cosOriginLatitude = cos(phi0)
        sinOriginLongitude = sin(lambda0)
        cosOriginLongitude = cos(lambda0)
        let origin = Self.ecef(latitudeRadians: phi0,
                               longitudeRadians: lambda0,
                               altitude: originAltitude)
        originX = origin.x
        originY = origin.y
        originZ = origin.z
    }

    // Only the defining fields participate in equality and serialisation; everything else is
    // a cache recomputed by the designated initialiser.
    public static func == (a: LocalENU, b: LocalENU) -> Bool {
        a.originLatitudeDegrees == b.originLatitudeDegrees
            && a.originLongitudeDegrees == b.originLongitudeDegrees
            && a.originAltitude == b.originAltitude
            && a.mode == b.mode
    }

    private enum CodingKeys: String, CodingKey {
        case originLatitudeDegrees, originLongitudeDegrees, originAltitude, mode
    }

    public init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        self.init(
            originLatitudeDegrees: try c.decode(Double.self, forKey: .originLatitudeDegrees),
            originLongitudeDegrees: try c.decode(Double.self, forKey: .originLongitudeDegrees),
            originAltitude: try c.decode(Double.self, forKey: .originAltitude),
            mode: try c.decodeIfPresent(Mode.self, forKey: .mode) ?? .exact
        )
    }

    public func encode(to encoder: Encoder) throws {
        var c = encoder.container(keyedBy: CodingKeys.self)
        try c.encode(originLatitudeDegrees, forKey: .originLatitudeDegrees)
        try c.encode(originLongitudeDegrees, forKey: .originLongitudeDegrees)
        try c.encode(originAltitude, forKey: .originAltitude)
        try c.encode(mode, forKey: .mode)
    }

    // MARK: - Ellipsoid helpers

    /// `R_N` (prime vertical) and `R_M` (meridional) radii of curvature, Appendix B.
    public static func radii(atLatitudeRadians phi: Double) -> (primeVertical: Double, meridional: Double) {
        let a = PTConstants.wgs84SemiMajorAxis
        let e2 = PTConstants.wgs84EccentricitySquared
        let sinPhi = sin(phi)
        let w = 1 - e2 * sinPhi * sinPhi
        let sqrtW = w.squareRoot()
        return (a / sqrtW, a * (1 - e2) / (w * sqrtW))
    }

    static func ecef(
        latitudeRadians phi: Double,
        longitudeRadians lambda: Double,
        altitude h: Double
    ) -> (x: Double, y: Double, z: Double) {
        let e2 = PTConstants.wgs84EccentricitySquared
        let n = radii(atLatitudeRadians: phi).primeVertical
        let cosPhi = cos(phi)
        return (
            (n + h) * cosPhi * cos(lambda),
            (n + h) * cosPhi * sin(lambda),
            (n * (1 - e2) + h) * sin(phi)
        )
    }

    // MARK: - Forward

    /// Geodetic → local ENU metres.
    public func project(latitudeDegrees: Double, longitudeDegrees: Double, altitude: Double) -> Point {
        switch mode {
        case .appendixBLiteral:
            let dPhi = (latitudeDegrees - originLatitudeDegrees) * .pi / 180
            let dLambda = (longitudeDegrees - originLongitudeDegrees) * .pi / 180
            return Point(
                east: dLambda * primeVerticalRadius * cosOriginLatitude,
                north: dPhi * meridionalRadius,
                up: altitude - originAltitude
            )

        case .exact:
            let p = Self.ecef(latitudeRadians: latitudeDegrees * .pi / 180,
                              longitudeRadians: longitudeDegrees * .pi / 180,
                              altitude: altitude)
            let dx = p.x - originX
            let dy = p.y - originY
            let dz = p.z - originZ
            return Point(
                east: -sinOriginLongitude * dx + cosOriginLongitude * dy,
                north: -sinOriginLatitude * cosOriginLongitude * dx
                    - sinOriginLatitude * sinOriginLongitude * dy
                    + cosOriginLatitude * dz,
                up: cosOriginLatitude * cosOriginLongitude * dx
                    + cosOriginLatitude * sinOriginLongitude * dy
                    + sinOriginLatitude * dz
            )
        }
    }

    public func project(_ geodetic: Geodetic) -> Point {
        project(latitudeDegrees: geodetic.latitudeDegrees,
                longitudeDegrees: geodetic.longitudeDegrees,
                altitude: geodetic.altitude)
    }

    // MARK: - Inverse

    /// Local ENU metres → geodetic.
    public func inverseProject(_ point: Point) -> Geodetic {
        switch mode {
        case .appendixBLiteral:
            let dPhi = point.north / meridionalRadius
            let dLambda = point.east / (primeVerticalRadius * cosOriginLatitude)
            return Geodetic(
                latitudeDegrees: originLatitudeDegrees + dPhi * 180 / .pi,
                longitudeDegrees: originLongitudeDegrees + dLambda * 180 / .pi,
                altitude: originAltitude + point.up
            )

        case .exact:
            // ENU → ECEF (transpose of the forward rotation), then ECEF → geodetic.
            let e = point.east, n = point.north, u = point.up
            let dx = -sinOriginLongitude * e
                - sinOriginLatitude * cosOriginLongitude * n
                + cosOriginLatitude * cosOriginLongitude * u
            let dy = cosOriginLongitude * e
                - sinOriginLatitude * sinOriginLongitude * n
                + cosOriginLatitude * sinOriginLongitude * u
            let dz = cosOriginLatitude * n + sinOriginLatitude * u
            return Self.geodetic(x: originX + dx,
                                 y: originY + dy,
                                 z: originZ + dz)
        }
    }

    /// ECEF → geodetic by Bowring's method.
    ///
    /// Bowring converges to sub-millimetre in a single pass for any altitude a car will ever
    /// see, which is why no iteration loop is needed here.
    static func geodetic(x: Double, y: Double, z: Double) -> Geodetic {
        let a = PTConstants.wgs84SemiMajorAxis
        let f = PTConstants.wgs84Flattening
        let e2 = PTConstants.wgs84EccentricitySquared
        let b = a * (1 - f)
        let ep2 = (a * a - b * b) / (b * b)

        let p = (x * x + y * y).squareRoot()
        let lambda = atan2(y, x)

        // Degenerate case: on the polar axis.
        guard p > 1e-12 else {
            let sign: Double = z >= 0 ? 1 : -1
            return Geodetic(latitudeDegrees: sign * 90,
                            longitudeDegrees: lambda * 180 / .pi,
                            altitude: abs(z) - b)
        }

        let theta = atan2(z * a, p * b)
        let sinTheta = sin(theta), cosTheta = cos(theta)
        let phi = atan2(
            z + ep2 * b * sinTheta * sinTheta * sinTheta,
            p - e2 * a * cosTheta * cosTheta * cosTheta
        )
        let n = radii(atLatitudeRadians: phi).primeVertical
        let h = p / cos(phi) - n

        return Geodetic(latitudeDegrees: phi * 180 / .pi,
                        longitudeDegrees: lambda * 180 / .pi,
                        altitude: h)
    }
}
