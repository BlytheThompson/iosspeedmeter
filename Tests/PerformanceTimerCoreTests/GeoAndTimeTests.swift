import XCTest
@testable import PerformanceTimerCore

/// Spec Appendix B — local ENU tangent plane.
final class LocalENUTests: XCTestCase {
    let originLatitude = 37.4275      // degrees
    let originLongitude = -122.1697
    let originAltitude = 30.0

    private func makeOrigin() -> LocalENU {
        LocalENU(originLatitudeDegrees: originLatitude,
                 originLongitudeDegrees: originLongitude,
                 originAltitude: originAltitude)
    }

    func testOriginMapsToZero() {
        let enu = makeOrigin()
        let p = enu.project(latitudeDegrees: originLatitude,
                            longitudeDegrees: originLongitude,
                            altitude: originAltitude)
        XCTAssertEqual(p.east, 0, accuracy: 1e-9)
        XCTAssertEqual(p.north, 0, accuracy: 1e-9)
        XCTAssertEqual(p.up, 0, accuracy: 1e-9)
    }

    func testNorthAndEastSignsAndMagnitude() {
        let enu = makeOrigin()
        let north = enu.project(latitudeDegrees: originLatitude + 0.001,
                                longitudeDegrees: originLongitude,
                                altitude: originAltitude)
        XCTAssertGreaterThan(north.north, 0)
        XCTAssertEqual(north.east, 0, accuracy: 1e-9)
        // 0.001 deg of latitude is about 111 m.
        XCTAssertEqual(north.north, 111.0, accuracy: 1.0)

        let east = enu.project(latitudeDegrees: originLatitude,
                               longitudeDegrees: originLongitude + 0.001,
                               altitude: originAltitude)
        XCTAssertGreaterThan(east.east, 0)
        // Shrinks with cos(latitude).
        XCTAssertEqual(east.east, 111_320.0 * 0.001 * cos(originLatitude * .pi / 180), accuracy: 1.0)
    }

    func testMeridionalRadiusAtEquatorMatchesClosedForm() {
        let enu = LocalENU(originLatitudeDegrees: 0, originLongitudeDegrees: 0, originAltitude: 0)
        // R_M at the equator is a(1 - e^2).
        let expected = PTConstants.wgs84SemiMajorAxis * (1 - PTConstants.wgs84EccentricitySquared)
        XCTAssertEqual(enu.meridionalRadius, expected, accuracy: 1e-6)
        // R_N at the equator is a.
        XCTAssertEqual(enu.primeVerticalRadius, PTConstants.wgs84SemiMajorAxis, accuracy: 1e-6)
    }

    func testUpIsAltitudeDifference() {
        let enu = makeOrigin()
        let p = enu.project(latitudeDegrees: originLatitude,
                            longitudeDegrees: originLongitude,
                            altitude: originAltitude + 12.5)
        XCTAssertEqual(p.up, 12.5, accuracy: 1e-9)
    }

    /// A point ~1.1 km from the origin on a diagonal, for the accuracy comparisons below.
    private func farPoint() -> (latitude: Double, longitude: Double) {
        let metresPerDegreeLat = 111_132.0
        let offset = PTConstants.mile / 2  // ~805 m along each axis => ~1138 m total
        return (
            originLatitude + offset / metresPerDegreeLat,
            originLongitude + offset / (metresPerDegreeLat * cos(originLatitude * .pi / 180))
        )
    }

    private func horizontalErrorVersusExact(mode: LocalENU.Mode) -> Double {
        let enu = LocalENU(originLatitudeDegrees: originLatitude,
                           originLongitudeDegrees: originLongitude,
                           originAltitude: originAltitude,
                           mode: mode)
        let far = farPoint()
        let approx = enu.project(latitudeDegrees: far.latitude,
                                 longitudeDegrees: far.longitude,
                                 altitude: originAltitude)
        let exact = Self.exactENU(
            latitudeDegrees: far.latitude, longitudeDegrees: far.longitude,
            altitude: originAltitude,
            originLatitudeDegrees: originLatitude,
            originLongitudeDegrees: originLongitude,
            originAltitude: originAltitude
        )
        let dEast = approx.east - exact.east
        let dNorth = approx.north - exact.north
        return (dEast * dEast + dNorth * dNorth).squareRoot()
    }

    /// Deviation D3. Appendix B claims its formula is accurate to "well under a centimetre
    /// over a mile". Measured against an exact ECEF→ENU transform it is not — this pins the
    /// real figure so the claim cannot quietly creep back into the code.
    func testAppendixBLiteralFormulaIsCentimetreLevelNotSubCentimetre() {
        let error = horizontalErrorVersusExact(mode: .appendixBLiteral)
        XCTAssertGreaterThan(error, 0.01,
                             "Appendix B's sub-centimetre claim would hold; error was \(error) m")
        XCTAssertLessThan(error, 0.5, "error should still be small in absolute terms: \(error) m")
    }

    func testDefaultModeIsExact() {
        XCTAssertEqual(makeOrigin().mode, .exact)
    }

    /// Independent validation of `.exact` that does not reuse the ECEF oracle.
    ///
    /// The north coordinate of a point due north of the origin must equal the meridian arc
    /// length `∫ R_M(φ) dφ`, computed here by Simpson's rule straight from Appendix B's
    /// `R_M` expression. The ENU north coordinate is a chord projection rather than an arc,
    /// but over 800 m the two differ by ~2 µm, far inside the tolerance.
    func testExactModeNorthMatchesNumericallyIntegratedMeridianArc() {
        let enu = makeOrigin()
        let targetLatitude = originLatitude + 0.0072   // ~800 m north

        let phi0 = originLatitude * .pi / 180
        let phi1 = targetLatitude * .pi / 180
        let steps = 2000
        let h = (phi1 - phi0) / Double(steps)
        var arc = 0.0
        for i in 0..<steps {
            let a0 = phi0 + Double(i) * h
            let a1 = a0 + h
            let mid = (a0 + a1) / 2
            // Simpson's rule on each sub-interval.
            let f0 = LocalENU.radii(atLatitudeRadians: a0).meridional
            let fm = LocalENU.radii(atLatitudeRadians: mid).meridional
            let f1 = LocalENU.radii(atLatitudeRadians: a1).meridional
            arc += (h / 6) * (f0 + 4 * fm + f1)
        }

        let p = enu.project(latitudeDegrees: targetLatitude,
                            longitudeDegrees: originLongitude,
                            altitude: originAltitude)
        // The ENU north coordinate is a chord projected onto the origin's tangent plane,
        // whereas the integral is arc length along the curved meridian, so the two agree only
        // to second order. Over 800 m the gap is ~4 mm (5 ppm); anything larger would mean a
        // real error in the transform. For comparison, the Appendix B literal formula is off
        // by 86 mm over a comparable diagonal.
        XCTAssertEqual(p.north, arc, accuracy: 0.01)
        XCTAssertEqual(p.east, 0, accuracy: 1e-6)
    }

    /// Independent validation of the east axis, exact rather than approximate.
    ///
    /// For two points at the same geodetic latitude and height, the ENU east coordinate works
    /// out in closed form as `(R_N + h)·cos φ·sin(Δλ)`. Note this is the chord *projected on
    /// the east axis*, which is `R sin(Δλ)` — not the chord length `2R sin(Δλ/2)`; the two
    /// differ by `RΔλ³/8`, about 2.5 µm here, because the chord also has a north component.
    func testExactModeEastMatchesClosedFormParallelGeometry() {
        let enu = makeOrigin()
        let dLambdaDegrees = 0.009
        let p = enu.project(latitudeDegrees: originLatitude,
                            longitudeDegrees: originLongitude + dLambdaDegrees,
                            altitude: originAltitude)

        let phi0 = originLatitude * .pi / 180
        let radius = (LocalENU.radii(atLatitudeRadians: phi0).primeVertical + originAltitude)
            * cos(phi0)
        let dLambda = dLambdaDegrees * .pi / 180
        XCTAssertEqual(p.east, radius * sin(dLambda), accuracy: 1e-9)
    }

    /// A pure altitude change must be purely `up` — this pins the basis orthogonality.
    func testExactModeAltitudeChangeIsPurelyVertical() {
        let enu = makeOrigin()
        let p = enu.project(latitudeDegrees: originLatitude,
                            longitudeDegrees: originLongitude,
                            altitude: originAltitude + 40)
        XCTAssertEqual(p.up, 40, accuracy: 1e-6)
        XCTAssertEqual(p.east, 0, accuracy: 1e-9)
        XCTAssertEqual(p.north, 0, accuracy: 1e-9)
    }

    /// Round-trip exercises Bowring's ECEF→geodetic inverse against the forward transform —
    /// genuinely separate code paths.
    func testExactModeRoundTripIsSubMillimetre() {
        let enu = makeOrigin()
        for (dLat, dLon, alt) in [(0.004, -0.006, 55.0), (-0.01, 0.013, -20.0), (0.0, 0.0, 30.0)] {
            let lat = originLatitude + dLat
            let lon = originLongitude + dLon
            let back = enu.inverseProject(
                enu.project(latitudeDegrees: lat, longitudeDegrees: lon, altitude: alt)
            )
            // 1e-9 deg of latitude is about 0.1 mm.
            XCTAssertEqual(back.latitudeDegrees, lat, accuracy: 1e-9)
            XCTAssertEqual(back.longitudeDegrees, lon, accuracy: 1e-9)
            XCTAssertEqual(back.altitude, alt, accuracy: 1e-4)
        }
    }

    func testRoundTripInverseInLiteralMode() {
        let enu = LocalENU(originLatitudeDegrees: originLatitude,
                           originLongitudeDegrees: originLongitude,
                           originAltitude: originAltitude,
                           mode: .appendixBLiteral)
        let lat = originLatitude + 0.004
        let lon = originLongitude - 0.006
        let back = enu.inverseProject(
            enu.project(latitudeDegrees: lat, longitudeDegrees: lon, altitude: 55)
        )
        XCTAssertEqual(back.latitudeDegrees, lat, accuracy: 1e-11)
        XCTAssertEqual(back.longitudeDegrees, lon, accuracy: 1e-11)
    }

    // MARK: - Exact reference implementation (test oracle only)

    /// Geodetic → ECEF → ENU. This is the textbook exact transform; the production code uses
    /// the cheaper tangent-plane approximation from Appendix B and is checked against this.
    private static func exactENU(
        latitudeDegrees: Double, longitudeDegrees: Double, altitude: Double,
        originLatitudeDegrees: Double, originLongitudeDegrees: Double, originAltitude: Double
    ) -> (east: Double, north: Double, up: Double) {
        func ecef(_ latDeg: Double, _ lonDeg: Double, _ h: Double) -> (Double, Double, Double) {
            let a = PTConstants.wgs84SemiMajorAxis
            let e2 = PTConstants.wgs84EccentricitySquared
            let lat = latDeg * .pi / 180
            let lon = lonDeg * .pi / 180
            let n = a / (1 - e2 * sin(lat) * sin(lat)).squareRoot()
            return (
                (n + h) * cos(lat) * cos(lon),
                (n + h) * cos(lat) * sin(lon),
                (n * (1 - e2) + h) * sin(lat)
            )
        }
        let (x, y, z) = ecef(latitudeDegrees, longitudeDegrees, altitude)
        let (x0, y0, z0) = ecef(originLatitudeDegrees, originLongitudeDegrees, originAltitude)
        let dx = x - x0, dy = y - y0, dz = z - z0
        let lat0 = originLatitudeDegrees * .pi / 180
        let lon0 = originLongitudeDegrees * .pi / 180
        let east = -sin(lon0) * dx + cos(lon0) * dy
        let north = -sin(lat0) * cos(lon0) * dx - sin(lat0) * sin(lon0) * dy + cos(lat0) * dz
        let up = cos(lat0) * cos(lon0) * dx + cos(lat0) * sin(lon0) * dy + sin(lat0) * dz
        return (east, north, up)
    }
}

/// Spec §2.2 — one monotonic session clock for three differently-stamped sources.
final class SessionClockTests: XCTestCase {
    func testMonotonicSourceIsOffsetBySessionEpoch() {
        let clock = SessionClock(sessionEpoch: 1000.0, bootWallClockUnixTime: 1_700_000_000.0)
        XCTAssertEqual(clock.sessionTime(monotonicTimestamp: 1000.0), 0.0, accuracy: 1e-12)
        XCTAssertEqual(clock.sessionTime(monotonicTimestamp: 1002.5), 2.5, accuracy: 1e-12)
    }

    func testWallClockSourceConvertsThroughBootReference() {
        // Session started 1000 s after boot; boot happened at unix 1_700_000_000.
        let clock = SessionClock(sessionEpoch: 1000.0, bootWallClockUnixTime: 1_700_000_000.0)
        // A fix stamped 1002.5 s after boot in wall-clock terms.
        let wall = 1_700_000_000.0 + 1002.5
        XCTAssertEqual(clock.sessionTime(wallClockUnixTime: wall), 2.5, accuracy: 1e-12)
    }

    func testSimultaneousEventsFromBothSourcesAgree() {
        // This is the property the whole of §2 exists to guarantee: an IMU sample and a GNSS
        // fix describing the same instant must land on the same session time.
        let clock = SessionClock(sessionEpoch: 421.75, bootWallClockUnixTime: 1_699_000_000.25)
        let uptimeOfEvent = 500.125
        let fromMonotonic = clock.sessionTime(monotonicTimestamp: uptimeOfEvent)
        let fromWallClock = clock.sessionTime(
            wallClockUnixTime: 1_699_000_000.25 + uptimeOfEvent
        )
        XCTAssertEqual(fromMonotonic, fromWallClock, accuracy: 1e-9)
    }

    func testBootWallClockDerivedFromUptimeAndDate() {
        // bootWallClock = now - systemUptime  (spec §2.2, captured once per session)
        let clock = SessionClock(sessionEpoch: 10, currentWallClockUnixTime: 1_700_000_500.0,
                                 currentUptime: 500.0)
        XCTAssertEqual(clock.bootWallClockUnixTime, 1_700_000_000.0, accuracy: 1e-9)
    }
}

/// Spec §2.4 — external receiver time alignment by least-squares fit.
final class ClockFitTests: XCTestCase {
    /// Deterministic pseudo-random noise so the test is reproducible.
    private struct LCG {
        var state: UInt64
        mutating func nextUnit() -> Double {
            state = state &* 6_364_136_223_846_793_005 &+ 1_442_695_040_888_963_407
            return Double(state >> 11) / Double(1 << 53)
        }
        /// Uniform in [-1, 1].
        mutating func nextSigned() -> Double { nextUnit() * 2 - 1 }
    }

    func testPerfectDataRecoversSlopeAndIntercept() {
        var fit = ClockFit()
        // t_session = 0.001 * iTOW + beta, iTOW in ms.
        let beta = -3_600.25
        for i in 0..<40 {
            let itow = 100_000.0 + Double(i) * 40.0     // 25 Hz
            fit.add(itowMilliseconds: itow, arrivalSessionTime: 0.001 * itow + beta)
        }
        let solution = try? fit.solve()
        XCTAssertNotNil(solution)
        XCTAssertEqual(solution!.slope, 0.001, accuracy: 1e-12)
        XCTAssertEqual(solution!.intercept, beta, accuracy: 1e-6)
        XCTAssertEqual(solution!.residualRMS, 0, accuracy: 1e-9)
    }

    func testFewerThanThirtyPairsIsRejected() {
        var fit = ClockFit()
        for i in 0..<29 {
            let itow = Double(i) * 1000.0
            fit.add(itowMilliseconds: itow, arrivalSessionTime: 0.001 * itow)
        }
        XCTAssertThrowsError(try fit.solve()) { error in
            XCTAssertEqual(error as? ClockFit.Error, .insufficientSamples(have: 29, need: 30))
        }
    }

    func testSlopeMoreThanOneHundredPPMFromNominalIsRejected() {
        var fit = ClockFit()
        let badSlope = 0.001 * (1 + 500e-6)   // 500 ppm out
        for i in 0..<40 {
            let itow = Double(i) * 1000.0
            fit.add(itowMilliseconds: itow, arrivalSessionTime: badSlope * itow)
        }
        XCTAssertThrowsError(try fit.solve()) { error in
            guard case .slopeOutOfTolerance = error as? ClockFit.Error else {
                return XCTFail("expected slopeOutOfTolerance, got \(error)")
            }
        }
    }

    func testSlopeWithinToleranceIsAccepted() {
        var fit = ClockFit()
        let okSlope = 0.001 * (1 + 50e-6)     // 50 ppm, inside the 100 ppm gate
        for i in 0..<40 {
            let itow = Double(i) * 1000.0
            fit.add(itowMilliseconds: itow, arrivalSessionTime: okSlope * itow)
        }
        XCTAssertNoThrow(try fit.solve())
    }

    func testResidualRMSReportsLinkJitter() {
        // Spec §2.4: "The residual scatter is your link jitter — log it."
        var rng = LCG(state: 12345)
        var fit = ClockFit()
        let jitter = 0.010   // +/- 10 ms uniform
        for i in 0..<200 {
            let itow = Double(i) * 40.0
            fit.add(itowMilliseconds: itow,
                    arrivalSessionTime: 0.001 * itow + rng.nextSigned() * jitter)
        }
        let solution = try! fit.solve()
        // RMS of a uniform +/-j distribution is j/sqrt(3) = 0.00577.
        XCTAssertEqual(solution.residualRMS, jitter / 3.0.squareRoot(), accuracy: 0.0015)
    }

    func testApplyMapsPacketTimeOntoSessionClock() {
        var fit = ClockFit()
        let beta = 12.5
        for i in 0..<40 {
            let itow = 200_000.0 + Double(i) * 40.0
            fit.add(itowMilliseconds: itow, arrivalSessionTime: 0.001 * itow + beta)
        }
        let solution = try! fit.solve()
        // A packet 1 s after the last one used in the fit.
        let itow = 200_000.0 + 40.0 * 39 + 1000.0
        XCTAssertEqual(solution.sessionTime(itowMilliseconds: itow),
                       0.001 * itow + beta, accuracy: 1e-6)
    }
}
