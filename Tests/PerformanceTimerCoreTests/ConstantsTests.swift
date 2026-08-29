import XCTest
@testable import PerformanceTimerCore

/// Spec Appendix A — Constants.
final class ConstantsTests: XCTestCase {
    func testStandardGravity() {
        XCTAssertEqual(PTConstants.g, 9.80665, accuracy: 0.0)
    }

    func testUnitConversions() {
        XCTAssertEqual(PTConstants.mphToMetersPerSecond, 0.44704, accuracy: 0.0)
        XCTAssertEqual(PTConstants.knotsToMetersPerSecond, 0.514444, accuracy: 0.0)
        XCTAssertEqual(PTConstants.kmhToMetersPerSecond, 0.277778, accuracy: 0.0)
        XCTAssertEqual(PTConstants.foot, 0.3048, accuracy: 0.0)
        XCTAssertEqual(PTConstants.mile, 1609.344, accuracy: 0.0)
    }

    func testWGS84() {
        XCTAssertEqual(PTConstants.wgs84SemiMajorAxis, 6378137.0, accuracy: 0.0)
        XCTAssertEqual(PTConstants.wgs84Flattening, 1.0 / 298.257223563, accuracy: 0.0)
        // e^2 = 2f - f^2
        let f = PTConstants.wgs84Flattening
        XCTAssertEqual(PTConstants.wgs84EccentricitySquared, 2 * f - f * f, accuracy: 1e-18)
    }
}
