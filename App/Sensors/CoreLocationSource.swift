#if canImport(CoreLocation)
import CoreLocation
import Foundation
import PerformanceTimerCore

/// Spec §3.3 — internal GNSS via Core Location.
public final class CoreLocationSource: NSObject, SensorSource, CLLocationManagerDelegate {
    /// Spec §3.3 carries an explicit warning about this setting, so it is surfaced rather than
    /// buried: Apple DTS has stated that declaring `.automotiveNavigation` causes Core Location
    /// to correct coordinate randomness by **pulling positions toward roads**. On a drag strip
    /// or an unmapped surface that map-matching corrupts the position trace and therefore the
    /// distance marks. Doppler speed should be unaffected, but until that is verified on the
    /// specific device, `.otherNavigation` is the default. Exposed as a debug setting so traces
    /// can be compared.
    public enum ActivityMode: String, CaseIterable, Sendable {
        case otherNavigation
        case other
        case automotiveNavigation

        var clValue: CLActivityType {
            switch self {
            case .otherNavigation: return .otherNavigation
            case .other: return .other
            case .automotiveNavigation: return .automotiveNavigation
            }
        }

        public var warning: String? {
            self == .automotiveNavigation
                ? "May snap positions to roads, corrupting distance marks. Debug only."
                : nil
        }
    }

    public let descriptor = SensorSourceDescriptor(
        identifier: "corelocation",
        displayName: "Internal GNSS (Core Location)",
        // Spec §3.3: "Core Location does not commit to any rate publicly." This is a hint;
        // the estimator uses the observed rate.
        nominalRate: 1,
        kind: .gnss
    )

    public private(set) var isRunning = false

    private let manager = CLLocationManager()
    private let clock: SessionClock
    private var handler: ((SensorEvent) -> Void)?
    public var activityMode: ActivityMode

    /// Set when authorisation is refused, so the UI can explain rather than silently stall.
    public private(set) var authorisationDenied = false

    public init(clock: SessionClock, activityMode: ActivityMode = .otherNavigation) {
        self.clock = clock
        self.activityMode = activityMode
        super.init()
        manager.delegate = self
    }

    public func start(handler: @escaping (SensorEvent) -> Void) throws {
        guard !isRunning else { throw SensorSourceError.alreadyRunning }
        self.handler = handler

        manager.desiredAccuracy = kCLLocationAccuracyBestForNavigation
        manager.distanceFilter = kCLDistanceFilterNone
        // Core Location will otherwise pause updates when it decides the user has stopped —
        // which is precisely the pre-launch window the whole design depends on.
        manager.pausesLocationUpdatesAutomatically = false
        manager.activityType = activityMode.clValue

        if manager.authorizationStatus == .notDetermined {
            manager.requestWhenInUseAuthorization()
        }
        manager.startUpdatingLocation()
        isRunning = true
    }

    public func stop() {
        guard isRunning else { return }
        manager.stopUpdatingLocation()
        handler = nil
        isRunning = false
    }

    // MARK: - CLLocationManagerDelegate

    public func locationManager(_ manager: CLLocationManager,
                                didUpdateLocations locations: [CLLocation]) {
        for location in locations {
            handler?(.gnss(Self.makeFix(location, clock: clock)))
        }
    }

    public func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
        // A transient failure is normal indoors; nothing to do but keep listening.
    }

    public func locationManagerDidChangeAuthorization(_ manager: CLLocationManager) {
        switch manager.authorizationStatus {
        case .denied, .restricted:
            authorisationDenied = true
        default:
            authorisationDenied = false
        }
    }

    static func makeFix(_ location: CLLocation, clock: SessionClock) -> GNSSFix {
        GNSSFix(
            // Spec §2.3: this is the FIX EPOCH, not the delivery time. Delivery lags by
            // 100–500 ms and the lag varies; using the arrival time here would alone be worth
            // 0.1–0.3 s of error on a 0–60. Because results are post-processed, inserting the
            // fix at its own timestamp costs nothing.
            t: clock.sessionTime(wallClockUnixTime: location.timestamp.timeIntervalSince1970),
            speed: location.speed,                       // negative means invalid
            speedAccuracy: location.speedAccuracy,       // negative means invalid
            course: location.course,
            courseAccuracy: location.courseAccuracy,
            latitudeDegrees: location.coordinate.latitude,
            longitudeDegrees: location.coordinate.longitude,
            horizontalAccuracy: location.horizontalAccuracy,
            altitude: location.altitude,
            verticalAccuracy: location.verticalAccuracy,
            // Core Location does not expose a fix type. A negative horizontal accuracy is its
            // way of saying the fix is invalid; anything else is treated as a 3D fix.
            fixType: location.horizontalAccuracy >= 0 ? 3 : 0,
            numSV: 0,
            gnssFixOK: location.horizontalAccuracy >= 0,
            source: .coreLocation
        )
    }
}
#endif
