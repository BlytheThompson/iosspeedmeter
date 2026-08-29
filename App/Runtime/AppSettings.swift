#if canImport(SwiftUI)
import Foundation
import PerformanceTimerCore

/// User-facing configuration, persisted to `UserDefaults`.
///
/// The estimator tuning parameters are here as well as in `FilterConfiguration` because spec
/// §6.2 is explicit that its starting values are starting values: "tune from logged data,
/// don't trust these blindly". Exposing them makes the retuning loop possible on-device as
/// well as in the replay harness.
public struct AppSettings: Codable, Equatable {
    public enum GNSSSource: String, Codable, CaseIterable, Identifiable {
        case internalGNSS
        case externalWiFi
        case externalBLE

        public var id: String { rawValue }

        public var title: String {
            switch self {
            case .internalGNSS: return "Internal"
            case .externalWiFi: return "External (Wi-Fi)"
            case .externalBLE: return "External (BLE)"
            }
        }

        /// The expected 0–60 uncertainty for this configuration, from the spec §13 table.
        public var expectedUncertainty: String {
            switch self {
            case .internalGNSS: return "±0.05–0.08 s"
            case .externalWiFi: return "±0.02–0.03 s"
            case .externalBLE: return "±0.04–0.06 s"
            }
        }
    }

    public var gnssSource: GNSSSource = .internalGNSS
    public var externalHost: String = "192.168.4.1"
    public var externalPort: UInt16 = 2947
    public var wheelSpeedEnabled = false
    public var notes: String = ""

    /// Spec §3.3 debug setting — see `CoreLocationSource.ActivityMode`.
    public var activityModeRaw: String = "otherNavigation"

    public var targetDistanceMeters: Double = DistanceMark.quarterMile.meters
    public var useMetricUnits = false

    // Spec §6.2 tuning.
    public var sigmaA: Double = 0.05
    public var sigmaB: Double = 0.002
    public var sigmaGPSFloor: Double = 0.05
    public var sigmaZUPT: Double = 0.01
    public var useSpecLiteralProcessNoise = false

    private var calibrationData: Data?

    private static let key = "com.performancetimer.settings"

    public static func load() -> AppSettings {
        guard let data = UserDefaults.standard.data(forKey: key),
              let decoded = try? JSONDecoder().decode(AppSettings.self, from: data)
        else { return AppSettings() }
        return decoded
    }

    public func save() {
        guard let data = try? JSONEncoder().encode(self) else { return }
        UserDefaults.standard.set(data, forKey: Self.key)
    }

    public var filterConfiguration: FilterConfiguration {
        FilterConfiguration(
            sigmaA: sigmaA,
            sigmaB: sigmaB,
            sigmaGPSFloor: sigmaGPSFloor,
            sigmaZUPT: sigmaZUPT,
            processNoiseModel: useSpecLiteralProcessNoise ? .specLiteral : .exact,
            wheelSpeedEnabled: wheelSpeedEnabled
        )
    }

    public var storedCalibration: VehicleFrameCalibration? {
        guard let calibrationData else { return nil }
        return try? JSONDecoder().decode(VehicleFrameCalibration.self, from: calibrationData)
    }

    public mutating func storeCalibration(_ calibration: VehicleFrameCalibration) {
        calibrationData = try? JSONEncoder().encode(calibration)
        save()
    }

    public mutating func clearCalibration() {
        calibrationData = nil
        save()
    }
}

#if canImport(CoreLocation)
import CoreLocation

extension AppSettings {
    public var activityMode: CoreLocationSource.ActivityMode {
        CoreLocationSource.ActivityMode(rawValue: activityModeRaw) ?? .otherNavigation
    }
}
#endif
#endif
