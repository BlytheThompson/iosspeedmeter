#if canImport(CoreMotion)
import CoreMotion
import Foundation
import QuartzCore
import PerformanceTimerCore

/// Spec §3.1 / §3.2 — the primary IMU.
///
/// Delivers `IMUSample`s already converted to SI and stamped on the session clock, so nothing
/// downstream ever sees a `CMDeviceMotion`. That is spec Decision 2 in practice.
///
/// Two details from §3 are load-bearing and easy to get wrong:
///
/// - **Timestamps.** `CMDeviceMotion.timestamp` is seconds since boot, from the same monotonic
///   clock as `CACurrentMediaTime()`. It is converted with `SessionClock`, never with
///   `Date()`.
/// - **Specific force.** The sample carries `(userAcceleration + gravity) · g`, which
///   reconstructs the raw accelerometer reading. Downstream code removes gravity using its own
///   propagated attitude, because CoreMotion's gravity estimate tilts toward sustained
///   longitudinal acceleration — see `AttitudePropagator`.
public final class CoreMotionSource: SensorSource {
    public let descriptor = SensorSourceDescriptor(
        identifier: "coremotion.devicemotion",
        displayName: "Internal IMU (CoreMotion)",
        nominalRate: 100,
        kind: .imu
    )

    public private(set) var isRunning = false

    private let manager = CMMotionManager()
    private let clock: SessionClock
    private let queue: OperationQueue

    /// Spec §3.1: 100 Hz is the third-party ceiling on `CMMotionManager`; requesting faster
    /// silently clamps. The IMU rate is not the limiting factor anyway.
    public var requestedRate: Double = 100.0

    public init(clock: SessionClock) {
        self.clock = clock
        queue = OperationQueue()
        // Spec §3.1: "Use a dedicated OperationQueue with maxConcurrentOperationCount = 1,
        // not the main queue." Serial delivery keeps samples in timestamp order and keeps the
        // 100 Hz callback off the UI thread.
        queue.maxConcurrentOperationCount = 1
        queue.qualityOfService = .userInitiated
        queue.name = "com.performancetimer.coremotion"
    }

    public func start(handler: @escaping (SensorEvent) -> Void) throws {
        guard !isRunning else { throw SensorSourceError.alreadyRunning }
        guard manager.isDeviceMotionAvailable else {
            throw SensorSourceError.unavailable("Device motion is not available")
        }

        manager.deviceMotionUpdateInterval = 1.0 / requestedRate
        isRunning = true

        // `.xArbitraryCorrectedZVertical` gives a gravity-aligned Z with a yaw reference that
        // is corrected over time but arbitrary at start. That is exactly right here: the
        // vehicle frame is solved separately in §5, and absolute heading is never used.
        manager.startDeviceMotionUpdates(
            using: .xArbitraryCorrectedZVertical, to: queue
        ) { [weak self] motion, error in
            guard let self, let motion, error == nil else { return }
            handler(.imu(Self.makeSample(motion, clock: self.clock)))
        }
    }

    public func stop() {
        guard isRunning else { return }
        manager.stopDeviceMotionUpdates()
        isRunning = false
    }

    static func makeSample(_ motion: CMDeviceMotion, clock: SessionClock) -> IMUSample {
        let attitude = motion.attitude.quaternion
        return IMUSample(
            // Boot-relative monotonic stamp (spec §2.2).
            t: clock.sessionTime(monotonicTimestamp: motion.timestamp),
            userAccelerationG: Vector3(motion.userAcceleration.x,
                                       motion.userAcceleration.y,
                                       motion.userAcceleration.z),
            gravityG: Vector3(motion.gravity.x, motion.gravity.y, motion.gravity.z),
            rotationRate: Vector3(motion.rotationRate.x,
                                  motion.rotationRate.y,
                                  motion.rotationRate.z),
            attitude: Quaternion(w: attitude.w, x: attitude.x, y: attitude.y, z: attitude.z)
        )
    }
}

/// Spec §3.4 — the barometer, for grade.
///
/// Fixed at 1 Hz; there is no interval property and requests to change it are ignored. Over a
/// 15 s run that is 15 samples, which is plenty to fit a linear elevation profile, and its
/// ~0.1 m resolution is far better than GNSS vertical.
public final class AltimeterSource: SensorSource {
    public let descriptor = SensorSourceDescriptor(
        identifier: "coremotion.altimeter",
        displayName: "Barometer",
        nominalRate: 1,
        kind: .barometer
    )

    public private(set) var isRunning = false

    private let altimeter = CMAltimeter()
    private let clock: SessionClock
    private let queue: OperationQueue

    public init(clock: SessionClock) {
        self.clock = clock
        queue = OperationQueue()
        queue.maxConcurrentOperationCount = 1
        queue.name = "com.performancetimer.altimeter"
    }

    public func start(handler: @escaping (SensorEvent) -> Void) throws {
        guard !isRunning else { throw SensorSourceError.alreadyRunning }
        guard CMAltimeter.isRelativeAltitudeAvailable() else {
            throw SensorSourceError.unavailable("Relative altitude is not available")
        }
        isRunning = true
        altimeter.startRelativeAltitudeUpdates(to: queue) { [weak self] data, error in
            guard let self, let data, error == nil else { return }
            handler(.baro(BaroSample(
                t: self.clock.sessionTime(monotonicTimestamp: data.timestamp),
                relativeAltitude: data.relativeAltitude.doubleValue,
                pressure: data.pressure.doubleValue
            )))
        }
    }

    public func stop() {
        guard isRunning else { return }
        altimeter.stopRelativeAltitudeUpdates()
        isRunning = false
    }
}
#endif
