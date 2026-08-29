// swift-tools-version:5.9
import PackageDescription

// PerformanceTimerCore is deliberately free of Apple-framework imports (no CoreMotion,
// CoreLocation, Network, CoreBluetooth). That is spec Decision 2: the estimator is built
// against a sensor protocol, not against CLLocationManager. The practical payoff is that
// the entire numerical core builds and tests on Windows/Linux CI as well as macOS, so the
// maths is verifiable without a device.
//
// The iOS application layer (CoreMotion/CoreLocation/Network adapters + SwiftUI) lives in
// App/ and is assembled by XcodeGen from project.yml; it consumes this package.
let package = Package(
    name: "PerformanceTimer",
    // Declaring the Apple platforms only sets minimum deployment versions for those
    // platforms; it does not restrict where the package builds, so Linux and Windows are
    // unaffected and the core keeps building and testing there.
    platforms: [
        .iOS(.v17),
        .macOS(.v13),
    ],
    products: [
        .library(name: "PerformanceTimerCore", targets: ["PerformanceTimerCore"]),
        .executable(name: "pt-replay", targets: ["PerformanceTimerReplay"]),
    ],
    targets: [
        .target(
            name: "PerformanceTimerCore",
            path: "Sources/PerformanceTimerCore"
        ),
        .executableTarget(
            name: "PerformanceTimerReplay",
            dependencies: ["PerformanceTimerCore"],
            path: "Sources/PerformanceTimerReplay"
        ),
        .testTarget(
            name: "PerformanceTimerCoreTests",
            dependencies: ["PerformanceTimerCore"],
            path: "Tests/PerformanceTimerCoreTests"
        ),
    ]
)
