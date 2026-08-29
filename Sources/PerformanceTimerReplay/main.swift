import Foundation
import PerformanceTimerCore

// Spec §10 / §15 step 2 — the offline replay harness.
//
// "Build an offline replay mode that reads a CSV and reruns the whole estimator. You will
// retune Q and R a dozen times and you do not want to drive for each iteration."
//
// This is a plain CLI over `SessionProcessor`, so the estimator it runs is byte-for-byte the
// one that runs on the phone. It also carries a `--sweep` mode for exactly the retuning loop
// the spec describes, and a `--synthetic` mode so the whole chain can be exercised with no
// recorded data at all.

let version = "1.0"

func printUsage() {
    print("""
    pt-replay \(version) — offline replay harness for the GNSS/IMU performance timer

    USAGE
      pt-replay <session.csv> [options]
      pt-replay --synthetic [options]
      pt-replay --list <directory>

    OPTIONS
      --sigma-a <value>        Accelerometer white noise, m/s^2/sqrt(Hz)   (default 0.05)
      --sigma-b <value>        Bias random walk, m/s^3/sqrt(Hz)           (default 0.002)
      --sigma-gps-floor <v>    Floor on GNSS speed sigma, m/s             (default 0.05)
      --sigma-zupt <value>     ZUPT measurement noise, m/s                (default 0.01)
      --gate <sigmas>          Innovation gate, in sigmas                 (default 3)
      --q-model <exact|spec>   Process-noise discretisation               (default exact)
      --wheel-speed            Enable the CAN wheel-scale state (spec 3.7)
      --sweep <param>          Sweep one parameter and print a table
                               (sigma-a | sigma-b | sigma-zupt | q-model)
      --csv-out <path>         Write the re-analysed trace back out as CSV
      --json                   Emit results as JSON instead of a table
      --quiet                  Suppress the per-mark table

    EXAMPLES
      pt-replay session-1700000000-1b4e28ba.csv
      pt-replay session.csv --sweep sigma-a
      pt-replay session.csv --q-model spec        # compare against the published table
      pt-replay --synthetic --json
    """)
}

// MARK: - Argument parsing

struct Options {
    var inputPath: String?
    var listDirectory: String?
    var synthetic = false
    var filter = FilterConfiguration()
    var sweep: String?
    var csvOut: String?
    var json = false
    var quiet = false
}

func parseArguments(_ arguments: [String]) -> Options? {
    var options = Options()
    var index = 0

    func next(_ flag: String) -> String? {
        index += 1
        guard index < arguments.count else {
            FileHandle.standardError.write(Data("error: \(flag) needs a value\n".utf8))
            return nil
        }
        return arguments[index]
    }

    while index < arguments.count {
        let argument = arguments[index]
        switch argument {
        case "--help", "-h":
            printUsage()
            exit(0)
        case "--version":
            print(version)
            exit(0)
        case "--synthetic":
            options.synthetic = true
        case "--wheel-speed":
            options.filter.wheelSpeedEnabled = true
        case "--json":
            options.json = true
        case "--quiet":
            options.quiet = true
        case "--list":
            guard let value = next(argument) else { return nil }
            options.listDirectory = value
        case "--sigma-a":
            guard let value = next(argument), let number = Double(value) else { return nil }
            options.filter.sigmaA = number
        case "--sigma-b":
            guard let value = next(argument), let number = Double(value) else { return nil }
            options.filter.sigmaB = number
        case "--sigma-gps-floor":
            guard let value = next(argument), let number = Double(value) else { return nil }
            options.filter.sigmaGPSFloor = number
        case "--sigma-zupt":
            guard let value = next(argument), let number = Double(value) else { return nil }
            options.filter.sigmaZUPT = number
        case "--gate":
            guard let value = next(argument), let number = Double(value) else { return nil }
            options.filter.innovationGateSigma = number
        case "--q-model":
            guard let value = next(argument) else { return nil }
            switch value {
            case "exact": options.filter.processNoiseModel = .exact
            case "spec", "specLiteral": options.filter.processNoiseModel = .specLiteral
            default:
                FileHandle.standardError.write(Data("error: unknown q-model '\(value)'\n".utf8))
                return nil
            }
        case "--sweep":
            guard let value = next(argument) else { return nil }
            options.sweep = value
        case "--csv-out":
            guard let value = next(argument) else { return nil }
            options.csvOut = value
        default:
            if argument.hasPrefix("-") {
                FileHandle.standardError.write(Data("error: unknown option '\(argument)'\n".utf8))
                return nil
            }
            options.inputPath = argument
        }
        index += 1
    }
    return options
}

// MARK: - Synthetic session

/// A deterministic synthetic run, so the harness is useful before any data has been recorded.
/// Mirrors the generator used by the test suite.
func syntheticEvents() -> [SensorEvent] {
    var state: UInt64 = 0x2545F4914F6CDD1D
    func nextUnit() -> Double {
        state ^= state << 13
        state ^= state >> 7
        state ^= state << 17
        return Double(state >> 11) / Double(1 << 53)
    }
    func gaussian() -> Double {
        var sum = 0.0
        for _ in 0..<12 { sum += nextUnit() }
        return sum - 6.0
    }

    let mount = Quaternion(axis: Vector3(0.15, -0.3, 0.94).normalized()!, angle: 0.75)
    let gravityVehicle = Vector3(0, 0, -PTConstants.g)
    let dt = 0.01
    let stationary = 3.0
    let total = 17.0
    let accelerometerBias = 0.12

    var events: [SensorEvent] = []
    var speed = 0.0
    var distance = 0.0
    var t = 0.0
    var sampleIndex = 0

    while t <= total {
        let a = t < stationary ? 0.0 : 5.0
        let measured = a + accelerometerBias + gaussian() * 0.02
        let specificForceDevice = mount.rotate(Vector3(measured, 0, 0) + gravityVehicle)
        let tilt = atan2(a, PTConstants.g) * 0.35
        let gravityDevice = mount.rotate(
            Vector3(-sin(tilt) * PTConstants.g, 0, -cos(tilt) * PTConstants.g))

        events.append(.imu(IMUSample(
            t: t,
            specificForce: specificForceDevice,
            rotationRate: Vector3(0.0008 + gaussian() * 0.0008,
                                  -0.0011 + gaussian() * 0.0008,
                                  0.0006 + gaussian() * 0.0008),
            gravity: gravityDevice,
            userAcceleration: specificForceDevice - gravityDevice,
            attitude: mount.conjugate
        )))

        if sampleIndex % 100 == 0 {
            events.append(.gnss(GNSSFix(
                t: t, speed: max(0, speed + gaussian() * 0.04), speedAccuracy: 0.05,
                course: 0, courseAccuracy: 1.0,
                latitudeDegrees: 37.4275, longitudeDegrees: -122.1697,
                horizontalAccuracy: 2.0, altitude: 30, verticalAccuracy: 3.0,
                fixType: 3, numSV: 14, gnssFixOK: true, source: .replay)))
            events.append(.baro(BaroSample(t: t, relativeAltitude: 0, pressure: 101.3)))
        }

        distance += speed * dt + 0.5 * a * dt * dt
        speed += a * dt
        t += dt
        sampleIndex += 1
    }
    return events
}


// MARK: - Formatting helpers

/// Left-justify to a fixed width. Using Swift string padding rather than `%-14s` keeps the
/// output identical on Linux and Windows, where bridging to `NSString` for `utf8String` is not
/// available.
func pad(_ text: String, _ width: Int) -> String {
    text.count >= width ? text : text + String(repeating: " ", count: width - text.count)
}

/// Right-justify to a fixed width.
func padLeft(_ text: String, _ width: Int) -> String {
    text.count >= width ? text : String(repeating: " ", count: width - text.count) + text
}

func fixed(_ value: Double, _ decimals: Int) -> String {
    String(format: "%.\(decimals)f", value)
}

// MARK: - Reporting

func describe(_ analysis: SessionAnalysis, quiet: Bool) {
    let diagnostics = analysis.diagnostics
    print("""
    Samples          \(diagnostics.imuSampleCount) IMU at \
    \(String(format: "%.1f", diagnostics.meanIMUInterval > 0 ? 1 / diagnostics.meanIMUInterval : 0)) Hz
    GNSS             \(diagnostics.acceptedFixCount) accepted, \
    \(diagnostics.rejectedFixCount) rejected \
    (\(diagnostics.gateRejectionCount) by the 3σ gate) at \
    \(fixed(diagnostics.achievedGNSSRate, 2)) Hz
    ZUPT             \(diagnostics.zuptSampleCount) samples
    Vehicle frame    \(diagnostics.usedProvisionalVehicleFrame ? "PROVISIONAL (gravity only)" : "solved")
    Confidence       \(analysis.confidence.badge.rawValue)
    """)
    for caveat in analysis.confidence.caveats {
        print("                 · \(caveat)")
    }

    if let grade = analysis.grade {
        let sign = grade.meanGradePercent >= 0 ? "+" : ""
        print("Grade            \(sign)\(fixed(grade.meanGradePercent, 2))% "
              + "(\(grade.source.rawValue) source, R² \(fixed(grade.fitQuality, 3)))"
              + (grade.exceedsReportingThreshold ? "  ⚠ exceeds 1%" : ""))
        if grade.barometerDisagreesWithGNSS {
            print("                 ⚠ barometer disagrees with GNSS by more than 1 m")
        }
    }

    guard let anchor = analysis.anchor, let results = analysis.results else {
        print("\nNo launch detected — no timing results. The trace is still complete.")
        return
    }
    print("Launch anchor    t = \(fixed(anchor.time, 3)) s")
    if quiet { return }

    print("\nSPEED MARKS                    ET        ±σ       at")
    for result in results.speedResults {
        print("  " + pad(result.mark.name, 24) + " "
              + padLeft(fixed(result.elapsed, 3), 7) + " s  "
              + padLeft(fixed(result.sigma, 3), 5) + "   "
              + padLeft(fixed(result.distance, 1), 6) + " m")
    }

    print("\nDISTANCE MARKS       from rest   1-ft rollout      trap        ±σ")
    for result in results.distanceResults {
        print("  " + pad(result.mark.name, 14) + " "
              + padLeft(fixed(result.elapsedFromRest, 3), 9) + " s "
              + padLeft(fixed(result.elapsedFromRollout, 3), 11) + " s "
              + padLeft(fixed(result.trapSpeedMph, 1), 9) + " mph  "
              + padLeft(fixed(result.sigma, 3), 5))
    }

    if !results.rollResults.isEmpty {
        print("\nROLL WINDOWS                   ET        ±σ")
        for result in results.rollResults {
            print("  " + pad(result.window.name, 24) + " "
                  + padLeft(fixed(result.elapsed, 3), 7) + " s  "
                  + padLeft(fixed(result.sigma, 3), 5))
        }
    }

    if let corrected = analysis.gradeCorrectedResults {
        print("\nGRADE-CORRECTED (approximation, not a transform — spec §7.2)")
        for result in corrected.distanceResults {
            print("  " + pad(result.mark.name, 14) + " "
                  + padLeft(fixed(result.elapsedFromRest, 3), 9) + " s")
        }
    }
}

func emitJSON(_ analysis: SessionAnalysis) {
    var output: [String: Any] = [
        "confidence": analysis.confidence.badge.rawValue,
        "caveats": analysis.confidence.caveats,
        "acceptedFixes": analysis.diagnostics.acceptedFixCount,
        "rejectedFixes": analysis.diagnostics.rejectedFixCount,
        "gnssRateHz": analysis.diagnostics.achievedGNSSRate,
        "zuptSamples": analysis.diagnostics.zuptSampleCount,
    ]
    if let grade = analysis.grade { output["gradePercent"] = grade.meanGradePercent }
    if let anchor = analysis.anchor { output["anchorTime"] = anchor.time }
    if let results = analysis.results {
        output["speedMarks"] = results.speedResults.map {
            ["name": $0.mark.name, "elapsed": $0.elapsed, "sigma": $0.sigma]
        }
        output["distanceMarks"] = results.distanceResults.map {
            ["name": $0.mark.name, "fromRest": $0.elapsedFromRest,
             "fromRollout": $0.elapsedFromRollout, "trapSpeedMph": $0.trapSpeedMph,
             "sigma": $0.sigma]
        }
    }
    if let data = try? JSONSerialization.data(withJSONObject: output,
                                              options: [.prettyPrinted, .sortedKeys]) {
        print(String(decoding: data, as: UTF8.self))
    }
}

/// Spec §6.2: "tune from logged data, don't trust these blindly." This is that loop.
func sweep(_ parameter: String, events: [SensorEvent], base: FilterConfiguration) {
    func run(_ configuration: FilterConfiguration) -> (Double, Double, Int)? {
        let processor = SessionProcessor(
            configuration: SessionProcessor.Configuration(filter: configuration))
        processor.ingest(events)
        let analysis = processor.analyse()
        guard let result = analysis.results?.speedResult(for: .zeroToSixty) else { return nil }
        return (result.elapsed, result.sigma, analysis.diagnostics.rejectedFixCount)
    }

    print("\(parameter.padding(toLength: 14, withPad: " ", startingAt: 0))  0–60 s    ±σ      rejected")
    switch parameter {
    case "sigma-a", "sigma-b", "sigma-zupt":
        let values: [Double] = parameter == "sigma-zupt"
            ? [0.002, 0.005, 0.01, 0.02, 0.05, 0.1]
            : (parameter == "sigma-a"
                ? [0.01, 0.02, 0.05, 0.1, 0.2, 0.5]
                : [0.0002, 0.0005, 0.001, 0.002, 0.005, 0.01])
        for value in values {
            var configuration = base
            switch parameter {
            case "sigma-a": configuration.sigmaA = value
            case "sigma-b": configuration.sigmaB = value
            default: configuration.sigmaZUPT = value
            }
            if let (elapsed, sigma, rejected) = run(configuration) {
                print(pad("\(value)", 14) + "  " + padLeft(fixed(elapsed, 3), 7)
                      + "  " + padLeft(fixed(sigma, 4), 6)
                      + "  " + padLeft("\(rejected)", 8))
            } else {
                print(pad("\(value)", 14) + "  " + padLeft("no launch", 7))
            }
        }
    case "q-model":
        for model in ProcessNoiseModel.allCases {
            var configuration = base
            configuration.processNoiseModel = model
            if let (elapsed, sigma, rejected) = run(configuration) {
                print(pad(model.rawValue, 14) + "  " + padLeft(fixed(elapsed, 3), 7)
                      + "  " + padLeft(fixed(sigma, 4), 6)
                      + "  " + padLeft("\(rejected)", 8))
            } else {
                print(pad(model.rawValue, 14) + "  " + padLeft("no launch", 7))
            }
        }
    default:
        FileHandle.standardError.write(Data("error: cannot sweep '\(parameter)'\n".utf8))
        exit(2)
    }
}

// MARK: - Entry point

let arguments = Array(CommandLine.arguments.dropFirst())
guard let options = parseArguments(arguments) else { exit(2) }

if let listDirectory = options.listDirectory {
    let store = SessionLogStore(directory: URL(fileURLWithPath: listDirectory))
    let sessions = (try? store.listSessions()) ?? []
    if sessions.isEmpty {
        print("No sessions in \(listDirectory)")
    }
    for session in sessions {
        let date = Date(timeIntervalSince1970: session.header.startedAtUnixTime)
        print("\(session.files.csvURL.lastPathComponent)  \(date)  "
              + "\(session.rows.count) rows  \(session.header.deviceModel)")
    }
    exit(0)
}

var events: [SensorEvent]
if options.synthetic {
    print("Replaying a synthetic session (no recorded data).")
    events = syntheticEvents()
} else if let path = options.inputPath {
    guard let data = FileManager.default.contents(atPath: path) else {
        FileHandle.standardError.write(Data("error: cannot read \(path)\n".utf8))
        exit(1)
    }
    let rows = SessionLogCSVReader.rows(from: data)
    guard !rows.isEmpty else {
        FileHandle.standardError.write(Data("error: no usable rows in \(path)\n".utf8))
        exit(1)
    }
    print("Replaying \(rows.count) rows from \(path)")
    events = SessionLogBuilder.events(from: rows)
} else {
    printUsage()
    exit(2)
}

if let parameter = options.sweep {
    sweep(parameter, events: events, base: options.filter)
    exit(0)
}

let processor = SessionProcessor(
    configuration: SessionProcessor.Configuration(filter: options.filter))
processor.ingest(events)
let analysis = processor.analyse()

if options.json {
    emitJSON(analysis)
} else {
    describe(analysis, quiet: options.quiet)
}

if let csvOut = options.csvOut {
    var writer = SessionLogCSVWriter()
    writer.append(contentsOf: SessionLogBuilder.rows(analysis: analysis, events: events))
    do {
        try writer.data().write(to: URL(fileURLWithPath: csvOut), options: .atomic)
        print("\nWrote re-analysed trace to \(csvOut)")
    } catch {
        FileHandle.standardError.write(Data("error: cannot write \(csvOut): \(error)\n".utf8))
        exit(1)
    }
}
