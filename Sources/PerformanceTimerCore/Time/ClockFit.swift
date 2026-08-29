import Foundation

/// Spec §2.4 — external receiver time alignment.
///
/// A UBX or BLE packet carries GPS time of week; what you observe locally is an arrival time
/// contaminated by transport delay and 5–40 ms of Wi-Fi/BLE jitter. Timestamping on arrival
/// puts that jitter straight into the results. Instead, fit `t_session = α·iTOW + β` once per
/// session over pairs collected at rest: `β` absorbs the constant transport delay and the
/// GPS-week offset, `α` absorbs any clock-rate mismatch, and the residual scatter is a direct
/// measurement of the link jitter worth logging.
public struct ClockFit: Equatable, Sendable {
    /// Nominal slope: iTOW is in milliseconds, session time in seconds.
    public static let nominalSlope = 0.001
    /// Spec §2.4: "if it deviates by more than 100 ppm your clocks disagree".
    public static let slopeTolerancePPM = 100.0
    /// Spec §2.4: "Collect ≥ 30 pairs at rest."
    public static let minimumSamples = 30

    public enum Error: Swift.Error, Equatable {
        case insufficientSamples(have: Int, need: Int)
        case slopeOutOfTolerance(slope: Double, ppm: Double)
        case degenerate
    }

    public struct Solution: Equatable, Codable, Sendable {
        public let slope: Double
        public let intercept: Double
        /// RMS of the fit residuals, seconds. This is the link jitter.
        public let residualRMS: Double
        public let sampleCount: Int

        public init(slope: Double, intercept: Double, residualRMS: Double, sampleCount: Int) {
            self.slope = slope
            self.intercept = intercept
            self.residualRMS = residualRMS
            self.sampleCount = sampleCount
        }

        /// Map a packet's iTOW onto the session clock.
        public func sessionTime(itowMilliseconds: Double) -> Double {
            slope * itowMilliseconds + intercept
        }

        /// Deviation of the fitted rate from nominal, in parts per million.
        public var slopeErrorPPM: Double {
            (slope / ClockFit.nominalSlope - 1) * 1e6
        }
    }

    private var itows: [Double] = []
    private var arrivals: [Double] = []

    public init() {}

    public var sampleCount: Int { itows.count }

    /// Record one `(iTOW, arrival)` pair. Collect these while the vehicle is at rest.
    public mutating func add(itowMilliseconds: Double, arrivalSessionTime: Double) {
        itows.append(itowMilliseconds)
        arrivals.append(arrivalSessionTime)
    }

    public mutating func reset() {
        itows.removeAll(keepingCapacity: true)
        arrivals.removeAll(keepingCapacity: true)
    }

    /// Ordinary least squares over the collected pairs.
    public func solve() throws -> Solution {
        let n = itows.count
        guard n >= Self.minimumSamples else {
            throw Error.insufficientSamples(have: n, need: Self.minimumSamples)
        }

        let count = Double(n)
        let meanX = itows.reduce(0, +) / count
        let meanY = arrivals.reduce(0, +) / count

        var sxx = 0.0
        var sxy = 0.0
        for i in 0..<n {
            let dx = itows[i] - meanX
            sxx += dx * dx
            sxy += dx * (arrivals[i] - meanY)
        }
        guard sxx > 0 else { throw Error.degenerate }

        let slope = sxy / sxx
        let intercept = meanY - slope * meanX

        let ppm = (slope / Self.nominalSlope - 1) * 1e6
        guard abs(ppm) <= Self.slopeTolerancePPM else {
            throw Error.slopeOutOfTolerance(slope: slope, ppm: ppm)
        }

        var sumSquaredResiduals = 0.0
        for i in 0..<n {
            let residual = arrivals[i] - (slope * itows[i] + intercept)
            sumSquaredResiduals += residual * residual
        }

        return Solution(
            slope: slope,
            intercept: intercept,
            residualRMS: (sumSquaredResiduals / count).squareRoot(),
            sampleCount: n
        )
    }
}
