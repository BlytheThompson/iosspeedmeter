import Foundation

/// Spec §7.2 — grade over the measured interval, and the corrected result.
public struct GradeEstimate: Equatable, Sendable {
    public enum Source: String, Equatable, Codable, Sendable {
        case barometric
        case gnss
    }

    /// Mean grade over the run, as a percentage. Positive is uphill.
    public var meanGradePercent: Double
    /// Same thing in radians, for `a + g·sin θ`.
    public var meanGradeRadians: Double { atan(meanGradePercent / 100) }
    public var source: Source
    /// Total elevation change over the run, m.
    public var elevationChange: Double
    /// Distance over which the fit was made, m.
    public var fittedDistance: Double
    /// R² of the elevation-vs-distance fit. A poor fit means the grade is not constant and the
    /// single-number correction is a cruder approximation than usual.
    public var fitQuality: Double
    /// Spec §3.4: barometric and GNSS altitude disagreeing by more than 1 m over the run points
    /// at cabin pressure — HVAC, an open window, a flexing door seal — not at real elevation.
    public var barometerDisagreesWithGNSS: Bool

    public init(
        meanGradePercent: Double, source: Source, elevationChange: Double,
        fittedDistance: Double, fitQuality: Double, barometerDisagreesWithGNSS: Bool
    ) {
        self.meanGradePercent = meanGradePercent
        self.source = source
        self.elevationChange = elevationChange
        self.fittedDistance = fittedDistance
        self.fitQuality = fitQuality
        self.barometerDisagreesWithGNSS = barometerDisagreesWithGNSS
    }

    /// Spec §7.2: "A run with more than 1% mean grade should be visually flagged."
    public var exceedsReportingThreshold: Bool { abs(meanGradePercent) > 1.0 }

    /// Re-integrate a trace with the grade component removed from the acceleration.
    ///
    /// Spec §7.2 is emphatic that this is "an approximation, not a transform" and that the UI
    /// must say so. What it does is exactly the §7.1 correction — `a_corrected = a + g·sin θ`
    /// — applied over the whole run and re-integrated from the same initial conditions. It
    /// does **not** model the change in aerodynamic drag, traction or gearing that a genuinely
    /// flat run would have had.
    public func correctedTrace(_ samples: [SmoothedSample]) -> [SmoothedSample] {
        guard samples.count > 1 else { return samples }
        let delta = PTConstants.g * sin(meanGradeRadians)

        var corrected: [SmoothedSample] = []
        corrected.reserveCapacity(samples.count)

        var speed = samples[0].speed
        var distance = samples[0].distance
        var previousTime = samples[0].t

        for (index, sample) in samples.enumerated() {
            let dt = index == 0 ? 0 : sample.t - previousTime
            previousTime = sample.t

            let acceleration = sample.correctedAcceleration + delta
            if dt > 0 {
                distance += speed * dt + 0.5 * acceleration * dt * dt
                speed += acceleration * dt
            }

            var updated = sample
            var state = sample.state
            state[StateIndex.distance, 0] = distance
            state[StateIndex.speed, 0] = speed
            state[StateIndex.bias, 0] = 0          // the trace is already bias-corrected
            updated.state = state
            updated.acceleration = acceleration
            corrected.append(updated)
        }
        return corrected
    }
}

/// Spec §7.2 — fit elevation against distance over the run.
public enum GradeEstimator {
    /// Minimum number of elevation samples worth fitting. Over a 15 s run the 1 Hz barometer
    /// gives about 15 (spec §3.4), so this is deliberately low.
    public static let minimumSamples = 4
    /// Spec §3.4: disagreement over this many metres means the barometer is untrustworthy.
    public static let baroGNSSDisagreementLimit = 1.0

    public static func estimate(
        samples: [SmoothedSample],
        barometric: [BaroSample],
        gnss: [GNSSFix]
    ) -> GradeEstimate? {
        guard samples.count > 1 else { return nil }

        let baroPoints = pair(times: barometric.map(\.t),
                              elevations: barometric.map(\.relativeAltitude),
                              samples: samples)
        let gnssPoints = pair(times: gnss.map(\.t),
                              elevations: gnss.map(\.altitude),
                              samples: samples)

        let baroFit = fit(baroPoints)
        let gnssFit = fit(gnssPoints)

        // Cross-check before choosing (spec §3.4).
        var disagrees = false
        if let b = baroFit, let g = gnssFit {
            let span = max(1e-9, b.span)
            let baroChange = b.slope * span
            let gnssChange = g.slope * span
            disagrees = abs(baroChange - gnssChange) > baroGNSSDisagreementLimit
        }

        // Barometer is primary — 0.1 m resolution against GNSS vertical's several metres —
        // unless the cross-check says its readings are a cabin-pressure artefact.
        let chosen: (fit: LinearFit, source: GradeEstimate.Source)?
        if let b = baroFit, !disagrees {
            chosen = (b, .barometric)
        } else if let g = gnssFit {
            chosen = (g, .gnss)
        } else if let b = baroFit {
            chosen = (b, .barometric)
        } else {
            chosen = nil
        }
        guard let (selected, source) = chosen else { return nil }

        return GradeEstimate(
            meanGradePercent: selected.slope * 100,
            source: source,
            elevationChange: selected.slope * selected.span,
            fittedDistance: selected.span,
            fitQuality: selected.rSquared,
            barometerDisagreesWithGNSS: disagrees
        )
    }

    // MARK: - Fitting

    struct LinearFit {
        /// Rise per unit run — the grade as a fraction.
        var slope: Double
        var intercept: Double
        var rSquared: Double
        /// Distance covered by the fitted points, m.
        var span: Double
    }

    /// Match each elevation reading to the distance travelled at that instant.
    private static func pair(
        times: [Double], elevations: [Double], samples: [SmoothedSample]
    ) -> [(distance: Double, elevation: Double)] {
        guard !times.isEmpty else { return [] }
        var points: [(Double, Double)] = []
        points.reserveCapacity(times.count)

        for (t, elevation) in zip(times, elevations) {
            guard let distance = interpolatedDistance(at: t, samples: samples) else { continue }
            points.append((distance, elevation))
        }
        return points
    }

    private static func interpolatedDistance(at t: Double, samples: [SmoothedSample]) -> Double? {
        guard let first = samples.first, let last = samples.last else { return nil }
        if t <= first.t { return first.distance }
        if t >= last.t { return last.distance }

        // Binary search for the bracketing pair.
        var low = 0
        var high = samples.count - 1
        while high - low > 1 {
            let mid = (low + high) / 2
            if samples[mid].t <= t { low = mid } else { high = mid }
        }
        let a = samples[low], b = samples[high]
        let dt = b.t - a.t
        guard dt > 0 else { return a.distance }
        let blend = (t - a.t) / dt
        return a.distance + (b.distance - a.distance) * blend
    }

    private static func fit(_ points: [(distance: Double, elevation: Double)]) -> LinearFit? {
        guard points.count >= minimumSamples else { return nil }
        let n = Double(points.count)
        let meanX = points.reduce(0) { $0 + $1.distance } / n
        let meanY = points.reduce(0) { $0 + $1.elevation } / n

        var sxx = 0.0, sxy = 0.0, syy = 0.0
        for p in points {
            let dx = p.distance - meanX
            let dy = p.elevation - meanY
            sxx += dx * dx
            sxy += dx * dy
            syy += dy * dy
        }
        guard sxx > 1e-9 else { return nil }

        let slope = sxy / sxx
        let intercept = meanY - slope * meanX
        // A perfectly flat road has syy == 0; the fit is exact, so R² is 1 by convention.
        let rSquared = syy > 1e-12 ? (sxy * sxy) / (sxx * syy) : 1.0

        let xs = points.map(\.distance)
        let span = (xs.max() ?? 0) - (xs.min() ?? 0)
        return LinearFit(slope: slope, intercept: intercept, rSquared: rSquared, span: span)
    }
}
