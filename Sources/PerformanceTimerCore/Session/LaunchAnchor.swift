import Foundation

/// Spec §8 — the live launch trigger.
///
/// This only decides *when to start recording*. It deliberately does not define `t = 0`;
/// that comes from the retroactive anchor below, which is what removes the arbitrariness of
/// the threshold choice from the reported number.
public struct LaunchDetector: Sendable {
    /// Spec §8: `a > 0.15 g`.
    public var thresholdAcceleration: Double
    /// Spec §8: sustained for ≥ 60 ms.
    public var sustainDuration: Double

    private var aboveSince: Double?

    public init(
        thresholdAcceleration: Double = 0.15 * PTConstants.g,
        sustainDuration: Double = 0.060
    ) {
        self.thresholdAcceleration = thresholdAcceleration
        self.sustainDuration = sustainDuration
    }

    /// Feed one sample; returns true on the sample where the trigger condition is first met.
    public mutating func update(t: Double, acceleration: Double) -> Bool {
        guard acceleration > thresholdAcceleration else {
            aboveSince = nil
            return false
        }
        guard let since = aboveSince else {
            aboveSince = t
            return false
        }
        return t - since >= sustainDuration
    }

    public mutating func reset() {
        aboveSince = nil
    }
}

/// Spec §8 — retroactive launch anchoring.
public enum LaunchAnchor {
    public struct Anchor: Equatable, Sendable {
        public var index: Int
        public var time: Double

        public init(index: Int, time: Double) {
            self.index = index
            self.time = time
        }
    }

    /// Spec §8: `v_smoothed < 0.02 m/s`.
    public static let maximumAnchorSpeed = 0.02
    /// Spec §8: the next 100 ms of `a` must be monotonically increasing.
    public static let risingWindow = 0.100

    /// Search **backward** from the live trigger for the last sample that was genuinely at
    /// rest and about to accelerate.
    ///
    /// Because results are post-processed, the anchor can be placed precisely rather than
    /// guessed live. Spec §8: "This routinely recovers 100–200 ms versus triggering on GNSS,
    /// and it removes the arbitrariness of a threshold choice from your reported number."
    ///
    /// - Parameters:
    ///   - samples: the smoothed trace.
    ///   - stationaryFlags: the stationary detector's verdict per sample, same length.
    ///   - triggerIndex: index at which the live trigger fired.
    public static func retroactiveAnchor(
        samples: [SmoothedSample],
        stationaryFlags: [Bool],
        triggerIndex: Int
    ) -> Anchor? {
        guard !samples.isEmpty else { return nil }
        let start = min(triggerIndex, samples.count - 1)
        guard start >= 0 else { return nil }

        for index in stride(from: start, through: 0, by: -1) {
            guard abs(samples[index].speed) < maximumAnchorSpeed else { continue }
            if index < stationaryFlags.count, !stationaryFlags[index] { continue }
            guard accelerationIsRising(from: index, in: samples) else { continue }
            return Anchor(index: index, time: samples[index].t)
        }
        return nil
    }

    /// Minimum rise across the 100 ms window for it to count as the start of a launch, m/s².
    /// A real launch builds far more than this; sensor noise does not.
    public static let minimumRise = 1.0

    /// Is the acceleration rising over the next 100 ms?
    ///
    /// Spec §8 words this as "monotonically increasing", but applied literally to 100 Hz data
    /// it never matches: accelerometer noise puts small reversals throughout, so a sample-wise
    /// staircase test fails at the samples closest to the launch and the search falls through
    /// to an earlier, quieter instant. Measured on a synthetic run that placed the anchor 80 ms
    /// early, which propagated directly into the reported 0–60.
    ///
    /// A least-squares slope over the window captures the same physical intent — "this is where
    /// acceleration starts to build" — without being hostage to individual noisy samples.
    private static func accelerationIsRising(from index: Int, in samples: [SmoothedSample]) -> Bool {
        let start = samples[index].t
        let endTime = start + risingWindow

        var xs: [Double] = []
        var ys: [Double] = []
        var i = index
        while i < samples.count, samples[i].t <= endTime {
            xs.append(samples[i].t - start)
            ys.append(samples[i].correctedAcceleration)
            i += 1
        }
        guard xs.count >= 3 else { return false }

        let n = Double(xs.count)
        let meanX = xs.reduce(0, +) / n
        let meanY = ys.reduce(0, +) / n
        var sxx = 0.0
        var sxy = 0.0
        for k in 0..<xs.count {
            let dx = xs[k] - meanX
            sxx += dx * dx
            sxy += dx * (ys[k] - meanY)
        }
        guard sxx > 0 else { return false }

        return (sxy / sxx) * risingWindow >= minimumRise
    }

    /// Spec §8 roll-race variant.
    ///
    /// "For a 60–130 style run there is no stationary anchor and no ZUPT. Anchor on the
    /// smoothed speed trace crossing the lower bound and accept the wider uncertainty. Flag
    /// roll results as a distinct confidence class."
    public static func rollAnchor(
        samples: [SmoothedSample],
        window: RollWindow
    ) -> CrossingSolver.Crossing? {
        CrossingSolver.speedCrossing(target: window.fromMetersPerSecond, samples: samples)
    }
}
