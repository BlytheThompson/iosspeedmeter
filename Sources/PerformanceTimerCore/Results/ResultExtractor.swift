import Foundation

/// One distance mark's result (spec §9.3).
public struct DistanceResult: Equatable, Sendable, Identifiable {
    public var mark: DistanceMark
    /// ET measured from the retroactive rest anchor `t₀` (spec §8).
    public var elapsedFromRest: Double
    /// ET measured from the 1-foot rollout — what a drag-strip timeslip reports and what
    /// Dragy displays.
    public var elapsedFromRollout: Double
    /// Speed at the mark, m/s. Unaffected by the choice of anchor.
    public var trapSpeed: Double
    /// 1σ on the elapsed time, seconds (spec §9.4).
    public var sigma: Double

    public var id: String { mark.id }
    public var trapSpeedMph: Double { trapSpeed / PTConstants.mphToMetersPerSecond }
    public var trapSpeedKmh: Double { trapSpeed / PTConstants.kmhToMetersPerSecond }
}

/// One standing-start speed mark's result.
public struct SpeedResult: Equatable, Sendable, Identifiable {
    public var mark: SpeedMark
    /// ET from the rest anchor.
    public var elapsed: Double
    /// Distance covered when the speed was reached, m.
    public var distance: Double
    /// 1σ on the elapsed time, seconds.
    public var sigma: Double

    public var id: String { mark.id }
}

/// One roll window's result.
public struct RollResult: Equatable, Sendable, Identifiable {
    public var window: RollWindow
    public var elapsed: Double
    public var distance: Double
    public var sigma: Double

    public var id: String { window.id }
}

/// Everything extracted from one smoothed trace.
public struct RunResults: Equatable, Sendable {
    public var anchorTime: Double
    /// Time from the rest anchor to the 1-foot rollout mark, seconds.
    public var rolloutTime: Double?
    public var distanceResults: [DistanceResult]
    public var speedResults: [SpeedResult]
    public var rollResults: [RollResult]

    public func distanceResult(for mark: DistanceMark) -> DistanceResult? {
        distanceResults.first { $0.mark == mark }
    }

    public func speedResult(for mark: SpeedMark) -> SpeedResult? {
        speedResults.first { $0.mark == mark }
    }

    public func rollResult(for window: RollWindow) -> RollResult? {
        rollResults.first { $0.window == window }
    }
}

/// Spec §9 — turn a smoothed trace into a timeslip.
///
/// Everything here reads the **smoothed** trace. Spec Decision 1 and §6.4 are explicit that
/// the forward trace is display-only and never produces a reported result.
public enum ResultExtractor {
    public static func extract(
        from samples: [SmoothedSample],
        anchorTime: Double,
        distanceMarks: [DistanceMark] = DistanceMark.standard,
        speedMarks: [SpeedMark] = SpeedMark.standardImperial,
        rollWindows: [RollWindow] = RollWindow.standard
    ) -> RunResults {
        // Spec §9.3: rollout is the time at which s = 0.3048 m.
        let rolloutCrossing = CrossingSolver.distanceCrossing(
            target: DistanceMark.rollout.meters, samples: samples
        )
        let rolloutTime = rolloutCrossing.map { $0.time - anchorTime }

        var distanceResults: [DistanceResult] = []
        for mark in distanceMarks {
            guard let crossing = CrossingSolver.distanceCrossing(target: mark.meters,
                                                                 samples: samples) else { continue }
            let fromRest = crossing.time - anchorTime
            distanceResults.append(DistanceResult(
                mark: mark,
                elapsedFromRest: fromRest,
                elapsedFromRollout: fromRest - (rolloutTime ?? 0),
                trapSpeed: crossing.speed,
                sigma: distanceMarkSigma(crossing)
            ))
        }

        var speedResults: [SpeedResult] = []
        for mark in speedMarks {
            guard let crossing = CrossingSolver.speedCrossing(target: mark.targetMetersPerSecond,
                                                              samples: samples) else { continue }
            speedResults.append(SpeedResult(
                mark: mark,
                elapsed: crossing.time - anchorTime,
                distance: crossing.distance,
                sigma: speedMarkSigma(crossing)
            ))
        }

        var rollResults: [RollResult] = []
        for window in rollWindows {
            guard let from = CrossingSolver.speedCrossing(target: window.fromMetersPerSecond,
                                                          samples: samples),
                  let to = CrossingSolver.speedCrossing(target: window.toMetersPerSecond,
                                                        samples: samples),
                  to.time > from.time
            else { continue }
            let sigma = (pow(speedMarkSigma(from), 2) + pow(speedMarkSigma(to), 2)).squareRoot()
            rollResults.append(RollResult(
                window: window,
                elapsed: to.time - from.time,
                distance: to.distance - from.distance,
                sigma: sigma
            ))
        }

        return RunResults(
            anchorTime: anchorTime,
            rolloutTime: rolloutTime,
            distanceResults: distanceResults,
            speedResults: speedResults,
            rollResults: rollResults
        )
    }

    /// Spec §9.4: `σ_t ≈ σ_v(t_cross) / a(t_cross)`.
    ///
    /// The acceleration is floored so a roll race at steady speed does not report an infinite
    /// uncertainty. When `a` really is near zero the time of a speed crossing genuinely is
    /// poorly determined, so a large-but-finite figure is the honest answer.
    static func speedMarkSigma(_ crossing: CrossingSolver.Crossing) -> Double {
        let a = max(abs(crossing.acceleration), 0.05)
        return crossing.speedSigma / a
    }

    /// The distance-mark analogue: `σ_t ≈ σ_s(t_cross) / v(t_cross)`.
    ///
    /// Spec §9.4 only gives the speed-mark form. The same first-order propagation applied to a
    /// distance crossing divides the distance sigma by the speed at the mark.
    static func distanceMarkSigma(_ crossing: CrossingSolver.Crossing) -> Double {
        let v = max(abs(crossing.speed), 0.05)
        return crossing.distanceSigma / v
    }
}
