import XCTest
@testable import PerformanceTimerCore

/// Builds a smoothed trace for an analytic motion so crossings have exact expected answers.
enum SyntheticTrace {
    /// Constant acceleration from rest: `v = a·t`, `s = ½a·t²`.
    static func constantAcceleration(
        _ a: Double, duration: Double, dt: Double = 0.01, speedSigma: Double = 0.02
    ) -> [SmoothedSample] {
        var out: [SmoothedSample] = []
        for t in stride(from: 0.0, through: duration, by: dt) {
            let distance: Double = 0.5 * a * t * t
            let speed: Double = a * t
            out.append(sample(t: t, s: distance, v: speed, a: a, speedSigma: speedSigma))
        }
        return out
    }

    /// Constant jerk from rest: `a = a₀ + j·t`, `v = a₀t + ½jt²`, `s = ½a₀t² + ⅙jt³`.
    static func constantJerk(
        a0: Double, jerk j: Double, duration: Double, dt: Double = 0.01
    ) -> [SmoothedSample] {
        var out: [SmoothedSample] = []
        for t in stride(from: 0.0, through: duration, by: dt) {
            let t2: Double = t * t
            let t3: Double = t2 * t
            let distance: Double = 0.5 * a0 * t2 + j * t3 / 6
            let speed: Double = a0 * t + 0.5 * j * t2
            let acceleration: Double = a0 + j * t
            out.append(sample(t: t, s: distance, v: speed, a: acceleration))
        }
        return out
    }

    static func sample(
        t: Double, s: Double, v: Double, a: Double, speedSigma: Double = 0.02
    ) -> SmoothedSample {
        var state = Matrix(rows: 3, columns: 1)
        state[0, 0] = s
        state[1, 0] = v
        state[2, 0] = 0            // zero bias, so correctedAcceleration == a
        let covariance = Matrix.diagonal([0.01, speedSigma * speedSigma, 1e-4])
        return SmoothedSample(
            t: t, dt: 0.01, acceleration: a,
            state: state, covariance: covariance,
            carriedForwardDueToSingularCovariance: false
        )
    }
}

/// Spec §9.1 — interpolated crossings. "Never report the nearest sample."
final class CrossingSolverTests: XCTestCase {
    func testSpeedCrossingIsExactForConstantAcceleration() {
        let a = 5.0
        let trace = SyntheticTrace.constantAcceleration(a, duration: 8)
        // 60 mph = 26.8224 m/s, reached at t = v/a.
        let target = 60 * PTConstants.mphToMetersPerSecond
        let crossing = CrossingSolver.speedCrossing(target: target, samples: trace)!
        XCTAssertEqual(crossing.time, target / a, accuracy: 1e-9)
        XCTAssertEqual(crossing.speed, target, accuracy: 1e-9)
    }

    func testSpeedCrossingIsExactUnderConstantJerk() {
        // The quadratic form of §9.1 is exact for constant jerk, which is the point of using
        // it rather than linear interpolation.
        let trace = SyntheticTrace.constantJerk(a0: 2.0, jerk: 0.8, duration: 8)
        let target = 20.0
        // v = 2t + 0.4t^2 = 20  ->  t = (-2 + sqrt(4 + 32)) / 0.8 = 5.0
        let crossing = CrossingSolver.speedCrossing(target: target, samples: trace)!
        XCTAssertEqual(crossing.time, 5.0, accuracy: 1e-9)
    }

    func testSpeedCrossingBeatsNearestSampleByAMeaningfulMargin() {
        // At 100 Hz the nearest sample can be up to 5 ms away; interpolation must do better,
        // because 5 ms is a sixth of the total error budget of a Dragy-class result.
        let a = 5.0
        let trace = SyntheticTrace.constantAcceleration(a, duration: 8, dt: 0.01)
        let target = 26.8224
        let exact = target / a
        let crossing = CrossingSolver.speedCrossing(target: target, samples: trace)!
        let nearest = trace.min { abs($0.speed - target) < abs($1.speed - target) }!
        XCTAssertLessThan(abs(crossing.time - exact), 1e-9)
        XCTAssertGreaterThan(abs(nearest.t - exact), 1e-4)
    }

    func testDistanceCrossingIsExactForConstantAcceleration() {
        let a = 5.0
        let trace = SyntheticTrace.constantAcceleration(a, duration: 15)
        let target = 402.336                        // quarter mile
        let expected = (2 * target / a).squareRoot()
        let crossing = CrossingSolver.distanceCrossing(target: target, samples: trace)!
        XCTAssertEqual(crossing.time, expected, accuracy: 1e-9)
        XCTAssertEqual(crossing.distance, target, accuracy: 1e-9)
    }

    func testDistanceCrossingIsExactUnderConstantJerk() {
        let trace = SyntheticTrace.constantJerk(a0: 2.0, jerk: 0.8, duration: 12)
        // s = t^2 + (0.8/6) t^3; at t = 6: 36 + 28.8 = 64.8
        let crossing = CrossingSolver.distanceCrossing(target: 64.8, samples: trace)!
        XCTAssertEqual(crossing.time, 6.0, accuracy: 1e-7)
    }

    func testDistanceCrossingWorksFromAStandstillWhereNewtonSeedWouldDivideByZero() {
        // Spec §9.1 seeds Newton at tau0 = (s_t - s[i]) / v[i]; at the launch sample v[i] = 0.
        // The rollout mark (0.3048 m) is reached in the first fraction of a second, so this
        // is not an edge case, it is the normal path.
        let a = 5.0
        let trace = SyntheticTrace.constantAcceleration(a, duration: 4, dt: 0.01)
        let crossing = CrossingSolver.distanceCrossing(target: PTConstants.foot, samples: trace)!
        XCTAssertEqual(crossing.time, (2 * PTConstants.foot / a).squareRoot(), accuracy: 1e-9)
    }

    func testSpeedTargetNeverReachedReturnsNil() {
        let trace = SyntheticTrace.constantAcceleration(5, duration: 2)   // tops out at 10 m/s
        XCTAssertNil(CrossingSolver.speedCrossing(target: 50, samples: trace))
    }

    func testDistanceTargetNeverReachedReturnsNil() {
        let trace = SyntheticTrace.constantAcceleration(5, duration: 2)   // covers 10 m
        XCTAssertNil(CrossingSolver.distanceCrossing(target: 402.336, samples: trace))
    }

    func testEmptyTraceReturnsNil() {
        XCTAssertNil(CrossingSolver.speedCrossing(target: 10, samples: []))
        XCTAssertNil(CrossingSolver.distanceCrossing(target: 10, samples: []))
    }

    func testCrossingReportsTheBracketingIndexAndOffset() {
        let trace = SyntheticTrace.constantAcceleration(5, duration: 8, dt: 0.01)
        let crossing = CrossingSolver.speedCrossing(target: 26.8224, samples: trace)!
        XCTAssertEqual(trace[crossing.index].t + crossing.tau, crossing.time, accuracy: 1e-12)
        XCTAssertLessThanOrEqual(trace[crossing.index].speed, 26.8224)
        XCTAssertGreaterThan(trace[crossing.index + 1].speed, 26.8224)
    }

    func testFirstCrossingIsUsedWhenSpeedIsNotMonotonic() {
        // A roll race can dip below the target and come back; the first crossing is the one
        // that defines the window.
        var trace = SyntheticTrace.constantAcceleration(5, duration: 6, dt: 0.01)
        // Append a decel then re-accel through the same speed.
        let last = trace.last!
        for i in 1...200 {
            let t = last.t + Double(i) * 0.01
            trace.append(SyntheticTrace.sample(t: t, s: last.distance + Double(i) * 0.1,
                                               v: 5.0, a: 0))
        }
        let crossing = CrossingSolver.speedCrossing(target: 10.0, samples: trace)!
        XCTAssertEqual(crossing.time, 2.0, accuracy: 1e-6)
    }
}

/// Spec §9.2 — standard marks.
final class MarksTests: XCTestCase {
    func testDistanceMarkValuesMatchTheSpecTable() {
        XCTAssertEqual(DistanceMark.sixtyFoot.meters, 18.288, accuracy: 1e-12)
        XCTAssertEqual(DistanceMark.threeThirtyFoot.meters, 100.584, accuracy: 1e-12)
        XCTAssertEqual(DistanceMark.eighthMile.meters, 201.168, accuracy: 1e-12)
        XCTAssertEqual(DistanceMark.thousandFoot.meters, 304.800, accuracy: 1e-12)
        XCTAssertEqual(DistanceMark.quarterMile.meters, 402.336, accuracy: 1e-12)
        XCTAssertEqual(DistanceMark.halfMile.meters, 804.672, accuracy: 1e-12)
        XCTAssertEqual(DistanceMark.mile.meters, 1609.344, accuracy: 1e-12)
        XCTAssertEqual(DistanceMark.rollout.meters, 0.3048, accuracy: 1e-12)
    }

    func testDistanceMarksAreDerivedFromTheConstantsNotHardCoded() {
        XCTAssertEqual(DistanceMark.sixtyFoot.meters, 60 * PTConstants.foot, accuracy: 1e-12)
        XCTAssertEqual(DistanceMark.quarterMile.meters, PTConstants.mile / 4, accuracy: 1e-12)
        XCTAssertEqual(DistanceMark.rollout.meters, PTConstants.foot, accuracy: 1e-12)
    }

    func testStandardSpeedMarksCoverTheImperialAndMetricSets() {
        let mph = SpeedMark.standardImperial.map(\.targetMph)
        XCTAssertEqual(mph, [30, 60, 100, 130])
        let kmh = SpeedMark.standardMetric.map(\.targetKmh)
        XCTAssertEqual(kmh, [50, 100, 160, 200])
    }

    func testSpeedMarkConvertsToMetersPerSecond() {
        XCTAssertEqual(SpeedMark.zeroToSixty.targetMetersPerSecond,
                       60 * PTConstants.mphToMetersPerSecond, accuracy: 1e-12)
    }

    func testRollWindowsMatchTheSpecList() {
        let windows = RollWindow.standard.map { [$0.fromMph, $0.toMph] }
        XCTAssertEqual(windows, [[40, 100], [60, 130], [100, 150], [100, 200]])
    }
}

/// Spec §9.3 and §9.4 — result extraction and confidence.
final class ResultExtractorTests: XCTestCase {
    private func quarterMileTrace() -> [SmoothedSample] {
        SyntheticTrace.constantAcceleration(5.0, duration: 14, dt: 0.01)
    }

    func testReportsBothRestAnchoredAndRolloutAnchoredElapsedTimes() {
        // Spec §9.3: report both. The 1-foot rollout figure is what matches a drag strip
        // timeslip and what Dragy displays.
        let trace = quarterMileTrace()
        let results = ResultExtractor.extract(from: trace, anchorTime: 0)
        let quarter = results.distanceResult(for: .quarterMile)!

        let a = 5.0
        let expectedFromRest = (2 * 402.336 / a).squareRoot()
        let rolloutTime = (2 * PTConstants.foot / a).squareRoot()
        XCTAssertEqual(quarter.elapsedFromRest, expectedFromRest, accuracy: 1e-9)
        XCTAssertEqual(quarter.elapsedFromRollout, expectedFromRest - rolloutTime, accuracy: 1e-9)
        XCTAssertLessThan(quarter.elapsedFromRollout, quarter.elapsedFromRest)
    }

    func testTrapSpeedIsUnaffectedByTheChoiceOfAnchor() {
        // Spec §9.3: "Trap speed is v at the distance mark, unaffected by which anchor."
        let trace = quarterMileTrace()
        let quarter = ResultExtractor.extract(from: trace, anchorTime: 0)
            .distanceResult(for: .quarterMile)!
        let expectedSpeed = (2 * 5.0 * 402.336).squareRoot()
        XCTAssertEqual(quarter.trapSpeed, expectedSpeed, accuracy: 1e-9)
        XCTAssertEqual(quarter.trapSpeedMph,
                       expectedSpeed / PTConstants.mphToMetersPerSecond, accuracy: 1e-9)
    }

    func testSpeedResultsUseTheRestAnchor() {
        let trace = quarterMileTrace()
        let results = ResultExtractor.extract(from: trace, anchorTime: 0)
        let zeroSixty = results.speedResult(for: .zeroToSixty)!
        XCTAssertEqual(zeroSixty.elapsed, 60 * PTConstants.mphToMetersPerSecond / 5.0,
                       accuracy: 1e-9)
    }

    func testAnchorTimeIsSubtractedFromEveryResult() {
        // The retroactive anchor of §8 may sit part-way into the recorded trace.
        let base = SyntheticTrace.constantAcceleration(5.0, duration: 14, dt: 0.01)
        let shifted = base.map { s -> SmoothedSample in
            var copy = s
            copy.t = s.t + 3.0
            return copy
        }
        let results = ResultExtractor.extract(from: shifted, anchorTime: 3.0)
        XCTAssertEqual(results.speedResult(for: .zeroToSixty)!.elapsed,
                       60 * PTConstants.mphToMetersPerSecond / 5.0, accuracy: 1e-9)
    }

    func testMarksBeyondTheRunAreAbsentRatherThanZero() {
        let short = SyntheticTrace.constantAcceleration(5.0, duration: 3, dt: 0.01)
        let results = ResultExtractor.extract(from: short, anchorTime: 0)
        XCTAssertNil(results.distanceResult(for: .quarterMile))
        XCTAssertNotNil(results.distanceResult(for: .sixtyFoot))
    }

    func testRollWindowMeasuresBetweenTwoSpeeds() {
        let trace = SyntheticTrace.constantAcceleration(4.0, duration: 25, dt: 0.01)
        let results = ResultExtractor.extract(from: trace, anchorTime: 0)
        let window = results.rollResult(for: RollWindow(fromMph: 60, toMph: 130))!
        let from = 60 * PTConstants.mphToMetersPerSecond / 4.0
        let to = 130 * PTConstants.mphToMetersPerSecond / 4.0
        XCTAssertEqual(window.elapsed, to - from, accuracy: 1e-9)
    }

    // MARK: §9.4 confidence

    func testSpeedMarkUncertaintyFollowsSigmaVOverA() {
        // Spec §9.4: sigma_t ~ sigma_v(t_cross) / a(t_cross).
        let a = 5.0
        let sigmaV = 0.04
        let trace = SyntheticTrace.constantAcceleration(a, duration: 10, dt: 0.01,
                                                        speedSigma: sigmaV)
        let result = ResultExtractor.extract(from: trace, anchorTime: 0)
            .speedResult(for: .zeroToSixty)!
        XCTAssertEqual(result.sigma, sigmaV / a, accuracy: 1e-6)
    }

    func testDistanceMarkUncertaintyFollowsSigmaSOverV() {
        // The spec only gives the speed-mark form; the distance-mark analogue is sigma_s / v.
        let a = 5.0
        let trace = SyntheticTrace.constantAcceleration(a, duration: 14, dt: 0.01)
        let quarter = ResultExtractor.extract(from: trace, anchorTime: 0)
            .distanceResult(for: .quarterMile)!
        let speedAtMark = (2 * a * 402.336).squareRoot()
        XCTAssertEqual(quarter.sigma, 0.1 / speedAtMark, accuracy: 1e-6)
    }

    func testUncertaintyIsFiniteWhenAccelerationIsNearlyZero() {
        // A roll race at steady speed has a ~ 0; sigma_v/a must not become infinite.
        let trace = (0..<500).map { i -> SmoothedSample in
            let t = Double(i) * 0.01
            return SyntheticTrace.sample(t: t, s: 30 * t, v: 30.0, a: 1e-9)
        }
        let results = ResultExtractor.extract(from: trace, anchorTime: 0)
        if let result = results.speedResult(for: SpeedMark(targetMph: 60)) {
            XCTAssertTrue(result.sigma.isFinite)
        }
    }

    func testConfidenceBadgeIsHighForACleanRun() {
        let badge = ConfidenceAssessment(
            rejectedFixCount: 0,
            meanSpeedAccuracy: 0.06,
            achievedGNSSRate: 25.0,
            calibrationAgeSeconds: 60,
            meanGradePercent: 0.2,
            usedSmoother: true
        ).badge
        XCTAssertEqual(badge, .high)
    }

    func testMoreThanTwoRejectedFixesForcesLowConfidence() {
        // Spec §6.3: "a run with more than 2 rejections should be flagged as low confidence".
        let badge = ConfidenceAssessment(
            rejectedFixCount: 3,
            meanSpeedAccuracy: 0.05,
            achievedGNSSRate: 25.0,
            calibrationAgeSeconds: 10,
            meanGradePercent: 0.0,
            usedSmoother: true
        ).badge
        XCTAssertEqual(badge, .low)
    }

    func testForwardOnlyResultsCanNeverBeHighConfidence() {
        // Spec Decision 1: results must come from the smoother. If they did not, say so.
        let badge = ConfidenceAssessment(
            rejectedFixCount: 0,
            meanSpeedAccuracy: 0.05,
            achievedGNSSRate: 25.0,
            calibrationAgeSeconds: 10,
            meanGradePercent: 0.0,
            usedSmoother: false
        ).badge
        XCTAssertNotEqual(badge, .high)
    }

    func testPoorSpeedAccuracyOrSlowRateDegradesTheBadge() {
        let poorAccuracy = ConfidenceAssessment(
            rejectedFixCount: 0, meanSpeedAccuracy: 0.8, achievedGNSSRate: 25.0,
            calibrationAgeSeconds: 10, meanGradePercent: 0, usedSmoother: true
        ).badge
        XCTAssertNotEqual(poorAccuracy, .high)

        let slowRate = ConfidenceAssessment(
            rejectedFixCount: 0, meanSpeedAccuracy: 0.05, achievedGNSSRate: 0.8,
            calibrationAgeSeconds: 10, meanGradePercent: 0, usedSmoother: true
        ).badge
        XCTAssertNotEqual(slowRate, .high)
    }

    func testSteepGradeDegradesTheBadge() {
        let badge = ConfidenceAssessment(
            rejectedFixCount: 0, meanSpeedAccuracy: 0.05, achievedGNSSRate: 25.0,
            calibrationAgeSeconds: 10, meanGradePercent: 3.0, usedSmoother: true
        ).badge
        XCTAssertNotEqual(badge, .high)
    }
}

/// Spec §7 — grade.
final class GradeEstimatorTests: XCTestCase {
    /// A 2% uphill run: elevation rises 2 m per 100 m travelled.
    private func upTrace(gradeFraction: Double, duration: Double = 14) -> ([SmoothedSample], [BaroSample]) {
        let samples = SyntheticTrace.constantAcceleration(5.0, duration: duration, dt: 0.01)
        var baro: [BaroSample] = []
        for sample in samples where Int((sample.t * 100).rounded()) % 100 == 0 {
            baro.append(BaroSample(t: sample.t,
                                   relativeAltitude: sample.distance * gradeFraction,
                                   pressure: 101.3))
        }
        return (samples, baro)
    }

    func testFitsMeanGradeFromBarometricAltitude() {
        let (samples, baro) = upTrace(gradeFraction: 0.02)
        let grade = GradeEstimator.estimate(samples: samples, barometric: baro, gnss: [])!
        XCTAssertEqual(grade.meanGradePercent, 2.0, accuracy: 0.05)
        XCTAssertEqual(grade.source, .barometric)
    }

    func testFallsBackToGNSSAltitudeWhenBaroIsUnavailable() {
        let (samples, _) = upTrace(gradeFraction: 0.02)
        var fixes: [GNSSFix] = []
        for sample in samples where Int((sample.t * 100).rounded()) % 100 == 0 {
            fixes.append(GNSSFix(t: sample.t, speed: sample.speed, speedAccuracy: 0.05,
                                 altitude: sample.distance * 0.02, verticalAccuracy: 3))
        }
        let grade = GradeEstimator.estimate(samples: samples, barometric: [], gnss: fixes)!
        XCTAssertEqual(grade.meanGradePercent, 2.0, accuracy: 0.1)
        XCTAssertEqual(grade.source, .gnss)
    }

    /// Spec §3.4: HVAC, an open window or a flexing door seal show up as false altitude.
    /// Cross-check against GNSS; disagreement over 1 m over the run means fall back and flag.
    func testBaroGnssDisagreementOverOneMetreFallsBackAndFlagsTheRun() {
        let (samples, baro) = upTrace(gradeFraction: 0.02)
        // GNSS says flat; baro says 8 m of climb. That is a cabin-pressure artefact.
        var fixes: [GNSSFix] = []
        for sample in samples where Int((sample.t * 100).rounded()) % 100 == 0 {
            fixes.append(GNSSFix(t: sample.t, speed: sample.speed, speedAccuracy: 0.05,
                                 altitude: 0, verticalAccuracy: 3))
        }
        let grade = GradeEstimator.estimate(samples: samples, barometric: baro, gnss: fixes)!
        XCTAssertTrue(grade.barometerDisagreesWithGNSS)
        XCTAssertEqual(grade.source, .gnss)
    }

    func testFlatRunReportsZeroGradeAndIsNotFlagged() {
        let (samples, baro) = upTrace(gradeFraction: 0.0)
        let grade = GradeEstimator.estimate(samples: samples, barometric: baro, gnss: [])!
        XCTAssertEqual(grade.meanGradePercent, 0, accuracy: 0.01)
        XCTAssertFalse(grade.exceedsReportingThreshold)
    }

    func testGradeAboveOnePercentIsFlaggedForDisplay() {
        // Spec §7.2: "A run with more than 1% mean grade should be visually flagged."
        let (samples, baro) = upTrace(gradeFraction: 0.015)
        let grade = GradeEstimator.estimate(samples: samples, barometric: baro, gnss: [])!
        XCTAssertTrue(grade.exceedsReportingThreshold)
    }

    func testDownhillGradeIsNegative() {
        let (samples, baro) = upTrace(gradeFraction: -0.02)
        let grade = GradeEstimator.estimate(samples: samples, barometric: baro, gnss: [])!
        XCTAssertEqual(grade.meanGradePercent, -2.0, accuracy: 0.05)
    }

    func testInsufficientElevationDataYieldsNoEstimate() {
        let samples = SyntheticTrace.constantAcceleration(5.0, duration: 5)
        XCTAssertNil(GradeEstimator.estimate(samples: samples, barometric: [], gnss: []))
    }

    /// Spec §7.2: report the raw result and a grade-corrected one, and do not silently apply
    /// the correction.
    func testCorrectedTraceIsFasterUphillAndUnchangedOnTheFlat() {
        let (samples, baro) = upTrace(gradeFraction: 0.02)
        let grade = GradeEstimator.estimate(samples: samples, barometric: baro, gnss: [])!

        let raw = ResultExtractor.extract(from: samples, anchorTime: 0)
        let corrected = ResultExtractor.extract(
            from: grade.correctedTrace(samples), anchorTime: 0
        )
        let rawQuarter = raw.distanceResult(for: .quarterMile)!.elapsedFromRest
        let correctedQuarter = corrected.distanceResult(for: .quarterMile)!.elapsedFromRest
        XCTAssertLessThan(correctedQuarter, rawQuarter,
                          "removing an uphill grade must shorten the elapsed time")

        let (flatSamples, flatBaro) = upTrace(gradeFraction: 0.0)
        let flatGrade = GradeEstimator.estimate(samples: flatSamples, barometric: flatBaro,
                                                gnss: [])!
        let flatCorrected = ResultExtractor.extract(
            from: flatGrade.correctedTrace(flatSamples), anchorTime: 0
        ).distanceResult(for: .quarterMile)!.elapsedFromRest
        let flatRaw = ResultExtractor.extract(from: flatSamples, anchorTime: 0)
            .distanceResult(for: .quarterMile)!.elapsedFromRest
        XCTAssertEqual(flatCorrected, flatRaw, accuracy: 0.005)
    }
}
