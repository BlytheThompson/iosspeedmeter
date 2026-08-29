#if canImport(SwiftUI)
import SwiftUI
import PerformanceTimerCore

/// The result screen, set as a drag-strip timeslip.
///
/// Inverted to paper because that is what a timeslip is, and because the screen exists to be
/// screenshotted and shared.
///
/// The one deliberate typographic decision here: **the uncertainty is set at the same weight
/// and nearly the same size as the number it qualifies.** Spec §9.4 is unambiguous — "A 0–60 of
/// 4.31 ± 0.04 s is a scientifically honest number; 4.31 s alone is not" — and setting the ±
/// small, grey and optional would quietly undo the entire point of post-processing. If the
/// figure is uncertain, the reader sees that at the same moment they see the figure.
public struct TimeslipView: View {
    let analysis: SessionAnalysis
    var useMetricUnits: Bool

    public init(analysis: SessionAnalysis, useMetricUnits: Bool = false) {
        self.analysis = analysis
        self.useMetricUnits = useMetricUnits
    }

    public var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 0) {
                masthead
                if let results = analysis.results {
                    speedSection(results)
                    distanceSection(results)
                    if !results.rollResults.isEmpty { rollSection(results) }
                    if let corrected = analysis.gradeCorrectedResults {
                        gradeSection(corrected)
                    }
                } else {
                    noResult
                }
                provenance
            }
            .padding(22)
            // Clearance for the "Run again" control overlaid by RootView — without it the
            // button sits on top of the last section.
            .padding(.bottom, 76)
        }
        .background(Theme.paper.ignoresSafeArea())
        .foregroundStyle(Theme.ink)
    }

    // MARK: Masthead

    private var masthead: some View {
        VStack(alignment: .leading, spacing: 4) {
            Text("Timeslip")
                .font(Theme.label(26, weight: .heavy))
                .textCase(.uppercase)
                .tracking(4)
            HStack(spacing: 8) {
                badge
                if let grade = analysis.grade, grade.exceedsReportingThreshold {
                    Text(String(format: "%+.1f%% grade", grade.meanGradePercent))
                        .trackLabel(10, color: Theme.ink.opacity(0.7))
                        .padding(.horizontal, 7).padding(.vertical, 3)
                        .background(Theme.stage.opacity(0.28))
                }
            }
            ForEach(analysis.confidence.caveats, id: \.self) { caveat in
                Text("· \(caveat)")
                    .font(Theme.label(11, weight: .regular))
                    .foregroundStyle(Theme.ink.opacity(0.6))
            }
        }
        .padding(.bottom, 18)
    }

    private var badge: some View {
        let colour: Color = {
            switch analysis.confidence.badge {
            case .high: return Theme.go
            case .medium: return Theme.stage
            case .low: return Theme.flag
            }
        }()
        return Text(analysis.confidence.badge.rawValue + " confidence")
            .trackLabel(10, color: Theme.ink)
            .padding(.horizontal, 7).padding(.vertical, 3)
            .background(colour.opacity(0.28))
    }

    // MARK: Sections

    private func speedSection(_ results: RunResults) -> some View {
        section("From a standstill") {
            ForEach(results.speedResults) { result in
                row(name: result.mark.name) {
                    timeWithUncertainty(result.elapsed, result.sigma)
                }
            }
        }
    }

    /// Distance marks, one two-line row each rather than a four-column table.
    ///
    /// A fixed-width table fits on a 393 pt phone only by leaving the name column 83 pt and
    /// dropping the uncertainty entirely — which would be the one number spec §9.4 insists must
    /// never be dropped. Promoting the rollout ET with its ±, and demoting from-rest and trap to
    /// a secondary line, keeps every figure and reads far better at a glance.
    private func distanceSection(_ results: RunResults) -> some View {
        section("Distance marks") {
            Text("The large figure is the 1-foot rollout ET — what a drag strip prints and "
                 + "what Dragy shows.")
                .font(Theme.label(11, weight: .regular))
                .foregroundStyle(Theme.ink.opacity(0.55))
                .padding(.bottom, 6)

            ForEach(results.distanceResults) { result in
                VStack(spacing: 2) {
                    HStack(alignment: .firstTextBaseline) {
                        Text(result.mark.name)
                            .font(Theme.label(14, weight: .semibold))
                        Spacer()
                        timeWithUncertainty(result.elapsedFromRollout, result.sigma)
                    }
                    HStack(spacing: 0) {
                        Spacer()
                        Text("from rest ")
                        Text(String(format: "%.3f", result.elapsedFromRest)).monospacedDigit()
                        Text(" s · trap ")
                        Text(trapText(result)).monospacedDigit()
                        Text(useMetricUnits ? " km/h" : " mph")
                    }
                    .font(Theme.label(11, weight: .regular))
                    .foregroundStyle(Theme.ink.opacity(0.55))
                }
                .padding(.vertical, 7)
                .accessibilityElement(children: .ignore)
                .accessibilityLabel(result.mark.name)
                .accessibilityValue(String(
                    format: "%.3f seconds with rollout, plus or minus %.3f. From rest %.3f "
                        + "seconds. Trap speed %.1f.",
                    result.elapsedFromRollout, result.sigma, result.elapsedFromRest,
                    useMetricUnits ? result.trapSpeedKmh : result.trapSpeedMph))
                Divider().overlay(Theme.ink.opacity(0.12))
            }
        }
    }

    private func trapText(_ result: DistanceResult) -> String {
        String(format: "%.1f", useMetricUnits ? result.trapSpeedKmh : result.trapSpeedMph)
    }

    private func rollSection(_ results: RunResults) -> some View {
        section("Rolling starts") {
            ForEach(results.rollResults) { result in
                row(name: result.window.name) {
                    timeWithUncertainty(result.elapsed, result.sigma)
                }
            }
            Text("No stationary anchor and no ZUPT — wider uncertainty than a standing start.")
                .font(Theme.label(11, weight: .regular))
                .foregroundStyle(Theme.ink.opacity(0.55))
                .padding(.top, 6)
        }
    }

    private func gradeSection(_ corrected: RunResults) -> some View {
        section("Grade-corrected") {
            ForEach(corrected.distanceResults) { result in
                row(name: result.mark.name) {
                    Text(String(format: "%.3f s", result.elapsedFromRest))
                        .font(Theme.numeric(17)).monospacedDigit()
                }
            }
            Text("An approximation, not a transform. Shown beside the raw result, never instead of it.")
                .font(Theme.label(11, weight: .regular))
                .foregroundStyle(Theme.ink.opacity(0.55))
                .padding(.top, 6)
        }
    }

    private var noResult: some View {
        VStack(alignment: .leading, spacing: 8) {
            Text("No launch detected")
                .font(Theme.label(17, weight: .bold))
            Text("The full sensor trace was still recorded and saved. Open it from Sessions to "
                 + "re-analyse, or check that the car was stationary before launching.")
                .font(Theme.label(13, weight: .regular))
                .foregroundStyle(Theme.ink.opacity(0.7))
        }
        .padding(.bottom, 18)
    }

    // MARK: Provenance

    private var provenance: some View {
        section("How this was measured") {
            provenanceRow("GNSS fixes",
                          "\(analysis.diagnostics.acceptedFixCount) used, "
                          + "\(analysis.diagnostics.rejectedFixCount) rejected")
            provenanceRow("GNSS rate",
                          String(format: "%.2f Hz", analysis.diagnostics.achievedGNSSRate))
            provenanceRow("ZUPT samples", "\(analysis.diagnostics.zuptSampleCount)")
            provenanceRow("Vehicle frame",
                          analysis.diagnostics.usedProvisionalVehicleFrame
                            ? "Provisional (gravity only)" : "Solved from launch")
            if let grade = analysis.grade {
                provenanceRow("Grade",
                              String(format: "%+.2f%% (%@)", grade.meanGradePercent,
                                     grade.source.rawValue))
            }
            provenanceRow("Method", "Forward Kalman + RTS smoother, post-processed")
        }
    }

    private func provenanceRow(_ name: String, _ value: String) -> some View {
        HStack {
            Text(name).trackLabel(10, color: Theme.ink.opacity(0.55))
            Spacer()
            Text(value).font(Theme.numeric(12)).foregroundStyle(Theme.ink.opacity(0.8))
        }
        .padding(.vertical, 3)
    }

    // MARK: Building blocks

    private func section<Content: View>(
        _ title: String, @ViewBuilder content: () -> Content
    ) -> some View {
        VStack(alignment: .leading, spacing: 6) {
            Text(title).trackLabel(10, color: Theme.ink.opacity(0.5))
                .padding(.bottom, 2)
            content()
        }
        .padding(.bottom, 24)
    }

    private func row<Content: View>(
        name: String, @ViewBuilder value: () -> Content
    ) -> some View {
        VStack(spacing: 0) {
            HStack {
                Text(name)
                    .font(Theme.label(14, weight: .semibold))
                Spacer()
                value()
            }
            .padding(.vertical, 6)
            Divider().overlay(Theme.ink.opacity(0.12))
        }
    }

    /// The honesty unit: figure and uncertainty as one typographic object.
    private func timeWithUncertainty(_ value: Double, _ sigma: Double) -> some View {
        HStack(alignment: .firstTextBaseline, spacing: 6) {
            Text(String(format: "%.3f", value))
                .font(Theme.numeric(24, weight: .semibold))
                .monospacedDigit()
            Text(String(format: "± %.3f", sigma))
                .font(Theme.numeric(15, weight: .medium))
                .monospacedDigit()
                .foregroundStyle(Theme.ink.opacity(0.65))
            Text("s").trackLabel(11, color: Theme.ink.opacity(0.5))
        }
        .accessibilityElement(children: .ignore)
        .accessibilityLabel(String(format: "%.3f seconds, plus or minus %.3f", value, sigma))
    }
}
#endif
