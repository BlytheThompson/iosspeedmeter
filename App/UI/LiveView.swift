#if canImport(SwiftUI)
import SwiftUI
import PerformanceTimerCore

/// The asphalt screen — what you see with the phone mounted and the car staged.
///
/// It shows exactly three things, in descending size: current speed, progress toward the
/// target distance, and why the app is or is not armed. Everything else is telemetry in a
/// single quiet row at the bottom.
///
/// Note what it deliberately does **not** show: an elapsed time. Spec Decision 1 says results
/// never come from the live path, so putting a running clock here would invite the user to read
/// a number that is not the answer. The time appears once, on the timeslip, after the smoother
/// has run.
public struct LiveView: View {
    @ObservedObject var controller: SessionController

    public init(controller: SessionController) {
        self.controller = controller
    }

    private var readout: LiveEstimator.Readout { controller.readout }

    private var speedText: String {
        let value = controller.settings.useMetricUnits ? readout.speedKmh : readout.speedMph
        return String(format: "%.1f", max(0, value))
    }

    private var speedUnit: String { controller.settings.useMetricUnits ? "km/h" : "mph" }

    private var stateColour: Color {
        switch readout.state {
        case .idle: return Theme.dim
        case .armed: return Theme.stage
        case .recording: return Theme.go
        case .complete, .analysing: return Theme.stage
        case .result: return Theme.go
        }
    }

    private var stateTitle: String {
        switch readout.state {
        case .idle: return "Not armed"
        case .armed: return "Armed"
        case .recording: return "Recording"
        case .complete: return "Run complete"
        case .analysing: return "Smoothing"
        case .result: return "Result ready"
        }
    }

    private var progress: Double {
        let target = controller.settings.targetDistanceMeters
        guard target > 0 else { return 0 }
        return min(1, max(0, readout.distance / target))
    }

    public var body: some View {
        ZStack {
            Theme.asphalt.ignoresSafeArea()

            VStack(spacing: 0) {
                statusStrip
                Spacer(minLength: 0)
                speedBlock
                Spacer(minLength: 0)
                distanceBlock
                telemetryRow
                controls
            }
            .padding(.horizontal, 20)
        }
        .foregroundStyle(Theme.paper)
    }

    // MARK: Status

    private var statusStrip: some View {
        HStack(alignment: .top) {
            VStack(alignment: .leading, spacing: 6) {
                Text(stateTitle)
                    .font(Theme.label(15, weight: .bold))
                    .textCase(.uppercase)
                    .tracking(1.8)
                    .foregroundStyle(stateColour)
                if let reason = readout.stopReason, readout.state != .recording {
                    Text(describe(reason)).trackLabel(11)
                }
                if let error = controller.lastError {
                    Text(error).trackLabel(11, color: Theme.flag)
                }
                if let external = controller.externalGNSSState {
                    Text(external).trackLabel(11)
                }
            }
            Spacer()
            StagingTree(
                lock: controller.armingConditions.lock,
                calibration: controller.armingConditions.calibration,
                still: controller.armingConditions.still,
                launched: readout.state == .recording
            )
        }
        .padding(.top, 12)
    }

    private func describe(_ reason: SessionStateMachine.StopReason) -> String {
        switch reason {
        case .targetDistanceReached: return "Target distance reached"
        case .vehicleStopped: return "Vehicle stopped"
        case .durationLimit: return "Time limit reached"
        case .userStopped: return "Stopped by you"
        }
    }

    // MARK: Speed

    private var peakText: String {
        let value = controller.settings.useMetricUnits
            ? readout.peakSpeedKmh : readout.peakSpeedMph
        return String(format: "%.1f", max(0, value))
    }

    /// The speed readout is the one thing that has to be legible at arm's length through a
    /// windscreen, so it is sized to actually occupy the space rather than float in it. The
    /// bottom padding shifts it above the true centre: optically centred text sits high, and
    /// it leaves the lower half to the progress and telemetry cluster.
    private var speedBlock: some View {
        VStack(spacing: -6) {
            Text(speedText)
                .font(Theme.numeric(148, weight: .semibold))
                .monospacedDigit()
                .minimumScaleFactor(0.4)
                .lineLimit(1)
            Text(speedUnit).trackLabel(13)

            if readout.peakSpeed > 0.5 {
                HStack(spacing: 6) {
                    Text("Peak").trackLabel(9)
                    Text(peakText)
                        .font(Theme.numeric(15))
                        .monospacedDigit()
                        .foregroundStyle(Theme.dim)
                }
                .padding(.top, 14)
            }
        }
        .padding(.bottom, 70)
        .accessibilityElement(children: .combine)
        .accessibilityLabel("Speed")
        .accessibilityValue("\(speedText) \(speedUnit), peak \(peakText)")
    }

    // MARK: Distance

    private var distanceBlock: some View {
        VStack(alignment: .leading, spacing: 8) {
            GeometryReader { geometry in
                ZStack(alignment: .leading) {
                    Rectangle().fill(Theme.graphite)
                    Rectangle()
                        .fill(readout.state == .recording ? Theme.go : Theme.stage)
                        .frame(width: geometry.size.width * progress)
                }
            }
            .frame(height: 4)

            HStack {
                Text("\(Int(readout.distance)) m").font(Theme.numeric(13)).monospacedDigit()
                Spacer()
                Text("\(Int(controller.settings.targetDistanceMeters)) m target")
                    .trackLabel(11)
            }
        }
        .padding(.bottom, 18)
    }

    // MARK: Telemetry

    private var telemetryRow: some View {
        HStack(spacing: 18) {
            telemetry("GNSS", readout.hasGNSSLock ? "lock" : "none",
                      readout.hasGNSSLock ? Theme.go : Theme.flag)
            telemetry("ZUPT", readout.isStationary ? "on" : "off",
                      readout.isStationary ? Theme.go : Theme.dim)
            telemetry("Bias", String(format: "%+.3f", readout.accelerometerBias), Theme.paper)
            telemetry("Accel", String(format: "%+.2f", readout.longitudinalAcceleration),
                      Theme.paper)
        }
        .padding(.vertical, 14)
        .overlay(alignment: .top) { Rectangle().fill(Theme.graphite).frame(height: 1) }
    }

    private func telemetry(_ name: String, _ value: String, _ colour: Color) -> some View {
        VStack(alignment: .leading, spacing: 3) {
            Text(name).trackLabel(9)
            Text(value)
                .font(Theme.numeric(13))
                .monospacedDigit()
                .foregroundStyle(colour)
        }
        .frame(maxWidth: .infinity, alignment: .leading)
    }

    // MARK: Controls

    @ViewBuilder
    private var controls: some View {
        HStack(spacing: 12) {
            switch readout.state {
            case .idle, .armed:
                Button("Stop session") { controller.stopSession() }
                    .buttonStyle(StripButtonStyle(tint: Theme.graphite))
            case .recording:
                Button("Stop run") { controller.requestStop() }
                    .buttonStyle(StripButtonStyle(tint: Theme.flag))
            case .complete, .analysing:
                ProgressView().tint(Theme.stage)
            case .result:
                Button("Run again") { controller.rearm() }
                    .buttonStyle(StripButtonStyle(tint: Theme.go))
            }
        }
        .padding(.bottom, 20)
    }
}

struct StripButtonStyle: ButtonStyle {
    var tint: Color

    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .font(Theme.label(13, weight: .bold))
            .textCase(.uppercase)
            .tracking(1.4)
            .foregroundStyle(Theme.paper)
            .frame(maxWidth: .infinity)
            .padding(.vertical, 15)
            .background(
                RoundedRectangle(cornerRadius: 4, style: .continuous)
                    .fill(tint.opacity(configuration.isPressed ? 0.7 : 1))
            )
    }
}
#endif
