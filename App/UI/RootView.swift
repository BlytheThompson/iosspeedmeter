#if canImport(SwiftUI)
import SwiftUI
import PerformanceTimerCore

@main
struct PerformanceTimerApp: App {
    @StateObject private var controller = SessionController()

    var body: some Scene {
        WindowGroup {
            RootView(controller: controller)
                .preferredColorScheme(.dark)
        }
    }
}

public struct RootView: View {
    @ObservedObject var controller: SessionController
    @State private var selection = Tab.timer

    enum Tab: Hashable { case timer, sessions, settings }

    public init(controller: SessionController) {
        self.controller = controller
    }

    public var body: some View {
        TabView(selection: $selection) {
            timerTab
                .tabItem { Label("Timer", systemImage: "stopwatch") }
                .tag(Tab.timer)

            SessionListView(controller: controller)
                .tabItem { Label("Sessions", systemImage: "list.bullet.rectangle") }
                .tag(Tab.sessions)

            SettingsView(controller: controller)
                .tabItem { Label("Settings", systemImage: "slider.horizontal.3") }
                .tag(Tab.settings)
        }
        .tint(Theme.stage)
    }

    @ViewBuilder
    private var timerTab: some View {
        if let analysis = controller.analysis {
            TimeslipView(analysis: analysis,
                         useMetricUnits: controller.settings.useMetricUnits)
                .overlay(alignment: .bottom) {
                    Button("Run again") { controller.rearm() }
                        .buttonStyle(StripButtonStyle(tint: Theme.ink))
                        .padding(20)
                }
        } else if controller.readout.state == .idle && !controller.hasStartedSession {
            startScreen
        } else {
            LiveView(controller: controller)
        }
    }

    /// Before a session begins there is nothing to measure, so the screen explains the one
    /// thing that matters: stop the car, then arm.
    private var startScreen: some View {
        ZStack {
            Theme.asphalt.ignoresSafeArea()
            VStack(spacing: 22) {
                Spacer()
                Text("Performance Timer")
                    .font(Theme.label(24, weight: .heavy))
                    .textCase(.uppercase)
                    .tracking(3)
                    .foregroundStyle(Theme.paper)
                Text("Park, hold still, and arm. The app records from the moment it arms so "
                     + "the launch can be placed exactly, then smooths the whole run before "
                     + "reporting anything.")
                    .font(Theme.label(14, weight: .regular))
                    .foregroundStyle(Theme.dim)
                    .multilineTextAlignment(.center)
                    .padding(.horizontal, 32)
                Spacer()
                Button("Arm") { controller.startSession() }
                    .buttonStyle(StripButtonStyle(tint: Theme.stage))
                    .padding(.horizontal, 24)
                Text(controller.settings.gnssSource.title + " · "
                     + controller.settings.gnssSource.expectedUncertainty + " expected")
                    .trackLabel(10)
                    .padding(.bottom, 28)
            }
        }
    }
}

/// Spec §10 — the saved sessions, and re-analysis on device.
public struct SessionListView: View {
    @ObservedObject var controller: SessionController

    public init(controller: SessionController) { self.controller = controller }

    public var body: some View {
        NavigationStack {
            List {
                if controller.savedSessions.isEmpty {
                    Text("Recorded sessions appear here. Every run is saved, including ones "
                         + "that produced no result — those are the ones worth reading.")
                        .font(Theme.label(13, weight: .regular))
                        .foregroundStyle(.secondary)
                }
                ForEach(controller.savedSessions, id: \.header.sessionID) { session in
                    Button {
                        controller.reanalyse(session)
                    } label: {
                        VStack(alignment: .leading, spacing: 4) {
                            Text(Date(timeIntervalSince1970: session.header.startedAtUnixTime),
                                 format: .dateTime.day().month().hour().minute())
                                .font(Theme.label(15, weight: .semibold))
                            Text("\(session.rows.count) samples · \(session.header.deviceModel)")
                                .trackLabel(10)
                        }
                    }
                    .swipeActions {
                        Button("Delete", role: .destructive) { controller.delete(session) }
                    }
                }
            }
            .navigationTitle("Sessions")
            .refreshable { controller.refreshSessions() }
        }
    }
}

public struct SettingsView: View {
    @ObservedObject var controller: SessionController

    public init(controller: SessionController) { self.controller = controller }

    public var body: some View {
        NavigationStack {
            Form {
                Section("GNSS source") {
                    Picker("Source", selection: $controller.settings.gnssSource) {
                        ForEach(AppSettings.GNSSSource.allCases) { source in
                            Text(source.title).tag(source)
                        }
                    }
                    Text("Expected 0–60 uncertainty: "
                         + controller.settings.gnssSource.expectedUncertainty)
                        .font(.caption).foregroundStyle(.secondary)

                    if controller.settings.gnssSource == .externalWiFi {
                        TextField("Host", text: $controller.settings.externalHost)
                        Text("Join the receiver's Wi-Fi network first. The app pins the socket "
                             + "to Wi-Fi so iOS cannot route it over cellular.")
                            .font(.caption).foregroundStyle(.secondary)
                    }
                }

                Section("Run") {
                    Picker("Target distance",
                           selection: $controller.settings.targetDistanceMeters) {
                        ForEach(DistanceMark.standard, id: \.name) { mark in
                            Text(mark.name).tag(mark.meters)
                        }
                    }
                    Toggle("Metric units", isOn: $controller.settings.useMetricUnits)
                    Toggle("CAN wheel speed", isOn: $controller.settings.wheelSpeedEnabled)
                }

                Section {
                    Picker("Activity type", selection: $controller.settings.activityModeRaw) {
                        Text("Other navigation").tag("otherNavigation")
                        Text("Other").tag("other")
                        Text("Automotive navigation").tag("automotiveNavigation")
                    }
                    if controller.settings.activityModeRaw == "automotiveNavigation" {
                        Label("Core Location may snap positions to roads, which corrupts "
                              + "distance marks on a strip or an unmapped surface. Compare "
                              + "traces before trusting it.",
                              systemImage: "exclamationmark.triangle.fill")
                            .font(.caption)
                            .foregroundStyle(Theme.stage)
                    }
                } header: {
                    Text("Core Location")
                } footer: {
                    Text("Change this only to compare recorded traces.")
                }

                Section {
                    stepper("σa (accel noise)", value: $controller.settings.sigmaA,
                            range: 0.005...0.5, step: 0.005, format: "%.3f")
                    stepper("σb (bias walk)", value: $controller.settings.sigmaB,
                            range: 0.0002...0.02, step: 0.0002, format: "%.4f")
                    stepper("σ GNSS floor", value: $controller.settings.sigmaGPSFloor,
                            range: 0.01...0.5, step: 0.01, format: "%.2f")
                    stepper("σ ZUPT", value: $controller.settings.sigmaZUPT,
                            range: 0.002...0.1, step: 0.002, format: "%.3f")
                    Toggle("Use the spec's published Q table",
                           isOn: $controller.settings.useSpecLiteralProcessNoise)
                } header: {
                    Text("Filter tuning")
                } footer: {
                    Text("The published Q table in §6.2 is dimensionally inconsistent and makes "
                         + "the filter about 30,000× too confident in its own dead reckoning at "
                         + "100 Hz. Kept only so the two can be compared on recorded data.")
                }

                Section("Vehicle frame") {
                    if let calibration = controller.settings.storedCalibration {
                        LabeledContent("Solved from",
                                       value: String(format: "%.1f m/s² launch",
                                                     calibration.launchAccelerationMagnitude))
                        LabeledContent("Averaged over",
                                       value: "\(calibration.sourceEventCount) run(s)")
                        Button("Clear calibration", role: .destructive) {
                            controller.settings.clearCalibration()
                        }
                    } else {
                        Text("Not yet solved. It is calculated automatically from your first "
                             + "straight-line launch on level ground.")
                            .font(.caption).foregroundStyle(.secondary)
                    }
                }
            }
            .navigationTitle("Settings")
            .onChange(of: controller.settings) { _, newValue in newValue.save() }
        }
    }

    private func stepper(
        _ title: String, value: Binding<Double>,
        range: ClosedRange<Double>, step: Double, format: String
    ) -> some View {
        Stepper(value: value, in: range, step: step) {
            LabeledContent(title, value: String(format: format, value.wrappedValue))
        }
    }
}
#endif
