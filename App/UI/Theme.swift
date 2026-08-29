#if canImport(SwiftUI)
import SwiftUI

/// The visual system, drawn from the drag strip rather than from a generic dashboard.
///
/// Two surfaces, deliberately inverted from one another:
///
/// - **Asphalt** — the live screen. Near-black, because it is read through a windscreen in
///   direct sun with the phone mounted, and because the staging tree only reads as a staging
///   tree against dark.
/// - **Paper** — the result screen. A timeslip is a printed receipt; the result inverts to
///   light so it looks like the thing a track hands you, and so it screenshots and shares
///   legibly.
///
/// The accent colours are not decoration: amber, green and red are the literal vocabulary of
/// a staging tree, and they are used only for the states those lights mean.
public enum Theme {
    // MARK: Surfaces
    public static let asphalt = Color(red: 0.055, green: 0.067, blue: 0.075)   // #0E1113
    public static let graphite = Color(red: 0.165, green: 0.188, blue: 0.204)  // #2A3034
    public static let paper = Color(red: 0.949, green: 0.937, blue: 0.914)     // #F2EFE9
    public static let ink = Color(red: 0.086, green: 0.098, blue: 0.106)       // #16191B

    // MARK: Staging tree
    /// Pre-stage / armed.
    public static let stage = Color(red: 1.0, green: 0.690, blue: 0.125)       // #FFB020
    /// Green light — recording, or a condition met.
    public static let go = Color(red: 0.208, green: 0.816, blue: 0.498)        // #35D07F
    /// Red light — rejection, low confidence, a foul.
    public static let flag = Color(red: 0.898, green: 0.282, blue: 0.302)      // #E5484D

    /// Muted text on asphalt.
    public static let dim = Color(red: 0.541, green: 0.576, blue: 0.596)       // #8A9398

    // MARK: Type
    //
    // Numbers are monospaced throughout, for two reasons that are not aesthetic: a timeslip is
    // machine-printed, and a proportional digit set makes a 100 Hz speed readout jitter
    // horizontally as it counts.

    public static func numeric(_ size: CGFloat, weight: Font.Weight = .medium) -> Font {
        .system(size: size, weight: weight, design: .monospaced)
    }

    /// Track signage: short, uppercase, widely tracked.
    public static func label(_ size: CGFloat, weight: Font.Weight = .semibold) -> Font {
        .system(size: size, weight: weight, design: .default)
    }
}

extension View {
    /// Uppercase, widely tracked label styling used for every field name in the app.
    func trackLabel(_ size: CGFloat = 11, color: Color = Theme.dim) -> some View {
        self
            .font(Theme.label(size))
            .textCase(.uppercase)
            .tracking(1.4)
            .foregroundStyle(color)
    }
}

/// A drag-strip staging tree.
///
/// This is the signature element, and it is structural rather than decorative: each bulb is one
/// of the three ARMED preconditions from spec §4 — GNSS lock, valid calibration, and detected
/// stillness. Watching them fill tells you *why* the app will not arm, which is the single most
/// common question in a car park, and the green bulb is the launch itself.
public struct StagingTree: View {
    public var lock: Bool
    public var calibration: Bool
    public var still: Bool
    public var launched: Bool

    public init(lock: Bool, calibration: Bool, still: Bool, launched: Bool) {
        self.lock = lock
        self.calibration = calibration
        self.still = still
        self.launched = launched
    }

    /// One bulb. A named type rather than a tuple so `ForEach` has a real `Identifiable`
    /// element and no key path ever has to address a tuple member.
    private struct Condition: Identifiable {
        let id: String
        let met: Bool
    }

    private var conditions: [Condition] {
        [
            Condition(id: "Lock", met: lock),
            Condition(id: "Frame", met: calibration),
            Condition(id: "Still", met: still),
        ]
    }

    private var accessibilityDescription: String {
        if launched { return "Launched" }
        let waiting = conditions.filter { !$0.met }.map(\.id)
        return waiting.isEmpty ? "Ready" : "Waiting for " + waiting.joined(separator: ", ")
    }

    public var body: some View {
        VStack(spacing: 10) {
            ForEach(conditions) { condition in
                bulb(lit: condition.met && !launched, colour: Theme.stage)
            }
            bulb(lit: launched, colour: Theme.go)
                .padding(.top, 4)
        }
        .padding(.vertical, 14)
        .padding(.horizontal, 12)
        .background(
            RoundedRectangle(cornerRadius: 6, style: .continuous)
                .fill(Color.black.opacity(0.35))
                .overlay(
                    RoundedRectangle(cornerRadius: 6, style: .continuous)
                        .strokeBorder(Theme.graphite, lineWidth: 1)
                )
        )
        .accessibilityElement(children: .ignore)
        .accessibilityLabel("Staging")
        .accessibilityValue(accessibilityDescription)
    }

    private func bulb(lit: Bool, colour: Color) -> some View {
        Circle()
            .fill(lit ? colour : Theme.graphite)
            .frame(width: 18, height: 18)
            .overlay(Circle().strokeBorder(Color.black.opacity(0.4), lineWidth: 1))
            // A lit bulb glows; an unlit one is inert. Respecting reduced motion is handled by
            // there being no animation here at all — the state change is the signal.
            .shadow(color: lit ? colour.opacity(0.7) : .clear, radius: lit ? 8 : 0)
    }
}
#endif
