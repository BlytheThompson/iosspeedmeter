import Foundation

/// A buffer that retains only the most recent `duration` seconds of elements.
///
/// Spec §4: "Recording actually begins at ARMED, not at RECORDING. You need the pre-launch
/// data for ZUPT and for retroactive launch anchoring. Keep a 10 s ring buffer while ARMED and
/// prepend it on transition."
///
/// The window is measured from the newest timestamp seen rather than from wall-clock now, so
/// replaying a recorded session behaves identically to a live one.
public struct TimeWindowBuffer<Element>: Sendable where Element: Sendable {
    public let duration: Double
    private let timestamp: @Sendable (Element) -> Double
    private var storage: [Element] = []
    private var newestTime: Double = -.infinity

    public init(duration: Double, timestamp: @escaping @Sendable (Element) -> Double) {
        self.duration = duration
        self.timestamp = timestamp
    }

    public var count: Int { storage.count }
    public var isEmpty: Bool { storage.isEmpty }
    public var elements: [Element] { storage }

    public mutating func append(_ element: Element) {
        storage.append(element)
        newestTime = max(newestTime, timestamp(element))
        trim()
    }

    /// Return everything and empty the buffer — used when ARMED transitions to RECORDING and
    /// the pre-launch window is prepended to the recorded session.
    public mutating func drain() -> [Element] {
        defer {
            storage.removeAll(keepingCapacity: true)
            newestTime = -.infinity
        }
        return storage
    }

    public mutating func removeAll() {
        storage.removeAll(keepingCapacity: true)
        newestTime = -.infinity
    }

    private mutating func trim() {
        // The epsilon matters: timestamps accumulated as `i * 0.01` do not land exactly on
        // `newest - duration`, so a bare `<` drops or keeps the boundary sample depending on
        // rounding. Retaining a sample a nanosecond too old costs nothing; losing the oldest
        // pre-launch sample to a rounding artefact is the kind of thing that shows up later as
        // an unexplained few-millisecond shift in an anchor.
        let cutoff = newestTime - duration - 1e-9
        // Only drop from the front, and only while the element is genuinely older than the
        // window. A late-arriving out-of-order sample (spec §2.3) must not truncate anything.
        var drop = 0
        while drop < storage.count, timestamp(storage[drop]) < cutoff { drop += 1 }
        if drop > 0 { storage.removeFirst(drop) }
    }
}
