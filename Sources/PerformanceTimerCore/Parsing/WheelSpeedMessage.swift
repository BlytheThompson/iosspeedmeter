import Foundation

/// Spec §3.7 — CAN wheel speed carried over the existing GNSS link.
///
/// §3.7: "The cleanest route is to put a CAN transceiver on the same ESP32 that carries the
/// GNSS, and ship wheel speed over the existing link. This avoids the External Accessory
/// framework and MFi entirely."
///
/// Rather than inventing a second wire format, wheel speed is sent as a **UBX-framed private
/// message**. UBX reserves classes `0xF0`–`0xFF` for non-u-blox use, so the receiver's existing
/// framer, Fletcher checksum and — most valuably — the §2.4 `iTOW` clock alignment all apply
/// unchanged. A wheel-speed sample therefore lands on the session clock with the same accuracy
/// as a position fix, instead of being timestamped on arrival with Wi-Fi jitter in it.
///
/// Frame: class `0xF1`, ID `0x01`, payload length 8.
///
/// | Offset | Type | Field | Unit |
/// |---|---|---|---|
/// | 0 | U4 | `iTOW` | ms, GPS time of week — the same timebase as `UBX-NAV-PVT` |
/// | 4 | I4 | `speed` | mm/s |
///
/// If you use an OBDLink adapter instead of a CAN transceiver, note §3.7's warning: on iOS
/// only the **MX+** works — the LX, MX and CX will not — and adapter throughput is shared
/// across logged channels, so log wheel speed alone if timing is the goal.
public struct WheelSpeedMessage: Equatable, Sendable {
    public static let messageClass: UInt8 = 0xF1
    public static let messageID: UInt8 = 0x01
    public static let payloadLength = 8

    /// GPS time of week, ms — feed through the §2.4 `ClockFit` exactly like a NAV-PVT `iTOW`.
    public var iTOW: UInt32
    /// Wheel speed, m/s.
    public var speed: Double

    public init(iTOW: UInt32, speed: Double) {
        self.iTOW = iTOW
        self.speed = speed
    }

    /// Decode the 8-byte payload. Returns `nil` for any other length, so a firmware revision
    /// that changes the layout surfaces as an unknown message rather than as plausible noise.
    public static func decode(_ payload: [UInt8]) -> WheelSpeedMessage? {
        guard payload.count == payloadLength else { return nil }
        func u4(_ o: Int) -> UInt32 {
            UInt32(payload[o]) | (UInt32(payload[o + 1]) << 8)
                | (UInt32(payload[o + 2]) << 16) | (UInt32(payload[o + 3]) << 24)
        }
        return WheelSpeedMessage(
            iTOW: u4(0),
            speed: Double(Int32(bitPattern: u4(4))) / 1000.0
        )
    }

    /// Build a complete frame. Used by the tests, and by anyone writing the ESP32 firmware.
    public static func makeFrame(iTOW: UInt32, speedMillimetresPerSecond: Int32) -> [UInt8] {
        var payload = [UInt8]()
        for shift in stride(from: 0, through: 24, by: 8) {
            payload.append(UInt8((iTOW >> UInt32(shift)) & 0xFF))
        }
        let raw = UInt32(bitPattern: speedMillimetresPerSecond)
        for shift in stride(from: 0, through: 24, by: 8) {
            payload.append(UInt8((raw >> UInt32(shift)) & 0xFF))
        }
        return UBXParser.makeFrame(messageClass: messageClass, messageID: messageID,
                                   payload: payload)
    }

    public func sample(sessionTime t: Double) -> WheelSpeedSample {
        WheelSpeedSample(t: t, speed: speed)
    }
}
