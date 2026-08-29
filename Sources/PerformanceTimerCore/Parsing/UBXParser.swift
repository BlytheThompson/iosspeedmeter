import Foundation

/// A decoded UBX message.
public enum UBXMessage: Equatable, Sendable {
    case navPVT(UBXNavPVT)
    /// Any other class/ID, or a `NAV-PVT` whose payload length is not the expected 92 bytes.
    case other(messageClass: UInt8, messageID: UInt8, payload: [UInt8])
}

/// `UBX-NAV-PVT` (class `0x01`, ID `0x07`, payload length 92) — spec §3.5.
///
/// Preferred over NMEA because `sAcc` gives a real per-fix speed sigma, which is what the
/// filter needs for `R`. NMEA `RMC` gives knots and no accuracy estimate at all.
public struct UBXNavPVT: Equatable, Codable, Sendable {
    public enum CarrierSolution: Int, Equatable, Codable, Sendable {
        case none = 0, float = 1, fixed = 2, reserved = 3
    }

    public var iTOW: UInt32
    public var year: UInt16
    public var month: UInt8
    public var day: UInt8
    public var hour: UInt8
    public var minute: UInt8
    public var second: UInt8
    public var validFlags: UInt8
    /// Time accuracy estimate, ns.
    public var tAcc: UInt32
    /// Signed fraction-of-second, ns.
    public var nano: Int32
    public var fixType: UInt8
    public var flags: UInt8
    public var flags2: UInt8
    public var numSV: UInt8

    public var longitudeDegrees: Double
    public var latitudeDegrees: Double
    /// Height above the ellipsoid, m.
    public var heightAboveEllipsoid: Double
    /// Height above mean sea level, m.
    public var heightAboveMSL: Double
    /// Horizontal accuracy, m.
    public var horizontalAccuracy: Double
    /// Vertical accuracy, m.
    public var verticalAccuracy: Double
    /// North/East/Down velocity, m/s.
    public var velocityNED: Vector3
    /// 2D ground speed, m/s — the primary measurement.
    public var groundSpeed: Double
    public var headingOfMotionDegrees: Double
    /// Speed accuracy, m/s — this is `R` for the GNSS update.
    public var speedAccuracy: Double
    public var headingAccuracyDegrees: Double
    public var pDOP: Double

    /// Bit 0 of `flags`.
    public var gnssFixOK: Bool { flags & 0x01 != 0 }
    /// Bits 6–7 of `flags`.
    public var carrierSolution: CarrierSolution {
        CarrierSolution(rawValue: Int((flags >> 6) & 0x03)) ?? .none
    }

    /// GPS time of week in seconds, including the signed nanosecond fraction.
    /// Spec §3.5: "Precise fix epoch = GPS-week-start + iTOW/1000 + nano/1e9."
    public var timeOfWeekSeconds: Double {
        Double(iTOW) / 1000.0 + Double(nano) / 1e9
    }

    /// Convert to the estimator's fix type. `t` must already be on the session clock — map it
    /// with the §2.4 `ClockFit`, never with the packet's arrival time.
    public func gnssFix(sessionTime t: Double) -> GNSSFix {
        GNSSFix(
            t: t,
            speed: groundSpeed,
            speedAccuracy: speedAccuracy,
            course: headingOfMotionDegrees,
            courseAccuracy: headingAccuracyDegrees,
            latitudeDegrees: latitudeDegrees,
            longitudeDegrees: longitudeDegrees,
            horizontalAccuracy: horizontalAccuracy,
            altitude: heightAboveMSL,
            verticalAccuracy: verticalAccuracy,
            fixType: Int(fixType),
            numSV: Int(numSV),
            gnssFixOK: gnssFixOK,
            source: .externalUBX
        )
    }
}

/// Streaming UBX frame decoder.
///
/// A TCP socket delivers whatever the network feels like: partial frames, several frames in
/// one read, or noise from a receiver that was mid-sentence when the connection opened. The
/// parser therefore buffers, resynchronises on the `B5 62` preamble, and never assumes a
/// frame arrives whole.
public struct UBXParser: Sendable {
    public static let sync1: UInt8 = 0xB5
    public static let sync2: UInt8 = 0x62
    public static let navClass: UInt8 = 0x01
    public static let pvtID: UInt8 = 0x07
    public static let pvtPayloadLength = 92

    /// Largest payload we will wait for. Real UBX payloads are well under this; a larger
    /// declared length means the "length" field was actually noise, so the frame is abandoned
    /// rather than stalling the parser until 64 KB of unrelated bytes arrive.
    public static let maximumPayloadLength = 2048

    private var buffer: [UInt8] = []

    /// Frames whose checksum did not verify. Worth surfacing: a nonzero count on a Wi-Fi link
    /// points at the link, not the receiver.
    public private(set) var checksumFailureCount = 0
    /// Bytes discarded while resynchronising.
    public private(set) var discardedByteCount = 0

    public init() {}

    public var bufferedByteCount: Int { buffer.count }

    /// 8-bit Fletcher checksum over class, ID, length and payload (spec §3.5).
    public static func fletcherChecksum(_ bytes: [UInt8]) -> (UInt8, UInt8) {
        var a: UInt8 = 0
        var b: UInt8 = 0
        for byte in bytes {
            a = a &+ byte
            b = b &+ a
        }
        return (a, b)
    }

    /// Build a complete frame, used by tests and by any command we send to the receiver.
    public static func makeFrame(messageClass: UInt8, messageID: UInt8, payload: [UInt8]) -> [UInt8] {
        let length = UInt16(payload.count)
        var body: [UInt8] = [messageClass, messageID, UInt8(length & 0xFF), UInt8(length >> 8)]
        body += payload
        let (ckA, ckB) = fletcherChecksum(body)
        return [sync1, sync2] + body + [ckA, ckB]
    }

    /// Feed received bytes; returns every complete message they completed.
    public mutating func consume(_ bytes: [UInt8]) -> [UBXMessage] {
        buffer += bytes
        var messages: [UBXMessage] = []

        while true {
            // Find the preamble.
            guard let start = indexOfPreamble() else {
                // No preamble at all. Keep at most one trailing byte, in case it is a `B5`
                // whose `62` has not arrived yet. This is what bounds the buffer on noise.
                if buffer.count > 1 {
                    discardedByteCount += buffer.count - 1
                    buffer = [buffer[buffer.count - 1]]
                }
                break
            }
            if start > 0 {
                discardedByteCount += start
                buffer.removeFirst(start)
            }

            // Need class, ID and length before we know the frame size.
            guard buffer.count >= 6 else { break }

            let messageClass = buffer[2]
            let messageID = buffer[3]
            let payloadLength = Int(buffer[4]) | (Int(buffer[5]) << 8)

            guard payloadLength <= Self.maximumPayloadLength else {
                // Implausible length: this was not really a frame header. Skip the preamble
                // and resynchronise rather than waiting forever.
                discardedByteCount += 2
                buffer.removeFirst(2)
                continue
            }

            let frameLength = 6 + payloadLength + 2
            guard buffer.count >= frameLength else { break }   // wait for the rest

            let body = Array(buffer[2..<(6 + payloadLength)])
            let (ckA, ckB) = Self.fletcherChecksum(body)
            let receivedA = buffer[6 + payloadLength]
            let receivedB = buffer[7 + payloadLength]

            guard ckA == receivedA, ckB == receivedB else {
                checksumFailureCount += 1
                // Drop the preamble only, so a real frame starting inside this one is found.
                discardedByteCount += 2
                buffer.removeFirst(2)
                continue
            }

            let payload = Array(buffer[6..<(6 + payloadLength)])
            buffer.removeFirst(frameLength)

            if messageClass == Self.navClass, messageID == Self.pvtID,
               payloadLength == Self.pvtPayloadLength,
               let pvt = Self.decodeNavPVT(payload) {
                messages.append(.navPVT(pvt))
            } else {
                messages.append(.other(messageClass: messageClass,
                                       messageID: messageID,
                                       payload: payload))
            }
        }

        return messages
    }

    public mutating func reset() {
        buffer.removeAll(keepingCapacity: true)
    }

    private func indexOfPreamble() -> Int? {
        guard buffer.count >= 2 else {
            // A single leading byte that cannot start a preamble is discardable.
            return buffer.first == Self.sync1 ? nil : (buffer.isEmpty ? nil : nil)
        }
        var i = 0
        while i + 1 < buffer.count {
            if buffer[i] == Self.sync1 && buffer[i + 1] == Self.sync2 { return i }
            i += 1
        }
        return nil
    }

    // MARK: - NAV-PVT decoding

    /// Offsets follow the table in spec §3.5. All multi-byte fields are little-endian.
    ///
    /// Verify this layout against the interface description for your module generation before
    /// shipping — u-blox has extended the trailing fields across generations, though offsets
    /// 0–76 have been stable. The length check in `consume` means a generation with a
    /// different payload size surfaces as `.other` rather than being silently misread.
    static func decodeNavPVT(_ p: [UInt8]) -> UBXNavPVT? {
        guard p.count == pvtPayloadLength else { return nil }

        func u2(_ o: Int) -> UInt16 { UInt16(p[o]) | (UInt16(p[o + 1]) << 8) }
        func u4(_ o: Int) -> UInt32 {
            UInt32(p[o]) | (UInt32(p[o + 1]) << 8) | (UInt32(p[o + 2]) << 16) | (UInt32(p[o + 3]) << 24)
        }
        func i4(_ o: Int) -> Int32 { Int32(bitPattern: u4(o)) }

        return UBXNavPVT(
            iTOW: u4(0),
            year: u2(4),
            month: p[6],
            day: p[7],
            hour: p[8],
            minute: p[9],
            second: p[10],
            validFlags: p[11],
            tAcc: u4(12),
            nano: i4(16),
            fixType: p[20],
            flags: p[21],
            flags2: p[22],
            numSV: p[23],
            longitudeDegrees: Double(i4(24)) * 1e-7,
            latitudeDegrees: Double(i4(28)) * 1e-7,
            heightAboveEllipsoid: Double(i4(32)) / 1000.0,
            heightAboveMSL: Double(i4(36)) / 1000.0,
            horizontalAccuracy: Double(u4(40)) / 1000.0,
            verticalAccuracy: Double(u4(44)) / 1000.0,
            velocityNED: Vector3(Double(i4(48)) / 1000.0,
                                 Double(i4(52)) / 1000.0,
                                 Double(i4(56)) / 1000.0),
            groundSpeed: Double(i4(60)) / 1000.0,
            headingOfMotionDegrees: Double(i4(64)) * 1e-5,
            speedAccuracy: Double(u4(68)) / 1000.0,
            headingAccuracyDegrees: Double(u4(72)) * 1e-5,
            pDOP: Double(u2(76)) * 0.01
        )
    }
}
