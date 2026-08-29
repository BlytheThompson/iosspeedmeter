import Foundation

/// Spec §3.6 — RaceChrono DIY BLE GPS format.
///
/// Service `00001ff8-0000-1000-8000-00805f9b34fb`, GPS main characteristic `0x0003`,
/// READ + NOTIFY, 20 bytes, **big-endian** (note the contrast with UBX, which is little-endian).
///
/// This path has no speed-accuracy field, so `R` has to be derived from HDOP. That is a much
/// weaker basis than UBX's `sAcc`, and `estimatedSpeedAccuracy` is deliberately conservative
/// about it.
public enum RaceChronoBLEParser {
    public static let serviceUUID = "00001FF8-0000-1000-8000-00805F9B34FB"
    public static let mainCharacteristicUUID = "00000003-0000-1000-8000-00805F9B34FB"
    public static let timeCharacteristicUUID = "00000004-0000-1000-8000-00805F9B34FB"

    /// Sentinel in the 6-bit satellite field meaning "not available" (spec §3.6).
    public static let invalidSatelliteCount: UInt8 = 0x3F

    /// Contents of characteristic `0x0004`: the hour and date the main packet's
    /// time-from-hour-start is relative to, plus its own copy of the sync bits.
    public struct TimeReference: Equatable, Sendable {
        public var syncBits: UInt8
        /// Unix time at the start of the hour.
        public var hourUnixTime: Double

        public init(syncBits: UInt8, hourUnixTime: Double) {
            self.syncBits = syncBits
            self.hourUnixTime = hourUnixTime
        }
    }

    public struct MainPacket: Equatable, Sendable {
        /// 3-bit counter used to pair this packet with a `TimeReference`.
        public var syncBits: UInt8
        /// Decoded from the 21-bit field: `(min × 30000) + (sec × 500) + (ms / 2)`.
        public var secondsFromHourStart: Double
        /// 2-bit fix quality.
        public var fixQuality: UInt8
        /// 6-bit locked-satellite count. Meaningless when `satellitesValid` is false.
        public var satellites: UInt8
        public var latitudeDegrees: Double
        public var longitudeDegrees: Double
        public var altitude: Double
        public var speedKmh: Double
        public var bearingDegrees: Double
        public var hdop: Double
        public var vdop: Double

        public var satellitesValid: Bool { satellites != RaceChronoBLEParser.invalidSatelliteCount }

        public var speedMetersPerSecond: Double {
            speedKmh * PTConstants.kmhToMetersPerSecond
        }

        /// Pair with characteristic `0x0004`. Spec §3.6: "Match the two by comparing sync
        /// bits; if they differ, wait for one to update." Returning `nil` on a mismatch is
        /// what makes that rule impossible to skip by accident.
        public func unixTime(pairedWith reference: TimeReference) -> Double? {
            guard reference.syncBits == syncBits else { return nil }
            return reference.hourUnixTime + secondsFromHourStart
        }

        /// Derived 1σ speed accuracy, m/s.
        ///
        /// There is no `sAcc` in this format, so this is a heuristic, not a measurement:
        /// a nominal user-equivalent range-rate error scaled by HDOP, floored at the §6.2
        /// `σ_gps` floor of 0.05 m/s and deliberately biased pessimistic. Anything derived
        /// this way should be treated as a Medium-confidence input at best (§9.4) — this is
        /// the concrete reason the spec calls `sAcc` "what the filter needs".
        public var estimatedSpeedAccuracy: Double {
            let nominalDopplerSigma = 0.10     // m/s at HDOP 1.0, conservative for a bare module
            let floor = 0.05                   // spec §6.2 sigma floor
            let dop = max(hdop, 0.5)
            return max(floor + 1e-9, nominalDopplerSigma * dop)
        }

        /// Convert to the estimator's fix type. `t` must already be on the session clock.
        public func gnssFix(sessionTime t: Double) -> GNSSFix {
            GNSSFix(
                t: t,
                speed: speedMetersPerSecond,
                speedAccuracy: estimatedSpeedAccuracy,
                course: bearingDegrees,
                courseAccuracy: -1,
                latitudeDegrees: latitudeDegrees,
                longitudeDegrees: longitudeDegrees,
                horizontalAccuracy: hdop * 3.0,
                altitude: altitude,
                verticalAccuracy: vdop * 3.0,
                // The 2-bit quality field maps 0 = no fix; anything else is at least 2D.
                // Treat >= 2 as a usable 3D fix, matching how the format is used in practice.
                fixType: fixQuality >= 2 ? 3 : Int(fixQuality),
                numSV: satellitesValid ? Int(satellites) : 0,
                gnssFixOK: fixQuality > 0,
                source: .externalBLE
            )
        }
    }

    /// Decode the 20-byte GPS main characteristic. Returns `nil` for any other length.
    public static func parseMain(_ bytes: [UInt8]) -> MainPacket? {
        guard bytes.count == 20 else { return nil }

        // Bytes 0–2: 3 sync bits, then a 21-bit time from the hour start.
        let head = (UInt32(bytes[0]) << 16) | (UInt32(bytes[1]) << 8) | UInt32(bytes[2])
        let syncBits = UInt8((head >> 21) & 0x07)
        let rawTime = head & 0x001F_FFFF
        // (min × 30000) + (sec × 500) + (ms / 2)  =>  units of 2 ms.
        let secondsFromHourStart = Double(rawTime) * 0.002

        // Byte 3: fix quality (2 bits), locked satellites (6 bits).
        let fixQuality = (bytes[3] >> 6) & 0x03
        let satellites = bytes[3] & 0x3F

        func i4(_ o: Int) -> Int32 {
            let u = (UInt32(bytes[o]) << 24) | (UInt32(bytes[o + 1]) << 16)
                | (UInt32(bytes[o + 2]) << 8) | UInt32(bytes[o + 3])
            return Int32(bitPattern: u)
        }
        func u2(_ o: Int) -> UInt16 {
            (UInt16(bytes[o]) << 8) | UInt16(bytes[o + 1])
        }

        let latitude = Double(i4(4)) * 1e-7
        let longitude = Double(i4(8)) * 1e-7

        // Bytes 12–13, two-form encoding — check the high bit before decoding.
        //   fine:   ((m + 500) × 10) & 0x7FFF
        //   coarse: ((m + 500) & 0x7FFF) | 0x8000
        let rawAltitude = u2(12)
        let altitude: Double = (rawAltitude & 0x8000) != 0
            ? Double(rawAltitude & 0x7FFF) - 500.0
            : Double(rawAltitude & 0x7FFF) / 10.0 - 500.0

        // Bytes 14–15, same trick.
        //   fine:   (km/h × 100) & 0x7FFF
        //   coarse: ((km/h × 10) & 0x7FFF) | 0x8000
        let rawSpeed = u2(14)
        let speedKmh: Double = (rawSpeed & 0x8000) != 0
            ? Double(rawSpeed & 0x7FFF) / 10.0
            : Double(rawSpeed & 0x7FFF) / 100.0

        return MainPacket(
            syncBits: syncBits,
            secondsFromHourStart: secondsFromHourStart,
            fixQuality: fixQuality,
            satellites: satellites,
            latitudeDegrees: latitude,
            longitudeDegrees: longitude,
            altitude: altitude,
            speedKmh: speedKmh,
            bearingDegrees: Double(u2(16)) / 100.0,
            hdop: Double(bytes[18]) / 10.0,
            vdop: Double(bytes[19]) / 10.0
        )
    }
}
