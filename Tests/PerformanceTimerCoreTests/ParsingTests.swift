import XCTest
@testable import PerformanceTimerCore

// MARK: - Frame construction helpers

/// Little-endian encoders for building synthetic UBX payloads.
enum LE {
    static func u2(_ v: UInt16) -> [UInt8] { [UInt8(v & 0xFF), UInt8(v >> 8)] }
    static func u4(_ v: UInt32) -> [UInt8] {
        [UInt8(v & 0xFF), UInt8((v >> 8) & 0xFF), UInt8((v >> 16) & 0xFF), UInt8((v >> 24) & 0xFF)]
    }
    static func i4(_ v: Int32) -> [UInt8] { u4(UInt32(bitPattern: v)) }
}

/// Builds a well-formed `UBX-NAV-PVT` frame with the field values given.
/// Offsets follow the table in spec §3.5.
struct NAVPVTBuilder {
    var iTOW: UInt32 = 100_000
    var year: UInt16 = 2026
    var month: UInt8 = 8
    var day: UInt8 = 29
    var hour: UInt8 = 13
    var minute: UInt8 = 45
    var second: UInt8 = 12
    var valid: UInt8 = 0x07
    var tAcc: UInt32 = 25
    var nano: Int32 = 500_000            // 0.5 ms
    var fixType: UInt8 = 3
    var flags: UInt8 = 0x01              // gnssFixOK set
    var flags2: UInt8 = 0
    var numSV: UInt8 = 14
    var lon: Int32 = -1_221_697_000      // -122.1697 deg
    var lat: Int32 = 374_275_000         // 37.4275 deg
    var height: Int32 = 40_000           // mm
    var hMSL: Int32 = 30_000             // mm
    var hAcc: UInt32 = 850               // mm
    var vAcc: UInt32 = 1_200             // mm
    var velN: Int32 = 12_000             // mm/s
    var velE: Int32 = -3_000
    var velD: Int32 = 100
    var gSpeed: Int32 = 12_369           // mm/s
    var headMot: Int32 = 4_512_000       // 1e-5 deg
    var sAcc: UInt32 = 62                // mm/s
    var headAcc: UInt32 = 1_200_000
    var pDOP: UInt16 = 137               // 1.37

    func payload() -> [UInt8] {
        var p = [UInt8]()
        p += LE.u4(iTOW)                                  // 0
        p += LE.u2(year)                                  // 4
        p += [month, day, hour, minute, second]           // 6..10
        p += [valid]                                      // 11
        p += LE.u4(tAcc)                                  // 12
        p += LE.i4(nano)                                  // 16
        p += [fixType, flags, flags2, numSV]              // 20..23
        p += LE.i4(lon)                                   // 24
        p += LE.i4(lat)                                   // 28
        p += LE.i4(height)                                // 32
        p += LE.i4(hMSL)                                  // 36
        p += LE.u4(hAcc)                                  // 40
        p += LE.u4(vAcc)                                  // 44
        p += LE.i4(velN)                                  // 48
        p += LE.i4(velE)                                  // 52
        p += LE.i4(velD)                                  // 56
        p += LE.i4(gSpeed)                                // 60
        p += LE.i4(headMot)                               // 64
        p += LE.u4(sAcc)                                  // 68
        p += LE.u4(headAcc)                               // 72
        p += LE.u2(pDOP)                                  // 76
        p += [UInt8](repeating: 0, count: 92 - p.count)   // 78..91 reserved
        precondition(p.count == 92)
        return p
    }

    func frame() -> [UInt8] {
        UBXParser.makeFrame(messageClass: 0x01, messageID: 0x07, payload: payload())
    }
}

// MARK: - UBX

/// Spec §3.5 — UBX binary framing and `UBX-NAV-PVT` decoding.
final class UBXParserTests: XCTestCase {
    func testFletcherChecksumMatchesWorkedExample() {
        // 8-bit Fletcher over class, ID, length and payload.
        // Hand-computed for class=0x01, id=0x07, len=0, empty payload:
        //   a = 1, b = 1; a += 7 -> 8, b += 8 -> 9; a += 0 -> 8, b += 8 -> 17; a += 0, b += 25
        let (ckA, ckB) = UBXParser.fletcherChecksum([0x01, 0x07, 0x00, 0x00])
        XCTAssertEqual(ckA, 8)
        XCTAssertEqual(ckB, 25)
    }

    func testDecodesAllNAVPVTFieldsWithCorrectScaling() {
        var parser = UBXParser()
        let builder = NAVPVTBuilder()
        let messages = parser.consume(builder.frame())

        XCTAssertEqual(messages.count, 1)
        guard case .navPVT(let pvt) = messages[0] else { return XCTFail("expected navPVT") }

        XCTAssertEqual(pvt.iTOW, 100_000)
        XCTAssertEqual(pvt.year, 2026)
        XCTAssertEqual(pvt.month, 8)
        XCTAssertEqual(pvt.day, 29)
        XCTAssertEqual(pvt.nano, 500_000)
        XCTAssertEqual(pvt.fixType, 3)
        XCTAssertTrue(pvt.gnssFixOK)
        XCTAssertEqual(pvt.numSV, 14)
        XCTAssertEqual(pvt.longitudeDegrees, -122.1697, accuracy: 1e-9)
        XCTAssertEqual(pvt.latitudeDegrees, 37.4275, accuracy: 1e-9)
        XCTAssertEqual(pvt.heightAboveEllipsoid, 40.0, accuracy: 1e-9)
        XCTAssertEqual(pvt.heightAboveMSL, 30.0, accuracy: 1e-9)
        XCTAssertEqual(pvt.horizontalAccuracy, 0.850, accuracy: 1e-9)
        XCTAssertEqual(pvt.verticalAccuracy, 1.200, accuracy: 1e-9)
        XCTAssertEqual(pvt.groundSpeed, 12.369, accuracy: 1e-9)
        XCTAssertEqual(pvt.speedAccuracy, 0.062, accuracy: 1e-9)
        XCTAssertEqual(pvt.headingOfMotionDegrees, 45.12, accuracy: 1e-7)
        XCTAssertEqual(pvt.headingAccuracyDegrees, 12.0, accuracy: 1e-7)
        XCTAssertEqual(pvt.pDOP, 1.37, accuracy: 1e-9)
        XCTAssertEqual(pvt.velocityNED.x, 12.0, accuracy: 1e-9)
        XCTAssertEqual(pvt.velocityNED.y, -3.0, accuracy: 1e-9)
        XCTAssertEqual(pvt.velocityNED.z, 0.1, accuracy: 1e-9)
    }

    func testTimeOfWeekCombinesITOWAndNanoFraction() {
        // Spec §3.5: precise fix epoch = week start + iTOW/1000 + nano/1e9.
        var parser = UBXParser()
        var builder = NAVPVTBuilder()
        builder.iTOW = 250_000
        builder.nano = -250_000          // signed: fix is 0.25 ms *before* the iTOW tick
        let messages = parser.consume(builder.frame())
        guard case .navPVT(let pvt) = messages[0] else { return XCTFail("expected navPVT") }
        XCTAssertEqual(pvt.timeOfWeekSeconds, 250.0 - 0.00025, accuracy: 1e-12)
    }

    func testCarrierSolutionIsReadFromFlagsBitsSixAndSeven() {
        var parser = UBXParser()
        var builder = NAVPVTBuilder()
        builder.flags = 0x01 | (2 << 6)   // gnssFixOK + carrSoln = 2 (fixed)
        let messages = parser.consume(builder.frame())
        guard case .navPVT(let pvt) = messages[0] else { return XCTFail("expected navPVT") }
        XCTAssertEqual(pvt.carrierSolution, .fixed)

        var floatBuilder = NAVPVTBuilder()
        floatBuilder.flags = 0x01 | (1 << 6)
        var p2 = UBXParser()
        guard case .navPVT(let pvt2) = p2.consume(floatBuilder.frame())[0] else {
            return XCTFail("expected navPVT")
        }
        XCTAssertEqual(pvt2.carrierSolution, .float)
    }

    func testGnssFixOKClearIsReported() {
        var parser = UBXParser()
        var builder = NAVPVTBuilder()
        builder.flags = 0x00
        let messages = parser.consume(builder.frame())
        guard case .navPVT(let pvt) = messages[0] else { return XCTFail("expected navPVT") }
        XCTAssertFalse(pvt.gnssFixOK)
    }

    func testFrameSplitAcrossArbitraryChunkBoundariesStillParses() {
        // A TCP stream delivers whatever it likes; the framer must not care.
        let frame = NAVPVTBuilder().frame()
        for splitPoint in [1, 2, 5, 6, 7, 50, 97, frame.count - 1] {
            var parser = UBXParser()
            let first = parser.consume(Array(frame[0..<splitPoint]))
            XCTAssertTrue(first.isEmpty, "split at \(splitPoint) produced a premature message")
            let second = parser.consume(Array(frame[splitPoint...]))
            XCTAssertEqual(second.count, 1, "split at \(splitPoint) lost the message")
        }
    }

    func testByteAtATimeDeliveryParses() {
        var parser = UBXParser()
        let frame = NAVPVTBuilder().frame()
        var messages: [UBXMessage] = []
        for byte in frame { messages += parser.consume([byte]) }
        XCTAssertEqual(messages.count, 1)
    }

    func testBadChecksumIsRejectedAndCounted() {
        var parser = UBXParser()
        var frame = NAVPVTBuilder().frame()
        frame[frame.count - 1] ^= 0xFF
        XCTAssertTrue(parser.consume(frame).isEmpty)
        XCTAssertEqual(parser.checksumFailureCount, 1)
    }

    /// The usual case: the socket opens while the receiver is mid-sentence, so the stream
    /// starts with a partial message. No false preamble here.
    func testResynchronisesAfterLeadingGarbage() {
        var parser = UBXParser()
        let garbage: [UInt8] = [0x00, 0xB5, 0x11, 0xFF, 0x24, 0x47, 0x4E, 0x01]
        let messages = parser.consume(garbage + NAVPVTBuilder().frame())
        XCTAssertEqual(messages.count, 1)
        XCTAssertGreaterThan(parser.discardedByteCount, 0)
    }

    /// If noise happens to contain a valid-looking `B5 62` preamble, the framer will believe
    /// the length field that follows and wait for that many bytes — which is the correct
    /// behaviour for a streaming decoder, since a real payload may itself contain `B5 62`.
    /// Recovery comes from the checksum failing once the declared length is satisfied, after
    /// which the parser resynchronises. This test pins that recovery down.
    func testRecoversFromAFalsePreambleOnceEnoughDataArrives() {
        var parser = UBXParser()
        // 0xB5 0x62 followed by class/id/length that declare a 354-byte payload.
        let falsePreamble: [UInt8] = [0xB5, 0x62, 0x01, 0xB5, 0x62, 0x01]
        _ = parser.consume(falsePreamble)

        var messages: [UBXMessage] = []
        for i in 0..<6 {
            var builder = NAVPVTBuilder()
            builder.iTOW = UInt32(500_000 + i * 1000)
            messages += parser.consume(builder.frame())
        }
        XCTAssertGreaterThan(parser.checksumFailureCount, 0,
                             "recovery should come via a checksum failure")
        XCTAssertFalse(messages.isEmpty, "parser must recover once the false frame is resolved")
        // Whatever was swallowed, the parser must be healthy afterwards.
        var builder = NAVPVTBuilder()
        builder.iTOW = 777_000
        let after = parser.consume(builder.frame())
        XCTAssertEqual(after.count, 1)
        guard case .navPVT(let pvt) = after[0] else { return XCTFail("expected navPVT") }
        XCTAssertEqual(pvt.iTOW, 777_000)
    }

    func testRecoversAfterACorruptedFrame() {
        var parser = UBXParser()
        var bad = NAVPVTBuilder().frame()
        bad[bad.count - 2] ^= 0xFF
        var good = NAVPVTBuilder()
        good.iTOW = 999_000
        let messages = parser.consume(bad + good.frame())
        XCTAssertEqual(messages.count, 1)
        guard case .navPVT(let pvt) = messages[0] else { return XCTFail("expected navPVT") }
        XCTAssertEqual(pvt.iTOW, 999_000)
    }

    func testNavPVTWithUnexpectedLengthIsNotDecodedAsPVT() {
        // u-blox has extended trailing fields across generations; a short payload must not be
        // silently misread.
        var parser = UBXParser()
        let short = UBXParser.makeFrame(messageClass: 0x01, messageID: 0x07,
                                        payload: [UInt8](repeating: 0, count: 84))
        let messages = parser.consume(short)
        XCTAssertEqual(messages.count, 1)
        guard case .other(let cls, let id, let payload) = messages[0] else {
            return XCTFail("expected .other for a non-92-byte NAV-PVT")
        }
        XCTAssertEqual(cls, 0x01)
        XCTAssertEqual(id, 0x07)
        XCTAssertEqual(payload.count, 84)
    }

    func testOtherMessageClassesArePassedThroughNotDropped() {
        var parser = UBXParser()
        let ack = UBXParser.makeFrame(messageClass: 0x05, messageID: 0x01, payload: [0x06, 0x01])
        let messages = parser.consume(ack)
        XCTAssertEqual(messages.count, 1)
        guard case .other(let cls, let id, _) = messages[0] else { return XCTFail("expected other") }
        XCTAssertEqual(cls, 0x05)
        XCTAssertEqual(id, 0x01)
    }

    func testOversizedLengthDoesNotStallTheParser() {
        var parser = UBXParser()
        // Declares a 60000-byte payload that never arrives, then a valid frame follows.
        let bogus: [UInt8] = [0xB5, 0x62, 0x01, 0x07, 0x60, 0xEA]
        _ = parser.consume(bogus)
        let messages = parser.consume(NAVPVTBuilder().frame())
        XCTAssertEqual(messages.count, 1, "parser must abandon an implausible length declaration")
    }

    func testBufferDoesNotGrowWithoutBoundOnPureNoise() {
        var parser = UBXParser()
        for _ in 0..<200 {
            _ = parser.consume([UInt8](repeating: 0xAA, count: 1024))
        }
        XCTAssertLessThan(parser.bufferedByteCount, 4096)
    }
}

// MARK: - RaceChrono BLE

/// Spec §3.6 — RaceChrono DIY BLE GPS main characteristic, 20 bytes, big-endian.
final class RaceChronoBLEParserTests: XCTestCase {
    /// Builds the 20-byte main characteristic payload.
    private func packet(
        sync: UInt8 = 0b101,
        minute: Int = 12, second: Int = 34, millisecond: Int = 500,
        fixQuality: UInt8 = 2, satellites: UInt8 = 11,
        latitude: Int32 = 374_275_000, longitude: Int32 = -1_221_697_000,
        altitudeRaw: UInt16 = UInt16(((30 + 500) * 10) & 0x7FFF),
        speedRaw: UInt16 = UInt16((4452) & 0x7FFF),      // 44.52 km/h in the fine form
        bearingRaw: UInt16 = 12_345,
        hdop: UInt8 = 9, vdop: UInt8 = 14
    ) -> [UInt8] {
        let time = UInt32(minute * 30000 + second * 500 + millisecond / 2)
        precondition(time < (1 << 21))
        let head = (UInt32(sync) << 21) | time
        var b = [UInt8]()
        b.append(UInt8((head >> 16) & 0xFF))
        b.append(UInt8((head >> 8) & 0xFF))
        b.append(UInt8(head & 0xFF))
        b.append((fixQuality << 6) | (satellites & 0x3F))
        for shift in stride(from: 24, through: 0, by: -8) {
            b.append(UInt8((UInt32(bitPattern: latitude) >> UInt32(shift)) & 0xFF))
        }
        for shift in stride(from: 24, through: 0, by: -8) {
            b.append(UInt8((UInt32(bitPattern: longitude) >> UInt32(shift)) & 0xFF))
        }
        b.append(UInt8(altitudeRaw >> 8)); b.append(UInt8(altitudeRaw & 0xFF))
        b.append(UInt8(speedRaw >> 8)); b.append(UInt8(speedRaw & 0xFF))
        b.append(UInt8(bearingRaw >> 8)); b.append(UInt8(bearingRaw & 0xFF))
        b.append(hdop)
        b.append(vdop)
        precondition(b.count == 20)
        return b
    }

    func testRejectsPacketsThatAreNotTwentyBytes() {
        XCTAssertNil(RaceChronoBLEParser.parseMain([UInt8](repeating: 0, count: 19)))
        XCTAssertNil(RaceChronoBLEParser.parseMain([UInt8](repeating: 0, count: 21)))
    }

    func testDecodesTwentyOneBitTimeFromHourStart() {
        let r = RaceChronoBLEParser.parseMain(packet(minute: 12, second: 34, millisecond: 500))!
        // (12 * 60) + 34 + 0.5 = 754.5 s from the hour start.
        XCTAssertEqual(r.secondsFromHourStart, 754.5, accuracy: 1e-9)
        XCTAssertEqual(r.syncBits, 0b101)
    }

    func testDecodesFixQualityAndSatelliteCount() {
        let r = RaceChronoBLEParser.parseMain(packet(fixQuality: 2, satellites: 11))!
        XCTAssertEqual(r.fixQuality, 2)
        XCTAssertEqual(r.satellites, 11)
        XCTAssertTrue(r.satellitesValid)
    }

    func testSatelliteSentinelMarksCountInvalid() {
        // Spec §3.6: 0x3F means invalid.
        let r = RaceChronoBLEParser.parseMain(packet(satellites: 0x3F))!
        XCTAssertFalse(r.satellitesValid)
    }

    func testDecodesSignedLatitudeAndLongitude() {
        let r = RaceChronoBLEParser.parseMain(packet())!
        XCTAssertEqual(r.latitudeDegrees, 37.4275, accuracy: 1e-9)
        XCTAssertEqual(r.longitudeDegrees, -122.1697, accuracy: 1e-9)
    }

    func testFineAltitudeFormIsDecodedWhenHighBitClear() {
        // ((m + 500) * 10) & 0x7FFF  =>  m = raw/10 - 500
        let raw = UInt16(((123 + 500) * 10) & 0x7FFF)
        let r = RaceChronoBLEParser.parseMain(packet(altitudeRaw: raw))!
        XCTAssertEqual(r.altitude, 123.0, accuracy: 1e-9)
    }

    func testCoarseAltitudeFormIsDecodedWhenHighBitSet() {
        // ((m + 500) & 0x7FFF) | 0x8000  =>  m = (raw & 0x7FFF) - 500
        let raw = UInt16(((1234 + 500) & 0x7FFF)) | 0x8000
        let r = RaceChronoBLEParser.parseMain(packet(altitudeRaw: raw))!
        XCTAssertEqual(r.altitude, 1234.0, accuracy: 1e-9)
    }

    func testFineSpeedFormIsDecodedWhenHighBitClear() {
        // (km/h * 100) & 0x7FFF
        let r = RaceChronoBLEParser.parseMain(packet(speedRaw: UInt16(4452)))!
        XCTAssertEqual(r.speedKmh, 44.52, accuracy: 1e-9)
        XCTAssertEqual(r.speedMetersPerSecond,
                       44.52 * PTConstants.kmhToMetersPerSecond, accuracy: 1e-9)
    }

    func testCoarseSpeedFormIsDecodedWhenHighBitSet() {
        // ((km/h * 10) & 0x7FFF) | 0x8000
        let raw = UInt16((2225) & 0x7FFF) | 0x8000
        let r = RaceChronoBLEParser.parseMain(packet(speedRaw: raw))!
        XCTAssertEqual(r.speedKmh, 222.5, accuracy: 1e-9)
    }

    func testDecodesBearingAndDilutionOfPrecision() {
        let r = RaceChronoBLEParser.parseMain(packet(bearingRaw: 12_345, hdop: 9, vdop: 14))!
        XCTAssertEqual(r.bearingDegrees, 123.45, accuracy: 1e-9)
        XCTAssertEqual(r.hdop, 0.9, accuracy: 1e-9)
        XCTAssertEqual(r.vdop, 1.4, accuracy: 1e-9)
    }

    /// Spec §3.6: characteristic 0x0004 carries hour/date in a matching 21-bit field, and the
    /// two are paired by comparing sync bits.
    func testTimePairingRequiresMatchingSyncBits() {
        let main = RaceChronoBLEParser.parseMain(packet(sync: 0b101))!
        let matching = RaceChronoBLEParser.TimeReference(syncBits: 0b101, hourUnixTime: 1_700_000_000)
        let mismatched = RaceChronoBLEParser.TimeReference(syncBits: 0b010, hourUnixTime: 1_700_000_000)
        XCTAssertNotNil(main.unixTime(pairedWith: matching))
        XCTAssertNil(main.unixTime(pairedWith: mismatched))
    }

    func testPairedUnixTimeAddsSecondsFromHourStart() {
        let main = RaceChronoBLEParser.parseMain(
            packet(sync: 0b011, minute: 12, second: 34, millisecond: 500)
        )!
        let reference = RaceChronoBLEParser.TimeReference(syncBits: 0b011, hourUnixTime: 1_700_000_000)
        XCTAssertEqual(main.unixTime(pairedWith: reference)!,
                       1_700_000_000 + 754.5, accuracy: 1e-9)
    }

    /// Spec §3.6 notes there is no speed-accuracy field, so `R` must be derived from HDOP.
    /// That derivation must be explicit and conservative rather than silently optimistic.
    func testSpeedAccuracyIsDerivedFromHDOPAndIsWorseThanTheUBXFloor() {
        let r = RaceChronoBLEParser.parseMain(packet(hdop: 9))!
        let sigma = r.estimatedSpeedAccuracy
        XCTAssertGreaterThan(sigma, 0.05, "must never claim better than the §6.2 sigma floor")
        XCTAssertLessThan(sigma, 5.0)

        let worse = RaceChronoBLEParser.parseMain(packet(hdop: 40))!
        XCTAssertGreaterThan(worse.estimatedSpeedAccuracy, sigma,
                             "higher HDOP must yield a larger sigma")
    }
}

/// Spec §3.7 — wheel speed carried over the existing GNSS link as a UBX private message.
final class WheelSpeedMessageTests: XCTestCase {
    func testRoundTripsThroughTheExistingUBXFramer() {
        var parser = UBXParser()
        let frame = WheelSpeedMessage.makeFrame(iTOW: 250_000, speedMillimetresPerSecond: 18_500)
        let messages = parser.consume(frame)

        XCTAssertEqual(messages.count, 1)
        guard case .other(let cls, let id, let payload) = messages[0] else {
            return XCTFail("expected a private message")
        }
        XCTAssertEqual(cls, WheelSpeedMessage.messageClass)
        XCTAssertEqual(id, WheelSpeedMessage.messageID)

        let decoded = WheelSpeedMessage.decode(payload)!
        XCTAssertEqual(decoded.iTOW, 250_000)
        XCTAssertEqual(decoded.speed, 18.5, accuracy: 1e-9)
    }

    func testDecodesNegativeSpeedForReverse() {
        let frame = WheelSpeedMessage.makeFrame(iTOW: 1000, speedMillimetresPerSecond: -2_250)
        var parser = UBXParser()
        guard case .other(_, _, let payload) = parser.consume(frame)[0] else {
            return XCTFail("expected a private message")
        }
        XCTAssertEqual(WheelSpeedMessage.decode(payload)!.speed, -2.25, accuracy: 1e-9)
    }

    func testWrongPayloadLengthIsRefused() {
        XCTAssertNil(WheelSpeedMessage.decode([UInt8](repeating: 0, count: 6)))
        XCTAssertNil(WheelSpeedMessage.decode([UInt8](repeating: 0, count: 12)))
    }

    func testInterleavesWithPositionFixesOnTheSameLink() {
        // The point of reusing the UBX framer: both message types share one stream, one
        // checksum, and one clock alignment.
        var parser = UBXParser()
        let stream = NAVPVTBuilder().frame()
            + WheelSpeedMessage.makeFrame(iTOW: 100_040, speedMillimetresPerSecond: 12_000)
            + NAVPVTBuilder().frame()
        let messages = parser.consume(stream)
        XCTAssertEqual(messages.count, 3)
        guard case .navPVT = messages[0], case .other = messages[1], case .navPVT = messages[2]
        else { return XCTFail("unexpected message sequence") }
    }
}
