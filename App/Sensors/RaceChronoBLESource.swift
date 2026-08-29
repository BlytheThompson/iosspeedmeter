#if canImport(CoreBluetooth)
import CoreBluetooth
import Foundation
import PerformanceTimerCore

/// Spec §3.6 — external GNSS over BLE, the alternative to the Wi-Fi path.
///
/// Speaks the public RaceChrono DIY BLE format. Note the trade-off §3.6 spells out: there is no
/// speed-accuracy field, so `R` has to be derived from HDOP, which is much weaker than UBX's
/// `sAcc`. BLE also cannot sustain more than 1–2 Hz of NMEA, so the 25 Hz tier needs the TCP
/// path.
///
/// Characteristic `0x0003` carries position and speed with a 21-bit time-from-hour-start;
/// characteristic `0x0004` carries the hour and date in a matching field. Spec §3.6: "Match
/// the two by comparing sync bits; if they differ, wait for one to update."
public final class RaceChronoBLESource: NSObject, SensorSource,
                                        CBCentralManagerDelegate, CBPeripheralDelegate {
    public let descriptor = SensorSourceDescriptor(
        identifier: "external.racechrono.ble",
        displayName: "External GNSS (BLE, RaceChrono)",
        nominalRate: 10,
        kind: .gnss
    )

    public enum State: Equatable, Sendable {
        case idle, scanning, connecting, connected, streaming, failed(String)
    }

    public private(set) var isRunning = false
    public private(set) var state: State = .idle
    public var onStateChange: ((State) -> Void)?

    private static let serviceUUID = CBUUID(string: "1FF8")
    private static let mainCharacteristicUUID = CBUUID(string: "0003")
    private static let timeCharacteristicUUID = CBUUID(string: "0004")

    private var central: CBCentralManager?
    private var peripheral: CBPeripheral?
    private var handler: ((SensorEvent) -> Void)?
    private let clock: SessionClock

    /// Latest hour/date reference from characteristic 0x0004.
    private var timeReference: RaceChronoBLEParser.TimeReference?
    /// A main packet waiting for a matching time reference.
    private var pendingMain: RaceChronoBLEParser.MainPacket?

    public init(clock: SessionClock) {
        self.clock = clock
        super.init()
    }

    public func start(handler: @escaping (SensorEvent) -> Void) throws {
        guard !isRunning else { throw SensorSourceError.alreadyRunning }
        self.handler = handler
        isRunning = true
        central = CBCentralManager(delegate: self, queue:
            DispatchQueue(label: "com.performancetimer.ble"))
        setState(.scanning)
    }

    public func stop() {
        guard isRunning else { return }
        if let peripheral { central?.cancelPeripheralConnection(peripheral) }
        central?.stopScan()
        central = nil
        peripheral = nil
        handler = nil
        isRunning = false
        setState(.idle)
    }

    private func setState(_ newState: State) {
        state = newState
        onStateChange?(newState)
    }

    // MARK: - CBCentralManagerDelegate

    public func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
        case .poweredOn:
            central.scanForPeripherals(withServices: [Self.serviceUUID])
            setState(.scanning)
        case .unauthorized:
            setState(.failed("Bluetooth permission denied"))
        case .poweredOff:
            setState(.failed("Bluetooth is off"))
        default:
            break
        }
    }

    public func centralManager(_ central: CBCentralManager,
                               didDiscover peripheral: CBPeripheral,
                               advertisementData: [String: Any],
                               rssi RSSI: NSNumber) {
        central.stopScan()
        self.peripheral = peripheral
        peripheral.delegate = self
        setState(.connecting)
        central.connect(peripheral)
    }

    public func centralManager(_ central: CBCentralManager,
                               didConnect peripheral: CBPeripheral) {
        setState(.connected)
        peripheral.discoverServices([Self.serviceUUID])
    }

    public func centralManager(_ central: CBCentralManager,
                               didFailToConnect peripheral: CBPeripheral,
                               error: Error?) {
        setState(.failed(error?.localizedDescription ?? "Failed to connect"))
    }

    public func centralManager(_ central: CBCentralManager,
                               didDisconnectPeripheral peripheral: CBPeripheral,
                               error: Error?) {
        setState(.failed(error?.localizedDescription ?? "Disconnected"))
    }

    // MARK: - CBPeripheralDelegate

    public func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        for service in peripheral.services ?? [] {
            peripheral.discoverCharacteristics(
                [Self.mainCharacteristicUUID, Self.timeCharacteristicUUID], for: service)
        }
    }

    public func peripheral(_ peripheral: CBPeripheral,
                           didDiscoverCharacteristicsFor service: CBService,
                           error: Error?) {
        for characteristic in service.characteristics ?? [] {
            peripheral.setNotifyValue(true, for: characteristic)
        }
        setState(.streaming)
    }

    public func peripheral(_ peripheral: CBPeripheral,
                           didUpdateValueFor characteristic: CBCharacteristic,
                           error: Error?) {
        guard let data = characteristic.value else { return }
        let bytes = [UInt8](data)

        if characteristic.uuid == Self.mainCharacteristicUUID {
            guard let packet = RaceChronoBLEParser.parseMain(bytes) else { return }
            pendingMain = packet
            emitIfPaired()
        } else if characteristic.uuid == Self.timeCharacteristicUUID {
            guard let reference = Self.parseTimeReference(bytes) else { return }
            timeReference = reference
            emitIfPaired()
        }
    }

    /// Spec §3.6: emit only when the sync bits of the two characteristics agree.
    private func emitIfPaired() {
        guard let packet = pendingMain, let reference = timeReference,
              let unixTime = packet.unixTime(pairedWith: reference)
        else { return }
        pendingMain = nil
        handler?(.gnss(packet.gnssFix(
            sessionTime: clock.sessionTime(wallClockUnixTime: unixTime))))
    }

    /// Decode characteristic `0x0004`: 3 sync bits then a 21-bit hour-and-date field.
    ///
    /// The field encodes hours since a fixed epoch; what matters for timing is that both
    /// characteristics agree on which hour the main packet's offset belongs to.
    static func parseTimeReference(_ bytes: [UInt8]) -> RaceChronoBLEParser.TimeReference? {
        guard bytes.count >= 3 else { return nil }
        let head = (UInt32(bytes[0]) << 16) | (UInt32(bytes[1]) << 8) | UInt32(bytes[2])
        let syncBits = UInt8((head >> 21) & 0x07)
        let hoursSinceEpoch = Double(head & 0x001F_FFFF)
        return RaceChronoBLEParser.TimeReference(
            syncBits: syncBits,
            hourUnixTime: hoursSinceEpoch * 3600
        )
    }
}
#endif
