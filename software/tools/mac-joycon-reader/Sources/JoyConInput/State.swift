import Foundation

public enum ControllerRole: String, Codable {
    case left, right, pair, unknown

    // Names are hints, never authority to move an arm. In particular an unknown
    // controller must not silently become the right arm.
    public static func classify(name: String, category: String) -> Self {
        let text = (name + " " + category).lowercased()
        let compact = text.filter { $0.isLetter || $0.isNumber }
        guard compact.contains("joycon") else { return .unknown }
        let left = text.contains("(l)") || compact.contains("joyconleft")
            || text.contains("left")
        let right = text.contains("(r)") || compact.contains("joyconright")
            || text.contains("right")
        if (left && right) || text.contains("pair") || text.contains("(l/r)") { return .pair }
        if left { return .left }
        if right { return .right }
        return .unknown
    }
}

public struct Vector3: Codable, Equatable {
    public let x: Double
    public let y: Double
    public let z: Double
    public init(_ x: Double, _ y: Double, _ z: Double) { self.x = x; self.y = y; self.z = z }
}

public struct Quaternion: Codable, Equatable {
    public let x: Double
    public let y: Double
    public let z: Double
    public let w: Double
}

public struct ButtonState: Codable {
    public let value: Float
    public let pressed: Bool
    public let physicalNames: [String]
}

public struct AxisState: Codable {
    public let raw: Float
    public let filtered: Float
}

public struct PadState: Codable {
    public let x: AxisState
    public let y: AxisState
}

public struct MotionState: Codable {
    public let sensorsActive: Bool
    public let attitude: Quaternion?
    public let rotationRate: Vector3?
    public let gravity: Vector3?
    public let userAcceleration: Vector3?
    public let acceleration: Vector3
    public let eventCount: UInt64
    public let lastEventUptime: Double?
}

public struct ControllerState: Codable, Identifiable {
    public let id: String
    public let name: String
    public let category: String
    public let role: ControllerRole
    public let connected: Bool
    public let remapped: Bool
    public let batteryLevel: Float?
    public let batteryState: String?
    public let buttons: [String: ButtonState]
    public let axes: [String: AxisState]
    public let pads: [String: PadState]
    public let motion: MotionState?
    public let inputEventCount: UInt64
    public let lastInputEventUptime: Double?

    public func disconnected() -> Self {
        Self(id: id, name: name, category: category, role: role, connected: false,
             remapped: remapped, batteryLevel: nil, batteryState: nil, buttons: [:],
             axes: [:], pads: [:], motion: nil, inputEventCount: inputEventCount,
             lastInputEventUptime: lastInputEventUptime)
    }
}

public struct InputFrame: Encodable {
    public let schemaVersion = 1
    public let inputOnly = true
    public let source: String
    public let sessionID: String
    public let sequence: UInt64
    public let timestamp: Double
    public let uptime: Double
    public let event: String
    public let controllers: [ControllerState]

    public init(source: String, sessionID: String, sequence: UInt64, event: String,
                controllers: [ControllerState]) {
        self.source = source; self.sessionID = sessionID; self.sequence = sequence
        self.timestamp = Date().timeIntervalSince1970
        self.uptime = ProcessInfo.processInfo.systemUptime
        self.event = event; self.controllers = controllers
    }

    public func jsonLine() throws -> Data {
        let encoder = JSONEncoder()
        encoder.keyEncodingStrategy = .convertToSnakeCase
        encoder.outputFormatting = [.sortedKeys, .withoutEscapingSlashes]
        var data = try encoder.encode(self)
        data.append(0x0a)
        return data
    }
}

public func deadzone(_ value: Float, threshold: Float) -> Float {
    guard value.isFinite, threshold.isFinite, threshold >= 0, threshold < 1 else { return 0 }
    let bounded = min(1, max(-1, value))
    guard abs(bounded) > threshold else { return 0 }
    return (bounded < 0 ? -1 : 1) * (abs(bounded) - threshold) / (1 - threshold)
}
