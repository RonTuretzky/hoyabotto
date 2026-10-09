import Foundation

// Deliberately synthetic, always labelled source=demo. No controller API involved.
public func demoFrame(sequence: UInt64, sessionID: String, stopping: Bool = false) -> InputFrame {
    let phase = Double(sequence) / 20
    let controllers: [ControllerState] = [ControllerRole.left, .right].map { role in
        let pressed = sin(phase) > 0
        let x = Float(sin(phase))
        let y = Float(cos(phase))
        let controller = ControllerState(
            id: "demo-\(role.rawValue)", name: "DEMO Joy-Con (\(role == .left ? "L" : "R"))",
            category: "Synthetic fixture", role: role, connected: true, remapped: false,
            batteryLevel: 0.75, batteryState: "discharging",
            buttons: ["Button A": ButtonState(value: pressed ? 1 : 0, pressed: pressed,
                                              physicalNames: ["Demo button"])],
            axes: [:], pads: ["Thumbstick": PadState(x: AxisState(raw: x, filtered: deadzone(x, threshold: 0.08)),
                                                    y: AxisState(raw: y, filtered: deadzone(y, threshold: 0.08)))],
            motion: MotionState(sensorsActive: true,
                                attitude: Quaternion(x: 0, y: sin(phase / 2), z: 0, w: cos(phase / 2)),
                                rotationRate: Vector3(0, 1, 0), gravity: Vector3(0, -1, 0),
                                userAcceleration: Vector3(0, 0, 0), acceleration: Vector3(0, -1, 0),
                                eventCount: sequence, lastEventUptime: ProcessInfo.processInfo.systemUptime),
            inputEventCount: sequence, lastInputEventUptime: ProcessInfo.processInfo.systemUptime)
        return stopping ? controller.disconnected() : controller
    }
    return InputFrame(source: "demo", sessionID: sessionID, sequence: sequence,
                      event: stopping ? "shutdown" : "sample", controllers: controllers)
}
