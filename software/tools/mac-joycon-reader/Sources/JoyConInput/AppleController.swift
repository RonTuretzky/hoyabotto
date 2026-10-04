import Foundation
import GameController

// Both the app and command-line reader use this exact capture path. It does not
// import a robot library, open serial ports, or create a network listener.
public func captureController(_ controller: GCController, id: String, threshold: Float,
                              inputEvents: UInt64 = 0, lastInput: Double? = nil,
                              motionEvents: UInt64 = 0, lastMotion: Double? = nil) -> ControllerState {
    let snapshot = controller.capture()
    let profile = snapshot.physicalInputProfile
    let original = controller.physicalInputProfile
    let name = controller.vendorName ?? "Unnamed controller"
    let category = controller.productCategory
    let buttons = Dictionary(uniqueKeysWithValues: profile.buttons.map { name, input in
        (name, ButtonState(value: input.value, pressed: input.isPressed,
                          physicalNames: original.mappedPhysicalInputNames(forElementAlias: name).sorted()))
    })
    func axis(_ value: Float) -> AxisState {
        AxisState(raw: value, filtered: deadzone(value, threshold: threshold))
    }
    let axes = profile.axes.mapValues { axis($0.value) }
    let pads = profile.dpads.mapValues { PadState(x: axis($0.xAxis.value), y: axis($0.yAxis.value)) }
    var motionState: MotionState?
    if let motion = snapshot.motion, let liveMotion = controller.motion {
        let q = motion.attitude
        let r = motion.rotationRate
        let g = motion.gravity
        let u = motion.userAcceleration
        let a = motion.acceleration
        motionState = MotionState(
            sensorsActive: !liveMotion.sensorsRequireManualActivation || liveMotion.sensorsActive,
            attitude: motion.hasAttitude ? Quaternion(x: q.x, y: q.y, z: q.z, w: q.w) : nil,
            rotationRate: motion.hasRotationRate ? Vector3(r.x, r.y, r.z) : nil,
            gravity: motion.hasGravityAndUserAcceleration ? Vector3(g.x, g.y, g.z) : nil,
            userAcceleration: motion.hasGravityAndUserAcceleration ? Vector3(u.x, u.y, u.z) : nil,
            acceleration: Vector3(a.x, a.y, a.z), eventCount: motionEvents, lastEventUptime: lastMotion)
    }
    let battery = controller.battery
    let batteryLevel = battery.flatMap { $0.batteryLevel >= 0 ? $0.batteryLevel : nil }
    let batteryState: String? = battery.map {
        switch $0.batteryState {
        case .charging: return "charging"
        case .full: return "full"
        case .discharging: return "discharging"
        default: return "unknown"
        }
    }
    return ControllerState(id: id, name: name, category: category,
                           role: .classify(name: name, category: category), connected: true,
                           remapped: original.hasRemappedElements,
                           batteryLevel: batteryLevel, batteryState: batteryState,
                           buttons: buttons, axes: axes, pads: pads, motion: motionState,
                           inputEventCount: inputEvents, lastInputEventUptime: lastInput)
}

public final class ControllerReader {
    public var onFrame: ((InputFrame) -> Void)?
    private let threshold: Float
    private let controllers: () -> [GCController]
    private let sessionID = UUID().uuidString
    private var sequence: UInt64 = 0
    private var records: [ObjectIdentifier: Record] = [:]
    private var observers: [NSObjectProtocol] = []
    private var timer: Timer?
    private var previousBackgroundEvents = false
    private var running = false

    private final class Record {
        let controller: GCController
        let id = UUID().uuidString
        let originalSensorsActive: Bool?
        var inputEvents: UInt64 = 0
        var lastInput: Double?
        var motionEvents: UInt64 = 0
        var lastMotion: Double?

        init(_ controller: GCController) {
            self.controller = controller
            originalSensorsActive = controller.motion?.sensorsActive
        }
    }

    public init(threshold: Float = 0.08, controllers: @escaping () -> [GCController] = GCController.controllers) {
        self.threshold = threshold
        self.controllers = controllers
    }

    // All reads, callbacks, and lifecycle events are serialized on the main queue.
    public func start(hz: Double = 30) {
        precondition(Thread.isMainThread)
        guard !running else { return }
        running = true
        previousBackgroundEvents = GCController.shouldMonitorBackgroundEvents
        GCController.shouldMonitorBackgroundEvents = true
        for notification in [Notification.Name.GCControllerDidConnect, .GCControllerDidDisconnect] {
            observers.append(NotificationCenter.default.addObserver(forName: notification, object: nil, queue: .main) {
                [weak self] _ in self?.reconcile()
            })
        }
        reconcile()
        timer = Timer(timeInterval: 1 / hz, repeats: true) { [weak self] _ in self?.reconcile() }
        RunLoop.main.add(timer!, forMode: .common)
    }

    private func state(_ record: Record) -> ControllerState {
        captureController(record.controller, id: record.id, threshold: threshold,
                          inputEvents: record.inputEvents, lastInput: record.lastInput,
                          motionEvents: record.motionEvents, lastMotion: record.lastMotion)
    }

    private func detach(_ record: Record) {
        record.controller.physicalInputProfile.valueDidChangeHandler = nil
        record.controller.motion?.valueChangedHandler = nil
        if let wasActive = record.originalSensorsActive,
           record.controller.motion?.sensorsRequireManualActivation == true {
            record.controller.motion?.sensorsActive = wasActive
        }
    }

    func reconcile() {
        guard running else { return }
        let controllers = controllers()
        let present = Set(controllers.map(ObjectIdentifier.init))
        var disconnected: [ControllerState] = []
        for key in Array(records.keys) where !present.contains(key) {
            if let record = records.removeValue(forKey: key) {
                disconnected.append(state(record).disconnected())
                detach(record)
            }
        }
        for controller in controllers {
            let key = ObjectIdentifier(controller)
            guard records[key] == nil else { continue }
            let record = Record(controller)
            records[key] = record
            controller.handlerQueue = .main
            controller.physicalInputProfile.valueDidChangeHandler = { [weak record] _, _ in
                record?.inputEvents += 1
                record?.lastInput = ProcessInfo.processInfo.systemUptime
            }
            controller.motion?.valueChangedHandler = { [weak record] _ in
                record?.motionEvents += 1
                record?.lastMotion = ProcessInfo.processInfo.systemUptime
            }
            if controller.motion?.sensorsRequireManualActivation == true {
                controller.motion?.sensorsActive = true
            }
        }
        emit(event: disconnected.isEmpty ? "sample" : "disconnect", tombstones: disconnected)
    }

    private func emit(event: String, tombstones: [ControllerState] = []) {
        sequence += 1
        let controllers = (records.values.map(state) + tombstones).sorted { $0.id < $1.id }
        onFrame?(InputFrame(source: "live", sessionID: sessionID, sequence: sequence,
                            event: event, controllers: controllers))
    }

    public func stop() {
        guard running else { return }
        running = false
        timer?.invalidate(); timer = nil
        observers.forEach(NotificationCenter.default.removeObserver)
        observers = []
        let final = records.values.map { state($0).disconnected() }
        records.values.forEach(detach)
        records = [:]
        emit(event: "shutdown", tombstones: final)
        GCController.shouldMonitorBackgroundEvents = previousBackgroundEvents
    }
}
