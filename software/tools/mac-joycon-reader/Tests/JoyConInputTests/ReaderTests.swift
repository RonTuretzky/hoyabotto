import XCTest
import GameController
@testable import JoyConInput

final class ReaderTests: XCTestCase {
    func testRoleNeverAssignsUnrecognizedHardwareToAnArm() {
        XCTAssertEqual(ControllerRole.classify(name: "Joy-Con (L)", category: ""), .left)
        XCTAssertEqual(ControllerRole.classify(name: "Joy-Con (R)", category: ""), .right)
        XCTAssertEqual(ControllerRole.classify(name: "Nintendo Switch Joy-Con (L/R)", category: ""), .pair)
        XCTAssertEqual(ControllerRole.classify(name: "", category: "NintendoSwitchJoyConPair"), .pair)
        XCTAssertEqual(ControllerRole.classify(name: "", category: "NintendoSwitchJoyConLeft"), .left)
        XCTAssertEqual(ControllerRole.classify(name: "Joy-Con", category: ""), .unknown)
        XCTAssertEqual(ControllerRole.classify(name: "Left Xbox Controller", category: ""), .unknown)
    }

    func testDeadzoneSuppressesDriftPreservesEndpointsAndRejectsNonfinite() {
        XCTAssertEqual(deadzone(0.07, threshold: 0.08), 0)
        XCTAssertEqual(deadzone(-0.08, threshold: 0.08), 0)
        XCTAssertEqual(deadzone(1, threshold: 0.08), 1)
        XCTAssertEqual(deadzone(-1, threshold: 0.08), -1)
        XCTAssertEqual(deadzone(0.54, threshold: 0.08), 0.5, accuracy: 0.0001)
        XCTAssertEqual(deadzone(.nan, threshold: 0.08), 0)
        XCTAssertEqual(deadzone(.infinity, threshold: 0.08), 0)
        XCTAssertEqual(deadzone(0.5, threshold: 1), 0)
    }

    func testAppleExtendedProfileReadsPressedButtonsSticksAndRelease() {
        let controller = GCController.withExtendedGamepad()
        let gamepad = controller.extendedGamepad!
        gamepad.buttonA.setValue(1)
        gamepad.leftThumbstick.setValueForXAxis(0.54, yAxis: -1)
        let state = captureController(controller, id: "fixture", threshold: 0.08)
        XCTAssertTrue(state.buttons[GCInputButtonA]!.pressed)
        XCTAssertEqual(state.pads[GCInputLeftThumbstick]!.x.filtered, 0.5, accuracy: 0.0001)
        XCTAssertEqual(state.pads[GCInputLeftThumbstick]!.y.filtered, -1)
        gamepad.buttonA.setValue(0)
        let released = captureController(controller, id: "fixture", threshold: 0.08)
        XCTAssertFalse(released.buttons[GCInputButtonA]!.pressed)
        // Earlier samples are immutable snapshots, not references to mutable inputs.
        XCTAssertTrue(state.buttons[GCInputButtonA]!.pressed)
    }

    func testMicroProfileIsReadWithoutRequiringExtendedGamepad() {
        let controller = GCController.withMicroGamepad()
        controller.microGamepad!.buttonA.setValue(1)
        let state = captureController(controller, id: "micro", threshold: 0.08)
        XCTAssertTrue(state.buttons.values.contains { $0.pressed })
        XCTAssertFalse(state.pads.isEmpty)
        XCTAssertEqual(state.role, .unknown)
    }

    func testDisconnectClearsHeldButtonsSticksAndMotion() {
        let state = demoFrame(sequence: 10, sessionID: "test").controllers[0]
        XCTAssertTrue(state.buttons.values.contains { $0.pressed })
        let disconnected = state.disconnected()
        XCTAssertEqual(disconnected.id, state.id)
        XCTAssertFalse(disconnected.connected)
        XCTAssertTrue(disconnected.buttons.isEmpty)
        XCTAssertTrue(disconnected.axes.isEmpty)
        XCTAssertTrue(disconnected.pads.isEmpty)
        XCTAssertNil(disconnected.motion)
    }

    func testJSONStreamIdentifiesDemoAndHasSessionSequenceAndTime() throws {
        let frame = demoFrame(sequence: 10, sessionID: "test-session")
        let data = try frame.jsonLine()
        XCTAssertEqual(data.last, 10)
        let object = try XCTUnwrap(JSONSerialization.jsonObject(with: data) as? [String: Any])
        XCTAssertEqual(object["source"] as? String, "demo")
        XCTAssertEqual(object["input_only"] as? Bool, true)
        XCTAssertEqual(object["schema_version"] as? Int, 1)
        XCTAssertEqual(object["session_id"] as? String, "test-session")
        XCTAssertEqual(object["sequence"] as? Int, 10)
        XCTAssertNotNil(object["uptime"])
        XCTAssertEqual((object["controllers"] as? [Any])?.count, 2)
        let stopped = demoFrame(sequence: 11, sessionID: "test-session", stopping: true)
        XCTAssertEqual(stopped.event, "shutdown")
        XCTAssertTrue(stopped.controllers.allSatisfy { !$0.connected && $0.buttons.isEmpty && $0.motion == nil })
    }

    func testReaderDisconnectReconnectionAndShutdownLifecycle() {
        let controller = GCController.withExtendedGamepad()
        controller.extendedGamepad!.buttonA.setValue(1)
        var attached = [controller]
        let reader = ControllerReader(controllers: { attached })
        var frames: [InputFrame] = []
        reader.onFrame = { frames.append($0) }
        reader.start()
        let original = frames.last!.controllers[0]
        attached = []
        reader.reconcile()
        XCTAssertEqual(frames.last!.event, "disconnect")
        XCTAssertEqual(frames.last!.controllers[0].id, original.id)
        XCTAssertFalse(frames.last!.controllers[0].connected)
        XCTAssertTrue(frames.last!.controllers[0].buttons.isEmpty)
        reader.reconcile()
        XCTAssertTrue(frames.last!.controllers.isEmpty)
        attached = [controller]
        reader.reconcile()
        XCTAssertNotEqual(frames.last!.controllers[0].id, original.id)
        reader.stop()
        XCTAssertEqual(frames.last!.event, "shutdown")
        XCTAssertTrue(frames.last!.controllers.allSatisfy { !$0.connected })
        XCTAssertEqual(frames.map(\.sequence), Array(1...UInt64(frames.count)))
        XCTAssertEqual(Set(frames.map(\.sessionID)).count, 1)
    }
}
