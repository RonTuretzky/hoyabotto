import SwiftUI
import JoyConInput

final class ReaderDisplay: ObservableObject {
    @Published var frame: InputFrame?
}

struct ReaderView: View {
    @ObservedObject var display: ReaderDisplay
    let demo: Bool

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 18) {
                Text("Joy-Con input reader").font(.largeTitle.bold())
                Text(demo ? "DEMO — synthetic inputs, no hardware" : "LIVE — controllers connected to this Mac")
                    .font(.headline)
                Text("Input display only. Robot motion is not connected.")
                if let frame = display.frame {
                    let connected = frame.controllers.filter(\.connected)
                    Text("\(connected.count) controller profile(s) • Sample \(frame.sequence)")
                        .font(.system(.body, design: .monospaced))
                    if connected.isEmpty {
                        VStack(alignment: .leading, spacing: 10) {
                            Text("Waiting for Joy-Cons").font(.title2.bold())
                            Text("Open System Settings → Bluetooth. Hold the small SYNC button between SL and SR until the lights flash, then connect Joy-Con (L) and Joy-Con (R).")
                            Text("Press buttons, move each stick, then tilt each controller. Values below should change. A Bluetooth connection alone does not verify inputs.")
                        }.padding().background(.quaternary).cornerRadius(12)
                    }
                    ForEach(frame.controllers) { controller in
                        ControllerCard(controller: controller)
                    }
                } else {
                    Text("Starting the native macOS reader…")
                }
                Text("Names are provided by macOS. Nintendo button labels and macOS aliases can differ; verify by pressing each physical button. A combined pair may expose only one motion stream.")
                    .font(.callout).foregroundStyle(.secondary)
                Text("Close this window or press ⌘Q to stop the reader.").font(.callout)
            }.padding(24).frame(maxWidth: .infinity, alignment: .leading)
        }.frame(minWidth: 740, minHeight: 620)
    }
}

private struct ControllerCard: View {
    let controller: ControllerState
    private let columns = [GridItem(.adaptive(minimum: 190), alignment: .leading)]

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("\(controller.role.rawValue.uppercased()) • \(controller.name)").font(.title2.bold())
            Text("\(controller.connected ? "CONNECTED" : "DISCONNECTED — inputs cleared") • \(controller.category)")
            if controller.role == .pair {
                Text("macOS reports one combined pair. Independent left/right wrist motion has not been verified.").bold()
            } else if controller.role == .unknown {
                Text("Side is unknown. No left/right assignment has been assumed.").bold()
            }
            if controller.remapped { Text("System button remapping is active.").bold() }
            HStack {
                Text(controller.batteryLevel.map { "Battery \(Int($0 * 100))%" } ?? "Battery unavailable")
                Text("• Input changes: \(controller.inputEventCount)")
            }.font(.system(.callout, design: .monospaced))
            ForEach(controller.pads.keys.sorted(), id: \.self) { name in
                if let pad = controller.pads[name] {
                    Text(String(format: "%@: X %+.3f  Y %+.3f", name, pad.x.filtered, pad.y.filtered))
                        .font(.system(.body, design: .monospaced))
                }
            }
            LazyVGrid(columns: columns, alignment: .leading, spacing: 8) {
                ForEach(controller.buttons.keys.sorted(), id: \.self) { name in
                    if let button = controller.buttons[name] {
                        VStack(alignment: .leading, spacing: 3) {
                            Text(name).bold()
                            Text(button.pressed ? "PRESSED" : "Released")
                            if !button.physicalNames.isEmpty {
                                Text(button.physicalNames.joined(separator: ", ")).font(.caption)
                            }
                        }.padding(8).frame(maxWidth: .infinity, alignment: .leading)
                            .background(button.pressed ? Color.primary.opacity(0.18) : Color.primary.opacity(0.04))
                            .overlay(RoundedRectangle(cornerRadius: 6).stroke(lineWidth: button.pressed ? 2 : 0.5))
                    }
                }
            }
            if let motion = controller.motion {
                Text("Motion \(motion.sensorsActive ? "enabled" : "inactive") • Updates: \(motion.eventCount)").bold()
                if let q = motion.attitude {
                    Text(String(format: "Attitude quaternion  x %.3f  y %.3f  z %.3f  w %.3f", q.x, q.y, q.z, q.w))
                        .font(.system(.callout, design: .monospaced))
                } else { Text("Attitude unavailable") }
                if let r = motion.rotationRate {
                    Text(String(format: "Rotation rad/s       x %.3f  y %.3f  z %.3f", r.x, r.y, r.z))
                        .font(.system(.callout, design: .monospaced))
                } else { Text("Rotation rate unavailable") }
                if motion.eventCount == 0 { Text("Waiting for the first motion event; move this controller to verify.") }
            } else {
                Text("Motion unavailable through macOS for this profile.")
            }
        }.padding(16).frame(maxWidth: .infinity, alignment: .leading)
            .background(.quaternary).cornerRadius(12)
    }
}
