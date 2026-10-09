import AppKit
import SwiftUI
import Darwin
import JoyConInput

struct Options {
    var json = false
    var demo = false
    var seconds: Double?
    var hz: Double = 30
    var deadzone: Float = 0.08

    static let help = """
    MacJoyConReader [--json] [--demo] [--seconds N] [--hz N] [--deadzone N]
      Default: native input display; no robot connection.
      --json        JSON Lines on stdout (diagnostics on stderr)
      --demo        Synthetic inputs labelled source=demo; no controller access
      --seconds N   Exit after N seconds, with a final shutdown frame
      --hz N        Sample frequency, 1–120 (default 30)
      --deadzone N  Per-axis deadzone, 0–0.95 (default 0.08)
    """

    init(_ arguments: [String]) throws {
        var i = 0
        while i < arguments.count {
            let argument = arguments[i]
            switch argument {
            case "--json": json = true
            case "--demo": demo = true
            case "--seconds", "--hz", "--deadzone":
                i += 1
                guard i < arguments.count, let value = Double(arguments[i]), value.isFinite else {
                    throw ReaderError.message("\(argument) requires a finite number")
                }
                if argument == "--seconds" {
                    guard value > 0 else { throw ReaderError.message("--seconds must be positive") }
                    seconds = value
                } else if argument == "--hz" {
                    guard (1...120).contains(value) else { throw ReaderError.message("--hz must be 1–120") }
                    hz = value
                } else {
                    guard (0...0.95).contains(value) else { throw ReaderError.message("--deadzone must be 0–0.95") }
                    deadzone = Float(value)
                }
            case "--help", "-h": print(Self.help); exit(0)
            default: throw ReaderError.message("Unknown argument: \(argument)")
            }
            i += 1
        }
    }
}

enum ReaderError: Error { case message(String) }

final class Application: NSObject, NSApplicationDelegate {
    let options: Options
    let display = ReaderDisplay()
    let reader: ControllerReader
    var window: NSWindow?
    var signals: [DispatchSourceSignal] = []
    var demoTimer: Timer?
    var demoSequence: UInt64 = 0
    let demoSession = UUID().uuidString
    var stopped = false
    var failed = false

    init(_ options: Options) {
        self.options = options
        reader = ControllerReader(threshold: options.deadzone)
    }

    func applicationDidFinishLaunching(_ notification: Notification) {
        if !options.json {
            window = NSWindow(contentRect: NSRect(x: 0, y: 0, width: 860, height: 780),
                              styleMask: [.titled, .closable, .miniaturizable, .resizable],
                              backing: .buffered, defer: false)
            window?.title = "Joy-Con Reader\(options.demo ? " — DEMO" : "")"
            window?.contentView = NSHostingView(rootView: ReaderView(display: display, demo: options.demo))
            window?.center(); window?.makeKeyAndOrderFront(nil)
            let menu = NSMenu()
            let item = NSMenuItem()
            let appMenu = NSMenu()
            appMenu.addItem(withTitle: "Quit Joy-Con Reader", action: #selector(NSApplication.terminate(_:)), keyEquivalent: "q")
            item.submenu = appMenu; menu.addItem(item)
            NSApp.mainMenu = menu
            NSApp.activate(ignoringOtherApps: true)
        }
        for number in [SIGINT, SIGTERM] {
            signal(number, SIG_IGN)
            let source = DispatchSource.makeSignalSource(signal: number, queue: .main)
            source.setEventHandler { NSApp.terminate(nil) }
            source.resume(); signals.append(source)
        }
        signal(SIGPIPE, SIG_IGN)
        if options.demo {
            demoTimer = Timer(timeInterval: 1 / options.hz, repeats: true) { [weak self] _ in
                guard let self = self else { return }
                self.demoSequence += 1
                self.receive(demoFrame(sequence: self.demoSequence, sessionID: self.demoSession))
            }
            RunLoop.main.add(demoTimer!, forMode: .common)
        } else {
            reader.onFrame = { [weak self] in self?.receive($0) }
            reader.start(hz: options.hz)
        }
        if let seconds = options.seconds {
            DispatchQueue.main.asyncAfter(deadline: .now() + seconds) { NSApp.terminate(nil) }
        }
    }

    func receive(_ frame: InputFrame) {
        guard !failed else { return }
        if options.json {
            do { try FileHandle.standardOutput.write(contentsOf: frame.jsonLine()) }
            catch {
                failed = true
                demoTimer?.invalidate()
                reader.stop()
                FileHandle.standardError.write(Data("Cannot write controller stream: \(error)\n".utf8))
                exit(1)
            }
        } else { display.frame = frame }
    }

    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool { true }

    func applicationWillTerminate(_ notification: Notification) {
        guard !stopped else { return }
        stopped = true
        if options.demo {
            demoTimer?.invalidate()
            receive(demoFrame(sequence: demoSequence + 1, sessionID: demoSession, stopping: true))
        } else { reader.stop() }
    }
}

@main
struct Main {
    static func main() {
        do {
            let options = try Options(Array(CommandLine.arguments.dropFirst()))
            let application = NSApplication.shared
            application.setActivationPolicy(options.json ? .accessory : .regular)
            let delegate = Application(options)
            application.delegate = delegate
            withExtendedLifetime(delegate) { application.run() }
        } catch ReaderError.message(let message) {
            FileHandle.standardError.write(Data("\(message)\n\(Options.help)\n".utf8))
            exit(2)
        } catch {
            FileHandle.standardError.write(Data("\(error)\n".utf8)); exit(2)
        }
    }
}
