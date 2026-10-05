// macOS two-camera publisher. Build: swiftc capture.swift -o capture
// No motors, network, model calls, permission changes, or hidden device selection.
import AVFoundation
import CoreImage
import CryptoKit
import Foundation
import ImageIO
import UniformTypeIdentifiers

func fail(_ message: String) -> Never {
    FileHandle.standardError.write(Data((message + "\n").utf8)); exit(1)
}
func fourCC(_ n: FourCharCode) -> String {
    String(bytes: [UInt8((n >> 24) & 255), UInt8((n >> 16) & 255), UInt8((n >> 8) & 255), UInt8(n & 255)], encoding: .ascii) ?? "unknown"
}
let discovery = AVCaptureDevice.DiscoverySession(deviceTypes: [.external, .builtInWideAngleCamera], mediaType: .video, position: .unspecified)
let args = Array(CommandLine.arguments.dropFirst())
if args == ["--list"] {
    let devices = discovery.devices.map { ["name": $0.localizedName, "camera_id": $0.uniqueID] }
    print(String(data: try! JSONSerialization.data(withJSONObject: devices, options: [.prettyPrinted]), encoding: .utf8)!)
    exit(0)
}
if args.count != 2 { fail("Usage: capture OUTPUT_DIR head=EXACT_DEVICE_ID right_wrist=EXACT_DEVICE_ID (or left_wrist); capture --list") }
if AVCaptureDevice.authorizationStatus(for: .video) != .authorized {
    fail("Camera permission is not authorized for this application. Run from the robot Mac's already-authorized Terminal.")
}
let directory = URL(fileURLWithPath: args[0], isDirectory: true)
try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
let streamID = UUID().uuidString

final class Sink: NSObject, AVCaptureVideoDataOutputSampleBufferDelegate {
    let name: String
    let device: AVCaptureDevice
    let context = CIContext()
    var seq = 0
    var skipped = 0
    var lastPTS: CMTime?
    var files: [URL] = []
    init(_ name: String, _ device: AVCaptureDevice) { self.name = name; self.device = device }
    func captureOutput(_ output: AVCaptureOutput, didOutput sample: CMSampleBuffer, from connection: AVCaptureConnection) {
        // Convert sample PTS (host-clock time) to wall time. A queued old buffer
        // must not get a falsely fresh timestamp merely because it arrived now.
        let pts = CMSampleBufferGetPresentationTimeStamp(sample)
        if !pts.isValid || (lastPTS != nil && CMTimeCompare(pts, lastPTS!) <= 0) { return }
        let host = CMClockGetTime(CMClockGetHostTimeClock())
        let age = CMTimeGetSeconds(CMTimeSubtract(host, pts))
        if !age.isFinite || age < 0 || age > 2 { skipped += 1; if skipped % 15 == 1 { print("Timestamp rejection", name, "age", age, "PTS", CMTimeGetSeconds(pts), "host", CMTimeGetSeconds(host)); fflush(stdout) }; return }
        lastPTS = pts
        let capturedAt = Date().timeIntervalSince1970 - age
        guard let pixel = CMSampleBufferGetImageBuffer(sample) else { return }
        let source = CIImage(cvPixelBuffer: pixel)
        guard let cg = context.createCGImage(source, from: source.extent) else { return }
        let encoded = NSMutableData()
        guard let destination = CGImageDestinationCreateWithData(encoded, UTType.jpeg.identifier as CFString, 1, nil) else { return }
        CGImageDestinationAddImage(destination, cg, [kCGImageDestinationLossyCompressionQuality: 0.9] as CFDictionary)
        guard CGImageDestinationFinalize(destination) else { return }
        seq += 1
        let data = encoded as Data
        let imageName = "\(name)-\(streamID)-\(seq).jpg"
        let imageURL = directory.appendingPathComponent(imageName)
        let hash = SHA256.hash(data: data).map { String(format: "%02x", $0) }.joined()
        let format = device.activeFormat.formatDescription
        let manifest: [String: Any] = [
            "schema": 1, "camera_id": device.uniqueID, "stream_id": streamID,
            "seq": seq, "captured_at": capturedAt, "received_at": Date().timeIntervalSince1970,
            "image": imageName, "sha256": hash, "width": cg.width, "height": cg.height,
            "device_format_fourcc": fourCC(CMFormatDescriptionGetMediaSubType(format)),
            "requested_fps": 5, "active_min_frame_s": CMTimeGetSeconds(device.activeVideoMinFrameDuration),
            "sample_pts_s": CMTimeGetSeconds(pts)
        ]
        do {
            try data.write(to: imageURL, options: .atomic)
            // Compatibility image for the existing viewer; control reads ONLY
            // the immutable image named in the manifest below.
            try data.write(to: directory.appendingPathComponent(name + ".jpg"), options: .atomic)
            let json = try JSONSerialization.data(withJSONObject: manifest, options: [.prettyPrinted, .sortedKeys])
            try json.write(to: directory.appendingPathComponent(name + ".json"), options: .atomic)
            files.append(imageURL)
            while files.count > 12 { try? FileManager.default.removeItem(at: files.removeFirst()) }
        } catch { fail("Frame publication failed: \(error)") }
    }
}
var sessions: [AVCaptureSession] = []
var sinks: [Sink] = []
var identities = Set<String>()
var names = Set<String>()
for item in args.dropFirst() {
    let pair = item.split(separator: "=", maxSplits: 1).map(String.init)
    if pair.count != 2 || !["head", "right_wrist", "left_wrist", "aux"].contains(pair[0]) || !names.insert(pair[0]).inserted || !identities.insert(pair[1]).inserted {
        fail("Use two distinct, explicitly named cameras")
    }
    guard let device = discovery.devices.first(where: { $0.uniqueID == pair[1] }) else { fail("Camera identity not found: \(pair[1])") }
    let formats = device.formats.filter {
        let d = CMVideoFormatDescriptionGetDimensions($0.formatDescription)
        return d.width == 640 && d.height == 480 && $0.videoSupportedFrameRateRanges.contains(where: { $0.minFrameRate <= 5 && $0.maxFrameRate >= 5 })
    }
    guard let format = formats.first(where: { fourCC(CMFormatDescriptionGetMediaSubType($0.formatDescription)) == "jpeg" }) ?? formats.first else {
        fail("Camera does not advertise 640x480 at 5 fps; inspect its formats before selecting another mode")
    }
    let session = AVCaptureSession()
    session.beginConfiguration()
    let input = try AVCaptureDeviceInput(device: device)
    guard session.canAddInput(input) else { fail("Cannot add camera input") }
    session.addInput(input)
    try device.lockForConfiguration()
    device.activeFormat = format
    device.activeVideoMinFrameDuration = CMTime(value: 1, timescale: 5)
    device.activeVideoMaxFrameDuration = CMTime(value: 1, timescale: 5)
    device.unlockForConfiguration()
    let output = AVCaptureVideoDataOutput()
    output.alwaysDiscardsLateVideoFrames = true
    output.videoSettings = [kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA]
    let sink = Sink(pair[0], device)
    output.setSampleBufferDelegate(sink, queue: DispatchQueue(label: "carton.capture.\(pair[0])"))
    guard session.canAddOutput(output) else { fail("Cannot add camera output") }
    session.addOutput(output)
    session.commitConfiguration()
    session.startRunning()
    // AVFoundation may reset active format/duration when a session starts.
    try device.lockForConfiguration()
    device.activeFormat = format
    device.activeVideoMinFrameDuration = CMTime(value: 1, timescale: 5)
    device.activeVideoMaxFrameDuration = CMTime(value: 1, timescale: 5)
    device.unlockForConfiguration()
    sessions.append(session); sinks.append(sink)
}
// Single-camera diagnostic capture; other cameras retain their existing owners.
signal(SIGINT, SIG_IGN); signal(SIGTERM, SIG_IGN)
let interrupt = DispatchSource.makeSignalSource(signal: SIGINT, queue: .main)
let terminate = DispatchSource.makeSignalSource(signal: SIGTERM, queue: .main)
let stop = { for session in sessions { session.stopRunning() }; exit(0) }
interrupt.setEventHandler(handler: stop); terminate.setEventHandler(handler: stop)
interrupt.resume(); terminate.resume()
print("Publishing two coherent streams at requested 640x480/5fps. No motor connection.")
RunLoop.main.run()
