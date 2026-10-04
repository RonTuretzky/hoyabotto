// swift-tools-version: 5.9
import PackageDescription

let package = Package(
    name: "MacJoyConReader",
    platforms: [.macOS(.v13)],
    products: [.executable(name: "MacJoyConReader", targets: ["MacJoyConReader"])],
    targets: [
        .target(name: "JoyConInput"),
        .executableTarget(name: "MacJoyConReader", dependencies: ["JoyConInput"]),
        .testTarget(name: "JoyConInputTests", dependencies: ["JoyConInput"]),
    ]
)
