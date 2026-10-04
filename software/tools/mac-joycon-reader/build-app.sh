#!/bin/bash
set -euo pipefail
reader_root="$(cd -- "$(dirname -- "$0")" && pwd)"
swift build --package-path "$reader_root" -c release >&2
reader_bin_dir="$(swift build --package-path "$reader_root" -c release --show-bin-path)"
reader_app="$reader_root/dist/Joy-Con Reader.app"
mkdir -p "$reader_app/Contents/MacOS"
cp "$reader_bin_dir/MacJoyConReader" "$reader_app/Contents/MacOS/MacJoyConReader"
cat > "$reader_app/Contents/Info.plist" <<'PLIST'
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
  <key>CFBundleIdentifier</key><string>org.xlerobot.joycon-reader</string>
  <key>CFBundleName</key><string>Joy-Con Reader</string>
  <key>CFBundleExecutable</key><string>MacJoyConReader</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleShortVersionString</key><string>0.1.0</string>
  <key>CFBundleVersion</key><string>1</string>
  <key>LSMinimumSystemVersion</key><string>13.0</string>
  <key>NSHighResolutionCapable</key><true/>
  <key>GCSupportsControllerUserInteraction</key><true/>
  <key>GCSupportedGameControllers</key><array>
    <dict><key>ProfileName</key><string>ExtendedGamepad</string></dict>
    <dict><key>ProfileName</key><string>MicroGamepad</string></dict>
  </array>
</dict></plist>
PLIST
# Local development signature only; this is not an Apple-notarized distribution.
codesign --force --sign - "$reader_app" >&2
printf '%s\n' "$reader_app"
