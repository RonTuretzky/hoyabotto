#!/bin/bash
set -euo pipefail
reader_root="$(cd -- "$(dirname -- "$0")" && pwd)"
if ! xcrun --find swift >/dev/null 2>&1; then
    echo 'Apple Command Line Tools are required. Run: xcode-select --install' >&2
    exit 1
fi
swift build --package-path "$reader_root" -c release >&2
reader_bin_dir="$(swift build --package-path "$reader_root" -c release --show-bin-path)"
exec "$reader_bin_dir/MacJoyConReader" "$@"
