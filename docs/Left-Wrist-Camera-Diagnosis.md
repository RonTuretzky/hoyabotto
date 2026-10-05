# Left-wrist camera diagnosis

Camera identity: `0x12140005a39230`.

The head/right-wrist feeds were previously live. A dedicated left capture briefly produced advancing frames, then froze. A diagnostic run with the right-wrist camera disabled kept the head delivering approximately five frames per second, but the left camera delivered zero callbacks for roughly seventy seconds. Timestamp rejection count was zero. This is capture failure, not the controller rejecting old video.

Actual negotiated head and left settings in that run: 640 × 480 YUV, minimum and maximum frame duration 0.2 seconds. A 320 × 240 / 5fps mode is advertised. Testing that lower mode encountered a disappearing device identity, followed by an AVFoundation “Cannot Use” device error. The device reappeared in inventory, but the subsequent final attempt also failed before delivering frames. No thirty-frame recovery was established.

The OAK publisher was also found to have reached its timed capture limit and is being restarted. Shared USB bandwidth is not confirmed as the cause; disabling the unused right feed did not recover the left. The observations support checking the camera, its USB data cable, hub path, or macOS camera driver state.

Minimal next physical action: unplug and reconnect only the left wrist camera USB data cable. Prefer a direct Mac connection temporarily to isolate the hub. Keep its cable slack. Do not change the motor power wiring for this camera check. After it reappears, require at least thirty consecutive fresh frames with advancing source timestamps before relying on it.

No motors were commanded and no calibration was changed during this diagnosis.
