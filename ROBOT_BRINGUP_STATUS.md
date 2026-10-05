# Robot bring-up status

Updated: 2026-10-05 15:36 JST (Asia/Tokyo).

## Verified physical progress

Motor calibration remains saved for both arms and the head. Earlier basic checks passed for all sixteen motors and earlier forward/back drive tests completed. No new autonomous motor movement, grasp, lift or carton fold has been verified since the previous report. The paddle remains ungrasped. This reporting run sent no motor commands and changed no calibration.

## Software progress

Measured-reference angle/encoder/normalized-unit conversion and guarded continuous-controller integration remain committed locally as e8d369c. The focused controller/reference suite passed 109 tests. The isolated pinned geometry solver passed three actual URDF numerical pose checks, with maximum numerical position error below 0.3 mm; 22 upstream geometry/perception tests passed. These are software results, not physical pickup evidence. Source publication remains pending exact-payload approval; only this status file is published by this automation.

## New camera diagnosis

The left-wrist camera briefly produced fresh frames after a restart, then froze. With the unused right-wrist producer disabled, the head continued at approximately five frames per second while the left camera delivered zero callbacks for roughly seventy seconds. No timestamp rejections occurred. The negotiated mode was 640 by 480 YUV with minimum and maximum frame duration 0.2 seconds, so this was not merely application-level frame throttling.

The camera advertises a lower 320 by 240 / five-fps mode. Attempts to use it encountered an intermittently missing device and a macOS camera-device error before frame delivery. Disabling the right feed did not recover the left; shared USB bandwidth is a hypothesis, not a confirmed cause. No sustained thirty-frame recovery has been verified.

The head and right-wrist publishers were restored. The depth publisher had reached its timed capture limit and was restarted. All three were confirmed fresh at the last camera check; this is not a claim of a new check during the reporting run.

## Current blockers and next steps

Reconnect only the left wrist camera USB data cable, preferably directly to the Mac temporarily to isolate the hub. Leave motor-power wiring unchanged. After reconnecting, verify at least thirty consecutive fresh frames with advancing source timestamps before relying on the stream. Use head/wrist views for jaw geometry and depth for distances rather than requiring one view to show everything.

Refresh scene references after camera or arm changes, establish a measured local approach or physical model reference, verify clearance, then attempt a bounded paddle grasp and lift. The earlier powered right-claw reading anomalies and uncertain narrow-handle depth remain unresolved.
