# Robot bring-up status

Updated: 2026-10-05 15:56 JST (Asia/Tokyo).

## Verified physical progress

Motor calibration remains saved for both arms and the head. Earlier basic movement checks passed for all sixteen motors, and earlier forward/back drive tests completed.

A new isolated left-claw opening moved from encoder tick 1342 to 1393, approximately 4.5 degrees. Its requested endpoint was 1410, so the endpoint check failed by 17 ticks. Recorded temperature was 37 degrees Celsius, with normal voltage and no motor fault flag. Torque-off and release of all left-board motors were verified afterward. The other board was not commanded during this test. This is confirmed claw movement, not a confirmed grip. No paddle grasp, lift or carton fold has been verified.

## Software progress

Explicit head-plus-OAK support is implemented locally in preparation, camera audit, packaged motor-owner readers, continuous-controller binding and grasp observation. OAK retains its actual identity and aligned metric depth; it is not relabeled as a wrist camera. Fixed-camera retention uses a metric paddle-to-tool displacement rather than wrist-relative image coordinates. No camera-to-arm transform or physical reference was invented.

The helper reported 126 focused tests passed, compilation passed and whitespace checks clean. Tests include left-arm preparation and synthetic two-camera pickup observations, STOP, depth loss, camera movement and dropped-object rejection. These are software tests; they do not establish a physical grasp or deployed full-arm reach.

The previously verified reference conversion and guarded controller integration remain committed locally as e8d369c. The isolated pinned geometry solver passed three numerical URDF pose checks with maximum position error below 0.3 mm, and 22 upstream geometry/perception tests passed. Source publication remains pending exact-payload approval; this automation publishes only this report.

## Cameras and current blockers

The user selected head plus OAK depth operation; right-wrist capture was disabled. Both selected feeds were confirmed fresh at the latest image check. The left-wrist capture fault remains unresolved but is not a prerequisite for this selected two-camera mode.

The latest reviewed images showed the user's hand beside the left claw and paddle. No further movement was started from that image. Handle occlusion and missing measured jaw baselines, model reference angles/direction signs and camera-to-arm registration still prevent a validated planned pickup. Earlier powered right-claw temperature anomalies remain unresolved. Thermal protections have not been removed or raised.

## Next steps

Refresh both images before any motion; measure an unobstructed claw/handle reference and opening baseline. Complete actual-runner integration without duplicate motor owners, record the real arm's model references and direction signs, establish the camera/station relationship, then validate a bounded approach, grip and lift. Continue to distinguish numerical solver success, motor movement and confirmed object pickup.

This reporting run issued no motor commands and changed no calibration or motor settings.
