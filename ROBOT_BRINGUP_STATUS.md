# Robot bring-up status

Updated: 2026-10-05 15:16 JST (Asia/Tokyo).

## Verified physical progress

Motor calibration is saved for both arms and the head. Earlier basic checks passed for all sixteen motors, and earlier forward/back driving tests completed. No new autonomous motor movement has been verified since the previous report. The paddle has not been securely grasped or lifted; the carton remains unfolded.

The latest recorded read-only check confirmed all sixteen motors released and both arms inside their saved ranges after the right arm was repositioned. This is the last verified state, not a claim of a fresh hardware check during this reporting run. The earlier powered right-claw reading anomalies remain unresolved.

## Software progress

Model-angle/encoder/normalized-driver conversion is implemented with an explicit measured-reference requirement. The continuous-controller integration passed 109 focused tests, including interruption, visual-tracking, encoder and fault handling. The isolated pinned geometry solver passed three actual URDF numerical pose checks, with maximum position error below 0.3 mm; 22 upstream geometry/perception tests also passed. These are software results, not physical pickup evidence.

Implementation is committed locally as e8d369c. Public source upload is pending approval of the exact source-file payload and destination; this status report does not publish that code.

## New scene findings

Fresh camera observations show the right claw near the paddle, partly obscuring the blade. Strict depth checks do not corroborate the earlier jaw-tip/base readings as reliable tool-surface measurements. The known paddle dimensions and partly occluded outline do not yet provide a bounded pose estimate. The healthy left claw and its approach route are not visible in the current depth view.

## Current blockers and next steps

Obtain one wider oblique view containing the left shoulder, elbow, wrist and open claw, the entire paddle including its handle, and the nearby table edge/apron. Then establish the measured arm reference and camera relationship or a verified local visual-control response, check the approach path, and attempt a bounded grasp and lift. Keep the right-claw anomaly unresolved until diagnosed. No motor commands or calibration changes are part of this reporting automation.
