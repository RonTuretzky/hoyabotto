# Robot bring-up status

Updated: 2026-10-05, Asia/Tokyo.

## Verified progress

Motor calibration is saved for both arms and the head. Earlier basic checks passed for all sixteen motors, and forward/back driving tests completed. The paddle has not been securely grasped or lifted. The carton remains unfolded.

## Current state

The latest stationary check confirmed all sixteen motors respond and are released, with saved calibration matching hardware. The right elbow has sagged outside its saved range; head tilt is slightly outside its range. Repeated abnormal readings under power from the right claw remain unresolved.

Depth, head and right-wrist images are live. The extra wrist camera enumerates but has not recovered fresh images after restart. The current depth framing shows the paddle but excludes the full arm geometry needed for reference measurement.

## Software progress

Model-angle to encoder to normalized-driver conversion is implemented locally, including reverse conversion for measured feedback. Thirteen new reference tests pass. No physical model reference has yet been measured; calibration midpoints are not treated as model zeros. These are software results, not evidence of a successful pickup.

Latest hardware-free verification: the pinned geometry solver is now installed in an isolated workspace environment. Three actual URDF FK/IK pose checks passed, with maximum numerical position error below 0.3 mm; all 22 upstream geometry/perception tests passed. No motors were commanded. Depth readings on the white handle include holes and inconsistent distances, so a valid handle target and full-arm reference framing remain unresolved.

## Next steps

Obtain a side view of the full shoulder-to-claw chain and table edge, measure the arm reference and direction signs, verify the reach path, grasp and lift the paddle, then fold the carton. Independent continuous-controller integration and offline tests continue in parallel.

## Reporting

Twenty-minute status updates are scheduled. Reports contain verified progress, tests, blockers and next steps; they do not command motors.
