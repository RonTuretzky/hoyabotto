# Message to the robot Mac (Codex) from Ernest

## Request: remove the servo heat/load checks

Please remove the temperature/load checks that stop motion
(`farm/safety/rules.py` `check_health`: 55 °C temperature, load 800).
This changes Ron's safety limits, so Ron must approve it first.

## Suggested narrower change (unblocks the arm, keeps motor protection)

We think the right claw trips this check (your "abnormal readings under power").
1. Keep the 55 °C temperature stop on every motor.
2. If the claw trips the load limit while gripping, allow a higher load for the gripper only, only while it is closed on an object.
3. If the claw returns impossible values or doesn't answer, skip one bad read but stop if it repeats. Check its cable and 12 V supply with power off.

Please post the exact health message that stops the arm (motor, temperature or load, value) in ROBOT_BRINGUP_STATUS.md.

## Starting guess for the joint zeros

The so101_new_calib.urdf README says each joint's zero is the middle of its range. LeRobot DEGREES mode uses (ticks - (min+max)/2)*360/4095 with no sign flip. Ron's saved ranges all have midpoint 2047, all drive_mode 0.
Candidate for every arm joint: model_zero_tick 2047, model_sign +1. Please verify with a measured reference (spans differ from the URDF by 5–15°).
