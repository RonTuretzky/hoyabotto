# Demo arm speed

Choose the speed before enabling an arm. `normal` stays the default.

- Joy-Con: select **Arm speed → Demo · up to26°/s**, then Arm. Keep using the
  matching rail deadman. Stop before changing the selection.
- Pilot/chat: on an explicitly requested free-space demo, call
  `robot_set_motor_enable` with all six names for the chosen arm, `enabled:true`
  and `speed_profile:"demo"`. Direct/waypoint arm commands then use a maximum
  300 ticks/s (26.37degrees/s). Request a short duration if the demo needs that
  rate; longer requested durations still produce slower paths. Inspect actual
  movement; the limit is not a guarantee of achieved physical speed.
- Release the arm and re-enable with `speed_profile:"normal"` (or omit the field)
  to return to100 ticks/s at the owner. Joy-Con's normal requested rate stays80.
- Grippers, head and wheels retain their prior settings. The40tick arm ramp,
  acceleration, torque/load, travel, following-error, contact, camera, watchdog,
  deadman and STOP checks remain in place. Trained policy streaming requires normal.
- No faster physical wave has been validated by the software tests. Carton contact
  tasks retain normal speed and require their usual observation/calibration evidence.
