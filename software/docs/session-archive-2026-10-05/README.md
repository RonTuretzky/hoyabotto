# Source actually used on the robot Mac

Historical snapshots from the October 5 session, retained separately from the portable packaged controller under `software/scripts/carton_robot`. They are not installed entry points and must not be launched alongside that sole motor owner.

The work-directory motor owner and isolated left-claw script depend on the original workspace layout and adjacent strict-reply/telemetry helpers. The packaged owner was prepared and tested but had not replaced the historical owner at handoff. The isolated claw test moved 51 ticks and failed its requested endpoint; it did not grasp the paddle. These files preserve that distinction rather than implying deployment.

The capture and viewer sources were used for head/OAK framing. The single-camera Swift publisher accepts an output directory and one explicit named device selection; its inherited usage string mentions two selections. The phone helper generates its own configuration/certificates. No runtime config, authorization token, certificate, key, image, binary or motor log is included. The viewer and phone server are no longer running.

Machine-specific paths and hardware IDs in these archived scripts are historical deployment details, not recommended defaults. Use the packaged runtime's explicit environment/configuration registration for a new deployment. Keep calibration/reference and safety checks intact.
