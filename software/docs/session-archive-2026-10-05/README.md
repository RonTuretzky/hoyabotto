# Source actually used on the robot Mac

Historical camera snapshots from the October 5 session, retained separately from the portable packaged controller under `software/scripts/carton_robot`. They are not installed entry points. The work-directory motor owner and isolated left-claw script that ran that day were removed from this archive on 2026-10-06 because they enforced a 55 °C servo limit; the packaged owner is the only motor owner.

The capture and viewer sources were used for head/OAK framing. The single-camera Swift publisher accepts an output directory and one explicit named device selection; its inherited usage string mentions two selections. The phone helper generates its own configuration/certificates. No runtime config, authorization token, certificate, key, image, binary or motor log is included. The viewer and phone server are no longer running.

Machine-specific paths and hardware IDs in these archived scripts are historical deployment details, not recommended defaults. Use the packaged runtime's explicit environment/configuration registration for a new deployment. Keep calibration/reference and safety checks intact.
