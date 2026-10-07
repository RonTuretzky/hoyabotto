# Paddle commissioning, 6–7 October 2026

The physical paddle pickup is **incomplete**. No jaw closure, verified lift, hold, or placement was performed. The requested 40-degree arm movements and 40-centimetre cart travel were not completed. Left-arm calibration remains mixed; only the accepted right-arm calibration was used in the reach tests.

## Verified actions and their limits

- On 6 October, eleven guarded wheel pulses produced a mean encoder estimate of 19.9916 cm forward (left 19.8957 cm, right 20.0875 cm). Actual floor distance was not independently measured. Both wheels stopped and released after each pulse.
- Right-arm non-contact movements used temporary lower torque settings and saved travel bounds. Holds expired and released, causing passive position changes. Later return movements folded the arm near rest; the wrist stopped short of its target and triggered cleanup. The final return and all-six release are recorded separately. No pregrasp or pickup was established.
- Native USB head-camera capture worked on 6 October; device identities changed, so no numbered OpenCV camera mapping was assumed.
- The OAK identity 1944301091DA1C2E00 returned fresh 640×360 RGB/depth with factory intrinsics and distortion metadata. Head orientation and the physical scene changed subsequently; prior camera/arm bindings cannot be reused unchanged.
- OAK and phone tag observations decoded IDs 1 (table), 2 (fixed right gripper housing), and 3 (paddle) when framed suitably. Cup IDs 11 and 14 were separately visible. Cup rotation alone did not establish the inter-tag transform or calibrate the cameras.
- The exploratory two-camera fit trained on tags 1/2 missed the excluded paddle tag by about 16 pixels. It did not pass registration. Phone intrinsics were fitted under simplified assumptions rather than independently calibrated.
- Five OAK frames compared tag-pose axial distance with depth patches; the largest disagreement was about 6.2 mm. This is limited scene consistency, **not** independently validated RGB/depth alignment or physical accuracy.
- Manual OAK depth feature selection estimated the fixed-jaw tip approximately 75 mm along tag 2 and 16 mm out of its plane. Partial-surface CAD fits placed the grasp point up to 30 mm apart. The nearest fit predicted the excluded tip within 4.4 mm. These remain exploratory candidates, not validated grasp offsets.
- Installed servo calibration supplies raw ranges and homing offsets, but not the five `model_zero_tick`/`model_sign` measurements required by `CalibratedArm`. No camera-to-arm registration matched to that binding was established.
- On 7 October, direct read-only checks recovered all sixteen status-zero/torque-zero replies after USB and power were restored. Those are historical snapshots, not a current-state guarantee.
- The OAK stream and phone feed were restored on 7 October. The first generated phone QR incorrectly used a `token=` fragment; it was corrected to the raw URL-encoded bearer key expected by the page. Private links, keys, QR images and raw camera footage are excluded from this publication.

## Archive contents

`evidence/` contains hardware and exploratory geometry results. The latest all-servo snapshot is dated by its recorded Unix timestamp. Failed checks are retained and are not motion-ready flags. Large repeated telemetry arrays were omitted from copied results; original full session files remain in the local task outputs.

`scripts/` preserves the five local commissioning prototypes used for wheel/reach and geometry experiments. They have workstation-specific paths, local dependencies and manually selected image features. They are **not turnkey production tools**. An explicit `XLEROBOT_RUN_ARCHIVED_PROTOTYPE=1` opt-in prevents accidental execution. In particular, the motor prototypes can move real hardware and their timed release can allow an unsupported arm to droop. Do not launch them with a suspended object or assume they implement the authenticated Gemma owner. Review the current single-owner contract, source, actual hardware state and physical clearance first. No temperature checks or tracked safety-limit changes were added.

The next session must establish a real arm/model binding and registration, validate actual jaw/handle geometry and approach clearance, and implement the grasp through the current sole motor owner. Successful simulation, tag detection, an ICP residual or a commanded movement is not physical pickup evidence. Verify actual paddle lift and table clearance, then place and release.
