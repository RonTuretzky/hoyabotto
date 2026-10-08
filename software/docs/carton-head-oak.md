# Explicit head + OAK mode

This local mode selects two real cameras named `head` and `oak`. It never labels OAK as a wrist camera. Existing head/wrist configurations retain their default behavior.

```sh
python -m carton.servo prepare --arm left --camera-pair head_oak --frames HEAD_MANIFEST_DIR --oak-frames OAK_MANIFEST_DIR --session SESSION_DIR --calibration CALIBRATION_JSON --out NEW_EXPERIMENT_DIR
python -m carton.servo camera-check --arm left --camera-pair head_oak --frames HEAD_MANIFEST_DIR --oak-frames OAK_MANIFEST_DIR --seconds 20
```

Preparation reads existing immutable manifests; it does not capture cameras or open motors. It writes an unseeded experiment with `camera_pair: head_oak`, raw saved encoder ranges and both actual camera identities. `seed --camera head` and `seed --camera oak` work normally. Review visible independent tool, paddle and stationary anchor regions and noncontact goals; preparation's zero placeholders are not physical targets.

The independently loaded continuous profile must also explicitly declare `camera_pair: head_oak`, match the station fingerprint and carry real commissioning evidence, all six selected-arm corridors/rates and measured waypoints. Owner deployment must separately register head and OAK camera IDs (`CARTON_HEAD_ID`, `CARTON_OAK_ID`, or session camera identities). The packaged owner opens only those configured manifest readers. Its OAK reader verifies RGB and aligned metric depth hashes, dimensions, units, identity and capture synchronization before any trajectory write; a missing/stale/corrupt depth frame stops operation.

A grasp recipe additionally requires a valid `depth` specification using the same OAK ID and manifest as station RGB. Seed six independent stationary table points plus separate tool, paddle and bottom features on the OAK reference, establish the measured unit up direction in its optical frame and unchanged 10–100 mm lift/clearance and ≤5 mm slip thresholds. Head evidence still requires separate tool, paddle, bottom and fixed table tracks, upward co-motion/clearance and a measured nonempty jaw baseline. Depth plane registration movement or depth loss fails the observer. Retention uses the paddle-minus-tool vector in OAK optical millimetres on each observation, rather than wrist-relative pixels. No arm/OAK or station extrinsic transform is inferred.

Current scene visibility alone supplies none of the measured jaw baseline, supported park, collision-safe joint corridor, timed waypoints or complete feature/depth registration. An occluded paddle handle/tool cannot be seeded reliably. Prepare/inspect remain available; execution still requires those measurements and a matching profile.

For the existing bounded single-joint owner without a continuous profile, session `camera` settings can explicitly use `source: robot`, `camera_pair: head_oak`, and `manifests: {head: ABS_HEAD_JSON, oak: ABS_OAK_JSON}`, with separately registered identities. Both immutable readers validate freshness and OAK depth without loading a trajectory profile. This permits supported isolated commissioning measurements under unchanged motor limits; it does not create a reference, grasp or Cartesian model.
