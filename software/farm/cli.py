"""`farm` command line.

  farm set-motor-id --name head_motor_1   give one loose servo its bus ID (replaces Feetech's Windows FD tool)
  farm devices                      list serial ports and cameras (fill the profile from this)
  farm calibrate  -p paper-tray-v0  one-time LeRobot range-of-motion calibration by hand (setup, not operation)
  farm calibrate --auto --arm left [--motor gripper | --unfold-only]   UNTESTED ON THE CART: the arm finds its own limits (LeRobot PR #3282)
  farm calibrate --head             hands-on, two joints: calibrate only the head
  farm calibration-report           read the saved calibration; flag wrapped, short or mismatched joint ranges (no motion)
  farm robot-test -p paper-tray-v0 [--move] [--ask] [--only head|left|right]   motors only: read every joint; --move nudges each one
  farm check      -p paper-tray-v0  connect everything, verify camera identities with the vision model, report
  farm teach      -p ... --arm right --goal "..." --save pour_B     LLM-servo the arm to a goal and save the keyframe
  farm teach-all  -p ...            learn every keyframe the profile needs, in order
  farm mcp        -p ...            stdio MCP server: state, camera frames and named skills for an agent (no raw joint access)
  farm calibrate-pour -p ... --tilt 25 --seconds 1.5 --ml 28        record a measured cup pour
  farm once       -p ... --tray B   run one care cycle (viewer included)
  farm run        -p ... [--every 3600]   run cycles for every tray on a schedule, viewer included
  farm viewer     -p ...            viewer only (evidence browsing, reconciliation)
  farm review     -p ...            Astra daily review -> proposal
  farm cup-test   -p ... --tilt 25 --seconds 1.5 --who you   pour into a measuring cup, record the mL
  farm policy-test --checkpoint DIR  run a trained checkpoint as a skill on the simulator (or --real), clamped; --server URL uses a policy server
  farm policy-server --checkpoint DIR [--port 8766]   training machine only: serve a checkpoint to the robot laptop (no hardware)
  farm light-monitor                stream the ESP32 lux readings
  farm sim        [--faults ...] [--record]   the same program on fakes
  farm backup     -p ...
  farm r2a        [-p r2a-assembly-v0] [--checkpoint DIR] [--station FILE]   R2a assembly readiness report: parts, station, grips, schema (no motion)
  farm r2a --simulate [--fault double_paper] [--variant R2a_with_frame]    offline supervisor rehearsal (no physics or hardware)
  farm r2a-station --measurements FILE --output FILE   fit actual measured fixture points; no motion
  --record on run/once/sim writes the robot's own runs to data/dataset (LeRobotDataset)
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from pathlib import Path

from .config import load_env


def _log():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")


def cmd_devices(a):
    """List serial ports and cameras; with --probe, ping each motor bus to tell bus 1 from bus 2 and snapshot every camera."""
    from .adapters.camera_opencv import list_cameras
    from .adapters.robot_lerobot import find_serial_ports
    ports = find_serial_ports()
    print("Serial ports:")
    for p in ports:
        print(f"  {p['device']:40} {p['description'] or '':30} vid={p['vid']} pid={p['pid']} sn={p['serial']}")
    cams = list_cameras()
    print("Cameras:")
    for c in cams:
        print(f"  id={c.get('id')!r:14} {c.get('name')}")
    if a.probe:
        try:
            from .tools.bus_probe import probe_ports
            usb = [p["device"] for p in ports if "usbmodem" in (p["device"] or "") or "ttyACM" in (p["device"] or "")]
            print("\nMotor bus probe:")
            for r in probe_ports(usb):
                print("  ", json.dumps(r, default=str))
        except ImportError:
            print("  (bus probe tool not available)")
        out = Path("data/devices"); out.mkdir(parents=True, exist_ok=True)
        import cv2
        from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig
        print("\nCamera snapshots (open these to decide which index is head / left_wrist / right_wrist):")
        for c in cams:
            try:
                cam = OpenCVCamera(OpenCVCameraConfig(index_or_path=c["id"], fps=30, width=640, height=480)); cam.connect()
                f = cam.read(); cam.disconnect()
                p = out / f"cam-{str(c['id']).replace('/', '_')}.jpg"
                cv2.imwrite(str(p), cv2.cvtColor(f, cv2.COLOR_RGB2BGR)); print("  ", p)
            except Exception as e:  # noqa: BLE001
                print(f"   camera {c.get('id')!r}: {e}")
    print("\nPut the two /dev/tty.usbmodem… ports and the camera indices into profiles/paper-tray-v0.yaml; `farm check` then asks the vision model to confirm which camera is which.")


def _system(a, **kw):
    from .system import build
    return build(a.profile, **kw)


def cmd_calibrate(a):
    _log()
    from .config import load_profile
    p = load_profile(a.profile)
    if a.auto or a.head:
        from .tools import auto_calibrate as ac
        if a.head:
            sys.exit(ac.calibrate_head(p.robot))
        mode = "motor" if a.motor else "unfold" if a.unfold_only else "full"
        rc = ac.run_arm(p.robot, a.arm, mode=mode, motor=a.motor or None, velocity=a.velocity or None)
        if rc == 0 and mode == "full":
            from .tools import calibration_report as cr
            cr.report(cr.calibration_path(p.robot))
        sys.exit(rc)
    from .adapters.robot_lerobot import LeRobotXLeRobot
    r = LeRobotXLeRobot(p.robot)
    print("Support both arms. Follow the prompts (move to mid-range, then sweep each joint). This is one-time setup.")
    path = r.calibrate_interactive()
    print("calibration saved:", path)


def cmd_check(a):
    _log()
    s = _system(a)
    problems = s.connect()
    print("problems:", problems or "none")
    print("joints:", s.robot.joints().status.value)
    print("health:", s.robot.health().status.value)
    for n, c in s.cameras.items():
        print(f"camera {n}: {c.frame().status.value}")
    if s.light is not None:
        print("light:", s.light.latest().to_record())
    print("views:", s.verify_views())
    print("keyframes:", s.keyframes.names())
    s.disconnect()


def cmd_calibration_report(a):
    """Read the saved calibration and flag wrapped, short or mismatched joint ranges. No motion."""
    from .config import load_profile
    from .tools import calibration_report as cr
    path = Path(a.file) if a.file else cr.calibration_path(load_profile(a.profile).robot)
    sys.exit(0 if cr.report(path)["ok"] else 1)


def cmd_robot_test(a):
    """Motors only: no cameras, no LLM, no trays. Needs ports in the profile and a calibration."""
    from .config import load_profile
    from .tools import robot_test
    p = load_profile(a.profile)
    if p.robot.kind == "sim" or p.simulated:
        from .adapters.sim import FakeRobot, Faults
        f = Faults(); f.set("empty_gripper")
        r = FakeRobot(f)
    else:
        from .adapters.robot_lerobot import LeRobotXLeRobot
        r = LeRobotXLeRobot(p.robot)
    if a.move:
        print("Arms and head will each move a little, one joint at a time. Start with the arms folded at rest.")
        print("When the test ends, or on Ctrl-C, the motors go limp.")
    r.connect()
    try:
        res = robot_test.run(r, move=a.move, delta=a.delta, only=a.only or None, ask=input if a.ask else None)
    except KeyboardInterrupt:
        r.stop()
        print("\nstopped")
        res = {"ok": False}
    finally:
        r.disconnect()
    sys.exit(0 if res["ok"] else 1)


def _needed_keyframes(s):
    t = s.profile.trays
    need = [("bottle_rest_above", s.profile.arms.bottle, "Position the gripper 6 cm directly above the bottle standing in its rest, fingers open, tool pitch level, ready to descend and grasp."),
            ("bottle_rest_grip", s.profile.arms.bottle, "Lower the open gripper around the bottle body in its rest so closing the fingers would hold it; do not touch the tray."),
            ("bottle_upright", s.profile.arms.bottle, "Hold the bottle vertical (upright) at chest height clear of everything."),
            ("paddle_rest_above", s.profile.arms.paddle, "Position the gripper 6 cm above the light paddle in its rest, fingers open."),
            ("paddle_rest_grip", s.profile.arms.paddle, "Lower the open gripper around the light paddle's handle so closing would hold it.")]
    for tr in t:
        need.append((f"look_{tr.id}", "head", f"Point the head camera so tray {tr.id} in the {tr.nest} nest fills the centre of the head view."))
        need.append((f"pour_{tr.id}", s.profile.arms.bottle, f"With the bottle held upright, bring its spout 3 cm above the refill opening of tray {tr.id} in the {tr.nest} nest, centred in the wrist view, without touching the tray."))
        need.append((f"measure_{tr.id}", s.profile.arms.paddle, f"Hold the light paddle level, sensor face up, at canopy height over the middle of tray {tr.id}, with the arm out of the light path."))
    return need


def _teach_one(s, name, arm, goal):
    from .skills.llm_servo import LLMServo
    servo = LLMServo(s.skills, s.cameras, s.backends.vision, s.store, None, max_steps=s.profile.limits.llm_servo_max_steps)
    out = servo.run(arm, goal, save_as=name, allow_gripper=(arm != "head"))
    print(f"{name}: {'OK' if out.ok else 'FAILED'} in {out.steps} steps, ${out.cost_usd:.3f}: {out.reason}")
    return out.ok


def cmd_teach(a):
    _log()
    s = _system(a)
    problems = s.connect()
    if any(p.startswith("robot") for p in problems):
        sys.exit("robot not connected: " + "; ".join(problems))
    from .viewer.app import serve_in_thread
    serve_in_thread(s, s.profile.viewer_port)
    ok = _teach_one(s, a.save, a.arm, a.goal)
    s.disconnect()
    sys.exit(0 if ok else 1)


def cmd_mcp(a):
    """Stdio MCP server: an agent can read state, look through a camera and run named skills. No raw joint access."""
    import contextlib
    from .mcp_server import serve
    with contextlib.redirect_stdout(sys.stderr):      # stdout belongs to the MCP transport; nothing else may print there
        s = _system(a)
        problems = s.connect()
        if any(p.startswith("robot") for p in problems):
            sys.exit("robot not connected: " + "; ".join(problems))
        from .viewer.app import serve_in_thread
        serve_in_thread(s, s.profile.viewer_port)
    serve(s)


def cmd_teach_all(a):
    _log()
    s = _system(a)
    problems = s.connect()
    if any(p.startswith("robot") for p in problems):
        sys.exit("robot not connected: " + "; ".join(problems))
    from .viewer.app import serve_in_thread
    serve_in_thread(s, s.profile.viewer_port)
    failed = []
    for name, arm, goal in _needed_keyframes(s):
        if name in s.keyframes.names() and not a.force:
            print(f"{name}: already taught, skipping")
            continue
        if not _teach_one(s, name, arm, goal):
            failed.append(name)
        s.skills.go_rest()
    s.disconnect()
    print("failed:", failed or "none")
    sys.exit(1 if failed else 0)


def cmd_cup_test(a):
    """Bottle in the gripper over a kitchen measure: pour with the given tilt/seconds, then record what was measured."""
    _log()
    s = _system(a)
    problems = s.connect()
    if any(p.startswith("robot") for p in problems):
        sys.exit("robot not connected: " + "; ".join(problems))
    from .viewer.app import serve_in_thread
    serve_in_thread(s, s.profile.viewer_port)
    try:
        if s.skills.held.get(s.profile.arms.bottle) != "bottle":
            r = s.skills.pick_tool("bottle", "bottle_rest_above", "bottle_rest_grip")
            if not r.ok:
                sys.exit(f"pick bottle failed: {r.note}")
        input("Hold a kitchen measuring cup under the spout (or place it and step back). Press ENTER to pour once … ")
        aid = s.store.intent(None, "pour", {"tray": "cup", "tilt_deg": a.tilt, "seconds": a.seconds, "authorized_by": f"human:{a.who}"})
        r = s.skills.pour(a.tilt, a.seconds, on_attempt=lambda: s.store.attempt(aid))
        s.store.result(aid, "VERIFIED" if r.ok else "ABORTED", note=f"cup test {r.note}")
        ml = float(input("Millilitres measured in the cup: ").strip() or "0")
        s.save_pour_calibration(a.tilt, a.seconds, ml, a.who)
        print(f"saved: tilt {a.tilt}°, {a.seconds}s → {ml} mL")
        s.skills.go_rest()
    finally:
        s.disconnect()


def cmd_calibrate_pour(a):
    s = _system(a)
    s.save_pour_calibration(a.tilt, a.seconds, a.ml, a.who)
    print("saved pour calibration")


def _run_cycles(s, trays, every: float | None):
    from .cycle.runner import CareCycle
    from .viewer.app import serve_in_thread
    serve_in_thread(s, s.profile.viewer_port)
    print(f"viewer: http://localhost:{s.profile.viewer_port}")
    while True:
        for tray in trays:
            if not s.ready_for_cycle():
                print("robot not ready (see viewer); retrying in 30 s")
                time.sleep(30)
                continue
            pre = s.state.get("preauthorized")
            if pre and pre.get("tray") == tray.id:
                s.state.pop("preauthorized", None)
            out = CareCycle(s, tray).run()
            print(json.dumps({"cycle": out.cycle_id, "tray": out.tray_id, "result": out.result, "note": out.note}))
            if out.result in ("PAUSED", "CRASHED"):
                print("waiting for a person in the viewer …")
                while s.store.open_actions() or s.state.get("needs_person"):
                    time.sleep(5)
        if every is None:
            return
        _daily_maintenance(s)
        s.idle_rest(every)


def _daily_maintenance(s):
    """Once per day inside `farm run`: Astra review (proposal) and a verified backup."""
    last = s.state.get("last_maintenance", 0.0)
    if time.time() - last < 86400:
        return
    s.state["last_maintenance"] = time.time()
    try:
        path = s.store.backup(s.profile.data_path / "backups")
        s.state["last_backup"] = s.store.verify_backup(path)
    except Exception as e:  # noqa: BLE001
        s.store.event(None, "backup_failed", {"error": str(e)})
    try:
        from .llm.astra import Astra
        from .llm.backends import NoLLM
        if not isinstance(s.backends.astra, NoLLM):
            Astra(s.backends.astra, s.store, s.profile.authority.astra, s.profile.data_path / "overrides.yaml").review(s.profile.raw)
    except Exception as e:  # noqa: BLE001
        s.store.event(None, "astra_failed", {"error": str(e)})


def cmd_once(a):
    _log()
    s = _system(a)
    problems = s.connect()
    if any(p.startswith("robot") for p in problems):
        sys.exit("robot not connected: " + "; ".join(problems))
    if a.record:
        s.enable_recording()
    try:
        _run_cycles(s, [s.profile.tray(a.tray)] if a.tray else s.profile.trays, None)
    finally:
        s.disconnect()


def cmd_run(a):
    _log()
    s = _system(a)
    problems = s.connect()
    if any(p.startswith("robot") for p in problems):
        sys.exit("robot not connected: " + "; ".join(problems))
    if a.record:
        s.enable_recording()
    try:
        _run_cycles(s, s.profile.trays, a.every)
    finally:
        s.disconnect()


def cmd_viewer(a):
    _log()
    s = _system(a)
    from .viewer.app import serve_in_thread
    serve_in_thread(s, s.profile.viewer_port)
    print(f"viewer: http://localhost:{s.profile.viewer_port} (evidence only; no devices connected)")
    while True:
        time.sleep(3600)


def cmd_review(a):
    _log()
    s = _system(a)
    from .llm.astra import Astra
    astra = Astra(s.backends.astra, s.store, s.profile.authority.astra, s.profile.data_path / "overrides.yaml")
    print(json.dumps(astra.review(s.profile.raw), indent=1, default=str))


def cmd_sim(a):
    _log()
    from .adapters.sim import Faults, ScriptedHuman
    from .config import load_profile
    from .system import System
    faults = Faults()
    for f in (a.faults or []):
        faults.set(f)
    human = ScriptedHuman({"pour:": "authorize one pour"}) if a.auto_answer else None
    s = System(load_profile("sim"), human=human, faults=faults)
    s.connect()
    _seed_sim_keyframes(s)
    if a.record:
        s.enable_recording()
    try:
        _run_cycles(s, s.profile.trays, None)
    finally:
        s.disconnect()


def _seed_sim_keyframes(s):
    """The simulator has no geometry to learn; give it trivial keyframes so skills have targets."""
    from .adapters.base import HEAD_JOINTS, arm_joint
    from .skills.arm import ArmPose
    b, p = s.profile.arms.bottle, s.profile.arms.paddle
    for name, arm, pose in [("bottle_rest_above", b, ArmPose(0.18, 0.12)), ("bottle_rest_grip", b, ArmPose(0.20, 0.08)), ("bottle_upright", b, ArmPose(0.16, 0.15)),
                            ("paddle_rest_above", p, ArmPose(0.18, 0.12)), ("paddle_rest_grip", p, ArmPose(0.20, 0.08))]:
        if name not in s.keyframes.names():
            s.keyframes.save(name, s.skills.models[arm].joints_for(pose), arm, "sim seed", "sim")
    for t in s.profile.trays:
        if f"look_{t.id}" not in s.keyframes.names():
            s.keyframes.save(f"look_{t.id}", {HEAD_JOINTS[0]: -20.0 if t.nest.startswith("left") else 20.0, HEAD_JOINTS[1]: -15.0}, "head", "sim seed", "sim")
        if f"pour_{t.id}" not in s.keyframes.names():
            s.keyframes.save(f"pour_{t.id}", s.skills.models[b].joints_for(ArmPose(0.21, 0.10, pan=-25 if t.nest.startswith("left") else 25)), b, "sim seed", "sim")
        if f"measure_{t.id}" not in s.keyframes.names():
            s.keyframes.save(f"measure_{t.id}", s.skills.models[p].joints_for(ArmPose(0.19, 0.13, pan=-25 if t.nest.startswith("left") else 25)), p, "sim seed", "sim")
        t.look_pose, t.pour_pose, t.measure_pose = f"look_{t.id}", f"pour_{t.id}", f"measure_{t.id}"


def cmd_policy_test(a):
    """Load a trained checkpoint and run it as a skill on the simulator (default) or the real robot, under the safety envelope."""
    _log()
    from .config import load_profile
    from .skills.policy import PolicySkill
    from .system import System
    from .learning.infer import PolicyRunner
    if a.real:
        s = _system(a)
    else:
        from .adapters.sim import Faults, ScriptedHuman
        s = System(load_profile("sim"), human=ScriptedHuman(), faults=Faults())
    problems = s.connect()
    if any(p.startswith("robot") for p in problems):
        sys.exit("robot not connected: " + "; ".join(problems))
    pc = s.profile.policy
    server = a.server or pc.server_url
    if server:
        from .learning.remote import RemotePolicy
        runner = RemotePolicy(server, timeout_s=pc.timeout_s)
        print("policy served from", server)
    else:
        runner = PolicyRunner(a.checkpoint or pc.checkpoint, device=a.device or pc.device)
    spec = runner.input_spec()
    print("policy expects:", json.dumps(spec, default=str))
    cam_map = dict(pc.camera_map)
    for key in list(spec.get("cameras", {})):      # keys the profile does not name: wrist views -> the bottle arm's wrist, anything else -> the head
        cam_map.setdefault(key, "right_wrist" if "wrist" in key else "head")
    skill = PolicySkill(s.skills, s.cameras, runner, pc.state_joints, {k: v for k, v in cam_map.items() if k in spec.get("cameras", {}) or not spec.get("cameras")},
                        hz=pc.hz, max_steps=a.steps, store=s.store)
    out = skill.run(a.goal)
    print(json.dumps({"ok": out.ok, "reason": out.reason, "steps": out.steps, "trace_tail": out.trace[-5:]}, default=str))
    s.disconnect()


def cmd_policy_server(a):
    """Run on the training machine: load a checkpoint once and answer the robot laptop over HTTP. Touches no hardware."""
    _log()
    from .learning.server import serve
    serve(a.checkpoint, device=a.device, host=a.host, port=a.port)


MOTOR_IDS = {"head_motor_1": (1, 7), "head_motor_2": (1, 8), "base_left_wheel": (2, 9), "base_right_wheel": (2, 10)}


def cmd_set_motor_id(a):
    """Give ONE loose STS3215 servo its bus ID (what Feetech's Windows 'FD' tool does in the vendor video).
    Connect only that servo to the motor board, power the board, then run this."""
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus
    if a.name:
        if a.name not in MOTOR_IDS:
            sys.exit(f"unknown motor name; choose from {sorted(MOTOR_IDS)}")
        board, new_id = MOTOR_IDS[a.name]
        print(f"{a.name}: ID {new_id}; this servo belongs on board {board} ({'left arm + head' if board == 1 else 'right arm + wheels'})")
    elif a.id:
        new_id = a.id
    else:
        sys.exit("give --name head_motor_1|head_motor_2|base_left_wheel|base_right_wheel or --id N")
    port = a.port
    if not port:
        from .adapters.robot_lerobot import find_serial_ports
        cands = [p["device"] for p in find_serial_ports() if "usbmodem" in (p["device"] or "") or "ttyACM" in (p["device"] or "")]
        if len(cands) != 1:
            sys.exit(f"found {len(cands)} motor boards {cands}; plug in exactly one or pass --port")
        port = cands[0]
    input(f"Only ONE servo connected to the board on {port}, board powered (12 V). Press ENTER to set its ID to {new_id} … ")
    bus = FeetechMotorsBus(port, {"target": Motor(new_id, "sts3215", MotorNormMode.RANGE_M100_100)})
    bus.setup_motor("target")     # finds the single servo at any baud/ID, writes the new ID and 1 Mbaud
    found = bus.broadcast_ping(num_retry=2) or {}
    bus.disconnect(False) if bus.is_connected else None
    print(f"done: servo now answers as ID {sorted(found)} (expected [{new_id}]). Label it before unplugging.")


def cmd_light_monitor(a):
    from .tools.light_monitor import run
    run(a.port or None, a.baud, a.seconds)


def cmd_backup(a):
    s = _system(a)
    path = s.store.backup(s.profile.data_path / "backups")
    print(path)
    if a.verify:
        print(json.dumps(s.store.verify_backup(path)))


def cmd_r2a(a):
    """R2a assembly contract: what is in place and what still blocks execution. Reads files only; never connects to the robot."""
    from .assembly import frames as fr
    from .assembly import r2a
    from .assembly.schema import DatasetSchema
    p = r2a.load_assembly_profile(a.profile)
    if a.variant:
        p.variant = r2a.Variant(a.variant)
    if a.simulate:
        from .assembly.rehearsal import Rehearsal, save_trace
        result = Rehearsal(p, a.fault).run()
        out = Path(a.output) if a.output else Path("data/r2a/rehearsals") / f"{time.time_ns()}.json"
        save_trace(result, out)
        print(f"{result['result']}: {result['variant']} fault={result['fault']}")
        print("Discrete supervisor rehearsal only; no physics, vision, hardware or training data.")
        print(f"Trace: {out}")
        if result["reason"]:
            print(result["reason"])
        if result["result"] != "SIMULATED_COMPLETE":
            raise SystemExit(2)
        return
    if a.fault != "none" or a.output:
        raise ValueError("--fault and --output require --simulate")
    st = fr.RobotFromAssembly.load(Path(a.station) if a.station else p.station_path())
    parts = r2a.verify_parts(p.parts_path())
    schema = DatasetSchema(p.controlled_arm, p.state_joints, p.cameras, p.fps, p.frame_hw)
    print(f"R2a assembly profile {p.name}: execution_enabled={p.execution_enabled} variant={p.variant.value} arm={p.controlled_arm}")
    print(f"  stages: {' -> '.join(s.value for s in r2a.stage_order(p.variant))}")
    print(f"  deadlines_s: {p.deadlines_s}")
    print(f"  max_attempts: {p.max_attempts}")
    print("Parts (parts/r2a):")
    for n, r in parts.items():
        print(f"  {'OK ' if r['ok'] else 'BAD'} {n:16} {r['note'] or r['sha256'][:16]}")
    print(f"Station transform ({a.station or p.station_path()}): {'measured by ' + st.by + ' on ' + st.measured_on + ' via ' + st.method if st.measured else 'NOT MEASURED'}")
    miss = p.grip.missing()
    print(f"Grip thresholds: {'all measured' if not miss else 'NOT MEASURED: ' + ', '.join(miss)}")
    print(f"Dataset schema {schema.schema_version}: state/action {len(schema.joints)} joints of {p.controlled_arm} arm; cameras {schema.cameras}; fps {p.fps}")
    print(f"  dataset root {p.dataset_path()} repo {p.dataset_repo_id}; holdout sessions {p.holdout_sessions or 'NONE DECLARED'}")
    info = p.dataset_path() / "meta" / "info.json"
    if info.exists():
        probs = schema.problems_with_info(json.loads(info.read_text()))
        print(f"  existing dataset: {'compatible' if not probs else 'INCOMPATIBLE: ' + '; '.join(probs)}")
    else:
        print("  existing dataset: none")
    if a.checkpoint:
        from .learning.infer import resolve_pretrained_dir
        cfg = json.loads((Path(resolve_pretrained_dir(a.checkpoint)) / "config.json").read_text())
        probs = schema.problems_with_checkpoint(cfg)
        print(f"Checkpoint {a.checkpoint}: {'matches schema' if not probs else 'DOES NOT MATCH: ' + '; '.join(probs)}")
    print("Nominal grasp candidates (CAD, assembly frame A, mm; not robot poses):")
    for g in fr.grasp_candidates():
        print(f"  {g.part:9} at {g.point_A_mm} close across {g.closure_axis} over {g.thickness_mm} mm  [{g.status}]")
    blockers = r2a.execution_blockers(p, station=st, parts_report=parts)
    print("Static contract: " + ("checks pass; hardware readiness still required" if not blockers else "BLOCKED"))
    for b in blockers:
        print(f"  - {b}")
    from .assembly.readiness import laptop_report
    report = laptop_report(p)
    print("Robot laptop (files only):")
    print(f"  calibration id: {report['calibration_id'] or 'unavailable'}")
    for problem in report["problems"]:
        print(f"  - {problem}")
    if st.measured and (not st.calibration_id or st.calibration_id != report["calibration_id"]):
        print("  - station transform does not match this laptop's current calibration")
    print(f"  R2a keyframes: {report['keyframes_file']}")
    for name in report["missing_keyframes"]:
        print(f"    missing: {name}")
    print("Physical checks still required: inspected print and hand fit; fixed trough and source stations; "
          "verified camera identities; measured grips; validated clear approach/transfer/release paths.")
    print("Live R2a execution is not exposed by this command. Rehearsal is available with --simulate.")


def cmd_r2a_station(a):
    from .assembly.measure import fit_station_file
    result = fit_station_file(Path(a.measurements), Path(a.output))
    print(json.dumps(result, indent=2))
    print(f"Station saved to {a.output}; execution remains disabled.")


def main(argv=None):
    load_env()
    ap = argparse.ArgumentParser(prog="farm", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name, fn, extra in [
        ("devices", cmd_devices, [("--probe", {"action": "store_true"})]), ("calibrate", cmd_calibrate, [("--auto", {"action": "store_true"}), ("--arm", {"choices": ["left", "right"], "default": ""}), ("--motor", {"default": ""}), ("--unfold-only", {"action": "store_true"}), ("--velocity", {"type": int, "default": 0}), ("--head", {"action": "store_true"})]), ("check", cmd_check, []),
        ("calibration-report", cmd_calibration_report, [("--file", {"default": ""})]),
        ("robot-test", cmd_robot_test, [("--move", {"action": "store_true"}), ("--delta", {"type": float, "default": 5.0}), ("--only", {"choices": ["head", "left", "right"]}), ("--ask", {"action": "store_true"})]),
        ("teach", cmd_teach, [("--arm", {"required": True}), ("--goal", {"required": True}), ("--save", {"required": True})]),
        ("mcp", cmd_mcp, []),
        ("teach-all", cmd_teach_all, [("--force", {"action": "store_true"})]),
        ("calibrate-pour", cmd_calibrate_pour, [("--tilt", {"type": float, "required": True}), ("--seconds", {"type": float, "required": True}), ("--ml", {"type": float, "required": True}), ("--who", {"required": True})]),
        ("cup-test", cmd_cup_test, [("--tilt", {"type": float, "default": 25.0}), ("--seconds", {"type": float, "default": 1.5}), ("--who", {"required": True})]),
        ("once", cmd_once, [("--tray", {}), ("--record", {"action": "store_true"})]), ("run", cmd_run, [("--every", {"type": float, "default": 3600}), ("--record", {"action": "store_true"})]),
        ("viewer", cmd_viewer, []), ("review", cmd_review, []), ("backup", cmd_backup, [("--verify", {"action": "store_true"})]),
        ("policy-server", cmd_policy_server, [("--checkpoint", {"required": True}), ("--device", {"default": "mps"}), ("--host", {"default": "0.0.0.0"}), ("--port", {"type": int, "default": 8766})]),
        ("policy-test", cmd_policy_test, [("--server", {"default": ""}), ("--checkpoint", {"default": ""}), ("--device", {"default": ""}), ("--steps", {"type": int, "default": 60}), ("--goal", {"default": "pour"}), ("--real", {"action": "store_true"})]),
        ("set-motor-id", cmd_set_motor_id, [("--name", {"default": ""}), ("--id", {"type": int, "default": 0}), ("--port", {"default": ""})]),
        ("light-monitor", cmd_light_monitor, [("--port", {"default": ""}), ("--baud", {"type": int, "default": 115200}), ("--seconds", {"type": float, "default": None})]),
        ("sim", cmd_sim, [("--faults", {"nargs": "*"}), ("--auto-answer", {"action": "store_true"}), ("--record", {"action": "store_true"})]),
        ("r2a", cmd_r2a, [("--checkpoint", {"default": ""}), ("--station", {"default": ""}),
            ("--simulate", {"action": "store_true"}), ("--fault", {"default": "none", "choices": ["none", "preflight_unknown", "no_pick", "lost_carrier", "seat_unknown", "double_paper", "no_paper", "paper_unknown", "paper_on_jaws", "frame_failed", "stale_evidence", "stop_during_transfer", "deadline", "intervention"]}),
            ("--variant", {"choices": ["R2a_no_frame", "R2a_with_frame"]}), ("--output", {"default": ""})]),
        ("r2a-station", cmd_r2a_station, [("--measurements", {"required": True}), ("--output", {"required": True})]),
    ]:
        sp = sub.add_parser(name)
        if name == "r2a":
            sp.add_argument("-p", "--profile", default="r2a-assembly-v0")
        elif name != "sim":
            sp.add_argument("-p", "--profile", default="paper-tray-v0")
        for flag, kw in extra:
            sp.add_argument(flag, **kw)
        sp.set_defaults(fn=fn)
    a = ap.parse_args(argv)
    a.fn(a)


if __name__ == "__main__":
    main()
