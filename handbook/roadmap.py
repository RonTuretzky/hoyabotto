"""Software roadmap deck: architecture first, then build order.

Every diagram is generated here as inline SVG so the slides explain the
software rather than reuse hardware renders. Three colours carry meaning:
green = exists upstream (LeRobot / XLeRobot), blue = we build it,
grey = deferred. Nothing described here has been implemented yet.
"""
from html import escape as e

INK, BLUE, GREEN, GREY, ORANGE, RED = '#172a34', '#154d77', '#2f6b3a', '#7d878d', '#a84c0b', '#9b2c2c'
KIND = {'bd': (BLUE, '#e8eff6'), 'ex': (GREEN, '#e9f2ea'), 'df': (GREY, '#f1f2f2'), 'hw': (INK, '#fffefa'), 'hu': (ORANGE, '#fbeee3'), 'bad': (RED, '#f8e7e7')}

def wrap(text, width):
    words, lines, cur = text.split(), [], ''
    for w in words:
        if len(cur) + len(w) + 1 > width and cur:
            lines.append(cur); cur = w
        else:
            cur = (cur + ' ' + w).strip()
    if cur: lines.append(cur)
    return lines

def svg(w, h, inner, title):
    return (f'<svg class="diagram" viewBox="0 0 {w} {h}" role="img" aria-label="{e(title, quote=True)}" xmlns="http://www.w3.org/2000/svg">'
            f'<defs><marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0 0L10 5L0 10z" fill="{INK}"/></marker>'
            f'<marker id="ah-grey" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0 0L10 5L0 10z" fill="{GREY}"/></marker>'
            f'<marker id="ah-red" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0 0L10 5L0 10z" fill="{RED}"/></marker></defs>{inner}</svg>')

def box(x, y, w, h, title, lines=(), kind='bd', fs=15, tfs=17, dashed=False, cw=None):
    stroke, fill = KIND[kind]
    dash = ' stroke-dasharray="7 5"' if dashed or kind == 'df' else ''
    out = f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="9" fill="{fill}" stroke="{stroke}" stroke-width="2"{dash}/>'
    cw = cw or max(8, int(w / (fs * 0.56)))
    ty = y + 8 + tfs
    out += f'<text x="{x + 14}" y="{ty}" font-size="{tfs}" font-weight="700" fill="{stroke}">{e(title)}</text>'
    ty += 6
    for line in lines:
        for part in wrap(line, cw):
            ty += fs * 1.35
            out += f'<text x="{x + 14}" y="{ty}" font-size="{fs}" fill="{INK}">{e(part)}</text>'
    return out

def tag(x, y, text, kind='bd', fs=12):
    stroke, fill = KIND[kind]
    w = len(text) * fs * 0.62 + 18
    return (f'<rect x="{x}" y="{y}" width="{w:.0f}" height="{fs + 12}" rx="{(fs + 12) / 2}" fill="{stroke}"/>'
            f'<text x="{x + 9}" y="{y + fs + 2}" font-size="{fs}" font-weight="700" fill="#fff">{e(text)}</text>')

def label(x, y, text, fs=14, weight=400, fill=INK, anchor='start'):
    return f'<text x="{x}" y="{y}" font-size="{fs}" font-weight="{weight}" fill="{fill}" text-anchor="{anchor}">{e(text)}</text>'

def arrow(x1, y1, x2, y2, text='', kind='ink', dashed=False, above=True):
    color = {'ink': INK, 'grey': GREY, 'red': RED}[kind]
    marker = {'ink': 'ah', 'grey': 'ah-grey', 'red': 'ah-red'}[kind]
    dash = ' stroke-dasharray="6 5"' if dashed else ''
    out = f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="2" marker-end="url(#{marker})"{dash}/>'
    if text:
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2 + (-8 if above else 18)
        out += f'<text x="{mx}" y="{my}" font-size="13" fill="{color}" text-anchor="middle">{e(text)}</text>'
    return out

def elbow(points, kind='ink', dashed=False):
    color = {'ink': INK, 'grey': GREY, 'red': RED}[kind]
    marker = {'ink': 'ah', 'grey': 'ah-grey', 'red': 'ah-red'}[kind]
    dash = ' stroke-dasharray="6 5"' if dashed else ''
    d = 'M' + ' L'.join(f'{x} {y}' for x, y in points)
    return f'<path d="{d}" fill="none" stroke="{color}" stroke-width="2" marker-end="url(#{marker})"{dash}/>'

def legend(x, y):
    out = ''
    for i, (k, t) in enumerate([('ex', 'Exists · LeRobot / XLeRobot'), ('bd', 'We build'), ('df', 'Deferred'), ('hu', 'Human')]):
        stroke, fill = KIND[k]
        out += f'<rect x="{x + i * 215}" y="{y}" width="18" height="18" rx="4" fill="{fill}" stroke="{stroke}" stroke-width="2"/>' + label(x + i * 215 + 26, y + 14, t, 13)
    return out

# ---------------------------------------------------------------- diagrams

def d_title():
    parts = [('1', 'Device adapters'), ('2', 'Skills'), ('3', 'Perception'), ('4', 'Care cycle'), ('5', 'Evidence'), ('6', 'Review'), ('7', 'Learning')]
    s = ''
    for i, (n, t) in enumerate(parts):
        x = 40 + i * 165
        s += box(x, 250, 150, 86, f'{n} · {t}', kind='bd', tfs=14)
        if i < len(parts) - 1: s += arrow(x + 150, 293, x + 165, 293)
    s += label(40, 60, 'What the deck covers', 18, 700, GREY)
    s += label(40, 120, 'The hardware we own → the code that already exists → the seven parts we must write →', 22, 600)
    s += label(40, 155, 'how one care cycle flows through all of them → what gets built on which day.', 22, 600)
    s += label(40, 400, 'One sentence:', 18, 700, GREY)
    s += label(40, 445, 'A Python program on the Mac that turns a teleoperated robot kit into a', 26, 700)
    s += label(40, 483, 'supervised plant caretaker that can explain every action it took.', 26, 700)
    s += legend(40, 560)
    return svg(1200, 600, s, 'Seven software parts in a row')

def d_gap():
    s = label(40, 40, 'What the kit does out of the box', 20, 700, GREEN)
    s += box(40, 60, 520, 340, 'XLeRobot + LeRobot today', [
        '• A person drives the arms, head and wheels with a keyboard, Xbox pad, Joy-Cons or VR.',
        '• Three cameras stream frames; joint positions read back 30× per second.',
        '• Demonstrations can be recorded to a dataset and a policy (ACT / SmolVLA / π0.5) trained on them.',
        '• An LLM-agent example turns camera + tools into “approach a human”.',
        '',
        'It has no idea what a tray, a plant, water, a schedule, a fault or “yesterday” is.',
        'The plant-watering clip in the showcase is teleoperated, sped up 8×.'], kind='ex', fs=16, tfs=20)
    s += label(640, 40, 'What a farm caretaker needs', 20, 700, BLUE)
    s += box(640, 60, 520, 340, 'The farm program', [
        '• Knows which tray it is looking at and whether the view is fresh.',
        '• Runs a fixed routine: inspect → measure → ask → pour → verify → park.',
        '• Asks a person before water moves; pours a bounded amount; returns upright.',
        '• Writes down intent, attempt and outcome before, during and after every action.',
        '• Pauses on doubt and never repeats a pour whose result is unknown.',
        '• Lets a person see why it paused and fix it from a screen.',
        '• Gets better from its own recorded episodes, in the open.'], kind='bd', fs=16, tfs=20)
    s += arrow(565, 230, 635, 230) + label(600, 212, 'the gap', 13, 700, INK, 'middle') + label(600, 262, '= this deck', 13, 700, INK, 'middle')
    return svg(1200, 430, s, 'Kit capabilities versus farm needs')

def d_hardware():
    s = box(40, 40, 260, 124, 'MacBook Neo', ['Runs everything. No Raspberry Pi.', 'Python · LeRobot · our farm code'], kind='hw')
    s += box(40, 230, 260, 112, 'RSHTECH 7-port hub', ['Own 5 V adapter. Carries USB data only. Never powers motors.'], kind='hw')
    s += arrow(170, 230, 170, 164, 'USB-C · USB 3', above=False)
    # motor boards
    s += box(380, 40, 380, 150, 'Motor board A  →  bus 1', ['Left arm: 6 × STS3215 servos, IDs 1–6', 'Head pan / tilt: IDs 7–8', 'USB serial on the Mac: /dev/tty.usbmodem…', '12 V / 8 A wall supply (separate)'], kind='hw')
    s += box(380, 210, 380, 150, 'Motor board B  →  bus 2', ['Right arm: 6 × STS3215 servos, IDs 1–6', 'Drive wheels: IDs 9–10 (disabled in V0)', 'USB serial on the Mac: /dev/tty.usbmodem…', '12 V / 8 A wall supply (separate)'], kind='hw')
    s += box(380, 380, 380, 96, 'Three USB cameras', ['head · left wrist · right wrist', 'OpenCV / AVFoundation, 640×480 @ 30 fps'], kind='hw')
    s += box(380, 496, 380, 96, 'ELEGOO ESP32 (one of three)', ['USB serial → JSON lines to the Mac', 'I²C: BH1750 light sensor on the paddle'], kind='hw')
    for y in (115, 285, 428, 544):
        s += arrow(300, 278, 380, y)
    # peripherals right
    s += box(840, 40, 320, 96, 'BH1750 light paddle', ['Gripped tool. 4 wires: 3V3, GND, SDA (GPIO21), SCL (GPIO22).'], kind='hw')
    s += arrow(760, 544, 840, 88)
    s += box(840, 160, 320, 80, 'Optional SHT31 air sensor', ['On the body, not the arm. Same I²C bus.'], kind='df')
    s += box(840, 260, 320, 80, 'Moisture probe + relay', ['Deferred for paper trays. Not wired.'], kind='df')
    s += box(840, 380, 320, 96, 'Bottle · trays · table', ['No electronics. Known only through cameras and taught positions.'], kind='hw')
    s += box(840, 496, 320, 96, 'Anker C300 (in hand) + wheels', ['Owned, not wired: wall power and parked base for the whole trip. 12 V comes from its car outlet via a fused harness, not USB-C.'], kind='df')
    s += label(40, 390, 'Two facts shape the code:', 16, 700)
    for i, t in enumerate(['Everything is a USB device on one Mac.', 'Motion is 14 servo positions;', 'sensing is 3 images + 1 lux number.', 'That is the whole interface.']):
        s += label(40, 418 + i * 24, t, 15)
    return svg(1200, 620, s, 'Hardware map from the Mac outward')

def d_upstream():
    cols = [
        ('FeetechMotorsBus', ['Serial protocol to STS3215 servos.', 'Read positions, load, temperature.', 'Write goal positions.']),
        ('XLerobot2Wheels', ['Two buses, 14 joints + 2 wheels.', 'get_observation() → dict of joint.pos + camera frames', 'send_action() → dict of joint.pos targets', 'Calibration file per robot id.']),
        ('SO101Kinematics', ['Analytical IK for the SO-101 arm.', 'x/y/pitch → joint angles.', 'Used by every teleop example.']),
        ('Teleop examples', ['Keyboard, Xbox, Joy-Con, VR.', 'Two-wheel keyboard script exists.', 'This is how we teach poses.']),
        ('lerobot-record', ['Records observation + action at fps into a LeRobotDataset (parquet + mp4).', 'Optionally runs a policy.']),
        ('Train / run policy', ['ACT, SmolVLA, π0.5 recipes.', 'Needs consistent camera poses.', 'Evaluate on the real robot.']),
    ]
    s = ''
    for i, (t, lines) in enumerate(cols):
        x = 40 + i * 190
        s += box(x, 120, 172, 250, t, lines, kind='ex', fs=13, tfs=15, cw=21)
        if i < len(cols) - 1: s += arrow(x + 172, 245, x + 190, 245)
    s += label(40, 60, 'Upstream stack, bottom to top (all Python, Apache-2.0):', 18, 700, GREEN)
    s += label(40, 88, 'motors → robot object → kinematics → human control → dataset → learned policy', 16)
    s += box(40, 410, 555, 120, 'What we call directly on the Mac', ['robot = XLerobot2Wheels(config); robot.connect()', 'obs = robot.get_observation()   # 14 joint.pos + 3 frames', 'robot.send_action({"right_arm_wrist_flex.pos": 40.0, ...})'], kind='ex', fs=13, tfs=15, cw=70)
    s += box(620, 410, 540, 120, 'What we do not use in V0', ['ZMQ host/client: only for a Raspberry Pi on the robot. We run in-process.', 'Wheel velocities: the base stays parked.', 'RoboCrew LLM agent: our orchestrator owns the decision loop instead.'], kind='df', fs=13, tfs=15, cw=68)
    return svg(1200, 560, s, 'The XLeRobot and LeRobot software stack')

def d_stack():
    layers = [
        (7, 'Learning loop', 'record episodes → train → eval vs rules → shadow → release', 'bd'),
        (6, 'Review', 'exception viewer (human) · Jev (shadow) · Astra (proposals)', 'bd'),
        (5, 'Evidence store', 'SQLite + images: intent → attempt → verified / UNKNOWN', 'bd'),
        (4, 'Care-cycle orchestrator', 'state machine; evidence + deadline per step; pauses on doubt', 'bd'),
        (3, 'Perception', 'tray id · fresh · opening visible · spill 3-state · light stats', 'bd'),
        (2, 'Skills', 'go_rest · look_at · pick_tool · measure · pour · upright · place · stop', 'bd'),
        (1, 'Device adapters', 'robot · cameras · ESP32 serial · human — one status vocabulary', 'bd'),
        (0, 'Hardware + upstream', 'Feetech bus · XLerobot2Wheels · cameras · IK · record · train', 'ex'),
    ]
    s = ''
    for i, (n, t, d, k) in enumerate(layers):
        y = 30 + i * 66
        s += box(160, y, 760, 56, f'{n} · {t}', [], kind=k, tfs=17)
        s += label(400, y + 34, d, 13.5, 400, INK)
    # side rails
    s += box(950, 30, 210, 160, 'Config profile', ['paper-tray-v0.yaml: ports, camera identities, tray positions, enabled sensors, motion limits.'], kind='bd', fs=13, tfs=15)
    s += box(950, 210, 210, 150, 'Simulator', ['Fake robot, fake cameras, fake ESP32 behind the same adapter interface. Fault injection.'], kind='bd', fs=13, tfs=15)
    s += box(950, 380, 210, 170, 'Local safety rules', ['Joint limits, max step per tick, servo temperature, watchdog, torque-off on disconnect. Never overridden by a model.'], kind='bd', fs=13, tfs=15)
    s += elbow([(60, 560), (60, 70), (150, 70)])
    s += label(20, 320, 'commands ↓', 13, 700, GREY)
    s += elbow([(120, 70), (120, 560), (150, 560)], kind='grey')
    s += label(80, 590, 'observations ↑ · evidence ↑', 13, 700, GREY)
    return svg(1200, 600, s, 'Eight-layer software stack')

def d_adapters():
    s = box(40, 30, 1120, 74, 'One interface for every device', ['read() → {value, t, seq, status}    status ∈ { OK, STALE, INVALID, NOT_APPLICABLE }    — nothing upstream ever sees a bare number'], kind='bd', fs=14, tfs=17, cw=140)
    cards = [
        ('Robot adapter', ['Wraps XLerobot2Wheels.', 'joints(), frames(), move_to(targets, max_step), stop(), torque_off().', 'Checks load/temperature each tick.', 'Disconnect → INVALID and a safe stop.'], 'bd'),
        ('Camera adapter', ['Names head / left_wrist / right_wrist by device identity, never by index 0-1-2.', 'Every frame carries capture time.', 'Old frame → STALE. No device → INVALID.'], 'bd'),
        ('ESP32 adapter', ['Serial JSON lines at ~5 Hz:', '{"seq":812,"t_ms":93410,', ' "lux":412.5,"ok":true}', 'Gap in seq or no line for 1 s → STALE.', 'Sensor error flag → INVALID.'], 'bd'),
        ('Human adapter', ['Buttons and typed confirmations from the viewer.', 'Each one is a timestamped, signed-by-name record.', 'Absent → the cycle waits; it does not assume.'], 'hu'),
    ]
    for i, (t, lines, k) in enumerate(cards):
        s += box(40 + i * 285, 130, 265, 250, t, lines, kind=k, fs=13.5, tfs=16)
    s += box(40, 410, 540, 130, 'Config decides what exists', ['enabled: robot, cameras, esp32_light', 'disabled: sht31, moisture, wheels, pump', 'A disabled device reads NOT_APPLICABLE forever. The orchestrator treats that differently from INVALID: “no probe” is not “dry”.'], kind='bd', fs=13.5, tfs=16, cw=64)
    s += box(620, 410, 540, 130, 'Why this layer first', ['The simulator implements the same four interfaces with fakes, so the whole program above runs on the Mac before the robot exists.', 'Swapping fake → real is one line in the config.'], kind='ex', fs=13.5, tfs=16, cw=64)
    return svg(1200, 570, s, 'Four device adapters sharing one status vocabulary')

def d_skills():
    rows = [
        ('go_rest()', 'any', 'arms at rest pose, grippers as-is', 'both'),
        ('look_at(tray)', 'tray known', 'head + wrist cameras framed on tray', 'head'),
        ('pick_tool(paddle | bottle)', 'gripper empty, tool in rest', 'tool gripped; grip verified by gripper position', 'one arm'),
        ('measure_pose(tray)', 'paddle gripped', 'paddle level at canopy height, arm out of light', 'left'),
        ('pour(tray, tilt°, s)', 'bottle gripped, fill known, human OK', 'tilt held for s seconds, then upright', 'right'),
        ('return_upright()', 'bottle gripped', 'bottle vertical; commanded before any release', 'right'),
        ('place_tool()', 'tool gripped, rest free', 'tool in rest, gripper open, arm clear', 'one arm'),
        ('stop()', 'any', 'motion halted; torque held or off per rule', 'both'),
    ]
    s = label(40, 40, 'Skill', 15, 700, GREY) + label(300, 40, 'Needs before', 15, 700, GREY) + label(580, 40, 'True after', 15, 700, GREY) + label(1000, 40, 'Arm', 15, 700, GREY)
    for i, (a, b, c, d) in enumerate(rows):
        y = 70 + i * 40
        s += f'<rect x="30" y="{y - 24}" width="1140" height="36" rx="6" fill="{"#e8eff6" if i % 2 == 0 else "#fffefa"}"/>'
        s += label(40, y, a, 15, 700, BLUE) + label(300, y, b, 14) + label(580, y, c, 14) + label(1000, y, d, 14)
    s += box(40, 410, 540, 150, 'v1 · taught keyframes (Sep 30 – Oct 5)', ['Drive the arm with the keyboard teleop, save joint positions as named keyframes, interpolate with the IK and a max-step clamp.', 'Every skill is a short list of keyframes plus checks. Boring, inspectable, fixable.'], kind='bd', fs=13.5, tfs=16, cw=66)
    s += box(620, 410, 540, 150, 'v2 · learned policies (after ~50 demos)', ['Record pick_tool / pour with lerobot-record, train ACT, run it behind the same skill interface.', 'Only replaces a v1 skill after it wins on held-out episodes. Camera poses must match recording.'], kind='ex', fs=13.5, tfs=16, cw=66)
    return svg(1200, 580, s, 'Eight motion skills with pre- and post-conditions')

def d_perception():
    s = box(40, 60, 220, 120, 'Inputs', ['3 frames + capture time', 'current joint positions', 'ESP32 lux stream'], kind='hw', fs=14, tfs=16)
    outs = [
        ('tray_id', 'which tray, from its fixed nest position (+ optional AprilTag)', 'UNKNOWN if the nest is empty or the view is off'),
        ('fresh', 'frame age < 1 s and camera matches its name', 'STALE otherwise; nothing downstream runs on stale'),
        ('opening_visible', 'refill opening clear and reachable in the wrist view', 'UNKNOWN if occluded or dark'),
        ('spill', 'CLEAR / SPILL / UNKNOWN around the tray after a pour', 'an unseen area can never be CLEAR'),
        ('growth', 'green-pixel fraction from the same head pose each day', 'a trend for people, not a decision input'),
        ('light', 'median lux, spread, saturation over ~10 fresh conversions', 'INVALID if arm shadow or sensor error'),
    ]
    for i, (n, what, rule) in enumerate(outs):
        y = 40 + i * 82
        s += box(380, y, 780, 70, n, [what, '↳ ' + rule], kind='bd', fs=13, tfs=15, cw=100)
        s += arrow(260, 120, 380, y + 35)
    s += box(40, 220, 220, 150, 'The one rule', ['Every output has an UNKNOWN value and the cycle treats UNKNOWN as “stop and ask”, never as “fine”.'], kind='bd', fs=14, tfs=16, cw=25)
    s += box(40, 400, 220, 130, 'v1 → v2', ['v1: OpenCV colour / tag / difference checks.', 'v2: a vision model on the same frames. Same typed outputs.'], kind='ex', fs=13, tfs=15)
    return svg(1200, 560, s, 'Perception outputs with their unknown states')

def d_cycle():
    states = [('IDLE', 60, 60), ('IDENTIFY', 240, 60), ('INSPECT', 420, 60), ('MEASURE LIGHT', 600, 60), ('ASK HUMAN', 800, 60),
              ('PICK BOTTLE', 1000, 60), ('APPROACH', 1000, 230), ('POUR', 800, 230), ('RETURN UPRIGHT', 600, 230), ('VERIFY', 420, 230), ('PARK', 240, 230)]
    s = ''
    for name, x, y in states:
        k = 'hu' if name == 'ASK HUMAN' else 'bd'
        s += box(x, y, 150, 56, name, [], kind=k, tfs=15)
    top = [('IDLE', 'IDENTIFY'), ('IDENTIFY', 'INSPECT'), ('INSPECT', 'MEASURE LIGHT'), ('MEASURE LIGHT', 'ASK HUMAN'), ('ASK HUMAN', 'PICK BOTTLE')]
    pos = {n: (x, y) for n, x, y in states}
    for a, b in top:
        s += arrow(pos[a][0] + 150, pos[a][1] + 28, pos[b][0], pos[b][1] + 28)
    s += arrow(1075, 116, 1075, 230)
    for a, b in [('APPROACH', 'POUR'), ('POUR', 'RETURN UPRIGHT'), ('RETURN UPRIGHT', 'VERIFY'), ('VERIFY', 'PARK')]:
        s += arrow(pos[a][0], pos[a][1] + 28, pos[b][0] + 150, pos[b][1] + 28)
    s += elbow([(240, 258), (135, 258), (135, 116)])
    s += label(150, 175, 'evidence + deadline on every arrow', 13, 700, GREY)
    s += label(830, 175, 'no “yes” → PARK, no water', 13, 700, ORANGE)
    s += elbow([(875, 116), (875, 175 + 10), (330, 185), (330, 230)], kind='grey', dashed=True)
    # paused
    s += box(420, 370, 360, 108, 'PAUSED (reason, evidence)', ['Any check fails, any deadline passes, any adapter goes INVALID or STALE. Motors stop; bottle is returned upright first if it is held.'], kind='bad', fs=13, tfs=16, cw=46)
    s += box(820, 370, 340, 108, 'UNKNOWN DELIVERY', ['Crash or lost acknowledgement between POUR and VERIFY. Stays UNKNOWN until a person reconciles. Automatic retry is not an option.'], kind='bad', fs=13, tfs=16, cw=44)
    for x in (300, 495, 675, 875, 1075):
        s += arrow(x, 286, 560, 370, kind='red', dashed=True)
    s += arrow(875, 286, 990, 370, kind='red', dashed=True)
    s += box(40, 370, 340, 108, 'Human review', ['Re-inspect, confirm physically, authorize, resolve, or take over. Only a person leaves PAUSED.'], kind='hu', fs=13, tfs=16, cw=44)
    s += arrow(420, 415, 380, 415)
    s += label(40, 515, 'Timeouts: identify 10 s · inspect 20 s · measure 15 s · ask 10 min · pick 30 s · pour ≤ 4 s tilt · verify 20 s · park 30 s (starting values, all in the config).', 13, 400, GREY)
    return svg(1200, 540, s, 'Care-cycle state machine with pause and unknown states')

def d_evidence():
    tables = [('cycles', 'cycle_id, tray_id, profile, code_version, config_hash, calibration_id, simulated, started, ended, result'),
              ('observations', 'obs_id, cycle_id, source, t, status, value_json, image_hash, extractor_version'),
              ('decisions', 'decision_id, cycle_id, kind (rule | human | jev_shadow), inputs_ref, choice, probabilities, schema_version'),
              ('actions', 'action_id, cycle_id, skill, params, INTENT_t, ATTEMPT_t, RESULT ∈ {VERIFIED, ABORTED, UNKNOWN}, VERIFY_t, evidence_ref'),
              ('interventions', 'who, t, what was physically checked, what was done, minutes spent')]
    s = ''
    for i, (t, cols) in enumerate(tables):
        y = 30 + i * 72
        s += box(40, y, 560, 64, t, [], kind='bd', tfs=15)
        for j, part in enumerate(wrap(cols, 58)):
            s += label(200, y + 26 + j * 16, part, 12)
    s += label(660, 40, 'One pour, written three times', 17, 700, BLUE)
    tl = [('INTENT', 'written before any motor command', 'bd'), ('ATTEMPT', 'written before the tilt can move water', 'bd'), ('VERIFY', 'written after the wrist camera confirms', 'bd')]
    for i, (t, d, k) in enumerate(tl):
        y = 80 + i * 90
        s += box(660, y, 500, 64, t, [d], kind=k, fs=13, tfs=15)
        if i < 2: s += arrow(910, y + 64, 910, y + 90)
    s += label(700, 252, 'crash here →', 13, 700, RED)
    s += f'<rect x="930" y="236" width="160" height="24" rx="6" fill="{RED}" opacity=".12"/>'
    s += label(1010, 253, 'RESULT = UNKNOWN', 12.5, 700, RED, 'middle')
    s += box(660, 390, 500, 140, 'Rules', ['Append-only. Images stored by content hash, never overwritten.', 'Duplicate action_id returns the existing record — the database cannot make a pour happen exactly once, only refuse to pretend.', 'Nightly backup with a restore test.'], kind='bd', fs=13, tfs=15, cw=62)
    s += box(40, 400, 560, 130, 'What the store never contains', ['A number without a status and a time.', 'An estimated volume labelled as measured.', '“Delivered” inferred from “the servo finished moving”.', 'Secrets, network credentials or addresses.'], kind='bad', fs=13.5, tfs=15, cw=70)
    return svg(1200, 540, s, 'Evidence store tables and the intent-attempt-verify timeline')

def d_viewer():
    s = f'<rect x="120" y="30" width="960" height="500" rx="14" fill="#fffefa" stroke="{INK}" stroke-width="2"/>'
    s += f'<rect x="120" y="30" width="960" height="44" rx="14" fill="{INK}"/>' + label(140, 59, 'localhost:8765 · farm viewer · tray B · cycle 0042', 15, 700, '#fff')
    s += f'<rect x="140" y="95" width="420" height="270" rx="8" fill="#dfe6e3" stroke="{GREY}"/>' + label(350, 235, 'latest wrist frame', 16, 700, GREY, 'middle') + label(350, 260, 'age 0.6 s · right_wrist', 13, 400, GREY, 'middle')
    s += tag(140, 380, 'PAUSED', 'bad', 13) + label(140, 424, 'reason: spill = UNKNOWN after pour', 14, 700) + label(140, 443, '(area under the rim not visible from right_wrist)', 13)
    s += label(140, 470, 'last verified: RETURN_UPRIGHT ✓ 12:04:31 · fill ~40 mL (human, 11:58)', 12.5, 400, GREY)
    s += label(140, 490, 'evidence: obs 9124 · action 771 · decision rule:spill_gate', 12.5, 400, GREY)
    s += label(600, 120, 'Eligible next steps', 16, 700, BLUE)
    btns = [('Re-inspect from head camera', 'bd'), ('I looked: no spill, paper wet', 'hu'), ('I looked: spill — cleaned, resolved', 'hu'), ('Caretaker takes over this tray', 'hu')]
    for i, (t, k) in enumerate(btns):
        y = 145 + i * 52
        stroke, fill = KIND[k]
        s += f'<rect x="600" y="{y}" width="450" height="40" rx="7" fill="{fill}" stroke="{stroke}" stroke-width="2"/>' + label(620, y + 26, t, 14, 700, stroke)
    y = 145 + 4 * 52
    s += f'<rect x="600" y="{y}" width="450" height="40" rx="7" fill="#f1f2f2" stroke="{GREY}" stroke-width="2" stroke-dasharray="6 4"/>' + label(620, y + 26, 'Retry pour — not offered while delivery is UNKNOWN', 14, 700, GREY)
    s += label(600, 420, 'Every press → intervention record: who, when, what was checked.', 12.5, 400, GREY)
    s += label(600, 442, 'Shadow (later): Jev suggested inspect_water 0.71 — logged, not acted on.', 12.5, 400, GREY)
    s += label(600, 500, 'Local network only. Not a safety device — stop rules live in the controller.', 12.5, 700, ORANGE)
    return svg(1200, 560, s, 'Mock-up of the exception viewer')

def d_models():
    s = box(40, 40, 300, 130, 'Evidence packet', ['tray_id, age, recent pours, sensor statuses, image features, device faults, last action, allowed next steps.', 'Small, typed, timestamped.'], kind='bd', fs=13, tfs=16)
    s += box(400, 40, 300, 130, 'Local rules  (must pass)', ['Numeric bounds, freshness, device health, human authorization present.', 'Own machine protection. Run without any network.'], kind='bd', fs=13, tfs=16)
    s += box(760, 40, 400, 130, 'Gate → one preapproved skill', ['Only a skill from the allowed list can execute, and only if rules passed.', 'A model label is never permission to move or pour.'], kind='bd', fs=13, tfs=16)
    s += arrow(340, 105, 400, 105) + arrow(700, 105, 760, 105)
    s += box(400, 220, 300, 170, 'Jev · typed classifier (shadow)', ['Q1 evidence quality → usable | reacquire | conflicting | unknown', 'Q2 next review → routine | inspect_water | inspect_image | review_machine | review_hygiene | unknown', 'Returns choice + probabilities. Abstains on unknown.'], kind='df', fs=12.5, tfs=15)
    s += arrow(190, 170, 480, 220, kind='grey', dashed=True)
    s += arrow(550, 220, 550, 170, kind='grey', dashed=True) + label(562, 200, 'suggestion only, logged', 12.5, 400, GREY)
    s += box(760, 220, 400, 170, 'Astra · reasoner (offline)', ['Reads exception packets and daily summaries.', 'Proposes one specific change with evidence, expected benefit and a rollback condition.', 'Never writes servo commands. Never on the protection path.'], kind='df', fs=12.5, tfs=15)
    s += box(40, 220, 300, 170, 'Human caretaker', ['Physical checks, authorizations, ambiguous crop calls, final incident labels.', 'The only party that leaves PAUSED.'], kind='hu', fs=13, tfs=16)
    s += box(40, 430, 1120, 100, 'Earning a role: shadow → compare → limited pilot', ['Replay the same recorded episodes through rules-only, rules+Jev, rules+Astra. Report recall per fault class, false clearances, abstention rate, review minutes, p95 latency, cost. Target ≥120 labelled episodes, ≥10 per priority fault (stale sensor, missing tray, obscured view, failed pour, camera loss). Jev gets an operational role only if it beats rules-only on a held-out set — and even then only to route reviews, not to pour.'], kind='bd', fs=13, tfs=16, cw=150)
    return svg(1200, 560, s, 'Bounded roles for rules, Jev, Astra and the human')

def d_learning():
    steps = [('Teleop', 'keyboard / Joy-Con drive of one arm; farm cameras fixed', 'ex'),
             ('lerobot-record', '≈50 episodes per skill: pick_tool, pour into cup', 'ex'),
             ('LeRobotDataset', 'parquet + mp4, our tray, our bottle, our poses', 'ex'),
             ('Train ACT', 'on the Mac or a rented GPU; version the checkpoint', 'ex'),
             ('Offline replay', 'candidate vs v1 keyframe skill on held-out episodes', 'bd'),
             ('Shadow', 'policy proposes, keyframes execute; log disagreement', 'bd'),
             ('Limited pilot', 'empty bottle, then cup, then tray; rollback ready', 'bd')]
    s = ''
    for i, (t, d, k) in enumerate(steps):
        x = 40 + i * 163
        s += box(x, 60, 150, 170, t, [d], kind=k, fs=12.5, tfs=14)
        if i < len(steps) - 1: s += arrow(x + 150, 145, x + 163, 145)
    s += label(40, 40, 'Robot learning: a skill graduates from v1 to v2', 17, 700, BLUE)
    s += label(40, 290, 'Crop learning: recipes change between frozen cohorts, never inside one', 17, 700, GREEN)
    s += box(40, 310, 350, 120, 'Cohort A · Sep 30', ['Cress on paper, recipe A. Every pour, light reading and human minute recorded.'], kind='bd', fs=13, tfs=15)
    s += box(425, 310, 350, 120, 'Cohort B · Oct 3', ['Same seed lot, one deliberate difference (e.g. light position). Same records.'], kind='bd', fs=13, tfs=15)
    s += box(810, 310, 350, 120, 'Proposal C', ['Astra summarises A vs B and proposes one change with a rollback condition. A person accepts or not.'], kind='df', fs=13, tfs=15)
    s += arrow(390, 370, 425, 370) + arrow(775, 370, 810, 370)
    s += box(40, 460, 1120, 80, 'What is honest to say today', ['No pretrained watering policy exists anywhere we looked; the two “water” datasets on the Hub are bottle transport. The upstream demo is teleoperated. Training happens on our own episodes or not at all, and logging alone improves nothing.'], kind='bad', fs=13.5, tfs=16, cw=150)
    return svg(1200, 560, s, 'Robot learning pipeline and crop cohort loop')

def d_sequence():
    lanes = ['Hardware', 'Adapters', 'Perception', 'Orchestrator', 'Evidence', 'Human', 'Jev (shadow)']
    lx = {n: 90 + i * 170 for i, n in enumerate(lanes)}
    s = ''
    for n, x in lx.items():
        s += label(x, 30, n, 14, 700, INK, 'middle') + f'<line x1="{x}" y1="42" x2="{x}" y2="560" stroke="{GREY}" stroke-dasharray="3 5"/>'
    steps = [
        ('Adapters', 'Hardware', 'read joints, 3 frames, lux', 'ink'),
        ('Adapters', 'Evidence', 'observations with status', 'ink'),
        ('Perception', 'Orchestrator', 'tray B · fresh · opening visible · light 410 lx', 'ink'),
        ('Orchestrator', 'Evidence', 'decision: rules pass → ASK_HUMAN', 'ink'),
        ('Orchestrator', 'Human', 'viewer: “tray B looks dry at edge — authorize 1 pour?”', 'ink'),
        ('Human', 'Orchestrator', 'authorize (name, time) → intervention record', 'ink'),
        ('Orchestrator', 'Evidence', 'action INTENT pour(B, 35°, 2 s)', 'ink'),
        ('Orchestrator', 'Adapters', 'skill pick_bottle → approach → pour', 'ink'),
        ('Adapters', 'Hardware', 'send_action(joint targets, clamped)', 'ink'),
        ('Orchestrator', 'Evidence', 'ATTEMPT written before tilt', 'ink'),
        ('Perception', 'Orchestrator', 'spill CLEAR · opening wet → VERIFIED', 'ink'),
        ('Orchestrator', 'Evidence', 'RESULT VERIFIED · PARK', 'ink'),
        ('Evidence', 'Jev (shadow)', 'packet → “routine” 0.88 (logged only)', 'grey'),
    ]
    for i, (a, b, t, k) in enumerate(steps):
        y = 70 + i * 37
        x1, x2 = lx[a], lx[b]
        s += arrow(x1, y, x2, y, kind=k, dashed=(k == 'grey'))
        s += label((x1 + x2) / 2, y - 6, f'{i + 1}. {t}', 12, 400, INK if k == 'ink' else GREY, 'middle')
    return svg(1200, 580, s, 'Sequence of one care cycle across layers')

def d_sim():
    s = box(40, 60, 300, 400, 'Farm program (unchanged)', ['orchestrator', 'perception', 'evidence store', 'viewer', 'skills'], kind='bd', fs=15, tfs=17)
    s += box(400, 60, 220, 400, 'Adapter interface', ['read() → status', 'move_to()', 'frames()', 'lux()', 'confirm()'], kind='bd', fs=15, tfs=17)
    s += arrow(340, 260, 400, 260)
    s += box(690, 60, 220, 180, 'Real devices', ['XLerobot2Wheels', 'OpenCV cameras', 'ESP32 serial', 'browser buttons'], kind='ex', fs=14, tfs=16)
    s += box(690, 280, 220, 180, 'Fakes', ['FakeRobot: kinematic model, instant or slow', 'FakeCamera: replays saved frames', 'FakeESP32: scripted lux', 'FakeHuman: scripted answers'], kind='bd', fs=13, tfs=16)
    s += arrow(620, 150, 690, 150) + arrow(620, 370, 690, 370)
    s += box(950, 60, 210, 400, 'Fault injection', ['stale frame', 'camera unplugged', 'serial gap', 'servo overtemp', 'crash after ATTEMPT', 'human never answers', 'wrong tray in nest', 'lost acknowledgement'], kind='bad', fs=13, tfs=16)
    s += arrow(910, 370, 950, 370)
    s += label(40, 510, 'Every simulated record carries simulated = true, so fake episodes can never be counted as farm evidence.', 14, 700, ORANGE)
    s += label(40, 540, 'This is what gets built first, on the Mac, before the robot exists — and it stays as the regression suite forever.', 14, 400, GREY)
    return svg(1200, 570, s, 'Simulator swaps fakes behind the adapter interface')

def d_config():
    cfg = ['profile: paper-tray-v0', 'robot:', '  port1: /dev/tty.usbmodem58A6…', '  port2: /dev/tty.usbmodem58B1…', '  max_relative_target: 8', '  wheels: disabled', 'cameras:', '  head: {id: "USB Camera 0C45:6366-1"}', '  right_wrist: {id: "…-2"}', 'sensors: {esp32_light: on, sht31: off, moisture: off}', 'trays:', '  B: {nest: right_front, pour_pose: kf_pour_B_v3}', 'limits: {tilt_max: 40, pour_s_max: 4, temp_max: 55}']
    s = f'<rect x="40" y="40" width="520" height="470" rx="10" fill="{INK}"/>'
    for i, line in enumerate(cfg):
        s += f'<text x="60" y="{72 + i * 30}" font-size="14" font-family="ui-monospace,monospace" fill="#fff">{e(line)}</text>'
    s += box(620, 40, 540, 220, 'Local safety rules (code, not config)', ['Joint range from calibration; refuse targets outside it.', 'Max step per tick — the arm cannot jump.', 'Servo temperature / load ceiling → stop.', 'Watchdog: no fresh observation for 0.5 s → stop.', 'Disconnect → torque off (upstream default), after return_upright if a bottle is held.', 'No model output can relax any of these.'], kind='bd', fs=13.5, tfs=16, cw=64)
    s += box(620, 290, 540, 220, 'Deferred — exists in the design, off in V0', ['Base driving and battery power.', 'Pump, relay, moisture probe, scale, leak pads.', 'Fixed overhead camera.', 'Printer pickup, table construction.', 'Any pour without a human “yes”.', 'Each returns when its own test passes; nothing is promised by date.'], kind='df', fs=13.5, tfs=16, cw=64)
    return svg(1200, 540, s, 'Config profile, safety rules and deferred items')

def d_timeline():
    phases = [('0', 'Sep 26–29 · Mac only', 'no robot yet', ['adapter interfaces + fakes', 'evidence store + schema', 'orchestrator on simulator', 'viewer skeleton', 'ESP32 firmware + serial parser'], 'Gate: full fake cycle, crash-after-ATTEMPT → UNKNOWN, restore from backup'),
              ('1', 'Sep 30–Oct 2 · connect', 'robot in Japan', ['assemble, ports, calibrate', 'real robot + camera adapters', 'keyframe skills v1', 'empty-bottle pick / return'], 'Gate: empty bottle picked, aimed, returned 10/10; stop works'),
              ('2', 'Oct 3–5 · one real cycle', 'water moves', ['measured pours into a cup', 'full cycle on tray with human gate', 'Jev shadow adapter (if 1 passed)'], 'Gate: 5 supervised cycles, every record complete, zero unknown left open'),
              ('3', 'Oct 6–8 · observe + record', 'longer windows', ['2 h → 8 h → 24 h watch windows', 'lerobot-record ≈50 demos', 'first ACT training attempt'], 'Gate: alerts fire on injected faults during a real window'),
              ('4', 'Oct 9–10 · hand over', 'someone else runs it', ['offline eval vs rules', 'export dataset + incident bundle', 'caretaker runbook, restore test'], 'Gate: another person resolves a staged exception')]
    s = ''
    for i, (n, when, sub, items, gate) in enumerate(phases):
        x = 40 + i * 226
        s += box(x, 60, 210, 300, f'Phase {n}', [when, sub, ''] + ['• ' + t for t in items], kind='bd', fs=12.5, tfs=16)
        s += f'<rect x="{x}" y="375" width="210" height="96" rx="8" fill="#fbeee3" stroke="{ORANGE}" stroke-width="2"/>'
        gy = 397
        for part in wrap(gate, 26):
            s += label(x + 12, gy, part, 12, 700, ORANGE); gy += 16
        if i < 4: s += arrow(x + 210, 210, x + 226, 210)
    s += label(40, 40, 'A failed gate holds the previous level. Dates are targets; the gates are the plan.', 15, 700, GREY)
    s += box(40, 492, 1120, 78, 'Status on Sep 26', ['None of this code exists. The “Sep 22–26 software before travel” from the previous roadmap did not happen; Phase 0 starts now and overlaps assembly if needed.'], kind='bad', fs=13.5, tfs=15, cw=150)
    return svg(1200, 585, s, 'Five phases with gates from Sep 26 to Oct 10')

def d_decisions():
    qs = [('Tray identity', 'Fixed nest positions (recommended) or AprilTags on each tray?', 'Nests: zero vision risk in V0; tags later for moved trays.'),
          ('Arms', 'Right arm = bottle, left arm = light paddle (recommended)?', 'Keeps the pour arm’s wrist camera on the opening.'),
          ('Jev / Astra', 'Shadow mode inside the trip, or after Oct 10 (recommended: only if Phase 2 passes by Oct 5)?', 'Neither gains authority during this trip either way.'),
          ('Viewer', 'Local web page (recommended) or terminal prompts?', 'Web page works from a phone on the same Wi-Fi.'),
          ('Light sensor', 'Keep the ESP32 + BH1750 in V0 (recommended) or drop all external sensors?', 'Dropping removes Phase 0 firmware work and the paddle skill.'),
          ('Learning', 'Record demos only (recommended) or also train ACT during the trip?', 'Training competes with commissioning time.')]
    s = ''
    for i, (t, q, why) in enumerate(qs):
        y = 30 + i * 88
        s += box(40, y, 1120, 76, f'{i + 1} · {t}', [q, '↳ ' + why], kind='hu', fs=13.5, tfs=16, cw=140)
    return svg(1200, 570, s, 'Six decisions with recommended defaults')

# ---------------------------------------------------------------- slides
# (key, kicker, title, lead, diagram, points[(label, text)], limit)
SLIDES = [
 ('start', 'Software roadmap · rebuilt Sep 26, 2026', 'The farm software, end to end',
  'What we are building, what already exists, how the parts connect, and the order to build them in.', d_title, [
  ('Scope', 'One parked robot, two arms, three cameras, one light sensor, two paper trays, one bottle. Watering stays human-authorized for the whole trip.'),
  ('Honesty', 'Nothing in this deck is implemented. Green boxes exist upstream; blue boxes are ours to write; grey is deferred.'),
  ('Navigate', 'Arrow keys or the buttons. O opens the outline; P presents full screen.')],
  'This replaces the September 22 slide set. Sources: XLeRobot repository and docs, LeRobot, the September 20 redline, purchase records.'),

 ('gap', 'Why write anything at all', 'The kit is a body with reflexes. The farm needs a routine, a memory and a conscience.',
  'XLeRobot gives us motors, cameras, kinematics and a way to train policies. It does not know what a tray is.', d_gap, [
  ('Exists', 'Teleoperation, calibration, dataset recording, policy training. All Python on a laptop.'),
  ('Missing', 'A schedule, tray identity, human authorization, bounded pouring, a record of what happened, pause-on-doubt, and a screen for a person.'),
  ('Therefore', 'We write a farm layer on top of LeRobot and call the robot object directly. We do not fork the robot code.')],
  'The showcase watering clip (0:56–0:59) is labelled “Teleoped with Switch Joycons”. No plant-care policy or checkpoint was found in the upstream repo or on the Hugging Face Hub.'),

 ('hardware', 'The physical system', 'Everything is a USB device on one Mac',
  'Two motor boards, three cameras and one ESP32 plug into a powered hub. Motor power is separate wall supplies.', d_hardware, [
  ('Motion', '14 servo positions (two arms + head) and, later, two wheel velocities. Motor IDs 1–8 on bus 1, 1–6 and 9–10 on bus 2 — exactly what XLerobot2Wheels expects.'),
  ('Sensing', 'Three 640×480 frames per tick plus one lux number over serial. The moisture probe, air sensor, wheels and battery stay off.'),
  ('Passive', 'Bottle, trays and table have no electronics. The software knows them only through taught positions and camera views.')],
  'Serial port names on macOS are /dev/tty.usbmodem…, discovered with lerobot-find-port; cameras with lerobot-find-cameras. Delivered board pinouts must be read before wiring; GPIO21/22 is the proposed I²C pair for the ELEGOO ESP32.'),

 ('upstream', 'What already exists', 'XLeRobot’s code base, layer by layer',
  'The repository is a thin extension of Hugging Face LeRobot: a robot class, an IK solver, teleop scripts and a recorder.', d_upstream, [
  ('Robot object', 'XLerobot2Wheels opens two FeetechMotorsBus instances and the cameras. get_observation() returns joint positions and frames; send_action() takes joint targets. Calibration is a JSON file per robot id.'),
  ('Kinematics', 'SO101Kinematics converts x / y / pitch to joint angles. Every teleop example uses it; our keyframe skills will too.'),
  ('Learning', 'lerobot-record writes a LeRobotDataset; the ACT / SmolVLA / π0.5 guides train from it. That pipeline is ours to reuse, unchanged.')],
  'Files read: software/src/robots/xlerobot_2wheels/*.py, software/src/model/SO101Robot.py, software/src/record.py, examples/4_xlerobot_2wheels_teleop_keyboard.py, docs install / teleop / VLA_ACT / LLM_agent. The ZMQ host/client exists for a Raspberry Pi and is not needed when the Mac is the controller.'),

 ('stack', 'Architecture', 'Eight layers, seven of them ours',
  'Commands flow down through adapters to the hardware. Observations and evidence flow up to the people and models that review them.', d_stack, [
  ('Bottom', 'Layer 0 is the kit and LeRobot. Layer 1 wraps every device in one status vocabulary so the rest of the program never sees a bare number.'),
  ('Middle', 'Skills move; perception judges; the orchestrator sequences them with deadlines and writes evidence as it goes.'),
  ('Top', 'A person reviews exceptions in a viewer. Jev and Astra observe in shadow. The learning loop turns recorded episodes into better skills and recipes.'),
  ('Cross-cutting', 'One config profile names what exists. A simulator implements the adapters with fakes. Safety rules live in code and outrank everything.')],
  'The next nine slides take one layer each, then trace one cycle through all of them.'),

 ('adapters', 'Layer 1', 'Device adapters: one interface, four devices',
  'Robot, cameras, ESP32 and the human each expose read() with a status. OK, STALE, INVALID and NOT_APPLICABLE mean different things.', d_adapters, [
  ('Robot', 'Wraps XLerobot2Wheels. Adds per-tick load and temperature checks, a clamped move_to, and stop / torque_off.'),
  ('Cameras', 'Bound by device identity, not by index — indices reshuffle on reconnect. Every frame carries its capture time.'),
  ('ESP32', 'Firmware prints one JSON line per reading with a sequence number. A gap or silence becomes STALE on the Mac side; an I²C error becomes INVALID.'),
  ('Human', 'Buttons in the viewer are an input device too: each press is a named, timestamped record.')],
  'NOT_APPLICABLE is the state of a device the config disabled. The orchestrator can plan around it; it can never treat it as a reading. “No probe” is not “dry”.'),

 ('skills', 'Layer 2', 'Skills: eight motions with contracts',
  'Each skill states what must be true before it runs and what is true after. The orchestrator only ever calls skills, never joints.', d_skills, [
  ('v1', 'Keyframes taught with the upstream keyboard teleop, interpolated with the IK solver and the max-step clamp. Readable, editable, no training.'),
  ('Contracts', 'pour() refuses to run without a gripped bottle, a known fill and a human authorization record. return_upright() is always commanded before any release.'),
  ('v2', 'A skill may later be replaced by an ACT policy trained on our own demonstrations, behind the same interface, after it wins on held-out episodes.')],
  'Pour parameters are tilt angle and seconds, calibrated into a kitchen measuring cup across fill levels. A tilt duration is never recorded as millilitres.'),

 ('perception', 'Layer 3', 'Perception: six outputs, each with an unknown',
  'From three frames, joint positions and the lux stream, perception answers a fixed set of questions with typed values.', d_perception, [
  ('Identity', 'Trays live in fixed nests, so tray_id comes from position; AprilTags are an optional upgrade for trays that move.'),
  ('Freshness', 'A frame older than one second, or from a camera that is not the one its name claims, blocks the cycle.'),
  ('Spill', 'Three values. An area the camera cannot see is UNKNOWN, and UNKNOWN after a pour pauses the cycle for a human look.'),
  ('Growth and light', 'Green fraction and lux statistics are for people and for cohort comparison. They never decide a pour.')],
  'v1 is OpenCV: nest occupancy, colour and frame differencing. v2 may be a vision model, but it must return the same typed outputs. Camera spill detection is experimental and gets its own fault tests.'),

 ('cycle', 'Layer 4', 'The care cycle: a state machine that would rather stop than guess',
  'Eleven states, one human gate, two ways to pause. Every arrow requires named evidence and has a deadline.', d_cycle, [
  ('Happy path', 'identify → inspect → measure light → ask human → pick bottle → approach → pour → return upright → verify → park.'),
  ('The gate', 'ASK_HUMAN shows the evidence in the viewer and waits. No answer within the deadline means park, not pour.'),
  ('Pause', 'Any failed check or timed-out step enters PAUSED with its reason and evidence. Only a person leaves it.'),
  ('Unknown', 'A crash between the tilt and the verify leaves the pour UNKNOWN. The cycle will not retry; a person reconciles.')],
  'Deadlines are starting values in the config. Pick / dock / weigh states from the redline belong to a future pod-moving profile and are not in this machine.'),

 ('evidence', 'Layer 5', 'Evidence store: intent, attempt, outcome — in that order',
  'SQLite for metadata, files for images, append-only everywhere. “We sent the command” and “water reached the tray” are separate rows.', d_evidence, [
  ('Five tables', 'cycles, observations, decisions, actions, interventions. Every row carries code version, config hash and calibration id.'),
  ('Three writes per pour', 'INTENT before any motor command, ATTEMPT before the tilt can move water, VERIFY after the camera confirms. A crash in between reads as UNKNOWN forever.'),
  ('Never', 'A value without a status and time; an estimate labelled as measured; “delivered” inferred from “servo finished”; any secret.')],
  'Idempotent action ids stop a restart from double-recording; they cannot stop physics from double-pouring. That is why UNKNOWN blocks retries. Nightly backup includes a restore test.'),

 ('viewer', 'Layer 6a', 'The exception viewer: the reason, the evidence and the eligible next steps',
  'A local web page on the Mac. When the robot pauses it shows the latest frame, its age, the reason and only the actions that are safe.', d_viewer, [
  ('Shows', 'Current state, last verified action, why it paused, which observation and rule produced the pause.'),
  ('Offers', 'Re-inspect, confirm a physical check, authorize one pour, mark resolved, hand the tray to a caretaker. Each press becomes an intervention record.'),
  ('Never offers', 'Retry pour while delivery is UNKNOWN. The button does not exist in that state.')],
  'Reachable from a phone on the same Wi-Fi; not exposed to the internet. Not a safety device — the stop rules run in the controller whether or not anyone is looking.'),

 ('models', 'Layer 6b', 'Jev and Astra: advice with no hands',
  'Rules own protection. Jev classifies evidence into typed choices. Astra investigates exceptions and proposes changes. Neither can move a servo.', d_models, [
  ('Jev', 'Two narrow questions per packet — evidence quality and next review — with probabilities and an explicit unknown. Runs in shadow: logged next to the rule’s decision, compared later.'),
  ('Astra', 'Reads exception packets and daily summaries offline. One proposal at a time, with evidence, expected benefit and a rollback condition.'),
  ('Earning a role', 'Shadow → replay comparison on ≥120 labelled episodes → limited pilot. Even then Jev only routes reviews.')],
  'The existing Goose/Jev helper routes documents with a 0.85 threshold; a farm adapter must add explicit farm questions, abstention labels and preserved confidence metadata. Cloud calls stay off the protection path.'),

 ('learning', 'Layer 7', 'The learning loop: skills graduate, recipes change between cohorts',
  'Robot learning reuses LeRobot’s recorder and trainers on our own episodes. Crop learning is A/B cohorts with one difference at a time.', d_learning, [
  ('Robot', 'Teleop ≈50 demonstrations per skill → LeRobotDataset → ACT → offline replay against the keyframe skill → shadow → limited pilot with rollback.'),
  ('Crop', 'Cohort A on Sep 30, B on Oct 3 from the same seed lot. Astra proposes C from their records; a person decides.'),
  ('Discipline', 'Simulated and real episodes are never mixed. Nothing is tuned on the evaluation set. A recipe never changes mid-cohort except as a recorded deviation.')],
  'No pretrained watering policy exists; both “water” datasets on the Hub are bottle transport. Training is optional inside the trip (see decisions).'),

 ('sequence', 'End to end', 'One cycle, thirteen messages, seven lanes',
  'Follow a single authorized pour from the first camera frame to the shadow classification.', d_sequence, [
  ('1–4', 'Adapters read; perception judges tray B fresh and reachable; rules pass; the orchestrator decides to ask.'),
  ('5–6', 'The viewer shows the edge-of-paper frame; a person authorizes one pour; the record names them.'),
  ('7–10', 'INTENT, then skills, then clamped joint targets, then ATTEMPT — written before the tilt.'),
  ('11–13', 'Perception verifies wetness and no spill; result VERIFIED; the packet goes to Jev in shadow.')],
  'Replace step 11 with “spill UNKNOWN” and the cycle enters PAUSED; replace a crash between 10 and 11 and the action stays UNKNOWN. Both paths are on the state-machine slide.'),

 ('simulator', 'Building it before the robot exists', 'The simulator is the same program with fakes plugged in',
  'Because everything above layer 1 talks to adapters, fakes make the whole farm program runnable on the Mac this week.', d_sim, [
  ('Fakes', 'FakeRobot with a kinematic model, FakeCamera replaying saved frames, FakeESP32 with scripted lux, FakeHuman with scripted answers.'),
  ('Faults', 'Stale frames, unplugged camera, serial gaps, overtemp, crash after ATTEMPT, silent human, wrong tray. Each has a test.'),
  ('Forever', 'The fakes become the regression suite. Every later change to the orchestrator runs against them first.')],
  'All simulated records carry simulated = true. They are never counted as farm evidence or used to tune thresholds for the real system.'),

 ('config', 'Config, safety, deferrals', 'One profile names what exists; code enforces what may move',
  'The config profile lists ports, camera identities, tray nests and enabled sensors. Safety limits are code and outrank every model.', d_config, [
  ('Profile', 'paper-tray-v0: two serial ports, three camera identities, esp32_light on, everything else off, tray B with its nest and pour keyframe.'),
  ('Safety', 'Calibrated joint ranges, max step per tick, servo temperature ceiling, 0.5 s watchdog, torque-off on disconnect after upright.'),
  ('Deferred', 'Driving, battery, pump, probe, scale, overhead camera, printer pickup, unsupervised pouring. Each returns when its own test passes.')],
  'A wrong profile is the most likely way to hurt the robot: the assembly guide’s port-discovery and calibration steps come before the first move.'),

 ('timeline', 'Build order', 'Five phases, five gates, Sep 26 → Oct 10',
  'Phase 0 is Mac-only and starts now. Each later phase opens only when the previous gate passes; a failed gate holds the previous level.', d_timeline, [
  ('Phase 0 · Sep 26–29', 'Adapters, fakes, evidence store, orchestrator on the simulator, viewer skeleton, ESP32 firmware. Gate: a fake cycle survives a crash after ATTEMPT and a restore.'),
  ('Phase 1 · Sep 30–Oct 2', 'Assemble, discover ports, calibrate, real adapters, keyframe skills, empty-bottle rehearsal. Gate: 10/10 pick-aim-return and a working stop.'),
  ('Phase 2 · Oct 3–5', 'Measured pours into a cup, then the full cycle on a real tray with the human gate. Gate: five complete supervised cycles, nothing UNKNOWN left open.'),
  ('Phases 3–4 · Oct 6–10', 'Observation windows 2 → 8 → 24 h, demo recording, first training attempt, offline eval, caretaker handover.')],
  'Status Sep 26: no code exists; the previous roadmap’s Sep 22–26 software work did not happen. The printed trays also do not exist yet, so Phase 2 assumes trays are printed or a household tray is substituted.'),

 ('decisions', 'Your call', 'Six decisions, each with a default',
  'Say “defaults” and Phase 0 starts as drawn. Change any line and the affected slides update.', d_decisions, [
  ('Cheap to change now', 'Tray identity, which arm holds what, viewer form. All three are config or keyframes.'),
  ('Scope', 'Jev/Astra timing, the light sensor and in-trip training each remove or add a Phase 0 work item.'),
  ('Not up for debate in V0', 'Human authorization before water, UNKNOWN blocks retry, base stays parked, safety rules outrank models.')],
  'Feedback on any slide is welcome; the deck is generated from one Python file so every diagram can be redrawn.'),
]

def render():
    n = len(SLIDES)
    out = '<link rel="stylesheet" href="roadmap.css"><script src="roadmap.js" defer></script><div class="deck" id="deck" data-count="%d">' % n
    out += '<div class="deck-bar" aria-label="Slideshow controls"><button type="button" id="deck-prev" aria-label="Previous slide">←</button><output id="deck-pos" aria-live="polite">01 / %02d</output><button type="button" id="deck-next" aria-label="Next slide">→</button><span class="deck-gap"></span><button type="button" id="deck-outline" aria-pressed="false">Outline</button><button type="button" id="deck-present" aria-pressed="false">Present</button></div><div class="deck-progress"><div id="deck-fill"></div></div>' % n
    out += '<ol class="deck-toc" id="deck-toc" hidden>'
    for i, s in enumerate(SLIDES, 1):
        out += f'<li><a href="#{s[0]}" data-go="{i - 1}"><span>{i:02}</span><strong>{e(s[2])}</strong><small>{e(s[1])}</small></a></li>'
    out += '</ol><div class="deck-slides">'
    for i, (key, kicker, title, lead, fig, points, limit) in enumerate(SLIDES, 1):
        out += f'<section id="{key}" class="slide" data-slide aria-labelledby="t-{key}"><header class="slide-head"><p class="kicker">{e(kicker)} · {i:02} / {n:02}</p><h2 id="t-{key}" tabindex="-1">{e(title)}</h2><p class="slide-lead">{e(lead)}</p></header><div class="slide-body"><figure class="slide-fig">{fig()}</figure><div class="slide-points"><dl>'
        for lab, text in points:
            out += f'<div><dt>{e(lab)}</dt><dd>{e(text)}</dd></div>'
        out += f'</dl></div></div><p class="slide-limit">{e(limit)}</p></section>'
    out += '</div><p class="deck-help">← → move · O outline · P present · Esc exit · Print shows every slide.</p>'
    out += ('<p class="deck-sources">Sources: <a href="https://github.com/Vector-Wangel/XLeRobot">XLeRobot repository</a> (software/src/robots/xlerobot_2wheels, model/SO101Robot.py, record.py, examples), '
            '<a href="https://xlerobot.readthedocs.io/en/latest/software/">XLeRobot software docs</a>, <a href="https://huggingface.co/docs/lerobot">LeRobot docs</a>, the September 20 redline deck, '
            '<a href="https://vector-wangel.github.io/XLeRobot-assets/videos/Real_demos/xlerobot030.mp4#t=56,60">the teleoperated watering clip</a>, <a href="hardware.html">hardware</a> and <a href="assembly.html">assembly</a> pages of this guide. '
            'This deck is a proposal generated from <code>handbook/roadmap.py</code>; it implements no robot control.</p></div>')
    return out
