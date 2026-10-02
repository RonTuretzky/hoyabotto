"""Community-projects deck: what the XLeRobot community page changes in our software.

Same engine as the roadmap deck (inline SVG from roadmap.py helpers). Colours:
green = already built, blue = to build on the farm program, orange = needs a
person or a decision, grey = unchanged / not used.
"""
from roadmap import INK, BLUE, GREEN, GREY, ORANGE, RED, KIND, svg, box, tag, label, arrow, elbow, render as render_deck
from html import escape as e


def key(x, y, items):
    out = ''
    for i, (k, t) in enumerate(items):
        stroke, fill = KIND[k]
        out += f'<rect x="{x + i * 230}" y="{y}" width="18" height="18" rx="4" fill="{fill}" stroke="{stroke}" stroke-width="2"/>' + label(x + i * 230 + 26, y + 14, t, 13)
    return out


LEGEND = [('ex', 'Built and pushed'), ('bd', 'Part of our program'), ('hu', 'Needs a person or a decision'), ('df', 'Unchanged / not used')]


def d_funnel():
    s = label(40, 44, '43 entries on the XLeRobot community page', 20, 700)
    groups = [(7, 'ex', 'useful to this plan'), (9, 'hu', 'related, nothing to take'), (27, 'df', 'not relevant to a parked, model-taught robot')]
    x = 40
    for n, k, _ in groups:
        stroke, fill = KIND[k]
        for i in range(n):
            s += f'<rect x="{x}" y="64" width="20" height="46" rx="4" fill="{stroke}" opacity="{1 if k != "df" else .45}"/>'
            x += 26
        x += 8
    s += box(40, 140, 350, 300, '7 · useful', ['xlerobot-onboard (installer + Claude skill)', 'RoboCrew (LLM agent loop)', 'xlerobot-mcp (MCP tool server)', 'Dexbotic (training for our exact robot)', 'Kinesthetic recorder (teach by hand)', 'XLeRobot-Pro measurement tools', 'Home Service Demo (grasp checkpoint)'], kind='ex', fs=14, tfs=18, cw=44)
    s += box(425, 140, 350, 300, '9 · related, nothing to take', ['AnchorVLA4D: pours on an XLeRobot, but no data, code or weights linked', 'Matcha Bot: liquid handling, no dataset', 'Perception Engine: head tracking of people', 'Laundry Bot, OneRobotAI, Grievous, XS-VLA', 'MakerMods parts, Cutting the Cord'], kind='hu', fs=14, tfs=18, cw=44)
    s += box(810, 140, 350, 300, '27 · not relevant', ['12 teleoperation (VR, Joy-Con, leader arms, ROS 2)', '4 simulation (Isaac, Gazebo, LeHome)', '3 different hardware (other arms, lift)', '5 voice agents and other stacks', '3 unrelated papers'], kind='df', fs=14, tfs=18, cw=44)
    s += box(40, 465, 1120, 70, 'Plus one find that is not on the page', ['LeRobot pull request #3282: automatic calibration for SO-101 arms. Open, not merged. Found through xlerobot-onboard.'], kind='hu', fs=14, tfs=16, cw=150)
    s += key(40, 560, LEGEND)
    return svg(1200, 600, s, 'Forty-three community entries sorted into three groups')


def d_stack():
    layers = [
        (7, 'Learning loop', [('policy server on this Mac', 'ex'), ('Dexbotic path · rented NVIDIA', 'df')]),
        (6, 'Review · viewer, Jev, Astra', [('unchanged', 'df')]),
        (5, 'Evidence store', [('soak-test log', 'ex'), ('who taught each pose', 'ex')]),
        (4, 'Care-cycle orchestrator', [('unchanged', 'df')]),
        (3, 'Perception', [('unchanged', 'df')]),
        (2, 'Skills', [('remote policy skill', 'ex'), ('by-hand teach · off by default', 'hu')]),
        (1, 'Device adapters', [('robot-test', 'ex'), ('calibration report', 'ex'), ('auto-calibration · not built', 'hu')]),
        (0, 'Hardware + upstream', [('bring-up skill', 'ex')]),
    ]
    s = label(40, 30, 'Our stack', 16, 700, GREY) + label(380, 30, 'What the community page adds to each layer', 16, 700, GREY)
    for i, (n, t, tags) in enumerate(layers):
        y = 46 + i * 60
        s += box(40, y, 320, 50, f'{n} · {t}', [], kind='bd' if n else 'ex', tfs=15)
        x = 380
        for text, k in tags:
            s += tag(x, y + 12, text, k, 13)
            x += len(text) * 13 * 0.62 + 30
    s += box(950, 46, 210, 230, 'New side door', ['farm mcp: any MCP client (Claude Code on the robot laptop) can read state, look through a camera, and run named skills.', 'No raw joint writes.'], kind='ex', fs=13, tfs=16, cw=26)
    s += box(950, 296, 210, 230, 'Untouched', ['Safety rules, authority ladder, UNKNOWN handling, parked base. Nothing from the community page relaxes them.'], kind='df', fs=13, tfs=16, cw=28)
    s += key(40, 560, LEGEND)
    return svg(1200, 600, s, 'The eight-layer stack with the change each community project brings')


def d_two():
    s = box(40, 40, 480, 330, 'Robot laptop · plugged into the robot', ['farm run --record', 'Skills, clamps, STOP, evidence', 'Cameras, motor boards, light sensor over USB', '', 'Sends: joint state + 3 camera frames', 'Receives: the next 50 actions', 'Every action still passes the clamps here'], kind='bd', fs=15, tfs=18, cw=52)
    s += box(680, 40, 480, 330, 'This Mac · M4 Max, 128 GB · the GPU server', ['Trains ACT (and SmolVLA) with LeRobot on Apple GPU', 'farm policy-server: loads a checkpoint, answers /act', 'Offline evaluation against held-out episodes', '', 'Never touches a motor', 'Holds no robot calibration', 'Can be switched off: the robot falls back to keyframes'], kind='ex', fs=15, tfs=18, cw=52)
    s += arrow(520, 130, 680, 130, 'recorded episodes')
    s += arrow(520, 215, 680, 215, 'observation')
    s += arrow(680, 285, 520, 285, 'action chunk')
    s += label(600, 330, 'same Wi-Fi', 13, 700, GREY, 'middle')
    s += box(40, 400, 480, 130, 'If the link drops', ['The skill stops, holds position and reports. It does not replay stale actions and never assumes the pour happened.'], kind='hu', fs=14, tfs=16, cw=60)
    s += box(680, 400, 480, 130, 'Dexbotic (the pipeline for our exact robot)', ['Needs Ubuntu and NVIDIA GPUs (RTX 4090 / A100 class). Not runnable on this Mac. Our recorder already writes the format it converts from, so it stays a rented-GPU option.'], kind='df', fs=14, tfs=16, cw=60)
    s += key(40, 560, LEGEND)
    return svg(1200, 600, s, 'Two computers: the robot laptop and this Mac as training and policy server')


def d_bringup():
    lanes = [('You', 150), ('Claude on the robot laptop', 450), ('farm program', 750), ('Robot', 1050)]
    s = ''
    for name, x in lanes:
        s += f'<rect x="{x - 120}" y="24" width="240" height="38" rx="8" fill="#e8eff6" stroke="{BLUE}" stroke-width="2"/>' + label(x, 49, name, 15, 700, BLUE, 'middle')
        s += f'<line x1="{x}" y1="62" x2="{x}" y2="540" stroke="{GREY}" stroke-width="1.5" stroke-dasharray="4 5"/>'
    steps = [
        (450, 750, 'farm devices --probe', 'ink'), (750, 1050, 'ping every servo ID', 'ink'),
        (450, 750, 'writes ports + camera indices into the profile', 'ink'),
        (150, 750, 'farm calibrate (your hands on the arms)', 'red'),
        (450, 750, 'farm robot-test', 'ink'), (750, 1050, 'read 14 joints, temperature, load', 'ink'),
        (150, 750, 'farm robot-test --move --ask (you watch)', 'red'), (750, 1050, 'nudge one joint, return', 'ink'),
        (450, 750, 'farm check; STATUS.md pushed', 'ink'),
    ]
    for i, (x1, x2, text, kind) in enumerate(steps):
        y = 96 + i * 49
        s += arrow(x1, y, x2, y, f'{i + 1}. {text}', kind)
    s += label(40, 572, 'Red = needs your hands or eyes on the robot. Everything else the agent runs and reads for you.', 14, 700, RED)
    return svg(1200, 600, s, 'Bring-up sequence across you, the agent, the farm program and the robot')


def d_calibration():
    s = box(40, 40, 350, 400, 'Today · by hand', ['farm calibrate', '', 'You hold each arm at mid-range, then sweep 14 joints to both stops.', '', 'Weak points: a joint left near a stop wraps its reading; a short sweep gives a short range. Neither is visible until something moves wrong.'], kind='ex', fs=17, tfs=20, cw=36)
    s += box(425, 40, 350, 400, 'Built · calibration report', ['farm calibration-report', '', 'Reads the saved calibration and prints each joint’s range in degrees.', '', 'Flags a wrapped reading, a range too short to be a full sweep, and left/right arms that disagree.', '', 'No motion. Runs in a second.'], kind='ex', fs=17, tfs=20, cw=36)
    s += box(810, 40, 350, 400, 'Candidate · automatic', ['LeRobot PR #3282 (open)', '', 'Each joint drives to its stops at low torque and detects the stall. No hands.', '', 'Tested by its author on one free-standing arm. On our cart a sweeping arm can reach the neck, the other arm and the tray rim.', '', 'About 1,800 lines of motion code we cannot test without the robot.'], kind='hu', fs=16, tfs=20, cw=38)
    s += arrow(390, 250, 425, 250)
    s += arrow(775, 250, 810, 250, '', 'grey', True)
    s += label(40, 490, 'The report is built and in the bring-up skill. Automatic calibration is not built: if you want it, it goes in', 15, 700)
    s += label(40, 514, 'one arm at a time with the other folded, and only after you say go.', 15, 700)
    s += key(40, 560, LEGEND)
    return svg(1200, 600, s, 'Three calibration paths: by hand, a new report, and a gated automatic option')


def d_teaching():
    rungs = [
        (360, 'ex', '1 · The vision model teaches (default, built)', ['farm teach / teach-all: the model looks through the cameras and moves the arm in clamped steps until the goal is met, then saves the keyframe.']),
        (220, 'hu', '2 · Teach by hand (fallback, built, off by default)', ['farm teach --by-hand --who NAME: one arm goes limp, you place it, press ENTER, the pose is saved under your name. From the kinesthetic recorder. It breaks the no-human-operation rule, so the command refuses until the profile says teaching.by_hand: true.']),
        (80, 'bd', '3 · A trained policy runs the motion (later)', ['Trained on this Mac from the robot’s own recorded runs, served to the robot laptop, clamped like everything else. Shadow first.']),
    ]
    s = ''
    for y, k, title, lines in rungs:
        s += box(260, y, 900, 118, title, lines, kind=k, fs=15, tfs=18, cw=108)
    s += f'<line x1="215" y1="478" x2="215" y2="90" stroke="{INK}" stroke-width="3" marker-end="url(#ah)"/>'
    s += label(40, 280, 'Move up a rung', 15, 700, GREY) + label(40, 302, 'only when the one', 15, 700, GREY) + label(40, 324, 'below is not enough', 15, 700, GREY)
    s += box(260, 495, 900, 50, 'Every saved keyframe records who taught it: model, hand, or policy', [], kind='bd', tfs=15)
    s += key(40, 560, LEGEND)
    return svg(1200, 600, s, 'Teaching ladder: model first, hand as fallback, trained policy later')


def d_remote():
    s = box(40, 60, 250, 150, 'Policy skill', ['on the robot laptop', 'asks for actions when a learned motion is selected'], kind='ex', fs=14, tfs=17, cw=28)
    s += box(360, 60, 250, 150, 'Policy server', ['on this Mac', 'checkpoint loaded once; answers in tens of milliseconds'], kind='ex', fs=14, tfs=17, cw=28)
    s += box(680, 60, 220, 150, 'Clamps', ['on the robot laptop', 'step size, joint range, temperature, watchdog'], kind='ex', fs=14, tfs=17, cw=24)
    s += box(970, 60, 190, 150, 'Motors', ['one small step at a time'], kind='hw', fs=14, tfs=17, cw=20)
    s += arrow(290, 110, 360, 110) + arrow(360, 165, 290, 165)
    s += label(325, 98, 'obs', 12, 700, GREY, 'middle') + label(325, 188, 'actions', 12, 700, GREY, 'middle')
    s += elbow([(165, 210), (165, 262), (790, 262), (790, 212)])
    s += label(478, 254, 'every action goes through the clamps before it reaches a motor', 14, 700, INK, 'middle')
    s += arrow(900, 135, 970, 135)
    rows = [('Server slow or unreachable', 'Skill stops, holds position, reports. No stale actions are replayed.'),
            ('Action outside the clamps', 'Clipped to the allowed step; repeated clipping ends the skill.'),
            ('STOP pressed in the viewer', 'Wins immediately, as now. The server is not asked.'),
            ('Server switched off', 'Farm keeps working on keyframes. A policy is never required to water.')]
    for i, (a, b) in enumerate(rows):
        y = 300 + i * 58
        s += box(40, y, 360, 48, a, [], kind='hu', tfs=15)
        s += f'<rect x="420" y="{y}" width="740" height="48" rx="9" fill="#fffefa" stroke="{GREY}" stroke-width="1.5"/>' + label(436, y + 30, b, 15)
    return svg(1200, 600, s, 'A remote policy: observation out, actions back, clamps stay on the robot laptop')


def d_soak():
    x0, y0, w, h = 90, 60, 620, 400
    s = f'<rect x="{x0}" y="{y0}" width="{w}" height="{h}" fill="#fffefa" stroke="{GREY}" stroke-width="1.5"/>'
    for t in (30, 40, 50, 60):
        y = y0 + h - (t - 25) / 40 * h
        s += f'<line x1="{x0}" y1="{y:.0f}" x2="{x0 + w}" y2="{y:.0f}" stroke="{GREY}" stroke-width=".6" stroke-dasharray="3 5"/>' + label(x0 - 10, y + 5, f'{t} °C', 13, 400, GREY, 'end')
    for m in (0, 10, 20, 30):
        x = x0 + m / 30 * w
        s += label(x, y0 + h + 22, f'{m} min', 13, 400, GREY, 'middle')
    yc = y0 + h - (55 - 25) / 40 * h
    s += f'<line x1="{x0}" y1="{yc:.0f}" x2="{x0 + w}" y2="{yc:.0f}" stroke="{RED}" stroke-width="2.5"/>' + label(x0 + w - 8, yc - 8, 'ceiling 55 °C: test stops, arm goes to rest', 13, 700, RED, 'end')
    import math
    pts = []
    for i in range(61):
        m = i / 2
        temp = 31 + 17 * (1 - math.exp(-m / 9))
        pts.append(f'{x0 + m / 30 * w:.1f},{y0 + h - (temp - 25) / 40 * h:.1f}')
    s += f'<polyline points="{" ".join(pts)}" fill="none" stroke="{BLUE}" stroke-width="3"/>'
    s += label(x0 + 330, y0 + 178, 'hottest servo', 14, 700, BLUE)
    s += label(x0, y0 - 14, 'Illustrative shape only. No temperatures have been measured on this robot yet.', 13, 700, ORANGE)
    s += box(760, 60, 400, 190, 'farm soak', ['Holds a taught pose (the pour pose is the hardest) and logs every servo’s temperature and load every two seconds.', 'Stops at the ceiling or after the set time.'], kind='ex', fs=14, tfs=18, cw=50)
    s += box(760, 270, 400, 190, 'What it tells you', ['Peak temperature and which servo', 'How fast it is still rising at the end', 'Whether an hourly cycle leaves time to cool', 'Saved as a CSV next to the evidence'], kind='ex', fs=14, tfs=18, cw=50)
    s += label(90, 530, 'From the XLeRobot-Pro measurement protocols. Run once before the robot is left alone for days.', 15, 700)
    return svg(1200, 600, s, 'Soak test: servo temperature while a pose is held, with a hard ceiling')


def d_mcp():
    s = box(40, 60, 250, 420, 'MCP client', ['Claude Code on the robot laptop, started in the repo.', '', 'It can see what the robot sees while it helps you bring it up, instead of asking you to describe it.'], kind='bd', fs=14, tfs=18, cw=30)
    s += arrow(290, 270, 350, 270)
    s += box(350, 60, 260, 420, 'Look', ['get_state: joints, temperatures, what the cycle is doing', 'get_camera_image: head or wrist', 'list_keyframes', 'recent evidence'], kind='ex', fs=14, tfs=18, cw=30)
    s += box(630, 60, 260, 420, 'Act through skills', ['stop', 'go_rest', 'go_keyframe (a saved pose)', '', 'Same clamps and the same STOP as the viewer. Every call is written to the evidence store.'], kind='bd', fs=14, tfs=18, cw=30)
    s += box(910, 60, 250, 420, 'Never exposed', ['Raw joint targets', 'Wheel commands', 'Changing limits', 'Authorizing a pour', 'Editing the profile'], kind='bad', fs=14, tfs=18, cw=28)
    s += label(40, 525, 'Modelled on xlerobot-mcp, which exposes raw servo positions. Ours stops at the skill layer, because our rule is that', 15, 700)
    s += label(40, 548, 'a model never writes a joint command directly.', 15, 700)
    return svg(1200, 600, s, 'MCP tool surface: what an agent may look at, what it may do, and what is never exposed')


def d_invariants():
    rows = [
        ('Water moves only when', 'rules pass and a named person, or Jev at approve level, authorizes', 'unchanged'),
        ('A pour with an unknown result', 'blocks every cycle until a person reconciles it', 'unchanged'),
        ('Models and joint commands', 'no model writes a joint target directly; policies and agents go through clamped skills', 'unchanged'),
        ('The base', 'stays parked; nothing sends wheel commands', 'unchanged'),
        ('Limits', 'live in code and the profile; no community tool relaxes them', 'unchanged'),
        ('Human operation', 'still none by default; teaching by hand exists only as a fallback you choose per pose', 'one new exception'),
        ('Calibration', 'still by hand unless you approve the automatic path', 'one new option'),
    ]
    s = label(40, 36, 'Rule', 14, 700, GREY) + label(330, 36, 'What it says', 14, 700, GREY) + label(1010, 36, 'After this work', 14, 700, GREY)
    for i, (a, b, c) in enumerate(rows):
        y = 52 + i * 70
        k = 'df' if c == 'unchanged' else 'hu'
        s += box(40, y, 270, 58, a, [], kind='bd', tfs=15)
        s += box(325, y, 660, 58, '', [b], kind='df', fs=14, tfs=2, cw=92)
        s += box(1000, y, 160, 58, c, [], kind=k, tfs=14)
    s += key(40, 560, LEGEND)
    return svg(1200, 600, s, 'Seven rules and whether this work changes them')


def d_plan():
    done = ['Bring-up skill, farm robot-test', 'Calibration report', 'Policy server + remote policy', 'Soak test', 'Teach by hand (off by default)', 'MCP tool surface', 'Review of all 43 entries']
    build = ['Automatic calibration: not built, waits for your decision', '', 'On the robot, in order: probe, calibrate, calibration-report, robot-test, teach, soak, then record episodes for this Mac to train on']
    s = box(40, 40, 340, 250, 'Built today, pushed', done, kind='ex', fs=15, tfs=18, cw=38)
    s += box(410, 40, 380, 250, 'Still open', build, kind='hu', fs=15, tfs=18, cw=42)
    s += box(820, 40, 340, 250, 'Not doing', ['Dexbotic on this Mac (needs NVIDIA)', 'Any teleoperation stack', 'ROS 2, simulators, voice', 'Raw-servo MCP tools'], kind='df', fs=15, tfs=18, cw=38)
    s += label(40, 335, 'Three decisions that are yours', 18, 700, ORANGE)
    q = [('Teaching by hand', 'Allowed as a per-pose fallback, or keep the rule absolute? It is built and switched off; one profile line turns it on.'),
         ('Automatic calibration', 'Run unproven limit-seeking motion on the cart, or stay with the hand sweep plus the report? I recommend the report.'),
         ('How episodes reach this Mac', 'Copy over the local network (default), or a private Hugging Face dataset.')]
    for i, (a, b) in enumerate(q):
        y = 352 + i * 64
        s += box(40, y, 270, 54, a, [], kind='hu', tfs=15)
        s += box(325, y, 835, 54, '', [b], kind='df', fs=14, tfs=2, cw=118)
    s += key(40, 560, LEGEND)
    return svg(1200, 600, s, 'What is done, what is built next, what is skipped, and three open decisions')


SLIDES = [
 ('summary', 'Community page review · Oct 2', '43 community projects, 7 that matter, 8 changes to our software',
  'Most of the page is about people driving the robot. What is left sharpens bring-up, calibration, teaching and training, without touching the safety model. Seven of the eight changes are built; automatic calibration waits for your decision.', d_funnel, [
  ('Checked', 'Every entry’s summary, and the README of the eleven closest. Nothing was installed or run on the robot.'),
  ('Biggest gap found', 'Nobody has published watering or pouring data for this robot. The one paper that pours on an XLeRobot released nothing.'),
  ('Full list', 'software/docs/community-projects.md in the repo.')],
  'An earlier message said 28 entries were not relevant; the correct count is 27 (one project was counted twice).'),

 ('stack', 'Where it lands', 'Each useful project touches one layer; four layers do not change',
  'Perception, the care cycle, the viewer and the authority ladder are untouched. The changes sit at the bottom (devices, calibration) and the top (training).', d_stack, [
  ('Bottom', 'Bring-up skill, robot-test and the calibration report are built. Automatic calibration is not built and waits for your decision.'),
  ('Middle', 'A remote policy skill, and teaching by hand as a fallback that is off unless the profile allows it.'),
  ('Top', 'This Mac becomes the training and policy server.'),
  ('Side', 'An MCP door so an agent on the robot laptop can look and run named skills.')],
  'Green is built and passes on the simulator. None of it has run on the real robot yet.'),

 ('two-computers', 'This Mac as the GPU server', 'Two computers: one holds the robot, one holds the model',
  'The robot laptop keeps every safety decision. This Mac trains on the robot’s recorded runs and answers “what next?” over the network.', d_two, [
  ('Why split', 'Training and a large policy would compete with the control loop on the robot laptop. An M4 Max with 128 GB handles ACT comfortably; the earlier pipeline test trained 5,000 steps here in about an hour.'),
  ('What crosses the wire', 'Joint state and three camera frames one way; a chunk of actions the other. No motor commands leave this Mac.'),
  ('Dexbotic', 'Written for our exact robot but needs NVIDIA hardware, so it is a rented-GPU option, not something this Mac runs.')],
  'Built and checked on this Mac alone: the trained checkpoint, served here, answered in 28 ms and drove the simulator through the remote client. It has not run between two machines, and the only policy that exists is a motion prior from someone else’s pouring data, not deployable.'),

 ('bringup', 'Built today', 'Bring-up: the agent runs the checks, you do the parts that need hands',
  'The farm-bringup skill is in the repo. On the robot laptop: git pull, start Claude Code in the folder, type /farm-bringup.', d_bringup, [
  ('From the community', 'xlerobot-onboard’s skill, rewritten for our kit: its servo numbers are for the three-wheel robot and would have been wrong here.'),
  ('Kept from it', 'Stop, report, ask. Never guess at hardware faults. The wrist-roll wrap trap during calibration.'),
  ('New', 'farm robot-test: reads every joint, then nudges each one and asks you to confirm the named part moved. Catches swapped boards and swapped head servos.')],
  'robot-test passes on the simulator. It has not run on the real robot yet.'),

 ('calibration', 'Calibration', 'Calibration: keep the hand sweep, add a report, hold the automatic option',
  'Calibration is the one step where a person still moves the arms. The community has an automatic method; it is unproven on a cart.', d_calibration, [
  ('Report', 'Turns a silent bad calibration into a visible one, before anything moves.'),
  ('Automatic', 'Would remove the last hands-on step. The risk is collision during limit-seeking, which the author’s single-arm test never met.'),
  ('Head', 'RoboCrew uses the same head mapping as we assumed (pan 7, tilt 8) and limits tilt to 0–85°. The report checks our tilt range against that.')],
  'Expected joint ranges for the report come from the SO-101 design; thresholds are deliberately loose until we have one good calibration to compare with.'),

 ('teaching', 'Teaching', 'Teaching: the model first, your hands only as a chosen fallback',
  'Nothing changes by default. If the vision model cannot reach a pose, there is now a second way that needs no extra hardware.', d_teaching, [
  ('Why a fallback', 'The model-taught approach has never run on the real robot. If one pose fails during the trip, the alternative today is no farm at all.'),
  ('What RoboCrew shows', 'Its agent also moves arms through saved poses or trained policies, not free-form joint commands. Same conclusion as ours, reached independently.'),
  ('Recorded', 'Hand-taught poses are marked as such in the evidence, so later analysis can separate them.')],
  'Teaching by hand contradicts “no human operation”. It is built, refuses to run by default, and is one of the decisions on the last slide.'),

 ('remote-policy', 'Learning', 'A trained policy is served from this Mac and clamped on the robot',
  'The policy skill already exists and runs a local checkpoint. The change is where the model lives.', d_remote, [
  ('Protocol', 'One HTTP call per chunk: state and JPEG frames in, about 50 actions out.'),
  ('Failure handling', 'A slow or missing server ends the skill safely. The care cycle treats it like any other failed step.'),
  ('Training', 'Same commands as now (farm.learning.train), run here against episodes copied from the robot laptop.')],
  'Built. There are no episodes from our robot yet, so this is plumbing until the robot has run recorded cycles. The care cycle does not call a policy yet; today it is reached through farm policy-test.'),

 ('soak', 'Endurance', 'A soak test before the robot is left alone',
  'We already refuse to act above 55 °C. What we do not know is how close a held pour pose gets, or how long cooling takes.', d_soak, [
  ('Source', 'XLeRobot-Pro’s thermal-endurance protocol: hold a pose under load, log per-servo telemetry, abort at the ceiling.'),
  ('Ours', 'One command, one CSV, one summary line. No licence on their repo, so the idea is reused, not the code.'),
  ('When', 'After the pour pose is taught, before the first unattended hour.')],
  'The command is built and tested with a simulated heating servo. The curve is drawn to show the idea. It is not data.'),

 ('mcp', 'Agent access', 'An MCP door that stops at the skill layer',
  'The community MCP server hands an agent raw servo control. Ours gives it eyes and named skills, under the existing clamps.', d_mcp, [
  ('Use', 'Once the robot is calibrated, the agent can read joint state and look through a camera itself, instead of asking you to describe it.'),
  ('Use later', 'A remote check-in while you are away: what does the head camera see, what is the cycle doing.'),
  ('Boundary', 'Authorizing water stays with a named person or Jev. An MCP client cannot do it.')],
  'Built; a live session against the simulator listed the seven tools, returned a camera frame and refused motion after STOP. While farm mcp runs it holds the robot’s serial ports, so no other farm command can run at the same time; it is started deliberately, not automatically.'),

 ('invariants', 'What does not change', 'Five rules untouched, two with a new option',
  'None of this is a reason to loosen the safety model. The two changes are both opt-in.', d_invariants, [
  ('Why this slide', 'Community code is written for demos with a person watching. Ours has to run unattended, so anything adopted goes through the existing rules.')],
  'If you want either orange row to stay absolute, say so and that piece is dropped.'),

 ('plan', 'Plan', 'Seven built today, one waiting, three decisions for you',
  'Everything built is tested on the simulator (65 tests). Nothing in this deck has run on the real robot yet.', d_plan, [
  ('Verified here', 'Policy server with the real checkpoint on this Mac; a live MCP session; soak with a simulated heating servo; the calibration report on synthetic files.'),
  ('Not verified', 'Anything on the real robot, and the policy path between two machines.')],
  'This deck is generated from handbook/community.py. The repo’s STATUS.md says what has actually run on the robot.'),
]

SOURCES = ('<p class="deck-sources">Sources: <a href="https://xlerobot.readthedocs.io/en/latest/relatedworks/index.html">XLeRobot community use cases</a> (43 entries, collected 2026-09-18), '
           'the README of each project named, <a href="https://github.com/huggingface/lerobot/pull/3282">LeRobot PR #3282</a>, and '
           '<a href="https://github.com/RonTuretzky/xlerobot-farm/blob/main/software/docs/community-projects.md">our full review</a>. '
           'Generated from <code>handbook/community.py</code>.</p></div>')


def render():
    return render_deck(SLIDES, SOURCES)
