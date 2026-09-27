"""The step-by-step guide: one page that takes a person from nothing to a running farm.

Every step has: why it exists, exactly what to do (commands included), what it needs,
a time estimate, links, a visual, and a checkbox that persists in the browser.
Diagrams are drawn here or imported from the roadmap deck so they stay consistent.
"""
from html import escape as e

from roadmap import BLUE, GREEN, GREY, INK, ORANGE, RED, arrow, box, d_cycle, d_hardware, d_models, d_parts, d_viewer, elbow, label, svg, tag, wrap

SW = "https://github.com/RonTuretzky/xlerobot-farm/tree/main/software"

# ------------------------------------------------------------------ diagrams

def d_journey():
    phases = [('0', 'Set up the Mac', '2 h'), ('1', 'Print', '1–4 days'), ('2', 'Electronics', '2 h'), ('3', 'Assemble', '4 h'), ('4', 'Bring-up', '2 h'),
              ('5', 'Station', '2 h'), ('6', 'Teach', '1 h'), ('7', 'Water', '2 h'), ('8', 'Run', 'daily'), ('9', 'Hand over', '2 h')]
    s = ''
    for i, (n, t, when) in enumerate(phases):
        x = 30 + i * 116
        k = 'ex' if i == 0 else 'bd'
        s += box(x, 40, 104, 80, f'{n} · {t}', [when], kind=k, fs=13, tfs=12)
        if i < len(phases) - 1:
            s += arrow(x + 104, 80, x + 116, 80)
    s += label(30, 160, 'Phases 0–2 happen before the robot exists (in NY or on arrival). Phases 3–9 start September 30 in Japan.', 14, 400, GREY)
    s += label(30, 186, 'Phase 1 runs in the background: start the printer first, then do everything else while it prints.', 14, 400, GREY)
    return svg(1200, 210, s, 'Ten phases of the project')


def d_print_plan():
    jobs = [('Cress planter (6 pieces)', 10.0, 275, 'bd'), ('Nest plate A', 6.7, 93, 'bd'), ('Nest plate B', 6.7, 93, 'bd'), ('Light paddle', 0.9, 13, 'bd'),
            ('Bottle rest', 1.1, 16, 'bd'), ('Tag tiles id1 + id2', 0.2, 3, 'bd'), ('Modular table (optional)', 45.0, 1000, 'df')]
    s = label(30, 30, 'Print queue on the AnkerMake M5C (hours, estimated; PLA grams)', 15, 700, GREY)
    scale = 1000 / 48.0
    for i, (n, h, g, k) in enumerate(jobs):
        y = 60 + i * 44
        stroke, fill = {'bd': (BLUE, '#e8eff6'), 'df': (GREY, '#f1f2f2')}[k]
        w = max(8, h * scale)
        s += label(30, y + 20, n, 13.5, 700, INK)
        s += f'<rect x="240" y="{y}" width="{w:.0f}" height="28" rx="5" fill="{fill}" stroke="{stroke}" stroke-width="2"{" stroke-dasharray=\"6 4\"" if k == "df" else ""}/>'
        s += label(248 + w, y + 19, f'{h:g} h · {g} g', 12.5, 400, INK)
    s += label(30, 390, 'Without the table: ≈ 26 h and ≈ 0.5 kg. With the table: ≈ 70 h and ≈ 1.5 kg (two spools).', 14, 700, INK)
    s += label(30, 414, 'Planter time is the K1 Max slice (10h06m); the M5C is usually faster. Nest/paddle/rest are volume estimates at ~14 g/h. Table = creator’s 63 h on a Prusa MINI × ~0.7.', 12.5, 400, GREY)
    return svg(1200, 430, s, 'Print plan with hours and grams')


def d_wiring():
    s = box(60, 60, 330, 300, 'ELEGOO ESP32 (ESP-WROOM-32)', ['USB-C → powered hub (data cable!)', '', 'Pins used:', '3V3   → BH1750 VCC', 'GND   → BH1750 GND', 'GPIO21 → BH1750 SDA', 'GPIO22 → BH1750 SCL', '', 'ADDR on the BH1750 left unconnected (= 0x23)'], kind='hw', fs=14, tfs=16)
    s += box(560, 100, 260, 200, 'BH1750 breakout', ['VCC', 'GND', 'SCL', 'SDA', 'ADDR (leave open)'], kind='hw', fs=14, tfs=16)
    for i, (y1, y2, t, c) in enumerate(((186, 150, '3V3', RED), (208, 176, 'GND', INK), (252, 228, 'SDA', BLUE), (230, 202, 'SCL', GREEN))):
        s += f'<line x1="390" y1="{y1}" x2="560" y2="{y2}" stroke="{c}" stroke-width="3"/>' + label(470, (y1 + y2) / 2 - 6, t, 12, 700, c, 'middle')
    s += box(880, 60, 280, 120, 'Paddle', ['Board sits in the paddle pocket, sensor face UP. Four flexible wires run down the handle groove to the ESP32 on the cart.'], kind='bd', fs=13, tfs=15)
    s += box(880, 200, 280, 160, 'Check before power', ['No short between 3V3 and GND (multimeter or careful look).', 'The breakout accepts 3.3 V (most BH1750 boards do; read the silkscreen).', 'Headers soldered if they arrived loose.'], kind='bad', fs=13, tfs=15)
    s += label(60, 400, 'Firmware prints one JSON line per reading at 115200 baud: {"seq":812,"t_ms":93410,"lux":412.5,"ok":true}. farm light-monitor shows it live.', 13.5, 400, GREY)
    return svg(1200, 420, s, 'ESP32 to BH1750 wiring')


def d_station():
    s = f'<rect x="60" y="80" width="520" height="360" rx="10" fill="#eeefec" stroke="{GREY}" stroke-width="2"/>' + label(70, 104, 'table (any stable surface at the right height)', 13, 700, GREY)
    s += f'<rect x="640" y="60" width="240" height="420" rx="10" fill="#fffefa" stroke="{INK}" stroke-width="2.5"/>' + label(760, 90, 'robot cart (parked)', 13, 700, INK, 'middle')
    s += f'<circle cx="700" cy="180" r="14" fill="#e8eff6" stroke="{BLUE}" stroke-width="2"/>' + label(700, 214, 'left arm', 12, 700, BLUE, 'middle')
    s += f'<circle cx="820" cy="180" r="14" fill="#e8eff6" stroke="{BLUE}" stroke-width="2"/>' + label(820, 214, 'right arm', 12, 700, BLUE, 'middle')
    s += f'<rect x="745" y="120" width="30" height="20" rx="4" fill="#e8eff6" stroke="{BLUE}" stroke-width="2"/>' + label(760, 112, 'head cam', 11, 400, GREY, 'middle')
    # reach arcs
    s += f'<path d="M 700 180 m -300 0 a 300 300 0 0 1 300 -300" fill="none" stroke="{ORANGE}" stroke-width="1.5" stroke-dasharray="6 5"/>'
    s += f'<path d="M 820 180 m -300 0 a 300 300 0 0 1 300 -300" fill="none" stroke="{ORANGE}" stroke-width="1.5" stroke-dasharray="6 5"/>'
    s += label(430, 60, 'comfortable reach ≈ 15–30 cm from each shoulder', 12, 700, ORANGE, 'middle')
    s += f'<rect x="120" y="140" width="200" height="120" rx="6" fill="#e8eff6" stroke="{BLUE}" stroke-width="2.5"/>' + label(220, 205, 'nest A · tray A', 14, 700, BLUE, 'middle')
    s += f'<rect x="350" y="140" width="200" height="120" rx="6" fill="#e8eff6" stroke="{BLUE}" stroke-width="2.5"/>' + label(450, 205, 'nest B · tray B', 14, 700, BLUE, 'middle')
    s += f'<circle cx="520" cy="360" r="26" fill="#fffefa" stroke="{INK}" stroke-width="2.5"/>' + label(520, 404, 'bottle rest (right side)', 12, 700, INK, 'middle')
    s += f'<rect x="120" y="345" width="130" height="18" rx="4" fill="#fffefa" stroke="{INK}" stroke-width="2"/>' + label(185, 390, 'paddle rest (left side)', 12, 700, INK, 'middle')
    s += box(920, 60, 250, 150, 'Heights', ['Tray rim about 5–10 cm below the arm shoulders; the spout must clear the rim with the bottle upright.', 'Measure before printing table legs.'], kind='bd', fs=13, tfs=15)
    s += box(920, 230, 250, 120, 'Nothing moves', ['Nests screwed or taped down. Rests taped. Cart brakes on. If anything shifts, re-run farm teach-all.'], kind='bad', fs=13, tfs=15)
    s += box(920, 370, 250, 110, 'Light', ['Bright, even, no direct sun on the cameras. A lamp behind the head camera is fine.'], kind='bd', fs=13, tfs=15)
    return svg(1200, 500, s, 'Station layout from above')


def d_day1():
    steps = [('farm devices --probe', 'ports + camera snapshots', 'bd'), ('edit profile', 'ports, camera indices', 'hu'), ('farm calibrate', 'one-time joint ranges', 'hu'),
             ('farm check', 'model confirms cameras', 'bd'), ('farm teach-all', 'model teaches poses', 'bd'), ('farm once (empty)', 'rehearsal, no water', 'bd'),
             ('farm cup-test', 'measure a pour', 'hu'), ('farm once --tray B', 'first real cycle', 'bd'), ('farm run --every 3600', 'hourly, viewer + Telegram', 'bd')]
    s = ''
    for i, (c, d, k) in enumerate(steps):
        x = 30 + (i % 5) * 232; y = 40 + (i // 5) * 150
        s += box(x, y, 210, 96, c, [d], kind=k, fs=13, tfs=14)
        if i < len(steps) - 1 and i % 5 != 4:
            s += arrow(x + 210, y + 48, x + 232, y + 48)
        if i == 4:
            s += elbow([(x + 105, y + 96), (x + 105, y + 120), (30 + 105, y + 120), (30 + 105, y + 150)])
    s += label(30, 330, 'Orange = a person does something physical or types a value (setup). Blue = the program and the model do it.', 13.5, 400, GREY)
    return svg(1200, 350, s, 'Day-one command sequence')


def d_authority_ladder():
    s = ''
    for i, (lvl, txt) in enumerate((('shadow', 'Jev answers, is recorded, decides nothing. You authorize every pour in the viewer.'),
                                    ('route', 'Jev’s “reinspect / pause” routes are honoured. You still authorize pours.'),
                                    ('approve', 'Jev authorizes routine pours (p ≥ 0.85) when rules pass. You are notified.'))):
        x = 40 + i * 390
        s += box(x, 40, 360, 140, f'{i + 1} · {lvl}', [txt], kind='bd', fs=14, tfs=18)
        if i < 2:
            s += arrow(x + 360, 110, x + 390, 110, 'earned', above=True)
    s += label(40, 230, 'Promotion is automatic after jev_shadow_cycles agreeing cycles (profile: 10). A spill at reconciliation demotes. Any named person can set the level in the viewer.', 14, 400, GREY)
    return svg(1200, 260, s, 'Authority ladder')


# ------------------------------------------------------------------ steps
# (phase, id, title, minutes, why, do[list of html], needs[list], links[(text,url)], visual html or None, check text, gotchas)
PHASES = [
 ('p0', '0 · Set up the Mac (do this today, anywhere)'),
 ('p1', '1 · Print the parts (start the printer, then keep going)'),
 ('p2', '2 · Electronics: the light sensor'),
 ('p3', '3 · Assemble the robot (Sep 30)'),
 ('p4', '4 · Bring the robot up on the Mac'),
 ('p5', '5 · Build the station'),
 ('p6', '6 · Let the model teach the robot'),
 ('p7', '7 · Water for the first time'),
 ('p8', '8 · Run it every day'),
 ('p9', '9 · Grow, review, hand over'),
]

def code(c):
    return f'<pre><code>{e(c)}</code></pre>'

STEPS = [
 # ---------------- phase 0
 ('p0', 'clone', 'Get the code and the tools', 20,
  'Everything runs from one folder on the Mac. uv makes the Python environment in seconds; the tests prove the install before any hardware is involved.',
  ['Install <a href="https://docs.astral.sh/uv/getting-started/installation/">uv</a> and Python 3.12 (<code>brew install uv python@3.12</code>).',
   'Clone the repo and create the environment:' + code('git clone https://github.com/RonTuretzky/xlerobot-farm.git\ncd xlerobot-farm/software\nuv venv --python 3.12 .venv && . .venv/bin/activate\nuv pip install -e .\npython -m pytest -q          # expect: 25 passed'),
   'Run the whole program on fakes and open the viewer:' + code('farm sim --auto-answer\n# in a browser: http://localhost:8765'),
   'You should see two POURED cycles and the viewer with frames, questions and the STOP button.'],
  ['MacBook (the Neo)', 'internet'], [('Software README', SW + '#readme'), ('uv install', 'https://docs.astral.sh/uv/getting-started/installation/')], None,
  'pytest passed and farm sim showed two POURED cycles', 'If pytest cannot import lerobot, the venv is not active: run <code>. .venv/bin/activate</code> first.'),

 ('p0', 'accounts', 'Log in to the model backends', 15,
  'Vision (looking at trays, teaching poses) runs on your Claude subscription through the claude command line. Jev and Astra run through OpenRouter.',
  ['Install and log in to Claude Code: <a href="https://docs.claude.com/en/docs/claude-code/quickstart">quickstart</a>, then run <code>claude</code> once and complete <code>/login</code>. Test headless: ' + code('claude -p "reply with the JSON {\\"ok\\": true}" --output-format json'),
   '<strong>Rotate the OpenRouter key</strong> (it was pasted in chat) at <a href="https://openrouter.ai/settings/keys">openrouter.ai/settings/keys</a>, put a small credit limit on it, then:' + code('cp .env.example .env\n# edit .env: OPENROUTER_API_KEY=sk-or-v1-…'),
   'Check both from the farm:' + code('farm review -p sim      # Astra proposal from the simulator’s evidence (uses OpenRouter)')],
  ['Claude subscription', 'OpenRouter account with a few dollars'], [('OpenRouter keys', 'https://openrouter.ai/settings/keys'), ('Claude Code quickstart', 'https://docs.claude.com/en/docs/claude-code/quickstart')], None,
  'claude -p returned JSON and farm review printed a proposal', 'The profile caps OpenRouter spend at $5/day (<code>llm.max_cost_usd_per_day</code>); when hit, the robot refuses to pour rather than run blind.'),

 ('p0', 'telegram', 'Optional: questions on your phone (Telegram)', 15,
  'The viewer works on the same Wi-Fi. Telegram lets the robot ask you anywhere: one message with buttons, your tap is the authorization record.',
  ['Message <a href="https://t.me/BotFather">@BotFather</a>: <code>/newbot</code>, copy the token.', 'Send your new bot any message, then open <code>https://api.telegram.org/bot&lt;TOKEN&gt;/getUpdates</code> and copy your chat id.',
   'Add to <code>.env</code>:' + code('TELEGRAM_BOT_TOKEN=123456:ABC…\nTELEGRAM_CHAT_IDS=987654321'),
   'From now on every question goes to the viewer <em>and</em> Telegram; the first answer wins.'],
  ['phone with Telegram'], [('BotFather', 'https://t.me/BotFather')], None, 'bot token and chat id in .env', 'Only the chat ids listed can answer; anything else is ignored and logged.'),

 ('p0', 'slicer', 'Install a slicer for the AnkerMake M5C', 15,
  'The M5C accepts G-code from AnkerMake Studio, PrusaSlicer or Cura. You only need one; AnkerMake Studio has the printer profile built in and sends jobs over Wi-Fi.',
  ['Install <a href="https://support.ankermake.com/s/article/Download">AnkerMake Studio</a> (or PrusaSlicer with the M5C profile) and pair the printer.',
   'Default profile for every part: 0.2 mm layers, 3 walls, 20 % infill, PLA 210–220 °C, bed 50–60 °C, no supports.',
   'Never run the Prusa MINI G-code from the modular-table download on the M5C: it is sliced for another machine.'],
  ['AnkerMake M5C on the same Wi-Fi', '1.5 kg PLA (two spools) if you print the table, 0.5 kg without'], [('AnkerMake downloads', 'https://support.ankermake.com/s/article/Download'), ('M5C specs (Fabbaloo)', 'https://www.fabbaloo.com/news/hands-on-with-the-ankermake-m5c-3d-printer-part-1')], None,
  'slicer installed and printer paired', 'The 0.4 mm nozzle kit for the M5C is a different part from the K1 Max kit.'),

 # ---------------- phase 1
 ('p1', 'print-plan', 'Print in this order', 5,
  'Trays first (they gate the growing), then the nests (they gate the robot), then the small parts. The table is optional and long; start it last or skip it.',
  ['Queue: cress planter (6 pieces) → nest A → nest B → paddle + bottle rest + tags on one plate → table (optional).',
   'Download: <a href="downloads/planter-source-stls.zip">planter STLs</a> · <a href="downloads/parts/nest_cress.stl">nest_cress.stl</a> (print twice) · <a href="downloads/parts/paddle_bh1750.stl">paddle</a> · <a href="downloads/parts/bottle_rest.stl">bottle rest</a> · <a href="downloads/parts/tag_36h11_id1.stl">tag id1</a> · <a href="downloads/parts/tag_36h11_id2.stl">tag id2</a>.',
   'Tag tiles: pause at Z = 1.0 mm and switch to black filament for the raised cells, or colour the raised cells with a black marker afterwards.'],
  ['M5C', 'PLA'], [('Parts README (assumptions to verify)', 'downloads/parts/README.md'), ('Printing page', 'prints.html#fit')], 'd_print_plan',
  'planter, two nests, paddle, rest and tags printed', 'The square germination kit’s 220 mm tray does not fit the M5C; skip it or print it at 95 %.'),

 ('p1', 'table', 'Decide about the table (and what it costs)', 10,
  'The robot needs the trays at a repeatable height within reach; it does not need a printed table. The chosen modular table works but is ~45 h of printing.',
  ['<strong>Fast path:</strong> any stable surface at the right height (a box, a shelf, a cheap side table) with the nest plates screwed or taped to it. Zero print time.',
   '<strong>Printed path:</strong> <a href="https://www.printables.com/model/177153-modular-table-100-printed">Modular Table (100 % printed)</a>, CC0. The creator’s table is 63 h / 909 g on a Prusa MINI; on the M5C expect ≈ 40–45 h and 0.9–1.2 kg for a top big enough for two nests (≈ 5 × 3 tiles). All parts fit the 220 mm bed.',
   'You must download “<strong>ALL MODEL FILES</strong>” (13 × .3mf, 538 KB) while logged in to Printables, then slice for the M5C. The ZIP we have holds Prusa MINI G-code only.',
   'Height: measure from the floor to the robot’s shoulders on the cart, subtract 5–10 cm: that is the tray-rim height you want.'],
  ['a decision'], [('Modular table files', 'https://www.printables.com/model/177153-modular-table-100-printed/files')], None,
  'table decided: surface chosen or table printing', 'Pre-sliced per-part times from the download: 4 legs 7h48m (452 g), 8 connectors 3h19m (384 g), tops 1.4–2.3 h each — those are MINI times.'),

 ('p1', 'print-check', 'Check each print before trusting it', 15,
  'A nest that is 1 mm tight or a rest that is 1 mm loose costs a whole teaching session later.',
  ['Nest: the trough drops in and lifts out freely; the tag tile sits flush in its recess; the spout notch faces where the bottle will come from.',
   'Paddle: the BH1750 board fits its pocket; the handle grooves face the gripper.', 'Bottle rest: your bottle drops in and stands vertical; if it wobbles, reprint with a smaller bore (edit <code>make_parts.py</code>).',
   'Planter: rinse, fill the trough, leave it on paper for 30 min: no wet patch underneath.'],
  ['printed parts', 'the real bottle', 'BH1750 board'], [('Growing page: leak test', 'growing.html#start')], None, 'all fits checked, no leaks', ''),

 # ---------------- phase 2
 ('p2', 'flash', 'Flash the ESP32', 30,
  'The ESP32 reads the light sensor and prints JSON over USB. Flashing is a one-time step with the Arduino IDE.',
  ['Install the <a href="https://www.arduino.cc/en/software">Arduino IDE</a>, then add the ESP32 boards: Settings → Additional boards manager URLs → <code>https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json</code> → Boards Manager → install “esp32”.',
   'If the board does not show a serial port on macOS, install the <a href="https://www.silabs.com/developer-tools/usb-to-uart-bridge-vcp-drivers">CP210x</a> or <a href="https://github.com/WCHSoftGroup/ch34xser_macos">CH34x</a> driver (check which chip your ELEGOO board has).',
   'Open <code>software/firmware/esp32_light/esp32_light.ino</code>, board “ESP32 Dev Module”, choose the port, Upload.',
   'Watch it: ' + code('farm light-monitor            # one line per second: lux, status, seq')],
  ['ELEGOO ESP32', 'USB data cable', 'Arduino IDE'], [('Arduino-ESP32 install', 'https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html'), ('Firmware source', SW + '/firmware/esp32_light')], None,
  'light-monitor shows lux changing when you cover the sensor', 'A charge-only USB cable shows nothing. The cable from the cart must be a data cable.'),

 ('p2', 'wire', 'Wire the BH1750 and mount it on the paddle', 45,
  'Four wires. Get them right once and the sensor is a status-bearing reading for the rest of the project.',
  ['Solder the header pins onto the BH1750 if they came loose.', 'Wire per the diagram (3V3, GND, GPIO21 → SDA, GPIO22 → SCL). Use the flexible 22 AWG wire; leave a service loop at the wrist.',
   'Screw the board into the paddle pocket, sensor face up; route the cable down the handle groove.', 'Confirm again with <code>farm light-monitor</code>: cover / uncover the paddle.'],
  ['BH1750', 'flexible wire', 'soldering iron (buy in Japan)', 'printed paddle'], [('Hardware page', 'hardware.html'), ('BH1750 datasheet', 'https://www.mouser.com/datasheet/2/348/bh1750fvi-e-186247.pdf')], 'd_wiring',
  'paddle wired, readings valid while the cable is flexed', 'Lux is not PPFD; the number is for comparing days, never for deciding water.'),

 # ---------------- phase 3
 ('p3', 'assemble', 'Assemble the robot', 240,
  'The WowRobo two-wheel Combo arrives mostly assembled. Follow the vendor video for brackets and screws; our assembly page tracks integration checkpoints.',
  ['Work through <a href="assembly.html#mechanical">assembly steps 1–4</a> (lay out, cart + drive base, arms + head, cabling) with the <a href="https://youtu.be/upB1CEFeOlk">WowRobo video</a>.',
   'Power: use the two supplied 12 V/8 A wall adapters; check the labels accept 100 V (Japan). Battery stays out.', 'Plug into the powered hub: 2 motor boards, 3 cameras, ESP32. Hub → Mac USB-3 port. Mac on its charger.'],
  ['WowRobo kit', 'IKEA cart', 'powered hub', 'hex keys', 'multimeter (borrowed)'], [('Assembly guide', 'assembly.html'), ('WowRobo video', 'https://youtu.be/upB1CEFeOlk'), ('Two-wheel reference', 'https://xlerobot.readthedocs.io/en/latest/hardware/getting_started/assemble_2wheel.html')], 'img:assets/blender/workbench.png',
  'robot assembled, all USB devices on the hub, motors powered from wall adapters', 'Never change a motor plug with motor power on.'),

 # ---------------- phase 4
 ('p4', 'devices', 'Find the ports and cameras', 15,
  'The profile must name the two motor buses and the three cameras by identity. The probe pings each bus and snapshots each camera so you do not guess.',
  [code('cd software && . .venv/bin/activate\nfarm devices --probe'),
   'The probe reports which port answers motor IDs 1–8 (bus 1: left arm + head) and which answers 9–10 (bus 2: right arm + wheels). Snapshots land in <code>data/devices/</code>; open them and decide which index is head, left wrist, right wrist.',
   'Edit <code>profiles/paper-tray-v0.yaml</code>: <code>robot.port1</code>, <code>robot.port2</code>, and <code>index_or_path</code> for each camera.'],
  ['assembled robot'], [('Profile file', SW + '/profiles/paper-tray-v0.yaml')], 'd_hardware',
  'profile has both ports and three camera indices', 'macOS asks for camera permission the first time; allow it for Terminal.'),

 ('p4', 'calibrate', 'Calibrate the joints once', 20,
  'LeRobot needs each servo’s range once. This is the one setup step where a person moves the arms by hand — it is not operation.',
  [code('farm calibrate'), 'Follow the prompts: move the left arm + head to mid-range, ENTER; sweep every joint through its full range, ENTER; same for the right arm.',
   'The file lands in <code>~/.cache/huggingface/lerobot/calibration/robots/xlerobot_2wheels/</code>; back it up.'],
  ['both arms supported', 'motor power on'], [('LeRobot calibration docs', 'https://huggingface.co/docs/lerobot/so101#calibrate')], None,
  'calibration file saved', 'If a joint is skipped the model will later push it toward an impossible target and the safety clamp will stop everything.'),

 ('p4', 'check', 'Prove every device with one command', 10,
  'Before any motion the program connects everything and asks the vision model to confirm which camera is which. A swapped camera is caught here, not over a tray.',
  [code('farm check'), 'Expect: <code>problems: none</code>, joints OK, health OK, three cameras OK, light OK, <code>views: {"status": "OK", …}</code>.',
   'If views says INVALID, two camera indices are swapped: fix the profile and re-run.'],
  [], [], None, 'farm check is all OK', 'Startup also marks any action that was ATTEMPTED before a crash as UNKNOWN and asks you to reconcile it in the viewer.'),

 # ---------------- phase 5
 ('p5', 'station', 'Lay out the station', 60,
  'The program assumes nothing moves: trays in nests, bottle in its rest, paddle in its rest, cart parked. Everything the model teaches is relative to this layout.',
  ['Place the surface so the tray rims are 5–10 cm below the arm shoulders and 15–30 cm in front of them (see diagram).',
   'Screw or tape both nest plates down; drop a tag tile in each recess; set the cress planter into nest B (and A when you have two).',
   'Bottle rest on the right side within the right arm’s reach; paddle rest on the left. Tape them. Cart brakes on.',
   'Fill the bottle only half for the first days. A spout that pours a thin stream matters more than the bottle: the Nalgene needs a fitted spout or use a small squeeze bottle.'],
  ['nests, rests, tags', 'planter with paper and water', 'bottle with a spout', 'tape / screws'], [('Growing page', 'growing.html'), ('Prints page: nests', 'prints.html#nests')], 'd_station',
  'nothing on the station can move; bottle and paddle in their rests', 'Direct sunlight on the wrist camera makes every judgement UNKNOWN. Shade the station.'),

 ('p5', 'sow', 'Sow the first tray', 20,
  'Cress on household paper: germinates in 2–4 days, so the robot has something to look at within the trip.',
  ['Follow <a href="growing.html#start">the setup steps</a>: leak test, wick contact, sow a small even patch, keep an unseeded paper edge visible near the refill opening (the model judges wetness there).',
   'Log it in the <a href="growing.html#journal">notebook</a>.'],
  ['cress seed (buy locally)', 'kitchen paper', 'water'], [], 'img:assets/blender/wick-cutaway.png', 'tray sown, paper edge visible', 'Do not carry seed or soil through customs; buy locally.'),

 # ---------------- phase 6
 ('p6', 'teach', 'Let the model teach every pose', 60,
  'Nobody drives the arm. For each needed pose the vision model looks through the wrist and head cameras, moves in small clamped steps and saves the joints as a keyframe when the goal is visibly met.',
  [code('farm teach-all'), 'Watch the viewer (and stand near the STOP button). Each pose prints OK or FAILED with the model’s reason and cost.',
   'Poses learned: bottle rest above/grip, bottle upright, paddle rest above/grip, and per tray: look, pour, measure.',
   'Redo a single pose:' + code('farm teach --arm right --goal "spout 3 cm above the refill opening of tray B" --save pour_B')],
  ['station laid out', 'bottle and paddle in rests'], [('LLM-servo source', SW + '/farm/skills/llm_servo.py')], 'd_day1',
  'teach-all reports failed: none', 'If the model aborts with “target not visible”, fix the lighting or the head pose first; it is telling the truth.'),

 # ---------------- phase 7
 ('p7', 'empty', 'Rehearse with an empty bottle', 20,
  'The first full cycle must be dry: it proves pick, approach, tilt, upright and park without a drop of water.',
  ['Empty the bottle. Run:' + code('farm once --tray B'), 'The viewer asks “Authorize one pour?” — type your name, press <em>authorize one pour</em>. Watch the arm pick, approach, tilt, return upright, verify, park.',
   'Repeat until it is boring. Ten clean runs is the gate.'],
  [], [], 'd_cycle', '10 clean empty-bottle cycles', 'Press STOP in the viewer if anything looks wrong; motors hold instantly and only a named person can clear it.'),

 ('p7', 'cup', 'Measure a real pour into a cup', 15,
  'A tilt for N seconds is not a volume until you measure it. The cup test records the millilitres so every later pour is a known dose.',
  [code('farm cup-test --tilt 25 --seconds 1.5 --who yourname'), 'Hold a kitchen measure under the spout when prompted, read the millilitres, type them. Repeat with a fuller bottle; the last measurement is the calibration.'],
  ['kitchen measuring cup', 'half-full bottle'], [], None, 'pour calibration saved (data/pour_calibration.yaml)', 'Aim for 20–30 mL per pour: enough to wet the reserve, never enough to flood the paper.'),

 ('p7', 'first', 'The first real cycle', 20,
  'Now water goes into the tray. Rules, Jev and you all have to agree.',
  [code('farm once --tray B'), 'Authorize in the viewer. After the pour the model verifies: water visible / paper edge damp / spill CLEAR. If it cannot see the rim, the cycle pauses with delivery UNKNOWN and asks you to look.',
   'Open the cycle record in the viewer: every observation, decision and action with its status.'],
  [], [], 'd_viewer', 'first POURED cycle with a VERIFIED pour', 'A pour that ends UNKNOWN blocks every later cycle until you reconcile it. That is by design.'),

 # ---------------- phase 8
 ('p8', 'run', 'Run hourly', 5,
  'The care loop runs on a schedule, rests the arms between cycles (torque off so servos cool), and reconnects if a USB device hiccups.',
  [code('farm run --every 3600'), 'Viewer: <code>http://&lt;mac-ip&gt;:8765</code> from your phone on the same Wi-Fi (or Telegram). Leave the Mac on its charger with sleep disabled.'],
  [], [], None, 'farm run has completed a day of cycles', 'Keep the Mac awake: System Settings → Battery → prevent sleep on power, or run <code>caffeinate -i farm run --every 3600</code>.'),

 ('p8', 'authority', 'Let Jev earn authority', 5,
  'Jev starts in shadow. After ten cycles where its route agreed with the rules and with you, it is promoted to route; after ten more agreeing pour decisions, to approve — it then authorizes routine pours and just notifies you.',
  ['Nothing to do: promotion is automatic and recorded as events. Watch the Authority panel in the viewer.', 'Demote at any time with the buttons; a spill at reconciliation demotes automatically.'],
  [], [('Deck: Jev and Astra', 'software.html#models')], 'd_authority_ladder', 'Jev reached route', ''),

 ('p8', 'review', 'Daily: Astra review, proposals, backup', 10,
  'Once a day Astra reads the evidence and proposes one change. You accept or reject in the viewer. The database is backed up every day.',
  [code('farm review\nfarm backup'), 'Open proposals appear in the viewer. <code>authority.astra: apply-safe</code> in the profile lets bounded numeric changes apply themselves.'],
  [], [], None, 'first proposal reviewed and a backup exists', ''),

 # ---------------- phase 9
 ('p9', 'grow', 'Daily plant checks', 10,
  'The robot inspects and waters; a person still looks at the crop once a day and writes two lines. The photos are in the viewer.',
  ['Follow <a href="growing.html#daily">the daily plan</a>: paper evenly damp, no puddles, cover off once shoots stand, bright light.', 'Log observations in the notebook; note every manual top-up as an intervention.'],
  [], [('Growing page', 'growing.html#daily')], None, 'day 6: germination photographed', ''),

 ('p9', 'handover', 'Hand over (Oct 9–10)', 60,
  'The project is done when someone else can run it: restore a backup, resolve a staged pause, refill the bottle, and know where the records are.',
  ['Export: <code>farm backup</code>; copy <code>data/</code> (SQLite + images + keyframes + calibration) somewhere safe.',
   'Stage a pause (cover the rim with a card during a cycle) and have the caretaker resolve it in the viewer.', 'Write down: viewer URL, Telegram bot, STOP, the reconcile buttons, who to call.'],
  [], [('Deck: build order', 'software.html#timeline')], None, 'another person resolved a staged exception', ''),
]

READINESS = [
 ('Robot kit (WowRobo two-wheel Combo), cart, hub, battery', 'ordered / owned', 'Confirm the packing list on arrival; battery stays out of V0.'),
 ('MacBook + Claude subscription + OpenRouter key', 'ready', 'Rotate the key; set a credit limit.'),
 ('ESP32 ×3, BH1750 ×3, wire, perfboard, breadboard', 'in cart / owned', 'Verify purchase; soldering tools bought in Japan.'),
 ('Cress planter prints', 'not printed', 'M5C, ≈ 10 h.'),
 ('Nest plates ×2, paddle, bottle rest, tag tiles', 'STLs ready, not printed', '≈ 16 h total.'),
 ('Table / surface at the right height', 'undecided', 'Fast path: any surface. Printed table ≈ 45 h + 13 model files from Printables.'),
 ('Bottle with a controlled spout', 'gap', 'Nalgene wide-mouth needs a fitted spout; a small squeeze bottle is the simplest fix.'),
 ('Seeds, paper, water, kitchen measure', 'buy in Japan', 'No seed/soil through customs.'),
 ('Japan mains for the 12 V adapters', 'check labels', '100 V input must be printed on the adapters.'),
 ('Arduino IDE + ESP32 board package + USB driver', 'to install', 'One-time, 30 min.'),
 ('Camera permission for Terminal on macOS', 'to grant', 'Prompted on first run.'),
 ('Wi-Fi for the viewer / Telegram bot', 'optional', 'Viewer works on LAN; Telegram needs internet.'),
 ('Software: adapters, skills, perception, cycle, evidence, viewer, Jev/Astra, STOP, cup-test, probe, reconnect', 'written, 25 tests', 'Hardware-facing parts unproven until Sep 30.'),
 ('Time: Sep 30 – Oct 10', '11 days', 'Phases 3–7 need ~2 days; the rest is running.'),
]


def render():
    total = len(STEPS)
    out = ['<link rel="stylesheet" href="guide.css"><script src="guide.js" defer></script>']
    out.append('<div class="gbar" id="gbar"><div class="gprog"><div id="gfill"></div></div><span id="gcount">0 / %d done</span><button type="button" id="gnext">Jump to next step</button><button type="button" id="gall" class="secondary">Show all</button></div>' % total)
    out.append('<section id="journey"><h2>The whole journey</h2>' + d_journey() + '<p class="small">Estimated hands-on time is about two working days plus printing. Print first; everything else can overlap.</p></section>')
    for pid, ptitle in PHASES:
        out.append(f'<section id="{pid}" class="gphase"><h2>{e(ptitle)}</h2>')
        for st in [s for s in STEPS if s[0] == pid]:
            _, sid, title, minutes, why, do, needs, links, visual, check, gotchas = st
            out.append(f'<article class="gstep" id="step-{sid}" data-step="{sid}"><header><label class="gcheck"><input type="checkbox" data-check="guide-{sid}"><span>done</span></label><h3>{e(title)}</h3><span class="gtime">≈ {minutes} min</span></header>')
            out.append(f'<p class="gwhy">{why}</p>')
            if visual:
                if visual.startswith('img:'):
                    src = visual[4:]
                    out.append(f'<figure class="gfig"><img loading="lazy" src="{src}" alt="{e(title)}"></figure>')
                else:
                    out.append('<figure class="gfig">' + globals()[visual]() + '</figure>')
            out.append('<ol class="gdo">' + ''.join(f'<li>{d}</li>' for d in do) + '</ol>')
            cols = []
            if needs:
                cols.append('<div><strong>You need</strong><ul>' + ''.join(f'<li>{e(n)}</li>' for n in needs) + '</ul></div>')
            if links:
                cols.append('<div><strong>Links</strong><ul>' + ''.join(f'<li><a href="{u}">{e(t)}</a></li>' for t, u in links) + '</ul></div>')
            cols.append(f'<div><strong>Done when</strong><p>{e(check)}</p></div>')
            if gotchas:
                cols.append(f'<div class="ggotcha"><strong>Watch out</strong><p>{gotchas}</p></div>')
            out.append('<div class="gcols">' + ''.join(cols) + '</div></article>')
        out.append('</section>')
    out.append('<section id="readiness"><h2>Readiness audit: everything the project needs</h2><div class="scroll"><table><thead><tr><th>Item</th><th>Status</th><th>Note</th></tr></thead><tbody>' +
               ''.join(f'<tr><td>{e(a)}</td><td>{e(b)}</td><td>{e(c)}</td></tr>' for a, b, c in READINESS) + '</tbody></table></div>' +
               '<p class="small">Gaps found in the audit: the bottle spout, the table height decision, and the Printables model download are the three things no amount of software fixes.</p></section>')
    return ''.join(out)
