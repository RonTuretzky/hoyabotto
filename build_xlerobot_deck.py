#!/usr/bin/env python3
"""xlerobot-deck.html — the autonomous-farm roadmap on XLeRobot, for the Sep 30 – Oct 10 Kesennuma window.
v3: accessible component map, A1 split trays, software-only prep, single-crop experiments and printer handoff."""
import re, pathlib, io, contextlib, json
HERE = pathlib.Path(__file__).parent
with contextlib.redirect_stdout(io.StringIO()):
    import shared_templates as B
style, extra, script = B.style, B.extra, B.script
gal, CREDIT = B.gal, B.CREDIT

DAYS = ["Sep 30","Oct 1","Oct 2","Oct 3","Oct 4","Oct 5","Oct 6","Oct 7","Oct 8","Oct 9","Oct 10"]
WD   = ["Wed","Thu","Fri","Sat","Sun","Mon","Tue","Wed","Thu","Fri","Sat"]

# ---------------- figures ----------------
def fig_site():
    pods = "".join(f'<rect x="{142+i*32}" y="{143+j*29}" width="24" height="21" rx="3" class="site-print"/>' for i in range(4) for j in range(3))
    return f'''<svg class="fig site-map" viewBox="0 0 1050 410" role="img" aria-label="Farm map. P diagonal stripes means printed, K circle means robot kit, H square means already owned or borrowed, S diamond means separate supplies, T cross means paper labels. Every part is labelled with its code.">
    <defs><pattern id="site-hatch" width="9" height="9" patternUnits="userSpaceOnUse"><path d="M-2,2 L2,-2 M0,9 L9,0 M7,11 L11,7" stroke="#d5e2ed" stroke-width="1" opacity=".5"/></pattern></defs>
    <text x="16" y="22" class="lbl">PLAN VIEW · SCHEMATIC, NOT TO SCALE</text>
    <path d="M135,42 H465" stroke="#e3edf6" stroke-width="6"/>
    <text x="300" y="65" class="lb" text-anchor="middle">[H] window · natural light</text>
    <rect x="100" y="82" width="440" height="185" rx="5" class="site-have"/>
    <text x="115" y="104" class="lb">[H] existing / borrowed table</text>
    <rect x="132" y="131" width="146" height="108" rx="4" class="site-print"/>{pods}
    <text x="205" y="125" class="lb" text-anchor="middle">[P] tray 1 + pods</text>
    <rect x="300" y="131" width="120" height="108" rx="4" class="site-print"/>
    <path d="M360,132 V238" stroke="#fff" stroke-dasharray="5 4"/>
    <text x="360" y="172" class="lbi" text-anchor="middle">[P] tray 2</text>
    <text x="360" y="190" class="lbs" text-anchor="middle">two 6-pod halves</text>
    <text x="360" y="207" class="lbs" text-anchor="middle">assembled by you</text>
    <path d="M438,162 L468,136 L498,162 L468,188 Z" class="site-shop"/>
    <text x="468" y="165" class="lbi" text-anchor="middle">S</text>
    <text x="468" y="211" class="lbs" text-anchor="middle">shop pots / bin</text>
    <text x="114" y="255" class="lb">[T] AprilTags + photo reference card</text>
    <rect x="150" y="290" width="155" height="97" rx="5" class="site-have"/>
    <text x="226" y="316" class="lb" text-anchor="middle">[H] IKEA cart</text>
    <text x="226" y="337" class="lbs" text-anchor="middle">Mac + battery + hub</text>
    <text x="226" y="361" class="lbs" text-anchor="middle">[P] scale plate</text>
    <text x="226" y="377" class="lbs" text-anchor="middle">[S] scale electronics</text>
    <circle cx="174" cy="282" r="15" class="site-kit"/><text x="174" y="286" class="lbi" text-anchor="middle">K</text>
    <circle cx="281" cy="282" r="15" class="site-kit"/><text x="281" y="286" class="lbi" text-anchor="middle">K</text>
    <path d="M174,267 L181,249 L199,226 M281,267 L271,247 L244,228" class="lime"/>
    <text x="335" y="300" class="lb">(K) arms + cameras</text>
    <text x="335" y="321" class="lbs">Park near table edge.</text>
    <text x="335" y="338" class="lbs">Measure reach on Sep 30.</text>
    <text x="335" y="371" class="lb">A1: separate dry workbench</text>
    <text x="335" y="389" class="lbs">keep its moving bed clear</text>
    <text x="580" y="25" class="lbl">LETTER + SHAPE + TEXT, NOT COLOUR</text>
    <rect x="580" y="46" width="26" height="26" class="site-print"/><text x="593" y="65" class="lbi" text-anchor="middle">P</text>
    <text x="622" y="63" class="lb">PRINT · diagonal stripes</text>
    <text x="622" y="84" class="lbs">Pods, tray halves, guides, scale plate, water bodies.</text>
    <circle cx="593" cy="115" r="14" class="site-kit"/><text x="593" y="119" class="lbi" text-anchor="middle">K</text>
    <text x="622" y="119" class="lb">KIT · circle</text>
    <text x="622" y="140" class="lbs">Arms, cameras, base parts: supplied with robot.</text>
    <rect x="580" y="157" width="26" height="26" class="site-have"/><text x="593" y="176" class="lbi" text-anchor="middle">H</text>
    <text x="622" y="176" class="lb">HAVE / BORROW · square</text>
    <text x="622" y="197" class="lbs">Cart, laptop, battery, hub; find a suitable table.</text>
    <path d="M593,211 l16,16 -16,16 -16,-16 Z" class="site-shop"/><text x="593" y="231" class="lbi" text-anchor="middle">S</text>
    <text x="622" y="231" class="lb">SEPARATE SUPPLIES · diamond</text>
    <text x="622" y="252" class="lbs">Seed, paper, shop pots, wicks; scale + HX711 + ESP32.</text>
    <text x="622" y="269" class="lbs">If scale parts are absent, start with photo logs.</text>
    <text x="583" y="303" class="lbi">✚</text><text x="622" y="303" class="lb">[T] PAPER · cross / letter T</text>
    <text x="622" y="324" class="lbs">Print tags and checkerboard on site from PDFs.</text>
    <text x="580" y="364" class="lb">Filament: you will arrange it.</text>
    <text x="580" y="385" class="lbs">No shelves needed this trip. Sensors ride the robot.</text>
    </svg>'''

FIG_SITE = fig_site()

def fig_cart():
    return '''
<svg class="fig" viewBox="0 0 900 380" role="img" aria-label="What rides on the cart">
  <text x="20" y="20" class="lbl">THE CART · SIDE VIEW · what sits on each tray</text>
  <line x1="60" y1="352" x2="840" y2="352" class="ln2"/>
  <line x1="330" y1="70" x2="330" y2="330" class="ln2"/><line x1="570" y1="70" x2="570" y2="330" class="ln2"/>
  <rect x="318" y="92" width="264" height="24" rx="6" class="fg"/><rect x="318" y="200" width="264" height="24" rx="6" class="fg"/><rect x="318" y="306" width="264" height="24" rx="6" class="fg"/>
  <circle cx="360" cy="340" r="14" class="fd"/><circle cx="540" cy="340" r="14" class="fd"/><circle cx="450" cy="344" r="8" class="fg"/>
  <!-- top: arms + head -->
  <polyline points="370,92 350,48 300,34 250,62" class="lime"/><circle cx="370" cy="92" r="4" class="fl"/><circle cx="350" cy="48" r="4" class="fl"/><circle cx="300" cy="34" r="4" class="fl"/>
  <rect x="240" y="60" width="10" height="34" rx="2" class="fa"/>
  <polyline points="530,92 552,50 604,40 650,66" class="lime"/><circle cx="530" cy="92" r="4" class="fl"/><circle cx="552" cy="50" r="4" class="fl"/><circle cx="604" cy="40" r="4" class="fl"/>
  <path d="M650,66 l10,-8 M650,66 l10,8" class="lime"/><rect x="660" y="54" width="22" height="26" rx="4" class="fl"/>
  <line x1="450" y1="92" x2="450" y2="36" class="ln2"/><rect x="434" y="24" width="32" height="14" rx="3" class="fd"/><circle cx="450" cy="31" r="3" class="fa"/>
  <text x="232" y="112" class="lb" text-anchor="end">LEFT ARM · senses</text><text x="232" y="126" class="lbs" text-anchor="end">wrist camera (in the kit)</text><text x="232" y="139" class="lbs" text-anchor="end">+ optional pH/EC probe holder</text>
  <text x="696" y="60" class="lb">RIGHT ARM · handles</text><text x="696" y="74" class="lbs">soft TPU gripper lifts a pod by its lip</text>
  <text x="474" y="30" class="lbs">head camera · reads AprilTags</text>
  <!-- mid: weigh dock -->
  <rect x="352" y="186" width="90" height="12" rx="3" class="fl"/><rect x="380" y="160" width="28" height="26" rx="4" class="fl"/>
  <rect x="470" y="180" width="34" height="18" rx="3" class="fd"/>
  <text x="600" y="186" class="lb">MIDDLE TRAY</text><text x="600" y="200" class="lbs">weigh dock: printed plate on a load cell</text><text x="600" y="213" class="lbs">+ a thumb-sized ESP32 that sends grams over USB</text><text x="600" y="226" class="lbs">(the MacBook has no sensor pins of its own)</text>
  <!-- bottom: mac, battery, hub -->
  <rect x="340" y="292" width="110" height="12" rx="3" class="fd"/><rect x="462" y="268" width="56" height="36" rx="5" class="fd"/><rect x="528" y="290" width="40" height="14" rx="3" class="fd"/>
  <text x="300" y="286" class="lb" text-anchor="end">BOTTOM TRAY</text><text x="300" y="300" class="lbs" text-anchor="end">MacBook (the brain) · battery · USB hub</text><text x="300" y="313" class="lbs" text-anchor="end">hub → the Mac's USB 3 port; 5 of 7 ports used</text>
  <text x="600" y="330" class="lbs">two 127 mm drive wheels + a caster (differential drive)</text>
</svg>'''
FIG_CART = fig_cart()

def fig_light():
    x = lambda h: 60 + h * 32.5
    g = ['<svg class="fig" viewBox="0 0 900 215" role="img" aria-label="Daylight and round times">',
         '<text x="60" y="18" class="lbl">ONE DAY AT THE WINDOW · Kesennuma 38.9° N · early October</text>',
         f'<rect x="{x(0)}" y="40" width="{x(24)-x(0)}" height="46" rx="8" class="fd"/>',
         f'<rect x="{x(5.5)}" y="40" width="{x(17.2)-x(5.5)}" height="46" class="fl" opacity=".55"/>',
         f'<rect x="{x(8)}" y="40" width="{x(15)-x(8)}" height="46" class="fl"/>']
    for h in range(0, 25, 3):
        g.append(f'<line x1="{x(h)}" y1="86" x2="{x(h)}" y2="94" class="ln2"/><text x="{x(h)}" y="108" class="axis" text-anchor="middle">{h:02d}:00</text>')
    g.append(f'<text x="{x(5.5)}" y="34" class="lbs" text-anchor="middle">sunrise ≈ 05:30</text><text x="{x(17.2)}" y="34" class="lbs" text-anchor="middle">sunset ≈ 17:10</text>')
    g.append(f'<text x="{x(11.5)}" y="68" class="lbi" text-anchor="middle">good window light ≈ 08:00 – 15:00</text>')
    for h, lab in ((10, "round 1 · 10:00"), (14, "round 2 · 14:00")):
        g.append(f'<circle cx="{x(h)}" cy="63" r="0" /><line x1="{x(h)}" y1="40" x2="{x(h)}" y2="130" class="wave"/><circle cx="{x(h)}" cy="134" r="6" class="fa"/><text x="{x(h)}" y="156" class="lba" text-anchor="middle">{lab}</text>')
    g.append(f'<text x="{x(12)}" y="176" class="lbs" text-anchor="middle">photo · weigh · (dip) — always at the same two times, always with the colour card in frame, so cloudy and sunny days can be compared</text>')
    g.append(f'<text x="{x(12)}" y="196" class="lbs" text-anchor="middle">daylight ≈ 11 h 45 min on Sep 30 → ≈ 11 h 25 min on Oct 10 · noon sun ≈ 45° high, so it reaches deep into a south window</text>')
    g.append('</svg>')
    return "".join(g)
FIG_LIGHT = fig_light()

def fig_waves():
    rows = [
      ("A · Sep 30", "12 shop pots", "Primary crop + baseline", "Assess Oct 7–10; harvest only if ready"),
      ("B · Oct 3", "12 printed pods, or shop pots", "Repeat control vs one change", "Growth data by Oct 10; harvest may be later"),
      ("C · after A harvest", "Reuse emptied slots", "Optional next recipe from A results", "Follow-up after this trip; no promised harvest"),
    ]
    g = ['<div class="crop-waves"><div class="wave-head">Sow / container</div><div class="wave-head">Why this batch exists</div><div class="wave-head">What the trip can measure</div>']
    for name, pots, why, result in rows:
        g += [f'<div><b>{name}</b><br><span>{pots}</span></div>', f'<div>{why}</div>', f'<div>{result}</div>']
    return ''.join(g) + '</div>'

FIG_WAVES = fig_waves()

def fig_agent():
    steps = [("recipe", "per pod: density|soak · cover · water"), ("robot runs it", "places pods · tops up|to the target weight"), ("measure", "grams · leaf area · colour|twice a day, every pod"),
             ("learn", "which recipe grew|fastest / cleanest?"), ("propose", "one change + controls|for the next batch"), ("you approve", "one tap — until|you trust it")]
    g = ['<svg class="fig" viewBox="0 0 900 215" role="img" aria-label="Botanist agent loop" style="overflow:visible">']
    for i, (t, sub) in enumerate(steps):
        cx = 80 + i * 148
        g.append(f'<circle cx="{cx}" cy="78" r="46" class="{"fa" if i in (3,4) else ("fl" if i==5 else "fg")}"/><text x="{cx}" y="82" class="lbi" text-anchor="middle">{t}</text><text x="{cx}" y="136" class="lbs" text-anchor="middle">{sub.split("|")[0]}</text><text x="{cx}" y="150" class="lbs" text-anchor="middle">{sub.split("|")[1]}</text>')
        if i < len(steps) - 1: g.append(f'<path d="M{cx+48},78 L{cx+100},78" class="ln2"/><path d="M{cx+94},73 l6,5 -6,5" class="ln2"/>')
    g.append('<path d="M866,78 L884,78 Q894,78 894,90 L894,170 Q894,182 880,182 L20,182 Q8,182 8,170 L8,90 Q8,78 20,78 L30,78" class="lime" stroke-dasharray="5 4"/><path d="M24,73 l7,5 -7,5" class="lime"/>')
    g.append('<text x="450" y="208" class="lbl" text-anchor="middle">one crop · controlled comparisons · later batches follow the evidence</text></svg>')
    return "".join(g)
FIG_AGENT = fig_agent()


def fig_plumb():
    return '''
<svg class="fig" viewBox="0 0 900 250" role="img" aria-label="Three ways to bring water back">
  <text x="20" y="18" class="lbl">A · POUR, DON'T PLUG</text>
  <path d="M40,120 L70,120 L62,150 L48,150 Z" class="fl"/><rect x="48" y="150" width="14" height="22" class="fl"/>
  <rect x="40" y="172" width="230" height="12" rx="3" class="fw"/><rect x="36" y="184" width="238" height="8" rx="3" class="fl"/>
  <g class="fl"><rect x="90" y="138" width="34" height="34" rx="4"/><rect x="140" y="138" width="34" height="34" rx="4"/><rect x="190" y="138" width="34" height="34" rx="4"/></g>
  <g class="sprout"><path d="M107,138 v-12 M101,130 l6,-7 6,7"/><path d="M157,138 v-12 M151,130 l6,-7 6,7"/><path d="M207,138 v-12 M201,130 l6,-7 6,7"/></g>
  <g class="ln2"><line x1="107" y1="172" x2="107" y2="180"/><line x1="157" y1="172" x2="157" y2="180"/><line x1="207" y1="172" x2="207" y2="180"/></g>
  <polyline points="10,40 40,52 58,86" class="lime"/><rect x="50" y="84" width="16" height="26" rx="4" class="fa" transform="rotate(25 58 97)"/><circle cx="56" cy="116" r="2.5" class="fw"/>
  <text x="20" y="212" class="lb">printed funnel + gutter under the pods</text><text x="20" y="226" class="lbs">wicks pull water up · no seals, no pressure</text><text x="20" y="240" class="lbs" style="fill:#c9f65b">printed body + real wicks · target tolerance ±10 mm</text>
  <line x1="300" y1="10" x2="300" y2="244" class="ln2" stroke-dasharray="3 6"/>
  <text x="320" y="18" class="lbl">B · FILL DOCK</text>
  <rect x="360" y="160" width="110" height="10" rx="3" class="fl"/><rect x="394" y="122" width="40" height="38" rx="5" class="fl"/>
  <path d="M414,122 v-12 M408,114 l6,-7 6,7" class="sprout"/>
  <path d="M500,172 L500,80 L424,80 L424,100" class="ln"/><path d="M420,100 l4,8 4,-8" class="ln"/>
  <circle cx="530" cy="150" r="14" class="fd"/><text x="530" y="154" class="lbs" text-anchor="middle">pump</text><path d="M500,150 L516,150 M544,150 L560,150 L560,176" class="ln"/>
  <rect x="546" y="176" width="30" height="28" rx="4" class="fw"/>
  <text x="320" y="212" class="lb">pod on the scale, nozzle above it</text><text x="320" y="226" class="lbs">pump runs until the pod hits its target weight</text><text x="320" y="240" class="lbs" style="fill:#5ed6de">printed nozzle arm · bought: 1 pump + tube</text>
  <line x1="600" y1="10" x2="600" y2="244" class="ln2" stroke-dasharray="3 6"/>
  <text x="620" y="18" class="lbl">C · SEALED SNAP-IN CONNECTOR</text>
  <rect x="630" y="120" width="110" height="50" rx="5" class="fl"/><path d="M740,132 L772,138 L772,152 L740,158 Z" class="fl"/><circle cx="758" cy="136" r="3" class="fc"/><circle cx="758" cy="154" r="3" class="fc"/>
  <path d="M800,124 L782,134 L782,156 L800,166 L860,166 L860,124 Z" class="fd"/><line x1="860" y1="145" x2="890" y2="145" class="ln"/>
  <path d="M776,145 h-2" class="ln"/><text x="685" y="149" class="lbs" text-anchor="middle">tile</text><text x="828" y="149" class="lbs" text-anchor="middle">socket</text>
  <text x="620" y="212" class="lb">cone finds the socket, O-rings seal it</text><text x="620" y="226" class="lbs">your spec's open question, as hardware</text><text x="620" y="240" class="lbs" style="fill:#ff8a70">printed housings · bought: O-rings, tube, pump</text>
</svg>'''
FIG_PLUMB = fig_plumb()

def roundtrip():
    steps = [("home", "charge|wake 09:55"), ("drive", "follow the|floor tags"), ("park", "table tag|within 2 cm"), ("photo", "every pod +|colour card"),
             ("weigh", "pod → scale|→ back"), ("top up", "to target|grams"), ("swap", "done → bin|fresh ← stack"), ("return", "log · call you|if unsure")]
    g = ['<svg class="fig" viewBox="0 0 900 200" role="img" aria-label="Daily round">']
    for i, (t, sub) in enumerate(steps):
        cx = 60 + i * 112
        g.append(f'<circle cx="{cx}" cy="80" r="30" class="{"fa" if i in (3,4,5) else "fg"}"/><text x="{cx}" y="85" class="lbi" text-anchor="middle">{t}</text><text x="{cx}" y="128" class="lbs" text-anchor="middle">{sub.split("|")[0]}</text><text x="{cx}" y="142" class="lbs" text-anchor="middle">{sub.split("|")[1]}</text>')
        if i < len(steps) - 1: g.append(f'<path d="M{cx+32},80 L{cx+80},80" class="ln2"/><path d="M{cx+74},75 l6,5 -6,5" class="ln2"/>')
    g.append('<path d="M844,110 Q850,170 60,170 Q40,170 40,112" class="ln2" stroke-dasharray="4 4"/><text x="450" y="190" class="lbs" text-anchor="middle">one round ≈ 40 min for 24 pods · 10:00 and 14:00 · an optional probe dip slots in after "weigh"</text></svg>')
    return "".join(g)
FIG_ROUND = roundtrip()

FIG_ARMS = '''
<svg class="fig" viewBox="0 0 960 300" role="img" aria-label="Arm roles">
  <text x="30" y="22" class="lba">LEFT ARM · SENSING</text><text x="470" y="22" class="lbl">RIGHT ARM · HANDLING</text>
  <line x1="450" y1="30" x2="450" y2="280" class="ln2" stroke-dasharray="3 6"/>
  <polyline points="80,250 70,180 130,130 190,150" class="lime"/><circle cx="80" cy="250" r="5" class="fl"/><circle cx="70" cy="180" r="5" class="fl"/><circle cx="130" cy="130" r="5" class="fl"/>
  <rect x="184" y="140" width="14" height="70" rx="3" class="fa"/><circle cx="191" cy="155" r="3" class="tag"/><circle cx="191" cy="175" r="3" class="tag"/><circle cx="191" cy="195" r="3" class="tag"/>
  <rect x="170" y="128" width="16" height="10" rx="2" class="fl"/>
  <text x="230" y="150" class="lb">wrist camera (in the kit)</text><text x="230" y="166" class="lbs">close-up of every pod, twice a day</text>
  <text x="230" y="192" class="lb">optional probe: pH · saltiness · temp</text><text x="230" y="208" class="lbs">one probe for all 24 pods — never one per pod</text>
  <text x="230" y="234" class="lb">never lifts anything</text><text x="230" y="250" class="lbs">stays cool: seconds of work, minutes of rest</text>
  <polyline points="520,250 510,180 570,130 630,150" class="lime"/><circle cx="520" cy="250" r="5" class="fl"/><circle cx="510" cy="180" r="5" class="fl"/><circle cx="570" cy="130" r="5" class="fl"/>
  <path d="M628,150 l12,-10 M628,150 l12,10" class="lime"/><rect x="640" y="132" width="34" height="36" rx="5" class="fl"/><rect x="644" y="136" width="6" height="6" class="tag"/>
  <text x="690" y="150" class="lb">gripper on the pod lip</text><text x="690" y="166" class="lbs">~200 g wet pod; cone base places it</text>
  <text x="690" y="192" class="lb">pod → scale → back to its slot</text><text x="690" y="208" class="lbs">the weigh dock rides on the cart</text>
  <text x="690" y="234" class="lb">tile slide, last 10 cm</text><text x="690" y="250" class="lbs">tile rides the tray; arm only pushes</text>
</svg>'''

def stage_ladder():
    stages = [
      ("0", "Instrument", "parked; cameras + scale logging", "table by a window · 2 printed tiles · 24 printed pods", "you sow, water, photograph", "D1 – D4 · Sep 30 – Oct 3"),
      ("1", "Supervise", "two rounds a day: photo · weigh · propose top-ups and recipes", "weigh dock on the cart · colour card · floor tags", "you approve each proposal", "D5 – D6 · Oct 4 – 5"),
      ("2", "Maintain", "after checks pass: top up by weight · swap pods · log interventions", "clean-pod stack · harvest bin", "refill the water jug, empty the bin", "D7 – D10 · Oct 6 – 9"),
      ("3", "Expand", "slides an empty tray between supported surfaces into a guide", "printed tile guide", "hand it the new tile", "D10 demo · full version later"),
      ("4", "Replicate", "installs trays in a human-assembled shelf; copies recipes", "complete metal shelving + printed tray guides", "assemble and secure the shelf", "a later trip"),
      ("5", "Fabricate", "picks released prints from a nest; direct pickup is research", "A1 + pickup nest; release initially by a human", "release prints; load filament", "a later trip"),
    ]
    g = ['<svg class="fig" viewBox="0 0 1000 400" role="img" aria-label="Stage ladder">']
    g.append('<text x="130" y="20" class="lbl">STAGE</text><text x="235" y="20" class="lba">THE ROBOT DOES</text><text x="560" y="20" class="lbl">WHAT IT NEEDS AROUND IT</text><text x="790" y="20" class="lbs" style="font-weight:700">YOU STILL DO · WHEN</text>')
    for i, (n, name, robot, infra, human, when) in enumerate(stages):
        y = 36 + i*60; cls = "fa" if i >= 3 else "fl"
        g.append(f'<rect x="20" y="{y}" width="960" height="52" rx="8" class="{"fg" if i%2 else "fd"}" opacity="{1 if i<4 else .55}"/>')
        g.append(f'<circle cx="50" cy="{y+26}" r="16" class="{cls}"/><text x="50" y="{y+31}" class="lbi" text-anchor="middle">{n}</text><text x="78" y="{y+31}" class="lb" style="font-weight:700;fill:#f4f6ed">{name}</text>')
        g.append(f'<foreignObject x="228" y="{y+6}" width="320" height="46"><div xmlns="http://www.w3.org/1999/xhtml" style="font:11px/1.35 ui-monospace,Menlo,monospace;color:#dce5dc">{robot}</div></foreignObject>')
        g.append(f'<foreignObject x="556" y="{y+6}" width="226" height="46"><div xmlns="http://www.w3.org/1999/xhtml" style="font:11px/1.35 ui-monospace,Menlo,monospace;color:#aebbb3">{infra}</div></foreignObject>')
        g.append(f'<foreignObject x="788" y="{y+6}" width="190" height="46"><div xmlns="http://www.w3.org/1999/xhtml" style="font:11px/1.35 ui-monospace,Menlo,monospace;color:#aebbb3">{human}<br/><b style="color:#c9f65b">{when}</b></div></foreignObject>')
    g.append('</svg>')
    return "".join(g)
FIG_STAGES = stage_ladder()

# ---------------- slides ----------------
S = []
def slide(kicker, body, foot, active=False, kick_cls="kicker eyebrow", fig=False):
    n = len(S) + 1
    kicker = re.sub(r"^\d\d · ", f"{n-1:02d} · ", kicker)
    S.append(f'<section class="slide{" active" if active else ""}{" figslide" if fig else ""}"><div class="{kick_cls}">{kicker}</div><div class="body">{body}</div><div class="footer"><span>{foot}</span><span>{n:02d}</span></div></section>')
def mrow(*cells): return "".join(f"<div>{c}</div>" for c in cells)
W = lambda s: f'<b style="color:#dce5dc">{s}</b>'
L = lambda s: f'<b style="color:#c9f65b">{s}</b>'

slide("Autonomous farm · the roadmap on XLeRobot · Kesennuma, Sep 30 – Oct 10", f'''
<div class="title-layout"><div>
  <h1>Eleven days. <span class="accent">One robot.</span> One window.</h1>
  <p class="lead" style="margin-top:22px">Your spec asks for a farm that thinks, operates and builds, and for a V0 that is "a testable system, not a miniature dream." This is that V0 for the Sep 30 – Oct 10 stay: XLeRobot, 24 printed pods on a table in daylight, one fast crop in repeated batches, and a botanist agent that proposes and measures recipe changes. Physical work starts Sep 30.</p>
  <div class="title-meta"><span class="pill">Stages 0 → 2 + a Stage 3 demo</span><span class="pill">natural light</span><span class="pill">nothing electronic in the pods</span><span class="pill">day-by-day plan inside</span></div>
</div><div><img src="assets/xlerobot.png" alt="XLeRobot" style="width:100%;border-radius:18px;border:1px solid var(--line)"></div></div>''', "XLeRobot · Kesennuma V0", active=True, kick_cls="kicker")

slide("01 · Roadmap slide 01 · system model", f'''
<h2>One system, three capabilities — as things on a cart.</h2>
<div class="grid-3" style="margin-top:18px">
  <div class="card"><div class="num">01 / THINKING</div><h3>Sense, reason, decide</h3><p>{W("Runs on:")} the MacBook riding on the cart. {W("Made of:")} a botanist agent that designs each wave's recipes and keeps a lab notebook · AprilTag localisation · leaf-area measurement from photos · a rule-based scheduler · a small database · a message to your phone when it is unsure.</p><span class="tag">the intelligence layer</span></div>
  <div class="card" style="background:linear-gradient(150deg, rgba(94,214,222,.10), rgba(255,255,255,.018))"><div class="num">02 / OPERATION</div><h3>Run the living system</h3><p>{W("Runs on:")} XLeRobot — left arm senses, right arm handles. {W("Serves:")} two 12-pod tiles on a table by a window, a weigh dock on the cart, a clean-pod stack and a harvest bin. Inspection, watering by weight, harvesting-by-swap, sanitation-by-swap, replacement, calibration.</p><span class="tag">the farm layer</span></div>
  <div class="card"><div class="num">03 / CONSTRUCTION</div><h3>Build, repair, replicate</h3><p>{W("This trip:")} printing starts on Sep 30; shop pots keep the first batch independent of printing. Day 10 targets an empty-tray installation demo after the handling checks pass. {W("Later:")} the cart picks released prints from a fixture and assembles pods; you assemble the furniture. Motors, chips and probes stay bought — your stated boundary.</p><span class="tag">the replication layer</span></div>
</div>''', "Reconciliation · system model")

slide("02 · Roadmap slide 02 · autonomy goals", f'''
<h2>Goal A in these eleven days; goal B gets a first demo.</h2>
<div class="grid-2" style="margin-top:18px">
  <div class="opt-card a"><div class="num">GOAL A · PRIMARY</div><h3>Indefinitely self-maintaining</h3><p>The cart parks at the table and runs a round at 10:00 and 14:00. It notices trouble (a dry pod, mould, a missing pod, a drifting scale), deals with it (top up, swap, recalibrate) or calls you. {W("Your definition, honoured:")} from day 1 we log how long it runs between your interventions and what share of jobs it does alone.</p><span class="tag">D1 – D10 · reduced intervention only after checks pass</span></div>
  <div class="opt-card b"><div class="num">GOAL B · SECOND FRONTIER</div><h3>Self-expanding &amp; replicating</h3><p>Day 10 targets a supported slide of an empty printed tray from the cart to the table: the smallest possible "install a new module". You assemble any later shelving. Robot assembly of pods and direct printer pickup are later experiments. {W("Why a robot on wheels:")} it can travel between the table and a future printer station.</p><span class="tag">D10 demo · the rest later</span></div>
</div>''', "Reconciliation · goals")

slide("03 · Roadmap slide 03 · physical architecture", f'''<h2>One table; printed pods and trays</h2><div style="margin-top:6px">{FIG_SITE}</div>
<p class="caption">Every component has a letter and a shape. Diagonal stripes mean printed plastic. The scale plate prints; its sensor and electronics do not. The Bambu A1 is the working assumption, still to be confirmed on Sep 30.</p>''', "Reconciliation · architecture", fig=True)

slide("04 · Water", f'''<h2>Plumbing: what prints, what doesn't, and how it comes back.</h2><div style="margin-top:4px">{FIG_PLUMB}</div>
<div class="grid-4" style="margin-top:6px">
  <div class="card" style="padding:12px 14px"><h3 style="font-size:14px">Why it was cut</h3><p style="font-size:12px;margin-top:4px">Microgreens in plain water need no dosing; shared water is how one mouldy pod becomes twelve; and a slow leak over four unattended days sits beside a laptop and a battery. That was caution about the schedule, not about printing.</p></div>
  <div class="card" style="padding:12px 14px"><h3 style="font-size:14px">What prints</h3><p style="font-size:12px;margin-top:4px">In PETG: funnels, gutters, manifold bodies, connector housings, nozzles, tube clips, brackets. {W("Separate components:")} pumps, tubing, wicks, O-rings and reliable valves. Printed threads and press-fits weep under pressure, so design for gravity or for seals.</p></div>
  <div class="card" style="padding:12px 14px"><h3 style="font-size:14px">The order to try them</h3><p style="font-size:12px;margin-top:4px">{L("A")} from D6: printed funnel / gutter with real wicks, forgiving of a loose arm; shares water only within one tile. {L("B")} when you have one pump. {L("C")} printed D8–D9 and tested in the D10 tile-install demo: the first real answer to your connector question.</p></div>
  <div class="card" style="padding:12px 14px"><h3 style="font-size:14px">Still true</h3><p style="font-size:12px;margin-top:4px">No electronics in pods, sensors stay on the arm, and at least one tile stays isolated and hand-or-robot watered as the control, so a leak or an infection in the plumbed tile never ends the experiment.</p></div>
</div>''', "Water", fig=True)

slide("04 · Light", f'''<h2>The window is the grow light.</h2><div style="margin-top:6px">{FIG_LIGHT}</div>
<div class="grid-3" style="margin-top:8px">
  <div class="card" style="padding:14px 16px"><h3>Why it works</h3><p style="font-size:13px">Microgreens live mostly off the seed. They spend days 1–2 covered in the dark anyway, then need only a few hours of bright indirect light to green up. Eleven-plus hours of October daylight is plenty.</p></div>
  <div class="card" style="padding:14px 16px"><h3>What it costs us</h3><p style="font-size:13px">Light changes with cloud, and this is a wet season. So every photo includes a reference card, rounds run at fixed times, and the agent compares pods within the same photo, never across days. Expect leggier shoots on grey days.</p></div>
  <div class="card" style="padding:14px 16px"><h3>Keep them indoors</h3><p style="font-size:13px">October days are about 19 °C but nights fall toward 10 °C, and it is typhoon season. A room at 18–22 °C by a south or east window is ideal; a sheer curtain stops midday glare from blinding the cameras.</p></div>
</div>''', "Light", fig=True)

# One pack per linked line; totals are calculated from the verified price snapshot.
SENSOR_BOM = json.loads((HERE / "sensor-bom.json").read_text())
def sensor_link(key):
    item = SENSOR_BOM["items"][key]
    return f'<a href="{item["url"]}" target="_blank" rel="noopener noreferrer">{item["name"]} · ${item["price"]:.2f}</a>'
def sensor_total(*groups):
    return sum(round(item["price"] * 100) for item in SENSOR_BOM["items"].values() if item["group"] in groups) / 100

def sensor_links(*keys):
    return " · ".join(sensor_link(key) for key in keys)

slide("05 · Roadmap slide 04 · closed-loop environment", f'''
<h2>Sensor parts and costs</h2>
<p class="sensor-intro"><b>Basic kit: ${sensor_total("basic"):.2f}</b> · <b>Basic + pH / conductivity: ${sensor_total("basic", "chemistry"):.2f}</b> · Optional upgrades below are extra.</p>
<div class="matrix sensor-shopping" style="grid-template-columns:.8fr 2.45fr 1.2fr">
  <div class="head">Measurement / priority</div><div class="head">Amazon links · price per retail pack</div><div class="head">What it tells us / placement</div>
  {mrow("Shared interface<br><b>BASIC · buy once</b>", sensor_links("esp32", "usb") + "<br>" + sensor_links("wires", "bolts"), "One ESP32 → USB → Mac. Other boards are spares.")}
  {mrow("Temperature + humidity<br><b>BASIC</b>", sensor_links("temp") + "<br>" + sensor_links("humidity"), "Water probe on arm; air chip on cart. Detect cold / damp.")}
  {mrow("Water use / weight<br><b>BASIC</b>", sensor_links("scale", "weights"), "Cart scale: water-loss proxy. Weigh cut shoots for yield.")}
  {mrow("Light<br><b>BASIC + optional</b>", sensor_links("lux") + "<br>Optional spectrum: " + sensor_links("spectrum"), "Wrist: brightness / optional spectrum. Not calibrated PAR.")}
  {mrow("Growth / symptoms / wear<br><b>ALREADY INCLUDED</b>", "Robot cameras + servo telemetry · <b>$0 extra</b><br>Paper size-reference tags; reuse your Mac and powered hub.", "Photos show growth / symptoms; telemetry flags strain.")}
  {mrow("Leaks<br><b>BASIC</b>", sensor_links("leak"), "One fixed catch-tray pad. No sensors inside pods.")}
  {mrow("pH + conductivity<br><b>OPTIONAL chemistry</b>", sensor_links("ph", "ec") + "<br>" + sensor_links("kcl"), "Arm: acidity + salts. Both kits include calibration fluids.")}
  {mrow("Probe interface + care<br><b>WITH chemistry</b>", sensor_links("adc", "resistors", "rinse"), "Share ADC; dip separately, rinse, store pH wet in KCl.")}
  {mrow("CO₂ + airflow<br><b>OPTIONAL</b>", sensor_links("co2") + "<br>" + sensor_links("airflow", "battery"), "CO₂ on cart. Airflow: manual fan check, not still-air sensing.")}
  {mrow("Dissolved oxygen<br><b>DEFER this trip</b>", sensor_links("oxygen") + "<br>Required electrolyte: " + sensor_links("electrolyte"), "Oxygen in water. Reuses ADC + temperature probe.")}
</div>
<p class="caption sensor-note">Amazon US · Sep 20, 2026 · before tax / shipping · one pack per link. Totals exclude robot, printer, filament and tools. Wiring / firmware required. Obtain probe liquids locally in Japan; DO electrolyte is corrosive and not included in the kit. <a href="sensor-shopping.html" target="_blank">Full checklist and integration notes ↗</a></p>''', "Sensing · linked parts and costs · USD", fig=True)

slide("06 · Roadmap slide 05 · crop strategy", f'''<h2>One crop makes the experiment easier to interpret</h2>
<p style="margin-top:14px">Start with <b>garden cress, one seed batch</b>. Multiple species would test whether the robot can handle different plants, but they also introduce different growth rates and water needs. That comparison can wait.</p>
<div style="margin-top:10px">{FIG_WAVES}</div>
<p style="margin-top:16px"><b>Within each 12-pod batch:</b> 6 controls + 6 with one change, initially seed density. Keep water, container type and light positions comparable within the batch. Compare recipes within A and within B; changing the pot makes A-versus-B yield an unfair comparison.</p>
<p class="caption">Cress is a candidate, not a deadline guarantee: <a href="https://www.rhs.org.uk/education-learning/school-gardening/resources/getting-started/make-a-cress-head">RHS gives 7–14 days</a>. If unavailable, use one locally available sprouting crop throughout. Repeated batches test reliability across plant ages; this is not three completed generations in eleven days.</p>''', "Reconciliation · crops", fig=True)

slide("07 · The botanist agent", f'''<h2>The part that improves itself: an autonomous botany agent.</h2><div style="margin-top:4px">{FIG_AGENT}</div>
<div class="grid-4" style="margin-top:6px">
  <div class="card" style="padding:12px 14px"><h3 style="font-size:14px">What it is</h3><p style="font-size:12px;margin-top:4px">Your spec's thinking layer: "diagnosis, experiments … learning across crop cycles." An application on the MacBook calling a language model, with four tools: read the log, propose a wave, schedule robot tasks, call you.</p></div>
  <div class="card" style="padding:12px 14px"><h3 style="font-size:14px">What it experiments on</h3><p style="font-size:12px;margin-top:4px">Each batch compares 6 controls with 6 test pods. Change seed density first; keep other settings fixed. Log individual pods and window positions. Water-interface trials use a separate bench setup.</p></div>
  <div class="card" style="padding:12px 14px"><h3 style="font-size:14px">What can improve this trip</h3><p style="font-size:12px;margin-top:4px">A establishes the baseline; B checks an early hypothesis using growth observations, not a completed harvest. After A is harvested, C can test a new recipe beyond this stay. Pod redesign is a separate experiment.</p></div>
  <div class="card" style="padding:12px 14px"><h3 style="font-size:14px">Guard rails</h3><p style="font-size:12px;margin-top:4px">It only picks from a fixed menu of safe actions; scripts, not the model, move the arms; you approve every proposal until day 7; every decision and its reasoning lands in a notebook you can read.</p></div>
</div>''', "Intelligence layer", fig=True)

slide("08 · Roadmap slide 06 · capability roadmap", f'''<h2>Instrument → supervise → maintain in eleven days; the rest on later trips.</h2><div style="margin-top:8px">{FIG_STAGES}</div>''', "Reconciliation · stages", fig=True)

slide("09 · Day by day · before you fly", f'''
<h2>Sep 19–29 · software and digital design only</h2>
<div class="matrix tight" style="margin-top:18px;grid-template-columns:.7fr 1.7fr 1.8fr">
  <div class="head">When</div><div class="head">Planned output</div><div class="head">Check without hardware</div>
  {mrow(L("Sep 19–20"), "Set up the development environment; record dependency versions. Draft pod, split-tray and scale-plate CAD for an A1.", "Imports work on the Mac. CAD exports; each tray half targets ≤ 150 × 200 mm including joints.")}
  {mrow(L("Sep 21–23"), "Draft tags.py, image measurement and the SQLite log. Prepare tag / checkerboard PDFs. Slice draft parts in Bambu Studio.", "Read sample images and generated tags. Record slicer dimensions, material and time estimates; no physical print.")}
  {mrow(L("Sep 24–25"), "Draft round.py and sensor interfaces, scale firmware, watchdog and stop / retry logic.", "Mock serial readings and robot commands. Inject stale data and missing tags; confirm the scheduler stops.")}
  {mrow(L("Sep 26–27"), "Draft the botany notebook and approval screen; run a complete simulated round.", "Synthetic data stays labelled simulated. Unapproved recipes cannot become robot tasks. Export logs and resume after a mock crash.")}
  {mrow(L("Sep 28–29"), "Freeze the environment; save repos, CAD, PDFs and draft slicer projects offline. Write the Sep 30 checklist.", "Re-run the software checks offline. Leave calibration values unset until the real robot is present.")}
</div>
<p style="margin-top:18px"><b>Sep 30 is the first physical day:</b> assembly, wiring, calibration, paper tags, first print and first sowing all start in Kesennuma.</p>
<p class="caption">This is a proposed software work plan, not software already implemented. Printer assumption: Bambu A1; filament is being arranged by you. Final slices depend on the actual nozzle, plate and material.</p>''', "Plan · software preparation", fig=True)

slide("10 · Day by day · on site, part 1", f'''
<h2>D1 – D5 · build it, teach it, start the rounds.</h2>
<div class="matrix tight" style="margin-top:10px;grid-template-columns:.62fr 1.5fr 1.4fr 1.3fr 1fr">
  <div class="head">Day</div><div class="head">Build · print</div><div class="head">Test — done when…</div><div class="head">Software</div><div class="head">Plants</div>
  {mrow(L("D1 · Wed<br>Sep 30"), "Check the box against the contents sheet. Build the IKEA cart. Bolt on both arms, the wheel base and the head (3–4 h). Run from the wall supplies first. " + L("Printer:") + " confirm A1 / nozzle / plate; print one cup + sieve + lid first, then check fit and leaks before batches.", "Every joint and both wheels move; all three cameras stream at once through the hub.", "Find the ports · set the base and head servo IDs · calibrate both arms and the head.", W("Sow wave A by hand") + " in 12 shop pots, 6 control + 6 test. Follow the seed packet; keep paper moist. Growth starts regardless of robot progress.")}
  {mrow(L("D2 · Thu<br>Oct 1"), "Battery in (USB-C → 12 V). Tape the home spot and two park spots; stick down floor and table tags. " + L("Printer:") + " after first-part checks, batch cups and inserts; print paper tags and checkerboard locally.", "Keyboard drive on the real floor; 10/10 returns to the home spot by hand. Inject a simulated fault to check the watchdog; do not force a real joint into a stall.", "Camera calibration (checkerboard) · tags.py prints each tag's position live.", "Mist wave A. Still covered.")}
  {mrow(L("D3 · Fri<br>Oct 2"), "Assemble and wire scale + HX711 + ESP32 if available; calibrate across the expected pod weights. Otherwise keep camera-only logs. " + L("Printer:") + " scale plate, then tray 1 as two 6-pod halves. Measure actual print time and adjust the queue.", "20/20: reach a tagged pod → grip the lip → lift → put it back. Log every attempt.", "First primitives: reach-to-tag · grasp · place. scale.py logging.", "Uncover wave A, move to the window. First photos with the colour card.")}
  {mrow(L("D4 · Sat<br>Oct 3"), "Join and fit-check the first printed tray if ready; shop pots remain the fallback. " + L("Printer:") + " tray 2 halves and a guide only after tray 1 fits.", "One full round on 12 pods by script: park → photo → weigh → return. Time it (target &lt; 25 min).", "round.py chained · database live · grow.py curves for wave A.", W("Sow wave B") + " in 12 matching containers: 6 controls + 6 test pods. Use printed pods if ready. Early observations may inform B; no harvest result exists yet.")}
  {mrow(L("D5 · Sun<br>Oct 4"), L("Printer:") + " funnel and separate gutter test piece if core parts are complete. Test water routing away from crop comparisons.", W("Stage 1 starts.") + " Rounds at 10:00 and 14:00. Break things on purpose: cover a tag, remove a pod, unplug a camera — does it notice?", "Approve-from-your-phone page · agent writes its first daily notebook.", "Top up by weight when the robot asks.")}
</div>''', "Plan · on site", fig=True)

slide("11 · Day by day · on site, part 2", f'''
<h2>D6 – D11 · hand it over, then read the results.</h2>
<div class="matrix tight" style="margin-top:10px;grid-template-columns:.62fr 1.4fr 1.6fr 1.3fr 1fr">
  <div class="head">Day</div><div class="head">Build</div><div class="head">Test — done when…</div><div class="head">Software</div><div class="head">Plants</div>
  {mrow(L("D6 · Mon<br>Oct 5"), "Water jug + a squeeze bottle the gripper can hold (or a pump at the weigh dock if you have one). " + L("Printer:") + " pod v2 if the grasp log asks for it.", "Robot tops up one pod to its target weight, 10/10. Robot carries a finished pod to the bin and a fresh one to the slot, 10/10.", "top-up and swap primitives · agent compares wave A recipes.", W("Measure A + B") + "; keep recipes fixed. Do not assume a harvest on day 6 or change half a crop batch to shared water.")}
  {mrow(L("D7 · Tue<br>Oct 6"), "—", W("Stage 2 only if D6 checks pass.") + " Otherwise continue supervised rounds. Log refills, charging, harvest cuts and every intervention.", "Scheduled start at boot · nightly backup of database + photos · phone alerts.", "Assess cress A; harvest only if ready. Robot moves pots; you cut and weigh shoots.")}
  {mrow(L("D8 · Wed<br>Oct 7"), L("Printer:") + " sealed snap-in connector, first try (water design C).", "Candidate autonomy window; log every intervention. Keep printer work separate from farm tasks.", "Tune only if it stops. Note each fix.", "—")}
  {mrow(L("D9 · Thu<br>Oct 8"), "Tape the tile guide to the table. Bench-test the connector over a tray: does it seal, does it leak when mis-seated?", "Review the autonomy log; remain supervised if the acceptance checks have failed.", "Agent's mid-run report: best recipe so far, with evidence.", "Assess A for harvest; photograph and weigh B. No fixed harvest deadline.")}
  {mrow(L("D10 · Fri<br>Oct 9"), "—", W("Stage 3 demo:") + " slide an empty tray from a supported cart surface into the guide, 10 timed attempts. Test the connector separately over a catch tray if ready. Log the demo as an intervention.", "tile-install primitive (slide-and-seat).", "Measure B; harvest only if ready. Keep unready plants for the final growth report.")}
  {mrow(L("D11 · Sat<br>Oct 10"), "Photograph the rig; label and pack, or leave it running.", "Read out the numbers on slide “Evaluation”. List every printed part you would change.", "Export database + photos · agent's final report: A/B comparisons with incomplete harvests clearly marked; proposal for C.", "Harvest ready A/B; record growth for the rest. Sow C only in freed slots with an agreed caretaker, or defer it.")}
</div>''', "Plan · on site", fig=True)

slide("12 · Software by when", f'''
<h2>What has to exist, by which day, and who writes it.</h2>
<div class="matrix tight" style="margin-top:10px;grid-template-columns:.8fr 2.2fr .9fr 1.6fr">
  <div class="head">By</div><div class="head">Software</div><div class="head">Source</div><div class="head">Done means</div>
  {mrow(L("Sep 21"), "macOS toolchain: Homebrew · a Python environment compatible with the selected LeRobot version · LeRobot (+ Feetech) · XLeRobot repo · OpenCV · pupil-apriltags · pyserial · SQLite · Arduino IDE + ESP32 core", "existing, open source", "imports pass; mock robot and recorded-image checks run without farm hardware")}
  {mrow(L("Sep 27"), "tags.py · grow.py · log database · round.py skeleton · scale firmware + scale.py · servo watchdog", "I draft, you run", "each passes on sample images or simulated readings; environment archived offline")}
  {mrow(L("D1 – D2"), "LeRobot motor setup (base + head IDs) · arm and head calibration · XLeRobot keyboard teleop · camera calibration", "existing", "every joint and wheel moves; three cameras at once; tags give positions")}
  {mrow(L("D3 – D4"), "Primitives: park-at-tag · reach-to-tag · grasp pod · place on scale · return — all referenced to tags, none learned", "written on site from the skeleton", "20/20 repetitions each, logged")}
  {mrow(L("D5 – D6"), "The scheduled round (10:00 / 14:00) · approve-from-phone page · top-up-to-weight · pod swap", "us", "two rounds a day with no crash; approvals arrive on your phone")}
  {mrow(L("D5 +"), "Botanist agent: a language model called over the internet with four tools — read_log · propose_wave · schedule_task · escalate — plus a daily notebook", "I draft before, tune on site", "one bounded recipe proposal is approved and logged; execution waits for available pots")}
  {mrow(L("D7 +"), "Start-at-boot job · watchdog · nightly backup of database and photos · alerts", "us", "target zero manual restarts; count actual hours, charging and human interventions")}
</div>
<p class="caption">No imitation learning on this trip: on this arm it buys 60–90 % success for hundreds of demonstrations and does not generalise. Tag-referenced scripts are both faster to write and more reliable. The 8 GB MacBook runs all of the above; it would not run training.</p>''', "Plan · software", fig=True)

slide("13 · Roadmap slide 07 · research landscape", f'''
<h2>Nine research areas, each with a first experiment on this rig.</h2>
<div class="matrix tight" style="margin-top:10px;grid-template-columns:1.35fr .55fr 2.5fr">
  <div class="head">Area</div><div class="head">When</div><div class="head">The experiment</div>
  {mrow(L("Autonomous botany agent — recipes that improve each cycle"), L("this trip"), "One crop, 6 controls + 6 test pods per batch. Change one variable, initially seed density. B uses early observations; any harvest-informed C continues after the trip. Record uncertainty.")}
  {mrow("Basic hydroponic control", "this trip", "Hold every pod within ± 5 g of its target water weight for four unattended days.")}
  {mrow("Longitudinal crop perception", "this trip", "Twice-daily photo stacks with a colour card: does green area predict harvest grams under changing daylight?")}
  {mrow("Failure diagnosis + recovery", "this trip", "Break things on purpose — covered tag, missing pod, unplugged camera, dry pod — and score whether it names the cause.")}
  {mrow("Robot-serviceable farm design", "this trip", "Docking success vs tag layout; grasp success vs lip and cone shape. The printed interface is the experiment.")}
  {mrow("Autonomous cleaning + sanitation", "this trip", "Swap instead of scrub: how many pods are lost to mould per wave when used pods are simply replaced?")}
  {mrow("Automated construction + commissioning", "D10 demo", "Slide an empty tray between supported surfaces: success rate and minutes per install. A second table is later.")}
  {mrow("Dense, deformable plant manipulation", "later", "Cutting tendrilled pea shoots; sowing single seeds. Deferred: every farm-automation post-mortem says tactile work is last.")}
  {mrow("Recursive system replication", "later", "Printer → nest → pod → tile → table: share made on site, human-minutes per copy.")}
</div>''', "Reconciliation · research", fig=True)

slide("14 · Roadmap slide 08 · highest-value thesis", f'''
<h2>Co-design the farm, the reasoner and the robot as one machine.</h2>
<div class="grid-3" style="margin-top:18px">
  <div class="card"><div class="num">ROBOT-SERVICEABLE INFRASTRUCTURE</div><h3>The interface is the product</h3><p>Everything the robot touches is a rigid plastic part with a ± 5 mm target: a pod lip, a cone base, a tile guide. This cheap arm is loose by about a degree, and it never has to be better than that. FarmBot, Jubilee, Opentrons and every 95 %-success transplanter teach the same lesson.</p></div>
  <div class="card" style="background:linear-gradient(150deg, rgba(94,214,222,.10), rgba(255,255,255,.018))"><div class="num">AI FAILURE DIAGNOSIS</div><h3>Three kinds of clue</h3><p>{W("Machine:")} motor strain, docking retries. {W("Sensor:")} scale disagreeing with itself, tags going missing. {W("Plant:")} growth slowing, colour shifting, weight not dropping. Every fault you cause on purpose is labelled; the agent learns from your farm's own failure log.</p></div>
  <div class="card"><div class="num">VERIFIED ROBOTIC REPAIR</div><h3>Repair = replace, then prove</h3><p>A deliberately small vocabulary: swap a pod, swap a tile, top up, recalibrate, re-seat. Each ends with a check — re-weigh, re-photograph, re-read the tag — before the log entry closes. The farm is designed so those five verbs are enough.</p></div>
</div>''', "Reconciliation · thesis")

slide("15 · Roadmap slide 09 · evaluation", f'''
<h2>Four questions. Fourteen numbers behind them.</h2>
<div class="grid-2" style="margin-top:14px;gap:12px">
  <div class="card" style="padding:16px 20px"><div class="num" style="margin-bottom:8px">THIS TRIP</div><h3>1 · How much did you have to help?</h3><ul style="padding-left:18px;margin:8px 0 0">
    <li style="font-size:13.5px">{W("Hands-on time per harvest")} — tap start/stop whenever you touch the farm; we add it up.</li>
    <li style="font-size:13.5px">{W("Longest stretch without you")} — hours between two interventions.</li>
    <li style="font-size:13.5px">{W("Share of jobs done alone")} — of every photo, weigh, top-up and swap, the % with no human.</li></ul></div>
  <div class="card" style="padding:16px 20px"><div class="num" style="margin-bottom:8px">THIS TRIP</div><h3>2 · Did it notice and fix problems?</h3><ul style="padding-left:18px;margin:8px 0 0">
    <li style="font-size:13.5px">{W("Caught it?")} — we break things on purpose and count how often it notices and names the right cause.</li>
    <li style="font-size:13.5px">{W("Fixed it alone?")} — % of problems solved without calling you.</li>
    <li style="font-size:13.5px">{W("Robot reliability")} — successful grasps and parks out of attempts; pods knocked over.</li>
    <li style="font-size:13.5px">{W("Wearing out?")} — do those success rates fall as the hours add up.</li></ul></div>
  <div class="card" style="padding:16px 20px"><div class="num" style="margin-bottom:8px">THIS TRIP</div><h3>3 · Did the plants do well, and at what cost?</h3><ul style="padding-left:18px;margin:8px 0 0">
    <li style="font-size:13.5px">{W("Survival, yield, speed")} — pods sown vs harvested; grams per pod; days to harvest.</li>
    <li style="font-size:13.5px">{W("Losses")} — pods lost to mould or drying out.</li>
    <li style="font-size:13.5px">{W("Inputs per gram")} — water (by weight) and electricity (battery %) per gram harvested.</li></ul></div>
  <div class="card" style="padding:16px 20px;opacity:.8"><div class="num" style="margin-bottom:8px;color:var(--aqua)">LATER TRIPS</div><h3>4 · Can it grow itself?</h3><ul style="padding-left:18px;margin:8px 0 0">
    <li style="font-size:13.5px">{W("Time to add a module")} — minutes from "new tile" to "growing" (first target on D10).</li>
    <li style="font-size:13.5px">{W("Made on site")} — share of parts that were printed, by count, weight and cost (count farm prints made in Kesennuma separately from kit parts).</li>
    <li style="font-size:13.5px">{W("Built by the robot")} — share of assembly steps it did itself.</li>
    <li style="font-size:13.5px">{W("Still shipped in")} — weight, cost and number of bought parts.</li></ul></div>
</div>
<p class="caption">Your slide's principle, unchanged: measure autonomy by what stops requiring people. Every number above is a count from the robot's log — nothing is estimated.</p>''', "Reconciliation · metrics", fig=True)

slide("16 · Roadmap slide 10 · open questions", f'''
<h2>Your twelve questions, sorted by when we will know.</h2>
<div class="grid-3" style="margin-top:14px;gap:12px">
  <div class="card" style="padding:16px 18px"><div class="num" style="margin-bottom:8px">ALREADY DECIDED · 5</div>
    <p style="font-size:13px;margin-top:0">{W("Smallest useful setup?")}<br>One cart, one table, 24 pods.</p>
    <p style="font-size:13px">{W("Rail, gantry or mobile robot?")}<br>Mobile: it must later reach a printer and a second table.</p>
    <p style="font-size:13px">{W("How is cleaning done?")}<br>Swap, don't scrub: used pod out, fresh pod in.</p>
    <p style="font-size:13px">{W("Disposable, replaceable or repairable?")}<br>Pods are disposable; tiles, probe and motors are replaceable; the cart is repairable.</p>
    <p style="font-size:13px">{W("What can be made on site?")}<br>Farm pods, tray halves and guides are printable; cart baskets, shelf decks and electronics are supplied parts.</p></div>
  <div class="card" style="padding:16px 18px;background:linear-gradient(150deg, rgba(201,246,91,.08), rgba(255,255,255,.018))"><div class="num" style="margin-bottom:8px">THESE ELEVEN DAYS ANSWER · 4</div>
    <p style="font-size:13px;margin-top:0">{W("What breaks most often?")}<br>The failure log will rank it.</p>
    <p style="font-size:13px">{W("Which jobs stay hard for the robot?")}<br>Failures counted per task: park, grasp, weigh, top up, swap.</p>
    <p style="font-size:13px">{W("Plant, sensor or machine problem — can it tell?")}<br>We break things on purpose and score its guesses.</p>
    <p style="font-size:13px">{W("When should it call you?")}<br>Starting rule: the same check fails twice, or any reading leaves its limits. We tune it on D5–D6.</p></div>
  <div class="card" style="padding:16px 18px;opacity:.85"><div class="num" style="margin-bottom:8px;color:var(--aqua)">LATER TRIPS · 3</div>
    <p style="font-size:13px;margin-top:0">{W("How do water and power plug in?")}<br>A water connector is a separate bench experiment this trip. Reliable automatic fluid and power connections remain later work.</p>
    <p style="font-size:13px">{W("Is a simulator worth it?")}<br>Only for checking drive paths and reach. Plants are grown for real.</p>
    <p style="font-size:13px">{W("How does a new module prove it is safe?")}<br>A self-check plus a quarantine day before it joins the others.</p></div>
</div>''', "Reconciliation · questions", fig=True)

slide("17 · Roadmap slide 11 · version zero", f'''
<h2>The first operational envelope, defined.</h2>
<div class="v0"><div class="v0-list">
  <div class="v0-row"><span>01</span><p>{W("Crop family + method:")} one crop, initially garden cress, on moist kitchen paper; water depth follows the tested container design: shop pots for A; printed pods for B if ready, with matching controls in each batch. No nutrients. Water parts arrive as printed experiments from D6.</p></div>
  <div class="v0-row"><span>02</span><p>{W("Dimensions, utilities, interfaces:")} one table (~72 cm) by a window; two 12-pod trays, each joined from two printed halves targeting ≤ 150 × 200 mm per half; daylight; one wall socket. Paper AprilTags on floor, table, tiles and lids.</p></div>
  <div class="v0-row"><span>03</span><p>{W("Robot class + workspace:")} XLeRobot — two SO-101 arms, reach 0.5–1.25 m high and 0.36 m past the cart edge, 0.6–1 kg per arm, two-wheel drive, 10–12 h per charge, a MacBook as the brain.</p></div>
  <div class="v0-row"><span>04</span><p>{W("Autonomous duration + cycles:")} eleven days to assemble and test; Oct 6–9 is a conditional autonomy target with charging and interventions logged. A is the primary harvest attempt; B may finish later.</p></div>
  <div class="v0-row"><span>05</span><p>{W("Permitted human intervention:")} refill, charge, cut harvests, replenish pots and answer alerts. Log every intervention; classify scheduled maintenance separately from failures.</p></div>
</div>
<div class="north-star"><div class="big">V0</div><p>"A testable system, not a miniature dream." It exposes real maintenance failures (dry pods, mould, missed grasps, lost tags, hot motors), supports repeatable experiments (one crop, controls and injected faults) and produces the data the next stage needs: a failure log and a ranked recipe book.</p></div></div>''', "Reconciliation · V0")

slide("18 · The robot", f'''
<h2>XLeRobot, in one slide.</h2>
<div class="grid-2" style="margin-top:14px;grid-template-columns:1fr 1.1fr;align-items:start">
  {gal([("xlerobot.png","<b>XLeRobot 0.4</b> · Vector Wang · Apache-2.0"),("so101.webp","<b>2 × SO-101</b> · TheRobotStudio / LeRobot"),("lekiwi.jpg","<b>LeKiwi drive</b> · SIGRobotics-UIUC"),("apriltag.png","<b>AprilTags</b> tell it where things are")], 2, "16/9")}
  <div class="stack">
    <div class="module"><b>Body</b>IKEA RÅSKOG cart · 2 SO-101 follower arms with soft TPU grippers · two 127 mm drive wheels · a 2-axis head · 3 cameras · a 288 Wh battery · ~12 kg</div>
    <div class="module"><b>Numbers</b>16 Feetech STS3215 servos · works 0.5–1.25 m high, 0.36 m past the cart edge · 0.6–1 kg per arm · 10–12 h per charge · 3–4 h to assemble with pre-built arms</div>
    <div class="module"><b>Known limits, all handled</b>about 1° of slop in the joints → cones and lips do the final millimetres · motors can reach 70 °C with no built-in cutoff → our watchdog stops them at 60 °C · no self-docking → floor tags, a taped home spot, three retries</div>
    <div class="module"><b>Brain</b>the MacBook on the bottom tray, through one powered USB hub. No Raspberry Pi, no depth camera.</div>
    <div class="module"><b>Software</b>LeRobot-native · keyboard or gamepad driving · a browser simulator · 4.8k★, 6k+ builders</div>
  </div>
</div>{CREDIT}''', "The robot", fig=True)

slide("19 · Roles", f'''<h2>Left arm senses, right arm handles.</h2><div style="margin-top:8px">{FIG_ARMS}</div>
<p class="caption">All sensing travels with the robot. If you add the pH / conductivity probe, it rides the left wrist in a soft holder, is rinsed between pods and rests in storage solution on the cart.</p>''', "The robot", fig=True)

slide("20 · The daily round", f'''<h2>Eight steps, scripted, twice a day.</h2><div style="margin-top:12px">{FIG_ROUND}</div>
<div class="grid-3" style="margin-top:8px">
  <div class="card"><h3>Finding its way</h3><p>Paper AprilTags on the floor and the table edge. It approaches slowly, stops within 2 cm, retries up to three times, then calls you.</p></div>
  <div class="card"><h3>Why scripted</h3><p>Teaching this arm by demonstration takes hundreds of examples for 60–90 % success and fails on anything new. A tag-referenced script is quicker to write and more reliable.</p></div>
  <div class="card"><h3>Protection</h3><p>A watchdog checks every motor's strain ten times a second. Work is seconds long, rests are minutes long.</p></div>
</div>''', "The robot", fig=True)

slide("21 · On the cart", f'''<h2>What rides where.</h2><div style="margin-top:8px">{FIG_CART}</div>''', "The robot", fig=True)

slide("22 · Beyond this trip", f'''<h2>What “tray” and “shelf” mean in this plan</h2>
<div class="matrix materials" style="margin-top:18px;grid-template-columns:1.1fr 1.55fr 1.65fr">
  <div class="head">Part</div><div class="head">Where it comes from</div><div class="head">Who assembles / moves it</div>
  {mrow("[H] Table · this trip", "Existing or borrowed furniture. No rack needed Sep 30–Oct 10.", "You position it; the robot docks at its edge.")}
  {mrow("[P] Plant tray / tile", "Custom 3D print: two 6-pod halves + joining clips. It locates pots; it is not the furniture shelf.", "You print and join it. Robot first slides an empty tray along guides; add pods individually.")}
  {mrow("[P] Pods + water insert", "Printed cups, sieve inserts and lids. Optional separate PETG gutter + funnel; real wicks and seals.", "You leak-test over a catch tray. Robot can handle tested pods; wet trays stay supported.")}
  {mrow("[H] Robot's cart trays", "The metal baskets supplied with your IKEA RÅSKOG. These are a third meaning of “tray”.", "You assemble the cart and mount the kit. We are not printing those baskets.")}
  {mrow("[S] Plant shelves · later", "Default: a complete metal shelving unit, including uprights and shelf decks. Printed guides attach to it.", "You assemble and secure the shelf. Robot installs pod trays into reachable slots; rack construction remains research.")}
</div>
<p style="margin-top:14px"><b>A1 print plan:</b> 256 × 256 mm bed. The old 260 × 190 mm tray is too long for a flat, axis-aligned print. Each proposed half fits inside 150 × 200 mm; validate joints, brim and clearances in the slicer.</p>
<p class="caption">CAD dimensions are design targets, not tested files. A loaded 12-pod tray is not an arm payload: at 200 g per pod, plants alone weigh 2.4 kg. Use supported sliding and carry pods individually. Sources: <a href="https://us.store.bambulab.com/products/a1?variant=41583355232392">A1 specifications</a> · <a href="https://xlerobot.readthedocs.io/en/latest/">XLeRobot payload limits</a>.</p>''', "Later · component origins", fig=True)

slide("23 · Risks", f'''
<h2>Risks and fallbacks</h2>
<div class="grid-3" style="margin-top:16px">
  <div class="card"><div class="num">01</div><h3>The pod design does not exist yet</h3><p>CAD can be drafted before Sep 30; first fit and leak checks happen on D1 in Kesennuma. Print one cup before batching. Shop pots cover A and remain the fallback for B.</p></div>
  <div class="card"><div class="num">02</div><h3>Print time is still unmeasured</h3><p>Queue: one pod test → cups → scale plate → tray halves → water parts. Replace the unsupported 75-hour estimate with actual slices. Reduce to 12 pots if needed.</p></div>
  <div class="card"><div class="num">03</div><h3>The robot is late or a part is missing</h3><p>Day 1 is an inventory against the contents sheet. Wave A is sown by hand and logged by phone camera regardless, so the plants never wait for the robot.</p></div>
  <div class="card"><div class="num">04</div><h3>Grey, wet weather</h3><p>Colour card in every photo; compare pods within a photo, not across days. A clip lamp is the backup for night rounds, not for growth.</p></div>
  <div class="card"><div class="num">05</div><h3>Driving and parking is unreliable</h3><p>Fallback: leave the cart parked at tile 1 for the whole stay. Twelve pods, no driving, every other experiment intact.</p></div>
  <div class="card" style="background:linear-gradient(150deg, rgba(94,214,222,.10), rgba(255,255,255,.018))"><div class="num">06</div><h3>An 8 GB laptop and hot motors</h3><p>Software is checked with mocks before Sep 30; hardware compatibility and timing are first tested on site. The watchdog goes in on D2, before any unattended minute.</p></div>
</div>''', "Risks", fig=True)

slide("End state", '''<p class="focus-quote">A farm that keeps going, on a cart that can build the next one.<br><span class="accent">Eleven days:</span> one crop, two measured batches, a first harvest attempt, and a logged autonomy trial.</p>
<div class="title-meta" style="margin-top:34px"><span class="pill">Sep 30 – Oct 10 · Kesennuma</span><span class="pill">Stages 0 → 2 + a Stage 3 demo</span><span class="pill">printed pods · daylight · water experiments</span><span class="pill">botanist agent in the loop</span></div>''', "XLeRobot · Kesennuma V0", kick_cls="kicker")

slide("Printer handoff · Bambu A1", f'''<h2>Can the robot take a print directly from the A1?</h2>
<p style="margin-top:14px"><b>Potentially, after it has released from the plate.</b> Picking up a loose cup is a reasonable first experiment. Peeling an attached print off the bed is a different task, and is not a demonstrated capability of this farm.</p>
<div class="matrix materials" style="margin-top:16px;grid-template-columns:.75fr 1.8fr 1.7fr">
  <div class="head">Step</div><div class="head">What happens</div><div class="head">What must be proved</div>
  {mrow("1 · Human handoff", "You let the plate cool, release the part and put it in a fixed pickup nest beside the printer.", "Robot picks the same rigid lip and places the part. Start with one small empty cup; log 20 attempts.")}
  {mrow("2 · Direct pickup", "Print one cup at a known reachable location. After cooling and a release check, robot lifts it from the stationary bed.", "Custom coordination: job finished, bed and head parked, temperatures acceptable, new jobs locked out, arm clear before restart. Stop if the part resists.")}
  {mrow("3 · Automated release", "A later fixture flexes or exchanges the plate, or a separately validated ejector frees the print.", "Plate seating, debris detection, collision clearance and repeated release must work before unattended reprints. A spare plate alone does not automate this.")}
</div>
<p style="margin-top:16px"><b>This trip:</b> human release → robot pickup is the fallback; direct bed pickup is optional after the farm works. The printer sits on a dry bench with clearance for its moving bed. Do not rely on automatic ejection or use the arm to pry a stuck print.</p>
<p class="caption">Bambu recommends letting the textured PEI plate cool before removal. The integration above is our proposed experiment; no stock A1 auto-eject feature is assumed. Sources: <a href="https://us.store.bambulab.com/collections/accessories-for-a1-mini/products/bambu-textured-pei-plate">Bambu plate guidance</a> · <a href="https://us.store.bambulab.com/products/a1?variant=41583355232392">A1 specifications</a> · <a href="https://xlerobot.readthedocs.io/en/latest/">XLeRobot limits</a>.</p>''', "Appendix · printer pickup", fig=True)

slide("Your comments", '''<h2>Your comments</h2>
<div style="display:flex;gap:10px;align-items:center;margin-top:14px"><button class="cm-btn cm-big" id="cm-copy-big" type="button">Copy all comments</button><button class="cm-btn" id="cm-clear" type="button">Clear all</button><span class="small">Saved in this browser only. Click a comment to jump to its slide.</span></div>
<div id="cm-summary"></div>''', "Review", kick_cls="kicker", fig=True)

n = len(S)
COMMENTS = (HERE / "deck-comments.inc.html").read_text().replace("xlerobot-deck-comments-v1", "xlerobot-deck-comments-v2")
html = f'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Autonomous farm — the roadmap on XLeRobot</title>
{style}
{extra}
<style>
.fig .lb, .fig .lbs, .fig .axis {{ fill: #c3cdd4; }}
.fig .ln2 {{ stroke: #91a0ab; }}
.site-map .site-print {{ fill: url(#site-hatch); stroke: #e3edf6; stroke-width: 1.5; }}
.site-map .site-have {{ fill: #172229; stroke: #c3cdd4; stroke-width: 2; }}
.site-map .site-kit {{ fill: #253b4d; stroke: #e3edf6; stroke-width: 2; }}
.site-map .site-shop {{ fill: #151e25; stroke: #e3edf6; stroke-width: 1.5; stroke-dasharray: 4 2; }}
.site-map .lbs {{ font-size: 11px; }}
.site-map .lb {{ font-size: 12px; }}
.crop-waves {{ display: grid; grid-template-columns: 1.1fr 1fr 1.45fr; border-top: 1px solid #71818d; }}
.crop-waves > div {{ padding: 14px 16px; font-size: 17px; line-height: 1.35; border-bottom: 1px solid #52616c; }}
.crop-waves span {{ font-size: 14px; color: #b9c4cc; }}
.crop-waves .wave-head {{ font-size: 12px; text-transform: uppercase; letter-spacing: .07em; color: #dce5dc; }}
.matrix.materials > div {{ font-size: clamp(13px, 1.05vw, 16px); padding: 12px 14px; line-height: 1.4; }}
.matrix.materials > .head {{ font-size: 12px; }}
.figslide h2 {{ max-width: 31ch; }}
.caption {{ color: #b2bfc6; }}
.sensor-intro {{ font-size: 16px; margin: 12px 0; }}
.matrix.sensor-shopping > div {{ font-size: clamp(11px, .93vw, 14px); padding: 5px 9px; line-height: 1.3; }}
.matrix.sensor-shopping > .head {{ font-size: 10px; }}
.matrix.sensor-shopping a {{ white-space: normal; }}
.sensor-note {{ font-size: 11px; line-height: 1.4; margin-top: 9px; }}
.body a, .body a:visited {{ color: #dcebf7; text-decoration: underline; text-underline-offset: 3px; }}
@media (max-height: 850px) {{
  .title-layout h1 {{ font-size: clamp(48px, 6vw, 84px); }}
  .title-layout .lead {{ font-size: 20px; }}
  .title-layout .pill {{ padding: 7px 10px; }}
}}
</style>
<style>.matrix.tight > div {{ font-size: clamp(10px, .84vw, 13px); padding: 7px 10px; line-height: 1.35; }} .matrix.tight > .head {{ font-size: 10px; }}</style>
</head><body><div class="deck">
{"".join(S)}
</div>
<div class="help">← → Navigate · F Fullscreen · Home / End</div>
<div class="controls" aria-label="Slideshow controls"><button class="nav-btn" id="prev" aria-label="Previous slide">←</button><span class="counter" id="counter">01 / {n:02d}</span><button class="nav-btn" id="next" aria-label="Next slide">→</button></div>
<div class="progress" id="progress"></div>
{script}
{COMMENTS}
</body></html>'''
(HERE / "xlerobot-deck.html").write_text(html)
from build_sensor_shopping import build as build_sensor_shopping
build_sensor_shopping(HERE)
print(f"wrote xlerobot-deck.html · {n} slides · {len(html)//1024} KB")
