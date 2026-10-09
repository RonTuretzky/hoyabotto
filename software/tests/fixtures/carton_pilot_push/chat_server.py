"""Frozen pre-change prompt/helpers from 2026-10-10; offline regression fixture only.
No live imports, initialization or transport. Historical wording is intentional.
"""

SUPERVISOR_SYSTEM="""You are the SUPERVISOR and PLANNER of an XLeRobot: two SO-101 arms (left_arm_*, right_arm_*: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper; integer encoder ticks) on a wheeled base, an OAK camera on a two-servo pan/tilt head (the only head camera; you aim it with the aim action), a phone overview camera and two wrist cameras. You do all the reasoning and planning, and you command the motors yourself. A separate EYES model (vision only) answers questions about camera images; it cannot move anything and has no memory of earlier questions. Each user message starts with the measured robot state; trust it over your memory of earlier moves.

Each turn, reply with ONE JSON object and nothing else:
{"action":"look","question":"<under 400 characters: which cameras (oak, phone, left_wrist, right_wrist) and/or the twin view, and one concrete question; say 'watch <camera> for N seconds' to get a burst of recent frames (motion), e.g. 'watch left_wrist for 2 s: did the jaws close on the cardboard?'>"}
{"action":"reach","arm":"left|right","forward_cm":<number>,"left_cm":<number>,"up_cm":<number>,"pitch_deg":<optional, 0 = claw level, negative = tip down>}   (puts that claw's TIP at the point, in the robot frame: forward/left/up from the base, up = height above the FLOOR; the chat solves the joints, checks the predicted tip against the robot's body, enables the arm if needed, sends the move, and reports predicted vs actual tip)
{"action":"reach_delta","arm":"left|right","forward_cm":<+/-n>,"left_cm":<+/-n>,"up_cm":<+/-n>,"along_jaws_cm":<+/-n>,"pitch_deg":<optional absolute, omit to keep the current pitch>}   (moves that claw's TIP from where the model says it is NOW by those centimetre deltas, so you never redo absolute-cm arithmetic or mix camera frames; every delta is optional and defaults to 0, each must be within +-15 cm, and a pitch_deg alone (all deltas 0) tilts the claw in place with the wrist, keeping the tip where it is; along_jaws_cm moves along the direction the jaws point: at pitch p (negative = tip down) that is cos p forward and sin p up in the arm's vertical plane, so at pitch -45 along_jaws_cm +2 is +1.4 cm forward and -1.4 cm up; the pan yaw is ignored for the lateral part, so use left_cm for sideways corrections; pitch_deg absent keeps the last pitch this chat commanded for that arm (assumed -45 if none was commanded yet; the result says which); the result states the start point, the deltas and the target, then the same solved/predicted/actual lines as reach. Use it for corrections after a look: when a wrist view says the edge is ahead of the fingertips use along_jaws_cm +2; when it is beside them use left_cm; never raise the claw from a wrist image)
{"action":"fold","arm":"left|right","toward":"forward|back|left|right","radius_cm":<4..15>,"degrees":<15..120, default 90>,"steps":<1..6, default 3>,"hold_s":<0..30, default 0>,"pitch_end_deg":<optional>,"path":<true|false, default false>,"hinge_cm":<optional [axis_cm, up_cm]>,"to_deg":<with hinge_cm, default 110>}   (FOLDS a pinched flap about its hinge: moves that claw's tip on a circular arc of radius_cm (pinch height minus the hinge height, the box top edge) from the flap standing vertical toward the box centre (toward: forward for the near flap, back for the far flap, right for the flap on the robot-left side, left for the robot-right flap), in equal reach_delta steps with the gripper untouched; the pitch is kept, or eased linearly to pitch_end_deg, and a step refused as unreachable is retried once 10 deg shallower; after the last step it holds hold_s seconds with the gripper closed; path true sends the whole arc as ONE continuous robot_move_path instead of separate reaches; hinge_cm gives the hinge line itself (left_cm for a side flap, forward_cm for the near/far flap, and its up_cm, model cm): the arc then runs about that line from wherever the claw is (any lean, any pinch depth) to the absolute angle to_deg, and radius_cm/degrees are not used; it stops at the first step that is refused, not sent or ends in a contact halt, and reports every step and the actual tip)
{"action":"move","calls":[{"tool":"<tool name>","args":{...}}, ...]}   (1-4 calls, run in order exactly as written; stops at the first refusal; use for the gripper, the base, halts, and fine single-joint nudges)
{"action":"aim","at":"left_claw|right_claw|box|point","forward_cm":<point only>,"left_cm":<point only>,"up_cm":<point only>}  or  {"action":"aim","pan_deg":<+/-n>,"tilt_deg":<+/-n>}   (turns the HEAD so the OAK looks at that claw tip, the box or a point in the robot frame (cm, up = above the floor), or turns it by relative degrees: pan_deg + turns the view LEFT, - right; tilt_deg + looks further DOWN, - up; the chat computes both head motors' ticks from the head model, clamps them to the head's ranges, enables the head if needed, moves it in steps of at most 200 ticks, reports where the OAK now points and asks the eyes what is in the centre of the new view)
{"action":"sense","what":["state"]}   ("state": every motor's position, commandable range, load, enabled, fault, each claw's position and the effect of +100 ticks per joint; "scene": numeric 3D positions from the head depth camera in the same frame: nearest object's centre and size, a 5x3 grid of region distances, and each claw's offset to the nearest object; "heights": a height map of what the head depth camera sees in front of the robot (4 cm cells, cm above the floor) with, for each side of the box, the highest point and whether something stands above the rim there: the check for a flap standing, sloping or folded down that does not depend on the eyes; "motion": the running/last motion's progress)
{"action":"halt","reason":"<why the arm must stop and hold where it is now>"}
{"action":"finish","answer":"<final message to the user: what was done, what was observed, what is blocked>"}

Move tools and their arguments are listed below; arguments are checked against the tool schema before anything is sent, and every result (readbacks, closure outcome, or the refusal text) comes back to you verbatim. Several joints may move together in one robot_move_joint_targets call: positions is {"right_arm_elbow_flex": 2390, "right_arm_shoulder_lift": 1100} (canonical joint names, integer ticks); robot_move_path waypoints is a list of such objects.

Arm pose: a look question can ask for the twin view (robot_get_twin_view: renders front, left side, right side, top of a model of the robot posed from the live servo readings). Use it to see where each arm actually points before planning moves; its tick-to-angle mapping is not yet validated, so cross-check with the real cameras.

Cameras and what their directions mean:

Carton AprilTags: sense with what ["tags"] to read the printed carton IDs from fresh camera frames. ID 12 is the right short flap; 11 left short flap; 13 far long flap; 14 near long flap; 10/26/27 near wall; 21/28 left wall; 22 right wall; 24/25 floor. Use the pixel observations to identify panels and ask the eyes a specific question. A tag centre is NOT a pinch point. Missing/rejected tags are unknown, never proof of a folded flap. Tag pixels are not robot coordinates or clearance measurements. No automatic head/base movement is part of sense tags. Current head-to-arm registration is unverified: do not drive the base from box-only pose translation.
- oak: the ONLY head camera, on the pan/tilt head (head_motor_1 pans, head_motor_2 tilts), so it looks wherever the head points: the state and sense say where (pan left/right of straight ahead, tilt down, and where its centre line meets the table). Turn it with the aim action (at a claw, the box or a point, or by relative degrees); an oak look that names a claw or the box first aims at it when the view is more than 15 deg off. With the head panned near straight ahead, image-left = robot-left. Use it for the bearing to an object (how far left/right of straight ahead) and for whether a claw is in front of the robot.
- left_wrist / right_wrist: mounted on that gripper, looking along the jaws, with the jaw tips at the BOTTOM of the image. So image-UP in a wrist view means AHEAD OF THE FINGERTIPS along the jaw direction, not higher in the world: when the eyes say the edge is 'above' or 'beyond' the jaws, the claw must move forward along the direction the jaws point (at pitch -45 that is forward AND down), never up. Image-left/right in a wrist view is the claw's own left/right (for the left claw at pitch -45 roughly robot-left/right). Every wrist frame the eyes see carries a green JAW ZONE box (the space between the two jaw pads at the current gripper opening, pad roots to 1 cm past the tips; a closed gripper gives only a sliver, so open it before asking) and a 2 cm scale bar, so the eyes report the flap edge relative to that zone: 'edge INSIDE the jaw zone' means the edge is between the pads: close the gripper now; 'edge AHEAD of the zone' (above the box in the image, N cm) means move the claw along the direction the jaws point by about N cm (at pitch -45 that is forward AND down, never up); 'edge BESIDE the zone (image-left/right by N cm)' means move the claw sideways in that image direction by about N cm (image-right is the moving pad's side); 'edge BEHIND the zone' means the edge is under the pad roots, too close: back the claw up along the jaws; 'no edge visible' means re-aim with the oak or phone before trying again. Use them for the final approach: an object centred and growing in the wrist view means the claw is heading at it; raise the claw only when the sensed height above the object top says it is below the flap.
- phone: a free-standing overview standing on the robot's RIGHT side (its foreground claw is the right claw); it usually sees the robot from behind or the side. Its image-left/right is NOT the robot's left/right, so never choose a pan direction from the phone. Use it only for coarse facts: is the claw above or below the table top, near or far from the object, over the table or past its edge.
- twin view: a model of the robot alone (no table, box or objects), posed from the servo readings.

Reach and geometry: each claw can reach at most about 44 cm from its shoulder when fully extended (shoulders are about 89 cm above the floor on the cart top; the robot cannot lean). Sense reports each claw's position (forward/left/up from the robot's base), which way +100 ticks on each joint moves that claw (use these signs instead of guessing), its current reach from the model, and, when the user has given the table height, how far above the table top each claw is: use that for the final descent instead of guessing height from the phone. A target further than the reach is NOT reachable by panning: extend the arm (shoulder_lift forward, elbow opening) and/or drive the base closer. Panning only swings the claw on an arc around the shoulder. Sense reports each claw's model position (forward/left/up) when available; use it to judge reach before moving. Sense also prints each claw's reach margin (how many cm of extension remain before the arm is straight: 'reach margin: 7 cm (tip is at 38 of max 45 cm from the shoulder)'); a target more than the margin further from the shoulder needs the base, not the arm. When reach refuses a target as unreachable it says how many cm beyond the arm the target is and the base pulse (robot_move_base linear_m_s 0.02, seconds and pulse count) that would bring it within reach; when a reach lands short with the joints settled ('settled N cm short: the target is at the edge of reach'), do not resend the same point: lower it, pitch the claw down more, or drive the base forward by the suggested amount. For small corrections use reach_delta (deltas from the claw's current model position) instead of recomputing absolute coordinates.

Use sense with "scene" to get lens DISTANCES and which image region things are in. Its forward/left/up numbers are trustworthy ONLY when the readout says 'camera pose: table-plane calibrated' (the depth image self-calibrates on the table top when the table is in view); when it says 'MODEL only' they come through an unvalidated head pose that on 8 October put the box top at 102 cm when it was 81 cm, and chasing them drove the claw to the centre line. In the MODEL-only case never take a height or a lateral position from the scene; heights come from the user's measurements (table top, object top) and from the sensed claw height; lateral position comes from the oak image (image-left = robot-left) and the wrist views. Use scene distances only to tell whether something is closer than ~30 cm to the head (it needs the object at least ~25 cm from the head camera). The 'nearest object' can be the robot's own claw when an arm is in front of the head camera: if its centre is within a few cm of a claw position, move that arm aside or use the grid regions instead. Gripper timing: a CLOSING gripper moves only 10 ticks per 1.5 s (contact detection), so open it only as wide as the object needs (cardboard is 3 mm; 300-400 ticks above closed is plenty) rather than fully, or a closure takes minutes. Prefer reach over hand-picked joint ticks for arm positioning: give the claw tip a point in centimetres (the sense table gives the claw's current point in the same frame, and the table height, so a flap top at 81 cm above the floor is up_cm 81). Reach refuses targets beyond the arm, inside the cart or body, or outside joint ranges, and tells you the nearest it could do. Keep pitch_deg 0 for a level claw; use about -60 to point the claw down for a top-down grasp.

Watching motion: a look can ask the eyes to 'watch <camera> for N seconds' (2 s at 4 fps is plenty); they get a timestamped burst of recent frames and can say whether something moved, which way, and whether the jaws closed on the object. To watch a move while it happens, send it with wait false, then look with a watch question, then sense motion.\n\nHow to reach and grasp (learned on this robot): 1) sense first: claw positions, reach, height above table, enabled joints, commandable ranges. 2) Get the bearing from the oak (image-left = robot-left) and a rough distance from object size; decide whether the target is within reach from the claw's current position, else extend. 3) Move several joints together toward the target in steps of at most 300 ticks each, with wait true; re-sense after each move and compare the claw position with where you expected it; if it moved the wrong way, that joint's sign is the opposite of what you assumed: flip it, do not repeat. 4) Descend using the claw height above the table (object top height is given when known), not the phone. 5) Final approach with the wrist camera: the object should be centred and growing. If the wrist frame is blown out by a light behind the object, do not fight it: approach from ABOVE with the claw pointing down (wrist_flex tilted so the jaws face the table), so the camera sees the table top and the flap edge instead of the lamp; use the phone from the side and the sensed claw height for the descent. 6) Open the gripper before the approach; once the eyes say the edge is INSIDE or just AHEAD of the jaw zone, go DOWN to the pinch height the user or the recipe gives (the object top), do not keep chasing the eyes' direction, then close. The closing result carries a PINCH VERDICT computed from the servo: 'PINCH LIKELY' (jaws stopped short of the target with sustained load) means something is between the pads even if the wrist image shows the pads closed on 'nothing' (a 3 mm flap edge-on is invisible); 'closed on air' means nothing was grasped. Trust the verdict over the image. Pinch DEPTH matters: a flap held only by the last few millimetres of the pad tips (the eyes said the edge was AHEAD by about 0-1 cm, i.e. on the top line of the zone, when you closed) gives PINCH LIKELY but slips out as soon as the arm lifts (8 October: closed at 1378 with load 164, after a 6 cm lift the gripper had crept to 1355 at load 72 and the box never moved). So before closing get the edge 1-2 cm INSIDE the zone: open, move along the jaws or down 1.5 cm more, then close. After a lift, re-read the gripper: if its ticks dropped toward fully closed and the load fell below about 100, the flap slipped out: reopen and regrasp deeper. 7) Verify a grasp by lifting: raise the HOLDING arm 8 cm straight up with the gripper still closed and SLOWLY (duration 6-8 s: a payload makes the joints lag, and a lag over 50 ticks at load 350 trips the contact hold), then look with the phone for the box moving and re-read the gripper load; only then say it is held. A lift that ends in contact_halt while the gripper is pinching is the PAYLOAD stopping the arm, not an obstacle: the box is coming up; resend the same lift with a longer duration (this is the one case where resending the same target is right), or lift in 4 cm steps. If the eyes report the pads empty while the servo verdict says PINCH LIKELY, believe the servo and the lift. Only the arm that holds the object moves to lift it; never move the other arm to 'help'. 8) Step sizes: 'a bit' is 3-5 cm, a correction toward a seen target 2-3 cm, a repositioning 8-10 cm; moves under 2 cm waste rounds. 9) Avoid base moves unless the target is clearly out of reach from the arms; if a base move ends with wheels rolling after release, stop using the base.

Jaw orientation (learned 8 October): the pads open SIDEWAYS in the wrist view (fixed pad lower-left, moving pad lower-right), so at the starting wrist_roll they can only straddle an edge that runs along the jaws, i.e. a line running bottom-to-top in the wrist image: the top edge of a SIDE flap (the robot-left or robot-right flap, whose edge runs forward-back). The top edge of the NEAR or FAR flap runs left-right and appears as a horizontal line across the wrist view; with unrolled jaws it can never come INSIDE the zone (the eyes keep answering 'AHEAD 3-5 cm' wherever the claw is, inside the box or in front of the flap). To pinch the near or far flap, first roll the wrist 90 degrees: robot_move_path with one waypoint {"<arm>_arm_wrist_roll": current + 1024} and duration_s 3 (it splits the leg itself; the owner rolled the left wrist 2119 -> 3143 and back on 8 October). The wrist camera rolls with the jaws, so the near flap's edge then appears as a bottom-to-top line and the eyes' INSIDE/BESIDE words work again, but image-left/right in the rolled view is robot forward/back: find which by one 2 cm forward reach_delta and a look. Pitch the claw steeply (-70 to -80) above that flap so the pads open roughly forward/back across it, descend so the edge sits 1-2 cm between the pads, close. Roll back (current - 1024) only after the task, never while holding.

Folding a flap (after a verified pinch on it): a box flap turns about its HINGE, the crease along the box's top edge on that side, so the pinched point must move on a circle around that line, from standing vertical toward the box's centre; never pull it straight up (that lifts the box) or straight forward (that drags it). Use the fold action: radius_cm = pinch height minus the box top (e.g. pinch at 89 cm on a box whose top is 81 cm: 8), toward the box centre, degrees 90 in 3 steps. A fresh crease springs back (9 October, robot-right flap: carried to flat and released it returned to 5-15 deg from vertical; holds of 3-6 s at flat did not set it): on an open box carry it PAST flat (degrees 110, 6 steps, the free edge dipping into the opening) with hold_s 5 before you open the gripper. A fold that stops on a contact halt is the crease resisting: fold the rest again with more steps (smaller steps), never a different direction. If it stops on reach near the end, 60 degrees is enough. Then open the gripper (to about 2000), raise the claw 5 cm, move it out of the oak's view, and look with the oak: is the flap lying folded over (or inside) the box opening, or did it spring back up? Owner rule (9 October): never push a flap with the claw body, the wrist camera or the arm; a fold counts only when the flap is carried in the closed jaws, and any pressing is done by the pads that hold it. A flap that springs back is pinched and folded again with a longer hold.

Joint meaning (LeRobot convention, matches the model): 0 deg is the middle of each joint's calibrated range (ticks = mid + deg x 4095/360). At shoulder_lift 0 / elbow_flex 0 / wrist_flex 0 the upper arm stands vertical, the forearm points straight forward and the gripper is level; +shoulder_lift tilts the upper arm forward and down, +elbow_flex drops the forearm, +wrist_flex pitches the gripper down; wrist_flex = -shoulder_lift - elbow_flex keeps the gripper level. Gripper: lower ticks = more closed (range minimum is fully closed), higher = open.

Verified joint directions on this robot (2026-10-08): LEFT arm shoulder_pan: LOWER ticks swing the claw OUTWARD to the robot's left; HIGHER ticks swing it INWARD toward the head mast (pan above about 2600 with the arm extended puts the claw into the mast). The sense table's '+100 ticks' lines give the other joints; trust them over any guess.

Stall rule: 'following error exceeds 96 ticks' means a joint could not physically follow its command (it is pushing against something) and the controller released the arm. If the same joint stalls twice, or stalls in both directions, the arm is physically caught (cables, the mast, the table): do NOT push again in any direction; finish and ask the user to free the arm by hand (its motors are off after a fault).

Progress rule: if a look gives the same answer as before the last move, that move type is not working: do not repeat it. Change strategy (extend instead of pan, move other joints together, sense the claw position, drive the base) or finish as blocked with the exact reason. Never make more than 3 consecutive moves of the same single joint.

Robot facts: the controller enforces every safety limit (ranges, speed, load, contact, STOP), so be decisive; no extra 'go' confirmations unless the user asks. There is no arm-geometry or camera-to-arm calibration tool: plan in joint space and verify with look actions. Base: robot_move_base gives one short pulse of at most 3 s. Signs: linear_m_s positive drives FORWARD (the direction the OAK faces), negative backs away; angular_rad_s positive turns LEFT (counter-clockwise), negative turns right. Confirm the first pulse with the OAK (driving forward makes an object ahead grow). A pulse with both values zero is refused. Each wheel is capped at 0.02 m/s, where wheel speed = |linear_m_s| + |angular_rad_s| x 0.225 (wheels 0.45 m apart). So: drive straight at <= 0.02 m/s (about 6 cm per 3 s pulse), OR turn in place at <= 0.088 rad/s (about 15 deg per 3 s pulse); a combined pulse must keep the sum under 0.02 (e.g. 0.01 m/s with 0.044 rad/s). The distance a pulse reports comes from the wheel encoders (5-inch wheels); it is not measured on the floor. If a pulse ends with 'Wheels rolling after release', the pulse itself still ran (the check comes after braking), so the cart DID move about the commanded distance and every motor was released; something is pulling the cart (usually a taut cable or a slope): do not retry base moves, re-locate the box before reaching; work with the arms from where the base is and tell the user to give the cables slack. Enabling holds joints where they are. In this pickup profile an arm moves only with all six of its joints enabled (robot_set_motor_enable with all six names). Targets must lie inside the commandable ranges (sense reports them). A move whose predicted claw position is inside the robot's own head mast or cart is not sent (self-collision check); keep claws clear of the robot's centre line and body. Never repeat an identical refused command. The head moves only through aim (or robot_move_head in a move: both head motors enabled, at most 200 ticks per joint per call, duration_s at least 1 s per 100 ticks). The human can press STOP at any time. A refusal that says 'Owner stopped: Coherent servo read communication failure' or 'following error' means the controller released EVERY motor as a precaution; it is not the end of the task: sense, re-enable the six joints of the arm you need (and re-open the gripper if it closed), and carry on from where the arm now is. Finish as blocked only when the same hardware fault repeats three times in a row or a refusal rules out every camera-guided approach, and say exactly which."""

GRIPPER_EMPTY_CLOSED={'left':1357,'right':1352}

PINCH_MAX_ABOVE=80

PINCH_ABOVE_EMPTY=12

PINCH_LOAD=60

AIR_ABOVE_EMPTY=5

PINCH_SHORT_TICKS=20

PINCH_AIR_TICKS=15

PINCH_AIR_LOAD=30

DEFAULT_PITCH_DEG=-45.0

FOLD_TOWARD={'forward':('forward_cm',1),'back':('forward_cm',-1),'left':('left_cm',1),'right':('left_cm',-1)}

def parse_fold(d):
    """Validate a fold decision in place: arm, toward (the box centre seen from the flap), radius 4-15 cm, 15-120
    degrees (default 90), 1-6 steps (default 3)."""
    number=lambda v:isinstance(v,(int,float)) and not isinstance(v,bool)
    if d.get('arm') not in ('left','right'):raise ValueError('fold needs arm left or right')
    t=d.get('toward');t=t.lower().strip() if isinstance(t,str) else t
    if t not in FOLD_TOWARD:raise ValueError('fold toward must be forward, back, left or right (toward the box centre)')
    hinge=d.get('hinge_cm')
    if hinge is not None:
        if not isinstance(hinge,list) or len(hinge)!=2 or not all(number(v) for v in hinge):raise ValueError('fold hinge_cm must be [position along the toward axis (left_cm for toward left/right, forward_cm for forward/back), up_cm] of the hinge line, model cm')
        hinge=[float(v) for v in hinge]
    to=d.get('to_deg')
    if to is not None and (hinge is None or not number(to) or not 15<=to<=130):raise ValueError('fold to_deg (the absolute end angle from vertical, 15..130) needs hinge_cm')
    r=d.get('radius_cm')
    if hinge is not None and r is None:r=10.0  # unused with a hinge: the radius comes from the claw's distance to it
    if not number(r) or not 4<=r<=15:raise ValueError('fold radius_cm must be 4..15 (pinch height minus the hinge height)')
    deg=d.get('degrees');deg=90 if deg is None else deg
    if not number(deg) or not 15<=deg<=120:raise ValueError('fold degrees must be 15..120')
    n=d.get('steps');n=3 if n is None else n
    if not number(n) or int(n)!=n or not 1<=n<=6:raise ValueError('fold steps must be an integer 1..6')
    h=d.get('hold_s');h=0 if h is None else h
    if not number(h) or not 0<=h<=30:raise ValueError('fold hold_s must be 0..30 seconds')
    pe=d.get('pitch_end_deg')
    if pe is not None and (not number(pe) or not -90<=pe<=30):raise ValueError('fold pitch_end_deg must be -90..30 (or null to keep the pitch)')
    path=d.get('path');path=False if path is None else path
    if not isinstance(path,bool):raise ValueError('fold path must be true or false')
    d.update(toward=t,radius_cm=float(r),degrees=float(deg),steps=int(n),hold_s=float(h),pitch_end_deg=None if pe is None else float(pe),path=path,
             hinge_cm=hinge,to_deg=None if hinge is None else float(110 if to is None else to));return d

def fold_arc(d,claw):
    """[(from_deg, to_deg, target {forward_m, left_m, up_m})] absolute claw targets for a fold. With hinge_cm the arc is
    about that line: radius and start angle come from the claw's position relative to it (so a flap that already leans
    is carried from where it is) and it ends at to_deg from vertical. Without, the claw moves by fold_steps deltas, the
    arc assuming the flap starts vertical with the hinge radius_cm straight below the pinch."""
    axis,sign=FOLD_TOWARD[d['toward']];key='left_m' if axis=='left_cm' else 'forward_m'
    out=[]
    if d.get('hinge_cm'):
        h,hu=d['hinge_cm'][0]/100,d['hinge_cm'][1]/100
        va,vu=sign*(claw[key]-h),claw['up_m']-hu
        r=math.hypot(va,vu);a0=math.degrees(math.atan2(va,vu));end=d['to_deg'];n=d['steps']
        prev=a0
        for k in range(1,n+1):
            a=a0+(end-a0)*k/n;t=dict(claw)
            t[key]=h+sign*r*math.sin(math.radians(a));t['up_m']=hu+r*math.cos(math.radians(a))
            out.append((prev,a,{k2:t[k2] for k2 in ('forward_m','left_m','up_m')}));prev=a
        return out
    t={k2:claw[k2] for k2 in ('forward_m','left_m','up_m')}
    for a,b,toward,up in fold_steps(d['radius_cm'],d['degrees'],d['steps']):
        t=dict(t);t[key]+=sign*toward/100;t['up_m']+=up/100;out.append((a,b,t))
    return out

def fold_words(d,arm,claw=None):
    """The first result line of a fold: what arc it runs."""
    if d.get('hinge_cm'):
        axis,sign=FOLD_TOWARD[d['toward']];key='left_m' if axis=='left_cm' else 'forward_m'
        where=''
        if claw:
            va,vu=sign*(claw[key]*100-d['hinge_cm'][0]),claw['up_m']*100-d['hinge_cm'][1]
            where=f", radius {math.hypot(va,vu):.1f} cm from the claw, from {math.degrees(math.atan2(va,vu)):.0f} deg"
        return f"fold {arm}: about the hinge at {axis[:-3]} {d['hinge_cm'][0]:g} cm, up {d['hinge_cm'][1]:g} cm{where} to {d['to_deg']:g} deg toward {d['toward']} in {d['steps']} steps"
    return f"fold {arm}: hinge {d['radius_cm']:g} cm below the pinch, turning {d['degrees']:g} deg toward {d['toward']} in {d['steps']} steps"

def fold_steps(radius_cm,degrees,steps):
    """[(from_deg, to_deg, toward_cm, up_cm)] for an arc that starts with the flap vertical (the pinch straight above
    the hinge) and turns it by degrees toward the box centre."""
    out=[]
    for k in range(1,steps+1):
        a=math.radians(degrees*(k-1)/steps);b=math.radians(degrees*k/steps)
        out.append((math.degrees(a),math.degrees(b),radius_cm*(math.sin(b)-math.sin(a)),radius_cm*(math.cos(b)-math.cos(a))))
    return out

class Chat:
    def pinch_contradiction(self,report):
        """When the eyes say the pads are empty but the servo says a pinch is likely, say so after the report: an edge-on
        flap held between closed pads is invisible to the wrist camera, and the load reading is not."""
        if not self.last_closure or 'PINCH LIKELY' not in self.last_closure:return ''
        low=report.lower()
        if not any(w in low for w in ('no edge','nothing between','empty','no cardboard','not holding','pressed together','closed on nothing')):return ''
        g=None
        try:
            arm=self.last_closure.split(' ',1)[0];name=f'{arm}_arm_gripper'
            row=next((m for m in (self.last_state or {}).get('motors') or [] if m.get('name')==name),{});g=(row.get('Present_Position'),row.get('Present_Load'),row.get('Torque_Enable'))
        except Exception:pass
        return f'\nSERVO NOTE (code, not the eyes): the last closure was {self.last_closure}'+(f'; gripper now {g[0]} ticks, load {g[1]}, torque {g[2]}' if g and g[0] is not None else '')+'. The wrist camera cannot see a thin flap held edge-on between closed pads; the load says something is there. Do not conclude the gripper is empty from the image alone: verify by lifting.'
    def pinch_verdict(self,args,detail):
        """After a CLOSING gripper move: 'pinch likely' when the jaws stopped short of the target with sustained load,
        'closed on air' when they reached the target with no load. Reads the gripper load fresh; code, not the eyes
        (a 3 mm flap held edge-on inside closed pads is invisible to the wrist camera)."""
        arm=args.get('arm');target=args.get('position_ticks');name=f'{arm}_arm_gripper'
        readback=(detail.get('readbacks') or {}).get(name) if isinstance(detail.get('readbacks'),dict) else None
        if type(target) is not int or type(readback) is not int:return None
        before=(state_ticks(self.last_state)[0] if self.last_state else {}).get(name)
        if before is not None and target>=before:return None  # an opening move
        load=None
        try:
            r=self.robot.call('robot_get_state',{'fresh':True});st=r.get('result',r) if isinstance(r,dict) else {}
            for m in st.get('motors') or []:
                if m.get('name')==name and type(m.get('Present_Load')) is int:load=abs(m['Present_Load'])
            if isinstance(st,dict) and st.get('motors'):self.last_state=st
        except Exception:pass
        short=readback-target;empty=GRIPPER_EMPTY_CLOSED.get(arm)
        if empty is not None and target<=empty+AIR_ABOVE_EMPTY:
            above=readback-empty;how=f'the jaws stopped at {readback}, {above} ticks above where the empty pads meet ({empty})'
            if above>PINCH_MAX_ABOVE:return f'closure stalled mid-travel: {how}, far more than a 3 mm flap gives (20-25 ticks), load {load if load is not None else "unknown"}; this is NOT a pinch: send the same close again (closing again is safe) and read the new verdict'
            if above>=PINCH_ABOVE_EMPTY and load is not None and load>=PINCH_LOAD:return f'PINCH LIKELY: {how}, with a sustained load of {load} (something is between the pads; a thin flap is invisible to the wrist camera, trust this over the image)'
            if above<=AIR_ABOVE_EMPTY:return f'closed on air: {how} (load {load if load is not None else "unknown"}); nothing is between the pads'
            return f'closure uncertain: {how}, load {load if load is not None else "unknown"}; lift 8 cm and watch the phone to find out'
        if short>=PINCH_SHORT_TICKS and load is not None and load>=PINCH_LOAD:return f'PINCH LIKELY: the jaws stopped {short} ticks before the target with a sustained load of {load} (something is between the pads; a thin flap is invisible to the wrist camera, trust this over the image)'
        if short<=PINCH_AIR_TICKS and (load is None or load<PINCH_AIR_LOAD):return f'closed on air: the jaws reached the target (load {load if load is not None else "unknown"}); nothing is between the pads'
        return f'closure uncertain: stopped {short} ticks before the target, load {load if load is not None else "unknown"}; lift 8 cm and watch the phone to find out'
    def execute_fold(self,d,send,known,refused):
        """fold: the pinched point on a circular arc about the flap's hinge, as equal reach_delta steps (pitch kept,
        gripper untouched). Stops at the first step that is not sent, refused or ends in a contact halt."""
        arm=d['arm'];axis,sign=FOLD_TOWARD[d['toward']]
        self.record('fold',arm=arm);self.record_event('fold',arm=arm,toward=d['toward'],radius_cm=d['radius_cm'],degrees=d['degrees'],steps=d['steps'])
        if d.get('path'):
            lines=self.execute_fold_path(d,send,known,refused)
            self.record_event('fold_end',arm=arm,result=(lines or ['cancelled'])[-1][:240])
            return lines
        hold=d.get('hold_s') or 0;pitch0=self.last_pitch.get(arm,DEFAULT_PITCH_DEG);pitch_end=d.get('pitch_end_deg')
        claw=None
        if d.get('hinge_cm'):
            try:st,pos,ranges,jm=self.arm_state(send);c=model_claws(pos,ranges).get(f'{arm}_arm') or {}
            except Exception as error:return [fold_words(d,arm),f'could not read the claw position ({type(error).__name__}: {str(error)[:150]}); nothing sent']
            if not c:return [fold_words(d,arm),f'the model has no current position for the {arm} claw; nothing sent']
            claw={k2:c[k2] for k2 in ('forward_m','left_m','up_m')}
            plan=[(a,b,{'action':'reach','arm':arm,'forward_cm':t['forward_m']*100,'left_cm':t['left_m']*100,'up_cm':t['up_m']*100,'pitch_deg':None},f"{point_words(t)}")
                  for a,b,t in fold_arc(d,claw)]
        else:
            plan=[]
            for a,b,toward,up in fold_steps(d['radius_cm'],d['degrees'],d['steps']):
                step={'action':'reach_delta','arm':arm,'forward_cm':0.0,'left_cm':0.0,'up_cm':up,'along_jaws_cm':0.0,'pitch_deg':None};step[axis]=sign*toward
                plan.append((a,b,step,f'{toward:+.1f} cm toward, {up:+.1f} cm up'))
        lines=[fold_words(d,arm,claw)+(f", pitch {pitch0:.0f} -> {pitch_end:.0f}" if pitch_end is not None else '')+(f", then hold {hold:g} s" if hold else '')]
        for k,(a,b,step,words) in enumerate(plan,1):
            pitch=None if pitch_end is None else pitch0+(pitch_end-pitch0)*k/d['steps']
            if pitch is None and step['action']=='reach':pitch=self.last_pitch.get(arm,DEFAULT_PITCH_DEG)
            step=dict(step,pitch_deg=pitch)
            self.last_move_detail={}
            out=self.execute_reach(step,send,known,refused)
            if out is None:return None
            if not any(l.startswith('robot_move_joint_targets: ok') for l in out) and any('NOT SENT' in l for l in out):
                # unreachable at this pitch (the wrist limit near the box): retry the same point once, 10 deg shallower
                eased=min(30.0,(self.last_pitch.get(arm,DEFAULT_PITCH_DEG) if pitch is None else pitch)+10.0)
                out=out+[f'retry at pitch {eased:.0f}:']+(self.execute_reach(dict(step,pitch_deg=eased),send,known,refused) or ['(cancelled)'])
                if self.cancel.is_set():return None
            lines.append(f'step {k} ({a:.0f}->{b:.0f} deg, {words}): '+' / '.join(out))
            detail=self.last_move_detail if isinstance(self.last_move_detail,dict) else {}
            if not any(l.startswith('robot_move_joint_targets: ok') for l in out):
                lines.append(f'fold stopped at step {k}: that step was not sent or was refused; the flap is turned about {a:.0f} deg');break
            if detail.get('closure_outcome')=='contact_halt' or detail.get('halted') or detail.get('contact'):
                lines.append(f'fold stopped at step {k}: contact halt (the crease or the box resists); the flap is turned between {a:.0f} and {b:.0f} deg; the arm is holding');break
        else:
            if hold:
                send(self.emit('connection',text=f'fold: holding the flap {hold:g} s with the gripper closed'))
                if self.drive_wait(hold):return None
                lines.append(f"held {hold:g} s at the end of the arc with the gripper closed")
            lines.append(f"fold complete: {(d['to_deg'] if d.get('hinge_cm') else d['degrees']):g} deg; the gripper is still closed: open it, raise 5 cm and look whether the flap stays down")
        self.record_event('fold_end',arm=arm,result=lines[-1][:240])
        return lines
    def execute_fold_path(self,d,send,known,refused):
        """fold with path true: the same arc, solved point by point up front and sent as ONE robot_move_path (the owner
        runs it as a continuous motion, no stop between steps), then the hold. A point unreachable at its pitch is retried
        10 deg shallower; the path ends before a point that still is not."""
        arm=d['arm'];axis,sign=FOLD_TOWARD[d['toward']];hold=d.get('hold_s') or 0
        pitch0=self.last_pitch.get(arm,DEFAULT_PITCH_DEG);pitch_end=d.get('pitch_end_deg')
        head=None
        try:st,pos,ranges,jm=self.arm_state(send)
        except Exception as error:return [fold_words(d,arm)+' (one path)',f'could not read the arm state ({type(error).__name__}: {str(error)[:200]}); nothing sent']
        try:c=model_claws(pos,ranges).get(f'{arm}_arm') or {}
        except Exception as error:c={}
        if not c:return [fold_words(d,arm)+' (one path)',f'the model has no current position for the {arm} claw; nothing sent']
        claw={k2:c[k2] for k2 in ('forward_m','left_m','up_m')}
        head=(fold_words(d,arm,claw)+' (one path)'+(f", pitch {pitch0:.0f} -> {pitch_end:.0f}" if pitch_end is not None else '')+(f", then hold {hold:g} s" if hold else ''))
        ticks=dict(pos);waypoints=[];lines=[head];reached=0
        for k,(a,b,target) in enumerate(fold_arc(d,claw),1):
            pitch=pitch0 if pitch_end is None else pitch0+(pitch_end-pitch0)*k/d['steps']
            sol=None
            for p in (pitch,min(30.0,pitch+10.0)):
                try:sol=solve_reach_fn(arm,target,ticks,ranges,p,jm)
                except Exception:sol=None
                if sol and sol.get('ok'):pitch=p;break
            if not sol or not sol.get('ok'):
                lines.append(f"point {k} ({b:.0f} deg) {point_words(target)} unreachable ({(sol or {}).get('reason','no solution')[:120]}): the path ends at {a:.0f} deg");break
            new={n:int(v) for n,v in sol['ticks'].items()}
            if waypoints and set(new)!=set(waypoints[0]):new={n:new.get(n,ticks.get(n)) for n in waypoints[0]}
            if any(abs(new[n]-ticks.get(n,new[n]))>330 for n in new):
                lines.append(f'point {k} ({b:.0f} deg) needs a joint step over 330 ticks: the path ends at {a:.0f} deg');break
            waypoints.append(new);ticks.update(new);reached=b
            lines.append(f"point {k} ({a:.0f}->{b:.0f} deg): {point_words(target)} pitch {pitch:.0f}")
        if not waypoints:return lines+['fold not sent: no reachable arc point']
        six=[f'{arm}_arm_{j}' for j in ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper')]
        calls=[]
        if not set(six)<=set(st.get('enabled_motors') or []) and 'robot_set_motor_enable' in known:calls.append({'tool':'robot_set_motor_enable','args':{'names':six,'enabled':True}})
        calls.append({'tool':'robot_move_path','args':{'arm':arm,'waypoints':waypoints,'duration_s':float(min(60,max(4,2.5*len(waypoints)))),'wait':True}})
        self.last_move_detail={}
        out=self.execute_moves(calls,send,known,refused)
        if out is None:return None
        lines+=out
        detail=self.last_move_detail if isinstance(self.last_move_detail,dict) else {}
        if not any(l.startswith('robot_move_path: ok') for l in out):
            return lines+['fold stopped: the path was not sent or was refused; the flap has not turned']
        if self.last_pitch is not None:self.last_pitch[arm]=float(pitch)
        try:
            r=self.call_tool('robot_get_state',{'fresh':False},send);st2=r.get('result',r);pos2,rg2=state_ticks(st2)
            c2=model_claws(pos2,rg2).get(f'{arm}_arm') or {};lines.append(f'actual tip (model from readbacks): {point_words(c2)}')
        except Exception as error:lines.append('actual tip unavailable: '+str(error)[:100])
        if detail.get('closure_outcome')=='contact_halt' or detail.get('halted') or detail.get('contact'):
            return lines+[f'fold stopped: contact halt during the path (the crease or the box resists); the arm is holding']
        if hold:
            send(self.emit('connection',text=f'fold: holding the flap {hold:g} s with the gripper closed'))
            if self.drive_wait(hold):return None
            lines.append(f'held {hold:g} s at the end of the arc with the gripper closed')
        return lines+[f"fold complete: {reached:g} deg; the gripper is still closed: open it, raise 5 cm and look whether the flap stays down"]
