# Prompt matrix, right short flap, simulator only (2026-10-10 02:50-03:50 JST)

Goal: improve `software/docs/prompts/carton-pilot-v8b-tags-v2.txt` (the v8b real-robot recipe plus the tags addendum)
after the 02:39 JST live attempt, where the pinch scan (-15, -12 on air; -18 at 1363 "stopped on something") started
the fold arc with the flap edge about 1 cm beside the pads and the arc pushed the flap. Prompts only; no simulator,
safety or bench code was changed. No real robot, no live chat, no tag scripts were touched.

## Set-up

- Bench: `pilot/bench/bench.py` (chat Mac, not in git) driving the live pilot chat code against
  `farm.sim.sim_robot.SimRobot` wrapped in `CartonTagRobot(TwinRobot(...))`, so `{"action":"sense","what":["tags"]}` is
  live. Software = this worktree (`XLEROBOT_SOFTWARE=.../seville-v2/software`), `--task fold --max-rounds 40
  --max-minutes 15 --no-real-time`, preset `real` (HACHIYO carton, 81 cm rim, 14 cm right flap, sticky right gripper,
  arm model 3 cm high). Supervisor DeepSeek Flash; eyes Cerebras Qwen; Opus 5.5 on the final candidate only.
- Two lean conditions, 8 fresh seeds each (20-27; the final candidate also 28-35 and 36-43):
  `default` = the preset's draw (-12..+15 deg, i.e. mostly upright or leaning IN), and `out` =
  `--fidelity '{"lean_jitter_deg": [-20, -12]}'` (12-20 deg OUTWARD, tonight's live case; negative = outward).
- Success = `score()['fold_by_pinch']` (folded, staying folded, at most 10 deg of the turn made by a non-pad contact).
  "pushed" = folded or not with more than 10 deg of push. Decisions = supervisor rounds.
- Run line: `.context/prompt-matrix-20261010/run.sh <variant> <default|out> [config] [first_seed] [n] [suffix]`;
  prompts in `prompts/`, every run (events.jsonl, chat-events.jsonl, progress.log, run.json, final JPEGs) under
  `runs/<variant>-<lean>-<config>[suffix]/`, summary via `summarize.py [--detail]`.

Two limits of the simulator that matter for reading the numbers:

1. The sim scene carries NO AprilTag textures: `sense tags` works but returns NO_VALID_TAGS on every camera. The tag
   lines were therefore tested for their cost (rounds, confusion, a stop on a missing tag), not for their benefit.
   Nothing in the sim can confirm "tag 11 gone = folded".
2. Sim pinches read 1367-1390 (real: 1358-1364), so the "check a close in 1358-1366 with a wrist look" rule fired in
   only about a quarter of the sim pinches. Where it fired (and in Flash's own extra looks) the closed-jaw wrist look
   was consistent: the sim eyes report a GENUINE pinch as "edge BESIDE the zone (image-left by 0.5-1 cm)" every time
   (the closed-jaw zone is a sliver narrower than the pads), an air close at a tip left of the edge as "beside
   image-left 1-2 / 2-3 cm". The live 02:39 look ("edge about 1 cm beside the pads") is the same wording.
3. Minutes are inflated by concurrency (up to 40 sims and DeepSeek streams at once, load average 59): compare
   decisions across variants, not minutes. Baseline-alone runs took about 1.0 min / 12.5 decisions.

## Variants

| variant | what changed vs v2 |
|---|---|
| V0 | v2 as is (baseline) |
| V1 | scan tips outward first: -18.5, -20.5, -15, -12; second pass descending to 93 |
| V2 | V1 + after any close in 1358-1366, one closed-jaw wrist look; fold only on "between the pads" |
| V3 | V2 + tags sensed after the fold (ID 11 gone from the phone agrees with folded; missing tag alone never proof) |
| V4 | compact rewrite (1745 words vs 1931): sectioned, image-left/right mapping, "beside image-right/left by N -> retry at T-3 / T+3", abort the fold after two contact halts before 60 deg |
| V5e / V5c | V2 lineage / V4 lineage with the SAFE-FIRST order -15, -18.5, -20.5, -12 and the sliver rule (beside <= 1 cm = pinch) |
| V6 | V3 + sliver rule + "1367 or more: fold at once, no look" + "a 1500-1600 close is a stall, never a pinch, whatever the eyes say" + fold abort after two contact halts before 60 deg + the owner's lean note selects the tip order |
| V7 (historical final candidate) | V6 + "if no tag is accepted at the start, say so at the end and carry on with the scan" |

## Results (fold_by_pinch out of 8 unless stated; sorted by rate)

| variant | lean | supervisor | runs | fold_by_pinch | folded (any) | pushed >10 deg | overclaims | stopped early | median decisions | median min | looks |
|---|---|---|---|---|---|---|---|---|---|---|---|
| V1-outward-order | out | deepseek-flash | 8 | 8 | 8 | 0 | 0 | 0 | 11.5 | 0.96 | 0 |
| V2-wrist-check | out | deepseek-flash | 8 | 8 | 8 | 0 | 0 | 0 | 12.0 | 1.06 | 2 |
| V3-tags-after | out | deepseek-flash | 8 | 8 | 8 | 0 | 0 | 0 | 12.0 | 2.19 | 2 |
| V6-final-candidate-s28 | out | deepseek-flash | 8 | 8 | 8 | 0 | 0 | 0 | 13.0 | 1.81 | 6 |
| V7-final-s36 | out | deepseek-flash | 8 | 8 | 8 | 0 | 0 | 0 | 13.5 | 1.88 | 4 |
| V6-final-candidate | out | opus55 | 4 | 4 | 4 | 0 | 0 | 0 | 14.5 | 3.16 | 2 |
| V5e-safe-first-order | default | deepseek-flash | 8 | 7 | 8 | 1 | 1 | 0 | 12.0 | 3.23 | 0 |
| V0-v2-baseline | default | deepseek-flash | 8 | 7 | 8 | 1 | 1 | 0 | 12.5 | 1.02 | 0 |
| V5c-safe-first-compact | default | deepseek-flash | 8 | 7 | 8 | 1 | 1 | 0 | 13.0 | 2.88 | 5 |
| V7-final | out | deepseek-flash | 8 | 7 | 7 | 1 | 0 | 1 | 13.0 | 1.90 | 6 |
| V4-compact-rewrite | out | deepseek-flash | 8 | 7 | 7 | 1 | 0 | 1 | 13.5 | 1.80 | 5 |
| V6-final-candidate | out | deepseek-flash | 8 | 6 | 7 | 1 | 1 | 0 | 12.5 | 2.19 | 4 |
| V0-v2-baseline | out | deepseek-flash | 8 | 6 | 6 | 2 | 0 | 2 | 12.5 | 1.02 | 1 |
| V5c-safe-first-compact | out | deepseek-flash | 8 | 6 | 7 | 2 | 1 | 1 | 15.5 | 3.98 | 4 |
| V4-compact-rewrite | default | deepseek-flash | 8 | 6 | 6 | 2 | 0 | 2 | 26.0 | 3.40 | 5 |
| V5e-safe-first-order | out | deepseek-flash | 8 | 5 | 6 | 2 | 0 | 3 | 15.0 | 3.82 | 8 |
| V2-wrist-check | default | deepseek-flash | 8 | 5 | 5 | 3 | 0 | 1 | 22.5 | 2.27 | 3 |
| V1-outward-order | default | deepseek-flash | 8 | 4 | 7 | 3 | 2 | 1 | 21.0 | 2.00 | 2 |
| V3-tags-after | default | deepseek-flash | 8 | 4 | 7 | 4 | 3 | 1 | 22.5 | 3.06 | 1 |
| V6-final-candidate | default | deepseek-flash | 8 | 3 | 5 | 4 | 1 | 1 | 20.0 | 2.63 | 1 |

Reading: with the flap leaning OUT 12-20 deg (tonight), every outward-first prompt (V1, V2, V3, V6, V7) folds by
pinch 8/8 or near it, the baseline 6/8 (seeds 21 and 26, leans -17.7 and -19.6, failed every time: -15 and -12 were
tried first, each descent nudged the flap further out, and -18 then closed on air at 1344-1354). With the preset's
default draw (upright or leaning IN), the same outward-first order costs 2-4 runs (descents at -18.5/-20.5 beside an
inward-leaning flap push it 13-30 deg, then -15/-12 catch it late or not at all), while -15-first prompts hold the
baseline's 7/8. The historical candidate selects scan order from the owner's lean note. It is retained for reproduction; the current push-authorized policy is separate.


## Failure modes seen

1. Scan order against the lean (the dominant one). Outward flap + -15/-12 first: the descents beside the flap push it
   further out, the later -18 tip closes on air (V0 out seeds 21, 26; V5e/V5c out, same seeds). Inward/upright flap +
   -18.5/-20.5 first: the descents push it 13-30 deg (V1/V2/V3/V4/V6 default seeds 20, 21, 24, 25, 27).
2. Closed-jaw wrist look read too strictly. "Fold only on BETWEEN" (V2, V4) turns real pinches (1359-1365) into
   misses because the sim eyes say "beside image-left by 0.5-1 cm" for a held edge; the run recovers on the next tip
   at a cost of ~6 decisions (V2 out seeds 21, 24) or, with V4's "beside image-left -> retry at T+3", moves the tip
   the wrong way and pushes the flap 46 deg (V4 out seed 21). The sliver rule (<= 1 cm beside = pinch) fixed it:
   V5e/V6/V7 folded on 1360-1363 closes after a "beside 0.5-1 cm" answer.
3. Folding on a stalled close. V5c out seed 21: the close stuck at 1568, the eyes said "INSIDE the jaw zone" (open
   pads around the flap), Flash folded and pushed the flap 80 deg. V6/V7 say a 1500-1600 close is a stall whatever the
   eyes show.
4. Scan descent onto the flap edge: one seed per batch (default seed 25, lean +13) lands the descent on the edge and
   moves the flap 17-18 deg before the pinch; counted as pushed, flap still ends folded. Present in every variant,
   including the baseline (it is the 1/8 that v8b lost on 9 October too).
5. Infrastructure: one DeepSeek `IncompleteRead` ended V6 out seed 21 after one decision (not a prompt effect);
   "supervisor ran out of reply tokens" notes appeared under the heaviest load. Flash writes -18.5 as -18 in its
   decision summary but sends the number as given.
6. Not seen: no stop on NO_VALID_TAGS, no extra tag confusion (tags cost 1-2 decisions per run), no wrong-arm or base
   moves, no resent stalled opens.

## Interpretation after the final runs and owner policy update

V7 outward-lean confirmation finished: seeds 20-27 scored **7/8**, seeds 36-43 **8/8**, combined **15/16** on the original fold_by_pinch metric. Seed 27 reached the 40-round limit after 10.56 minutes without folding; it pushed the flap 19.9 degrees and accumulated 140.9 seconds of illegal contact under the old scorer. API/reply-token trouble occurred, but this is still a failed run, not an excluded denominator.

The simulator renders no AprilTags: sense tags returns NO_VALID_TAGS. This matrix evaluates the scan recipe and missing-tag fallback, not tag-derived approach coordinates or a tags-on/off comparison. Even successful V7 runs report contact outside the old allowed pad pair. The simulator's score does not establish physical clearance or camera safety.

V7's rule that an edge beside the closed pads by <=1 cm establishes pinch is a simulator workaround. Similar wording occurred in a failed physical attempt; do not promote it as physical pinch evidence. Numeric gripper ticks alone also do not prove a flap is held. Keep the exact V7 artifact for reproduction, not as the current live policy.

On 10 October the owner explicitly authorized deliberate pushing with the claw and reported grip material beneath the carton. This supersedes the earlier pinch-only task rule. Historical fold_by_pinch scores above remain unchanged. A future push-fold comparison should measure final flap angle after withdrawal, carton translation/tipping, camera clearance, and owner contact stops. No push-fold trial was run as part of this report.
