# Training a pour/water policy for the farm XLeRobot: what exists, what transfers

Date: 2026-09-27. Every dataset/model row below comes from a live Hugging Face API response
(`/api/datasets/<id>`, `/api/datasets/<id>?expand[]=usedStorage`, `resolve/main/meta/info.json`,
`meta/tasks.jsonl` or `meta/tasks.parquet` via pandas/pyarrow 25.0.1 in the project venv, and
`resolve/main/config.json` + `train_config.json` + `README.md` for models). "not fetched" means the
request failed or the file does not exist. Sizes are the Hub's `usedStorage` (whole repo).

## 0. Our embodiment (from the repo, not the Hub)

| Item | Ours (farm, lerobot 0.6.1) | Where |
|---|---|---|
| Robot class / `robot_type` | vendored `XLerobot2Wheels`, registered `--robot.type=xlerobot_2wheels`; recorder default `robot_type="xlerobot_2wheels"` | `farm/vendor/config_xlerobot_2wheels.py:51`, `farm/learning/recorder.py:54` |
| Joints we record (14) | `left_arm_{shoulder_pan,shoulder_lift,elbow_flex,wrist_flex,wrist_roll,gripper}`, `right_arm_{...}`, `head_motor_1`, `head_motor_2` — **no `.pos` suffix** in `farm` names | `farm/adapters/base.py:15-19`, `farm/system.py:71` |
| Vendored robot's own state features | 16: the 14 above **with `.pos`** + `x.vel`, `theta.vel` (2-wheel) | `farm/vendor/xlerobot_2wheels.py:128-150` |
| Normalisation | body joints -100..100, gripper 0..100 (LeRobot `MotorNormMode`) | `farm/vendor/xlerobot_2wheels.py:87,116` |
| Cameras | `head`, `left_wrist`, `right_wrist`, all 640x480 @ 30 fps → dataset keys `observation.images.{head,left_wrist,right_wrist}` | `profiles/paper-tray-v0.yaml:17-20`, `recorder.py:37` |
| Dataset layout | `LeRobotDataset.create(... use_videos=False)` → images in parquet, v3.0 | `recorder.py:77` |
| Tools available | `lerobot-record`, `lerobot-train`, `lerobot-rollout`, `lerobot-edit-dataset`, `lerobot/scripts/convert_dataset_v21_to_v30.py`; policies act, diffusion, smolvla, pi0, pi05, xvla, groot, vqbet, tdmpc … | `.venv/bin`, `.venv/.../lerobot/policies/` |

Public XLeRobot datasets use **17-dim** state/action: our 14 with `.pos` plus `x.vel, y.vel, theta.vel`
(3-wheel Lekiwi base). Single-arm SO-100/101 datasets use **6-dim** `shoulder_pan.pos … gripper.pos`.
Neither matches our 14 without a slice/pad step (section 3).

## 1. Datasets on the Hub

### 1a. Single-arm SO-100/SO-101 pouring (6-dim state/action)

| id | robot_type / ver | fps | eps / frames | cameras (key: HxW) | joint names | task string | licence | size |
|---|---|---|---|---|---|---|---|---|
| [UNITAmanipulation/so101_pour_water_20260919_142610](https://huggingface.co/datasets/UNITAmanipulation/so101_pour_water_20260919_142610) | so_follower / v3.0 | 30 | 100 / 127,279 | `top` 480x640, `wrist` 480x640 | `shoulder_pan.pos…gripper.pos` | "Pick up the water bottle and pour water into the blue cup" | apache-2.0 | 1.64 GB |
| [ammiellewb/so101_liquid_pouring](https://huggingface.co/datasets/ammiellewb/so101_liquid_pouring) | so_follower / v3.0 | 30 | 51 / 53,409 | `front` 480x640 | same | "Pour the liquid from the cup into the bowl" | apache-2.0 | 995 MB |
| [omkarputti/SO101_liquid_pouring](https://huggingface.co/datasets/omkarputti/SO101_liquid_pouring) | so_follower / v3.0 | 30 | 43 / 33,185 | `front` 480x640 | same | "" (empty task) | apache-2.0 | 171 MB |
| [fhliang/so101_pour_coffee](https://huggingface.co/datasets/fhliang/so101_pour_coffee) | so_follower / v3.0 | 30 | 78 / 49,671 | `scene` 480x640 | same | "Pour from the silver pitcher into the white mug" | apache-2.0 | 456 MB |
| [fhliang/so101_pour_train_4x_scene](https://huggingface.co/datasets/fhliang/so101_pour_train_4x_scene) | so_follower / v3.0 | 30 | 225 / 148,530 | `scene` 480x640 | same | same as above | apache-2.0 | 735 MB |
| [fhliang/so101_pour_hero](https://huggingface.co/datasets/fhliang/so101_pour_hero) | so_follower / v3.0 | 30 | 3,969 / 2,436,060, **94 tasks** | `scene` 480x640 | same | sub-segments: "hover over the silver pitcher.", "pick up the silver pitcher.", "hold the silver pitcher over the white mug.", "pour milk and bring the silver pitcher back upright.", … | none stated | 763 MB (frame count vs size implies re-segmented copies of the same videos) |
| [di-techinnova/so-arm-101-pouring-0.1](https://huggingface.co/datasets/di-techinnova/so-arm-101-pouring-0.1) | so_follower / v3.0 | **15** | 135 / 65,250, 5 tasks | `camera1` **720x1280**, `camera2` 360x640 | same | "Pour sunflower seeds from the orange cup into the clear cup." + coffee variants | apache-2.0 | 1.52 GB |
| [di-techinnova/so-arm-101-pouring-0.2](https://huggingface.co/datasets/di-techinnova/so-arm-101-pouring-0.2) | so_follower / v3.0 | **15** | 96 / 43,200 | as above | same | "Pour from orange cup into blue cup." | apache-2.0 | 558 MB |
| [SurajCreation/so101_pour_v1](https://huggingface.co/datasets/SurajCreation/so101_pour_v1) | so_follower / v3.0 | 30 | 35 / 26,250 | `overhead`, `wrist` 480x640 | same | "Grasp the bottle, position it above the cup, and pour the contents." | none | 534 MB |
| [LeRobot-worldwide-hackathon/91-AM-PM-pouring-liquid](https://huggingface.co/datasets/LeRobot-worldwide-hackathon/91-AM-PM-pouring-liquid) | so101_follower / **v2.1** | 30 | 60 / 30,371 | `gripper`, `top` 480x640 | same | "Pick up a kettle and pour liquid into a cup." | apache-2.0 | 588 MB |
| [SoloHack/pouring-community-v21](https://huggingface.co/datasets/SoloHack/pouring-community-v21) | so101_follower / **v2.1** | 30 | 60 / 30,371 (same data as row above, cams renamed `image`,`image2`) | 480x640 | same | same | apache-2.0 | 1.18 GB |
| [DavidABV/watering_big_dataset](https://huggingface.co/datasets/DavidABV/watering_big_dataset) | so101_follower / **v2.1** | 30 | 62 / 40,982 | `up` **480x848** | same | "Pick up cup and pour water" | apache-2.0 | 393 MB |
| [Ishah8840/so101_pouring](https://huggingface.co/datasets/Ishah8840/so101_pouring) | so101 / v3.0 | **10** | 60 / 9,600 | `front`, `wrist` **240x320, image dtype** | `joint_1…joint_6` | "Pour the water from the source cup into the target cup." | apache-2.0 | 478 MB |
| [roboticshack/team16-water-pouring](https://huggingface.co/datasets/roboticshack/team16-water-pouring) | so100 / **v2.1** | 30 | 50 / 29,887 | `wrist`, `head` 480x640 | `main_shoulder_pan…main_gripper` | "Pouring water from cup to cup" | apache-2.0 | 266 MB |
| [yuz1wan/so100_pour_cup](https://huggingface.co/datasets/yuz1wan/so100_pour_cup) | so100 / **v2.0** | 30 | 10 / 3,139 | `wrist`, `side` 480x640 | `main_*` | "Pick the paper cup and pour sth out." | apache-2.0 | 23 MB |
| [sucrammal/plant_watering_reupload](https://huggingface.co/datasets/sucrammal/plant_watering_reupload) | so-100 / **v2.0** (phosphobot) | **33** | 43 / 8,225 | `main`, `secondary_0` **240x320** | `motor_1…motor_6` | "Pick up the square black cup from the side. Lift the cup towards the blue plant pot, and tip your gripper forward to pour water." | none | 37 MB |
| [sucrammal/plant_watering_reupload_single_cam](https://huggingface.co/datasets/sucrammal/plant_watering_reupload_single_cam) | so-100 / v2.0 | 33 | 43 / 8,225 (same episodes) | same two keys | same | same | none | 16 MB |
| [feiyu05/so101_grab_pour_dataset](https://huggingface.co/datasets/feiyu05/so101_grab_pour_dataset) | so101_follower / v3.0 | 30 | 25 / 9,249 | `front`, `front_depth`, `wrist` 480x640 | `.pos` names | "Put brick into the yellow box and pour it out." (granular, not liquid) | none | 872 MB |
| [RajatDandekar/so101_pour_chocolates](https://huggingface.co/datasets/RajatDandekar/so101_pour_chocolates) | so_follower / v3.0 | 30 | 10 / 7,014 | `webcam`, `arm_cam` 480x640 | `.pos` names | "Pick up the white plastic cup and place it in the blue bowl" (name says pour, task does not) | apache-2.0 | 76 MB |
| [wangranryan/so101_water](https://huggingface.co/datasets/wangranryan/so101_water) | so101_follower / v2.1 | 30 | 10 / 3,686 | `fixed`, `hand_eye` 480x640 | `.pos` names | **"Give me the toilet paper"** — mislabeled, not water | apache-2.0 | 86 MB |
| [arjunsinghyadav2/so101_pick_pour_stack](https://huggingface.co/datasets/arjunsinghyadav2/so101_pick_pour_stack) | — | — | — | — | — | repo holds only `.gitattributes` + 24-byte README; **no data** | mit | 0 |

### 1b. Bimanual SO-100/SO-101 pouring (12-dim: `left_*.pos` ×6 then `right_*.pos` ×6)

| id | robot_type / ver | fps | eps / frames | cameras | task | licence | size |
|---|---|---|---|---|---|---|---|
| [UNITAmanipulation/bi_so101_pour_water_20260920_194823](https://huggingface.co/datasets/UNITAmanipulation/bi_so101_pour_water_20260920_194823) | bi_so_follower / v3.0 | 30 | 100 / 117,619 | `left_wrist` **240x320**, `right_wrist` **240x320**, `top` 480x640 | "Pick up the cup with the left arm and pour water from the bottle into it with the right arm" | apache-2.0 | 1.70 GB |
| [JazSanchez/bimanual-so101_pouring_task](https://huggingface.co/datasets/JazSanchez/bimanual-so101_pouring_task) | bi_so101_follower / v3.0 | 30 | 25 / 30,838 | `wrist_left`, `wrist_right`, `logicamera` 480x640 | "Pour marbles into blue container" | apache-2.0 | 628 MB |
| [LeRobot-worldwide-hackathon/46-3580-bi_so100_pour_water_real](https://huggingface.co/datasets/LeRobot-worldwide-hackathon/46-3580-bi_so100_pour_water_real) | bi_so100_follower / v2.1 | 30 | 14 / 6,569 | `wrist_camera_1`, `wrist_camera_2`, `top_camera`, `side_camera` 480x640 | "bi soarm100 pour water" | apache-2.0 | 327 MB |
| [LeRobot-worldwide-hackathon/46-3580-bi_so100_pour_water_sim](https://huggingface.co/datasets/LeRobot-worldwide-hackathon/46-3580-bi_so100_pour_water_sim) | bi_so100_maniskill / v2.1 (sim) | 30 | 32 / 48,916 | same 4 keys | "Pour the mineral water into the cup." | apache-2.0 | 2.96 GB |
| [Nostalld/bi_so100_pour_water_demo](https://huggingface.co/datasets/Nostalld/bi_so100_pour_water_demo) | bi_so100_maniskill / v2.1 (sim) | 30 | 1 / 650 | same 4 keys | same | apache-2.0 | 39 MB |

### 1c. XLeRobot datasets (real robot; 17-dim unless noted)

| id | robot_type / ver | fps | eps / frames | cameras | state/action names | task | licence | size |
|---|---|---|---|---|---|---|---|---|
| [xlerobot-team/xlerobot-pick-cup-lr-merge-20260904](https://huggingface.co/datasets/xlerobot-team/xlerobot-pick-cup-lr-merge-20260904) | xlerobot / v3.0 | 30 | 202 / 60,464, 2 tasks | `left_arm_wrist` 480x640, `right_arm_wrist` 480x640, `head` **240x424** | 17: `left_arm_*.pos`×6, `right_arm_*.pos`×6, `head_motor_1.pos`, `head_motor_2.pos`, `x.vel`, `y.vel`, `theta.vel` | "Pick up the cup with the right arm." / "…left arm." | apache-2.0 | 1.01 GB |
| [xlerobot-team/xlerobot-right-pick-cup-0826sep8-full-20260910](https://huggingface.co/datasets/xlerobot-team/xlerobot-right-pick-cup-0826sep8-full-20260910) | xlerobot / v3.0 | 30 | 73 / 32,216 | same 3 | same 17 | "Pick up the cup with the right arm." | none | 472 MB |
| [xlerobot-team/xlerobot-right-pick-cup-sep2-trim-20260906](https://huggingface.co/datasets/xlerobot-team/xlerobot-right-pick-cup-sep2-trim-20260906) | xlerobot / v3.0 | 30 | 51 / 13,307 | same 3 | same 17 | same | apache-2.0 | 221 MB |
| [xlerobot-team/xlerobot-right-pick-cup-20260826-0042](https://huggingface.co/datasets/xlerobot-team/xlerobot-right-pick-cup-20260826-0042) | xlerobot / v3.0 | 30 | 40 / 16,487 | same 3 | same 17 | same | apache-2.0 | 263 MB |
| [xlerobot-team/xlerobot-left-pick-cup-sep2-20260902-1819](https://huggingface.co/datasets/xlerobot-team/xlerobot-left-pick-cup-sep2-20260902-1819) | xlerobot / v3.0 | 30 | 34 / 9,705 | same 3 (+ `depth/` dir) | same 17 | "Pick up the cup with the left arm." | none | 357 MB |
| [xlerobot-team/xlerobot-left-pick-cup-sep3-20260903-2015](https://huggingface.co/datasets/xlerobot-team/xlerobot-left-pick-cup-sep3-20260903-2015) | xlerobot / v3.0 | 30 | 29 / 6,507 | same 3 (+ depth) | same 17 | same | none | 757 MB |
| [xlerobot-team/xlerobot-right-place-cup-20260912-0108](https://huggingface.co/datasets/xlerobot-team/xlerobot-right-place-cup-20260912-0108) | xlerobot / v3.0 | 30 | 3 / 1,391 | same 3 + `head_depth` 480x640x1 | same 17 | "Place the cup down with the right arm." | none | 120 MB |
| [xlerobot-team/xlerobot-cup-grasp-20260820-0159](https://huggingface.co/datasets/xlerobot-team/xlerobot-cup-grasp-20260820-0159) (one of ~20 near-identical `xlerobot-cup-grasp-*` shards, 2 eps each) | xlerobot / v3.0 | 30 | 2 / 895 | same 3 | same 17 | "Pick up the cup and place it down" | none | 13 MB |
| [Suyang99/xlerobot-right-pick-cup1-20260902-1637](https://huggingface.co/datasets/Suyang99/xlerobot-right-pick-cup1-20260902-1637) | xlerobot / v3.0 | 30 | 2 / 620 | same 3 (+ depth) | same 17 | "Pick up the cup with the right arm." | none | 27 MB |
| Suyang99/xlerobot-cup-grasp-20260820-0230 (training set of the Suyang99 checkpoints) | not fetched (README of the model: 50 eps / 19,453 frames / 30 fps, 5 cup positions, right arm only) | | | | | | | |
| [yihao-brain-bot/xlerobot-get-water_v_2_1](https://huggingface.co/datasets/yihao-brain-bot/xlerobot-get-water_v_2_1) | xlerobot / **v2.1** | 30 | 50 / 18,808 | `left`, `right`, `top` 480x640 | 17 but **different order/names**: `left_shoulder_pan.pos…right_gripper.pos`, `x.vel`, `y.vel`, `theta.vel`, `mount_pan.pos`, `mount_tilt.pos` | "Move towards the table, align the right arm, grab the drink, and place it in the robot's basket." (fetch a bottle, **no pouring**) | none | 475 MB |
| [yihao-brain-bot/xlerobot_v_2_1](https://huggingface.co/datasets/yihao-brain-bot/xlerobot_v_2_1) | xlerobot / v2.1 | 30 | 277 / 157,338, 13 tasks | same | same | drawers, candy bars, the get-drink task; no pouring | none | 2.30 GB |
| [wangranryan/xlerobot_water](https://huggingface.co/datasets/wangranryan/xlerobot_water) | xlerobot_single_arm_client / v3.0 | 30 | 75 / 22,631 | `left_wrist` **640x480 (portrait)**, `head(RGDB)` 480x640 | state **9** (`left_arm_*.pos`×6 + `x.vel,y.vel,theta.vel`), action **6** (`shoulder_pan.pos…`) | "把水瓶放到篮子里" (put the water bottle in the basket — **no pouring**) | apache-2.0 | 1.13 GB |
| [ArthurWangSawau/xlerobot_multitask_part10](https://huggingface.co/datasets/ArthurWangSawau/xlerobot_multitask_part10) | None / v3.0 | 30 | 5 / 3,694 | `main`, `left_wrist`, `right_wrist` 480x640 | **14**: `right_arm_*.pos`×6, `left_arm_*.pos`×6, `head_motor_1.pos`, `head_motor_2.pos` (right first!) | "右手将圆柱体拿起来递给左手" (right hand hands cylinder to left) | apache-2.0 | 643 MB |
| [lissajous/xlerobot-glue-stick-grasp-30](https://huggingface.co/datasets/lissajous/xlerobot-glue-stick-grasp-30) | None / v3.0 | 30 | 30 / 6,149 | `head`, `wrist` [3,480,640] | **6**: `right_arm_shoulder_pan…right_arm_gripper` (no `.pos`) | "拿起黄色胶棒" (pick up yellow glue stick) | cc-by-4.0 | 246 MB |
| [F-Fer/xlerobot-0](https://huggingface.co/datasets/F-Fer/xlerobot-0) | bi_so_follower / v3.0 | 30 | 50 / 18,998 | `left_front`, `left_left`, `left_right` 480x640 | 12 bimanual `.pos` | "Pick up the sock and place it in the white tray." | apache-2.0 | 770 MB |

No public XLeRobot dataset contains a pour. The two "water" XLeRobot sets are fetch-a-bottle tasks.

### 1d. Plant watering on other embodiments (for task knowledge, not weight reuse)

| id | robot_type / ver | fps | eps / frames | cameras | state/action | task | licence | size |
|---|---|---|---|---|---|---|---|---|
| [villekuosmanen/agilex_water_plant_1](https://huggingface.co/datasets/villekuosmanen/agilex_water_plant_1) (+ `_2` 19 eps 210 MB, `_4` 30 eps 340 MB) | arx5_bimanual / v2.1 | 25 | 18 / 7,947 | `cam_high`, `cam_left_wrist`, `cam_right_wrist` 480x640 | 14 / 14 (names null) | "Pick up the watering can from the table, water the potted plant, and put the can back." | apache-2.0 | 172 MB |
| [DorayakiLin/water_plants_1000_25_08_09_lerobotv21](https://huggingface.co/datasets/DorayakiLin/water_plants_1000_25_08_09_lerobotv21) (+ `_25_08_03_lerobotv21` 120 eps 91 MB) | franka / v2.1 (RLBench-style sim) | 30 | 1000 / 109,369 | `image_1…image_5` **128x128** | 8 / 8 joints+gripper | "water plant" | none | 763 MB |
| [DorayakiLin/water_plants_25_08_03_parquet](https://huggingface.co/datasets/DorayakiLin/water_plants_25_08_03_parquet) | no `meta/info.json`; 120 loose `lerobot_episode_N.parquet` | | | | | | none | 1.89 GB |
| [KEVIN04087/rlbench-lerobot-train-water_plants](https://huggingface.co/datasets/KEVIN04087/rlbench-lerobot-train-water_plants) | rlbench_panda / v3.0 (sim) | 20 | 100 / 10,826 | `front` 128x128 image | state 121 (`low_dim_state_*`), action 7 joints | "water plant" | apache-2.0 | 308 MB |
| [WWZzz/rlbench_water_plants](https://huggingface.co/datasets/WWZzz/rlbench_water_plants) | panda / v3.0 (sim) | 20 | 50 / 5,466, 6 tasks | `front`,`wrist`,`left_shoulder`,`right_shoulder`,`overhead` 256x256 | 8 / 8 | RLBench strings: "pour some water on the plant", "water the soil", "pick up the watering can by its handle and water the plant", … | none | 2.75 GB |
| [RoboChallenge/task_table30_water_potted_plant](https://huggingface.co/datasets/RoboChallenge/task_table30_water_potted_plant) | raw tar (3 parts, 13.7 GB); **not LeRobot**; Table30 paper: ARX-5 arm, "Water the potted plant using the kettle", up to 1000 demos/task, converter script promised | | | | | | none stated | 13.7 GB |
| [BAAI-DataCube/AgiBotWorld-Beta_G1_task_410_Restaurant_pouring_water](https://huggingface.co/datasets/BAAI-DataCube/AgiBotWorld-Beta_G1_task_410_Restaurant_pouring_water) | a2d (AgiBot G1 humanoid) / v3.0 | 30 | 1,090 / 971,498 | `head`, `hand_left`, `hand_right` 480x640 + 5 fisheye 768x960 | AgiBot's own state keys (not `observation.state`) | 餐厅倒水 (restaurant pour water) | none in tags | large |
| [BAAI-DataCube/AgiBotWorld-Beta_G1_task_598_Pouring_tea](https://huggingface.co/datasets/BAAI-DataCube/AgiBotWorld-Beta_G1_task_598_Pouring_tea) | a2d / v3.0 | 30 | 225 / 1,003,724 | same 8 | same | 倒茶 (pour tea) | none in tags | large |
| [RoboSynChallenge/cobotmagic_Real_water_pouring](https://huggingface.co/datasets/RoboSynChallenge/cobotmagic_Real_water_pouring) | aloha (Piper) / v2.1 | 10 | 60 / 34,716 | `cam_high`, `cam_left_wrist`, `cam_right_wrist` | 32 (joints + EE pose, both arms) | water pouring | none | — |
| [cadene/droid_1.0.1](https://huggingface.co/datasets/cadene/droid_1.0.1) (DROID, Franka) | API ok (apache-2.0); `meta/info.json` not fetched; pour episodes exist in DROID's long tail but I did not count them | | | | | | | |

### 1e. Bottle / cup grasp on SO-100/101 (6-dim; for the pick/place halves)

| id | robot_type / ver | fps | eps / frames | cameras | task | licence | size |
|---|---|---|---|---|---|---|---|
| [Jiamo0912/so101-upright-bottle](https://huggingface.co/datasets/Jiamo0912/so101-upright-bottle) | so_follower / v3.0 | 30 | 100 / 35,934 | `front`, `wrist` 480x640 | "Pick up the fallen juice bottle and stand it upright on the table." | apache-2.0 | 344 MB |
| [adhjlm/so101-bottle](https://huggingface.co/datasets/adhjlm/so101-bottle) | so_follower / v3.0 | 30 | 49 / 29,400 | `top`, `wrist` 480x640 | "pick up the bottle and place it to the right outside the white paper" | apache-2.0 | 641 MB |
| [Raakshass/so100_pick_bottle](https://huggingface.co/datasets/Raakshass/so100_pick_bottle) | so100 / v3.0 | **15** | 56 / 28,613 | `cam_high`, `cam_wrist` [3,480,640] | "pick up bottle and place it in a yellow square" | apache-2.0 | 826 MB |
| [Elvinky/so101_pick_place_bottle](https://huggingface.co/datasets/Elvinky/so101_pick_place_bottle) | so_follower / v3.0 | 30 | 21 / 10,058 | `front` 480x640 | "Pick and place" | none | 95 MB |
| [yunlongguo2000/so101_il_cup_grasp](https://huggingface.co/datasets/yunlongguo2000/so101_il_cup_grasp) | so101_follower / v3.0 | 30 | 50 / 16,355 | `top`, `gripper` 480x640 | "Grab the cup from the top and place it in the marked zone" | none | 316 MB |
| [mtitg/so100_grasp_cup](https://huggingface.co/datasets/mtitg/so100_grasp_cup) | so100 / v2.1 | 30 | 41 / 28,546 | `laptop`, `phone` 480x640 | "Grasp a cup and keep it." | apache-2.0 | 586 MB |
| Shivanand709 (SO-100 bottle lift) | `/api/datasets?author=Shivanand709` returns **no datasets**; search "shivanand" finds nothing robotic — does not exist | | | | | | |

## 2. Checkpoints on the Hub

| id | type | input features (config.json) | action | chunk / n_action_steps | trained on | steps × bs | success-rate claims (README) |
|---|---|---|---|---|---|---|---|
| [Suyang99/xlerobot-act-cup-grasp-right](https://huggingface.co/Suyang99/xlerobot-act-cup-grasp-right) | act (resnet18, dim 512, VAE, kl 10) | state [17]; `left_arm_wrist` [3,480,640], `right_arm_wrist` [3,480,640], `head` [3,240,424] | [17] | 100 / 100 | Suyang99/xlerobot-cup-grasp-20260820-0230 (50 eps, 5 cup positions, right arm only) | 20k × 2 (≈2 epochs) | **none** — "never driven the physical arm"; training loss only; left-arm dims constant (arm parked) |
| [Suyang99/xlerobot-smolvla-cup-grasp-right](https://huggingface.co/Suyang99/xlerobot-smolvla-cup-grasp-right) | smolvla (from lerobot/smolvla_base; VLM frozen, expert-only) | config.json still shows the **base** features (state [6], `camera1..3` [3,256,256]); README: `head→camera1`, `right_arm_wrist→camera2`, `left_arm_wrist→camera3`; real state 17 (padded to 32) | [17] | 50 / 50 | same dataset, 40 of 50 eps | 16k (of 20k) × 2, Jetson Orin Nano | offline held-out loss 0.230 vs train 0.235 (ratio 0.98); "Zero physical trials so far" |
| [Suyang99/xlerobot-act-20260820-step20000](https://huggingface.co/Suyang99/xlerobot-act-20260820-step20000) | act | identical to the first row | [17] | 100/100 | same | 20k × 2 | none |
| [lissajous/xlerobot-act-local-grasp-v1](https://huggingface.co/lissajous/xlerobot-act-local-grasp-v1) | act | state [6]; `wrist` [3,480,640] only | [6] (right arm: pan, lift, elbow, wrist_flex, wrist_roll, gripper) | 100 / 100 | local copy of lissajous/xlerobot-glue-stick-grasp-30 (30 eps) | 5k × 8 | "No statistical success rate or general grasp capability is claimed" |
| [F-Fer/pi0-xlerobot-0](https://huggingface.co/F-Fer/pi0-xlerobot-0) | pi0 (paligemma 2b + gemma 300m expert), **LoRA adapter** under `001000/pretrained_model/` | `base_0_rgb`, `left_wrist_0_rgb`, `right_wrist_0_rgb` [3,224,224]; state [32] | [12] | 50 / 50 | F-Fer/xlerobot-0 (sock → tray, bi_so_follower) | 10k × 16 from lerobot/pi0_base | none |
| [ArthurWangSawau/xlerobot_act_policy_v3](https://huggingface.co/ArthurWangSawau/xlerobot_act_policy_v3) | act | state [12]; `main`, `left_wrist`, `right_wrist` [3,480,640] | [12] | 100 / 100 | train_config.json not fetched (not in repo) | — | no README |
| [bigdra/smolvla_xlerobot_handkerchief](https://huggingface.co/bigdra/smolvla_xlerobot_handkerchief) | smolvla (**full fine-tune**: vision unfrozen, `train_expert_only=False`) | state [17]; `head`, `wrist_left`, `wrist_right` [3,480,640] | [17] | 50 / 50 | rk000000/xlerobot-handkerchief-quarters-lowdesk-20260907-480 (50 eps) | 100k × 4 | none |
| [Globalmysterysnailrevolution/xlerobot-pick-cup-smolvla](https://huggingface.co/Globalmysterysnailrevolution/xlerobot-pick-cup-smolvla) | smolvla | state [6]; `hand_cam`, `front_cam`, `side_cam` [3,480,640] | [6] | 50 / 50 | Globalmysterysnailrevolution/xlerobot-pick-cup-20ep | 5k × 4 | none (template README) |
| [wangranryan/xlerobot_lsd_smolvla2](https://huggingface.co/wangranryan/xlerobot_lsd_smolvla2) | smolvla, `resize_imgs_with_padding=[128,128]` | base features (state [6], camera1..3) | [6] | 50 / 50 | wangranryan/xlerobot_lsd | 20k × 8 | none |
| [madokalif/xlerobot-pi05-bottle](https://huggingface.co/madokalif/xlerobot-pi05-bottle) | pi05 **openpi/JAX checkpoint (orbax), not LeRobot** | ManiSkill sim, head + wrist 224x224 | — | — | 1024 + 512 scripted-IK sim eps | 33k + 12k, bs 32 | **sim only**: 71.9 % fixed scene, 0 % → 56–72 % after domain randomisation; no real robot |
| [Histochemichael/act-so101-rack-centered-approach-2cam-25](https://huggingface.co/Histochemichael/act-so101-rack-centered-approach-2cam-25) | act (chunk 30, n_action_steps 3, AMP) | state [6]; `gripper`, `scene` [3,**720,1280**] | [6] | 30 / 3 | Histochemichael/so101-rack-centered-approach-2cam-25 (20 train + 5 val eps) | 25k × 7 | "Offline errors are not physical grasp success rates" |
| [edge-inference/smolvla-so101-pick-orange](https://huggingface.co/edge-inference/smolvla-so101-pick-orange) | smolvla (from base, expert-only) | state [6]; `front`, `wrist` [3,480,640] | [6] | 50 / 50 | LightwheelAI/leisaac-pick-orange (60 sim eps) | 30k × 8 | Isaac-Sim eval: 56 % (multi-rank) / 64 % (single-rank) |
| [DylM0nster22/act_so101_cup_pour_test](https://huggingface.co/DylM0nster22/act_so101_cup_pour_test) | smolvla (despite the name) | state [6]; `wrist`, `side`, `top` [3,480,640] | [6] | 50 / 50 | DylM0nster22/record-test | 15k × 64 | none |
| [lerobot/smolvla_base](https://huggingface.co/lerobot/smolvla_base) | smolvla, SmolVLM2-500M-Video-Instruct, 16 VLM layers, `max_state_dim=max_action_dim=32`, `resize_imgs_with_padding=[512,512]` | state [6]; `camera1`,`camera2`,`camera3` [3,256,256] (placeholders; real keys mapped at fine-tune) | [6] | 50 / 50 | SO-100 community data (paper 2506.01844) | — | docs: "~50 episodes"; "25 episodes… not enough"; 20k steps ≈ 4 h on one A100 |
| [lerobot/pi05_base](https://huggingface.co/lerobot/pi05_base) | pi05 (gemma_2b + gemma_300m), licence **gemma**, `tokenizer_max_length=200`, config `device: mps` | `base_0_rgb`, `left_wrist_0_rgb`, `right_wrist_0_rgb` [3,224,224]; state [32] | [32] | 50 / 50 | Physical Intelligence pretraining | — | none for our setting |

Suyang99/xlerobot-smolvla-right-0826sep8-p0.2-b16-u20000 exists but holds only `.gitattributes` (empty).

## 3. Joint-name and camera mapping

| Ours (`farm`, 14, no suffix) | Public XLeRobot 17-dim | Single-arm SO-101 6-dim | `main_*` SO-100 v2.x | phosphobot | Suyang99 SmolVLA cams | pi0/pi05 cams |
|---|---|---|---|---|---|---|
| `right_arm_shoulder_pan` … `right_arm_gripper` (idx 6–11) | `right_arm_shoulder_pan.pos` … (idx 6–11) | `shoulder_pan.pos` … `gripper.pos` (idx 0–5) | `main_shoulder_pan` … `main_gripper` | `motor_1…motor_6` | — | — |
| `left_arm_*` (idx 0–5) | `left_arm_*.pos` (idx 0–5) | n/a | n/a | n/a | — | — |
| `head_motor_1`, `head_motor_2` (idx 12–13) | same + `.pos` (idx 12–13) | n/a | n/a | n/a | — | — |
| (no base) | `x.vel`, `y.vel`, `theta.vel` (idx 14–16) — pad with 0 | | | | | |
| `observation.images.head` 480x640 | `observation.images.head` **240x424** | `top` / `front` / `scene` / `overhead` | `side` / `head` / `laptop` | `main` | `camera1` | `base_0_rgb` 224² |
| `observation.images.right_wrist` | `observation.images.right_arm_wrist` 480x640 | `wrist` / `gripper` / `hand_eye` | `wrist` / `phone` | `secondary_0` | `camera2` | `right_wrist_0_rgb` |
| `observation.images.left_wrist` | `observation.images.left_arm_wrist` | — | — | — | `camera3` | `left_wrist_0_rgb` |

Watch-outs: (1) `ArthurWangSawau/*` puts **right arm first**; (2) `yihao-brain-bot/*` uses `left_*`/`right_*`
without `_arm` and appends `mount_pan/mount_tilt` after the base velocities; (3) `lissajous` uses our
prefix but no `.pos`; (4) our recorder omits `.pos` while the vendored robot emits `.pos` — pick one
before recording for real (adding `.pos` makes our files mergeable with every `xlerobot` set);
(5) SO-101 datasets are normalised the same way as ours (-100..100 / 0..100) but against **their**
calibration ranges, so absolute joint targets can be off by several degrees on our arm; (6) the public
`head` camera is 424x240 from a RealSense-style module, ours is a 640x480 webcam.

## 4. Literature and prior work

| Work | What it is | What transfers to a 5-DoF+gripper SO-101 |
|---|---|---|
| [AlphaGarden (arXiv 2111.06014)](https://arxiv.org/abs/2111.06014), [Can Machines Garden? (2306.17162)](https://arxiv.org/abs/2306.17162) | Berkeley gantry robot tending a 1.5 x 3 m polyculture bed; irrigation and pruning policies learned in a plant simulator; 60-day cycles, ~44 % less water than horticulturalists. | The *decision* layer (when/how much to water from overhead imagery + moisture) — not the arm motion; irrigation is a gantry nozzle, no pouring. |
| [Autonomous Mobile Plant Watering Robot: A Kinematic Approach (2508.08607)](https://arxiv.org/abs/2508.08607) | 6-DoF arm on a 4WD chassis, YOLOv5 plant detection, moisture probe, hose; scripted IK, no learning. | Confirms the scripted keyframe route is the norm for watering; nothing to reuse as weights. |
| [Explainable Hierarchical IL for Robotic Drink Pouring (2105.07348)](https://arxiv.org/abs/2105.07348) | Hierarchical imitation: high-level logic graph + low-level pouring controller; claims better success/adaptability than flat BC. | Structure: split pick / align / tilt / return into sub-policies or keyframes — exactly what `fhliang/so101_pour_hero` did with its 94 sub-task strings. |
| [PourIt! (ICCV 2023, 2307.11299)](https://arxiv.org/abs/2307.11299) | Weakly-supervised liquid segmentation from one RGB image (CAM), closed-loop pour on a Franka. | Cheap liquid-in-flight detector for our wrist camera to stop the tilt; input is RGB only. |
| Visual closed-loop pouring ([1610.02610](https://arxiv.org/abs/1610.02610)), audio/vibration height estimation ([1903.00650](https://arxiv.org/abs/1903.00650)), vision+audio fusion ([ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0019057822004815)), capacitive RoboCAP ([2405.07423](https://arxiv.org/abs/2405.07423)), transparent-container lab pouring ([2404.16529](https://arxiv.org/abs/2404.16529)) | Pouring as a perception-driven tilt-angle/flow control problem. | For a fixed 1–2 s pour into a fixed opening, open-loop tilt with a vision "did water come out" check is sufficient; these become relevant only if we need dosed volumes. |
| [RLBench `water_plants`](https://github.com/stepjam/RLBench/blob/master/rlbench/tasks/water_plants.py) | Panda sim task; success = physics droplets hit a sensor after the can reaches the pour point; 6 language variants (listed in 1d). | Task strings and success definition; sim data (128–256 px, 7–8 DoF) is not usable for our weights. |
| [RoboCOIN (2511.17441)](https://arxiv.org/abs/2511.17441) | 180k bimanual demos, 15 platforms (Agilex Cobot Magic/Split ALOHA, Galaxea R1 Lite, AgiBot G1, Unitree G1 …), CC BY 4.0, "Built on the LeRobot framework"; taxonomy has "pour" but no SO-101 platform. | Language/segment annotation scheme; no embodiment match. |
| [AgiBot World Colosseo (2503.06669)](https://arxiv.org/abs/2503.06669) | 1M trajectories on AgiBot G1 humanoids; "Pour Water" (kettle → cup) is an evaluated task; HF LeRobot-v3 conversions exist (1d). | Pour-phase visual priors only via a VLA that saw it (GO-1, not open in LeRobot). |
| [RoboChallenge / Table30 (2510.17950)](https://arxiv.org/abs/2510.17950) | 30 real tasks on UR5, Franka, Cobot Magic Aloha, ARX-5; "water potted plant" is ARX-5 with a kettle; up to 1000 demos/task, raw video+json with a LeRobot converter. | Task definition and a reference for how many demos a real pour needs (hundreds); embodiment mismatch. |
| [Open X-Embodiment (2310.08864)](https://arxiv.org/abs/2310.08864), [DROID (2403.12945)](https://arxiv.org/abs/2403.12945), BridgeData V2 | 1M+ / 76k / 60k episodes; pour instructions exist in the long tail (not counted here). | Only through pretrained VLAs (pi0.5, SmolVLA's mix); not for direct fine-tuning. |
| [LAPA (2410.11758)](https://arxiv.org/abs/2410.11758), [UniVLA (2505.06111)](https://arxiv.org/abs/2505.06111), [ConLA (2602.00557)](https://arxiv.org/abs/2602.00557), [survey 2604.04974](https://arxiv.org/abs/2604.04974) | Latent-action pretraining from action-free video, then a small labelled set to decode to robot actions. | The only route by which footage helps; still needs our own action-labelled episodes; research-grade, not in LeRobot. |

## 5. Footage

| Source | Verified | Notes |
|---|---|---|
| https://vector-wangel.github.io/XLeRobot-assets/videos/Real_demos/xlerobot030.mp4#t=56,60 | HTTP 200, `video/mp4`, ~26.2 MB (etag 0x18f1489), last-modified 2026-01-12 | Official 0.3.0 "household chores showcase" (README 2025-08-30). I could not decode the 56–60 s segment here (no working ffmpeg on this Mac), so the watering content of that window is the caller's claim, not verified by me. |
| https://www.youtube.com/watch?v=upB1CEFeOlk | oEmbed: "XLeRobot 0.3.0 Assembly Guide" by WowRobo Robotics | Assembly, not a pour demo. |
| https://www.bilibili.com/video/BV1AGWFzUEJf/ | HTTP 200 | Linked next to the YouTube one in README/docs; presumably the same assembly guide; content not verified. |
| Hub dataset videos | every `videos/` dir in 1a–1c | Actually useful footage: each SO-101 pour dataset is 30 fps MP4 **with** synchronised actions — the only "footage" worth anything for training. |
| Mobiusi/Watering-Operation-Video-Dataset | found by search (cc-by-nc-sa-4.0, video-classification) | Human videos, no robot, no actions; NC licence. Not fetched further. |

Video without actions cannot train a LeRobot policy: ACT/SmolVLA/pi0.5 all need `action` per frame.
Options that use video anyway — latent-action pretraining (LAPA/UniVLA), or hand-labelling keyframes
from video and replaying them with our IK — are future work, not a plan.

## 6. Recommendations

### (a) Datasets to fine-tune on for a right-arm SO-101 pour

1. **UNITAmanipulation/so101_pour_water_20260919_142610** — the only large (100 ep), v3.0, bottle-to-cup *water* pour with a wrist camera at our fps/resolution. Mapping: `observation.images.top→head`, `observation.images.wrist→right_wrist`; state/action 6-dim → our indices 6–11 (`right_arm_*`). Mismatch: table-mounted arm and top camera geometry differ from our cart mount; bottle and cup differ from our bottle and tray opening.
2. **UNITAmanipulation/bi_so101_pour_water_20260920_194823** — 100 ep; the *right* arm pours a bottle while the left holds a cup, i.e. our arm assignment. Take indices 6–11 and `top`; wrist cams are 240x320 (upscale or drop). v3.0.
3. **LeRobot-worldwide-hackathon/91-AM-PM-pouring-liquid** (kettle, 60 ep, `gripper`+`top`) or **fhliang/so101_pour_coffee/4x_scene** (pitcher, 78/225 ep, single `scene` cam) as a third pour source. Both v2.1/v3.0 respectively; v2.1 needs `python -m lerobot.scripts.convert_dataset_v21_to_v30` first. `DavidABV/watering_big_dataset` is 848x480 single-cam v2.1 — usable only after resize.
   Skip: `sucrammal/plant_watering_*` (33 fps, 320x240, `motor_N` names, v2.0 phosphobot), `Ishah8840` (10 fps images), `di-techinnova` (15 fps, 720p), `wangranryan/so101_water` (mislabelled), `arjunsinghyadav2` (empty).
   Pooling any of these with our data requires: rename camera keys to a common set, slice our 14→6 (or pad theirs 6→14 with our recorded left-arm/head values — not zeros), and accept per-robot calibration offsets. Expect these sets to teach the *pour motion prior*; the final 10–20 cm of approach to our tray opening will come only from our own episodes.

### (b) Base model

- **ACT from scratch** (local baseline): trains natively on MPS (`--policy.device=mps`, documented), ~50 M params, takes our three 640x480 cams unchanged, no language. Community XLeRobot ACT runs used chunk 100, 20k steps; the one SO-101 run with held-out eval used chunk 30 / `n_action_steps` 3. Fixed trays + fixed bottle rest is exactly ACT's comfort zone. Needs ≥50 of *our* episodes; pretraining on 1a data only helps if features are made identical.
- **SmolVLA (lerobot/smolvla_base)** (recommended if a GPU is rented): 450 M / 100 M trainable with the default frozen VLM; pretrained on SO-100/101 community data so the embodiment prior is real; three cams map to `camera1..3` (community convention: head, right wrist, left wrist), state 14 auto-padded to 32. Cost: docs say 20k steps ≈ 4 h on one A100 at bs 64; a Jetson Orin Nano did ~1 step/s at bs 2. On an M-series Mac expect bs ≤ 4 and hours per few-k steps — fine for smoke tests, not for 20k. `lerobot-train --job.target=a10g-small` (HF Jobs) is the documented rental path. Run with `--inference.type=rtc` on the Mac at deploy time.
- **pi0.5 (lerobot/pi05_base)**: 3 B params, Gemma licence, 224² inputs, 32-dim padded state. Only viable as LoRA on a rented GPU (F-Fer did pi0 LoRA at bs 16); not for MPS training. Treat as a later experiment.

### (c) XLeRobot checkpoints worth a zero-shot try

None will pick our bottle; the candidates were trained on one table, one cup, 40–50 episodes, and none has been physically evaluated. Two are worth a **30-minute pipeline test** because their camera set is ours one-to-one (head + two wrists, 17-dim): **Suyang99/xlerobot-act-cup-grasp-right** (right-arm cup pick; feed our 14 joints + `x.vel,y.vel,theta.vel = 0`, rename `right_wrist→right_arm_wrist`, `left_wrist→left_arm_wrist`, downscale head to 424x240) and **Suyang99/xlerobot-smolvla-cup-grasp-right** (same data, has the documented `camera1..3` mapping and a 0.98 held-out/train loss ratio). Use them to validate `farm/skills/policy.py`'s camera_map/state plumbing and the safety wrapper, then discard. `lissajous/xlerobot-act-local-grasp-v1` is the closest *shape* to a right-arm-only skill (6-dim, one wrist cam) but is a glue-stick grasp.

### (d) What we must record ourselves, and how

- Counts (LeRobot docs): "at least 50 episodes, with 10 episodes per location"; SmolVLA: "~50 episodes… 25 episodes… not enough". Our variations are small (tray A/B, bottle pose jitter in the rest, water level), so target **60–100 episodes: 30–50 per tray**, plus 10 % deliberately perturbed starts. Real pours in Table30 used hundreds; we are not aiming for that generality.
- Source of demonstrations: the LLM-servo keyframe runs (`farm run --record` / `farm once --record`) via `farm/learning/recorder.py`, which writes a v3.0 LeRobotDataset (`robot_type=xlerobot_2wheels`, 14-dim state/action, keys `observation.images.{head,left_wrist,right_wrist}` 480x640, images in parquet). `lerobot-record --robot.type=xlerobot_2wheels` would also work since the config is registered, but it expects a teleop/policy source; the farm recorder is the path.
- Before the first real recording: (i) add `.pos` to joint names so the files merge with public `xlerobot` sets; (ii) fix one task string per tray, e.g. "Pick up the bottle, pour water into tray A's refill opening, and put the bottle back."; (iii) record `action` as the commanded target for the *next* tick, `observation.state` as measured — the ACT/SmolVLA convention; (iv) keep 30 fps and cameras fixed; (v) log the pour outcome (ml poured / VLM verdict) in the evidence store so failed episodes can be dropped.
- Honest caveat: keyframe-interpolated demos are smooth and low-diversity, so a policy will mostly learn to reproduce them with visual re-alignment of the grasp and spout. That is the intended win (robustness to bottle pose), not a new skill.

## 7. Sources

Hub API/JSON as listed in each row; LeRobot docs [il_robots](https://huggingface.co/docs/lerobot/il_robots), [smolvla](https://huggingface.co/docs/lerobot/smolvla); XLeRobot [README](https://github.com/Vector-Wangel/XLeRobot), [docs](https://xlerobot.readthedocs.io/); papers linked inline. Raw fetch dumps: `/tmp/xlr/datasets_round{1,2}.json`, `/tmp/xlr/models_round{1,2}.json` (not committed).
