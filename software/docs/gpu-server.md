# Two computers: the robot laptop and the training Mac

The robot laptop is plugged into the robot and keeps every safety decision. A second machine (the M4 Max with 128 GB) trains on the robot's recorded runs and serves the trained policy over the network. It never opens a serial port.

Status: built and checked on one machine (the trained test checkpoint, served locally, answered in about 28 ms and drove the simulator through the remote client). Not yet run between two machines, and there are no episodes from our robot yet.

## 1. Record on the robot laptop

```sh
farm run --every 3600 --record        # or: farm once --record
```

Episodes are written to `software/data/dataset` (LeRobotDataset v3).

## 2. Copy the episodes to the training Mac

Either machine can start the copy. Over the local network, with Remote Login enabled on the robot laptop (System Settings → General → Sharing):

```sh
# on the training Mac, inside software/
rsync -a <user>@<robot-laptop>.local:xlerobot-farm/software/data/dataset/ data/dataset-robot/
```

A private Hugging Face dataset also works; the recorder writes the standard format.

## 3. Train on the training Mac

```sh
python -m farm.learning.train --policy act --dataset-root data/dataset-robot --steps 20000 --device mps
python -m farm.learning.evaluate --checkpoint data-train/act_farm --dataset-root data/dataset-robot
```

The earlier pipeline test trained ACT for 5,000 steps here in about an hour. SmolVLA is untested on this machine.

## 4. Serve the policy

```sh
# on the training Mac
export FARM_POLICY_TOKEN=<any long random string>     # optional but recommended on shared Wi-Fi
farm policy-server --checkpoint data-train/act_farm --port 8766
```

It prints what the policy expects (cameras, state size) and listens on all interfaces. macOS will ask once whether to allow incoming connections.

## 5. Use it from the robot laptop

```sh
export FARM_POLICY_TOKEN=<the same string>
farm policy-test --server http://<training-mac>.local:8766              # simulator first
farm policy-test --server http://<training-mac>.local:8766 --real --steps 100
```

or set it in the profile:

```yaml
policy:
  server_url: "http://<training-mac>.local:8766"
  timeout_s: 2.0
```

One request fetches a whole chunk of actions; the robot laptop plays them through the usual clamps and asks again with a fresh observation.

## What happens when it goes wrong

| Situation | Result |
|---|---|
| Server slow (over `timeout_s`), unreachable or returns an error | The skill stops, holds position and reports. Nothing is retried, no stale action is replayed. |
| Action outside the clamps | Clipped to the allowed step, as for any skill. |
| STOP pressed in the viewer | Wins immediately. |
| Server switched off | Keyframe skills keep working. A policy is never required to water. |

The care cycle does not call a policy yet; today the policy path is reached through `farm policy-test`.

## Dexbotic

[Dexbotic's XLeRobot integration](https://github.com/dexmal/dexbotic/blob/HEAD/hardware/docs/xlerobot_inference_example.md) is written for our exact robot (0.4.0 two-wheel, 16-dimension action, head and two wrist cameras) but requires Ubuntu with NVIDIA GPUs (RTX 4090 / A100 class). It does not run on the training Mac. Our recorder writes the LeRobot format its converter takes, so it remains an option on a rented GPU.
