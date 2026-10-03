"""R2a cress-planter assembly: an isolated, initially disabled contract.

Nothing in this package moves a motor. It defines the stage machine, the
evidence each stage needs before the next may start, the frames and units
that relate CAD millimetres to robot metres, the dataset schema a demonstration
must satisfy, and the episode accounting that decides what may be called a
success. Execution is gated by `r2a.assert_execution_allowed`, which refuses
while the profile says `execution_enabled: false`, while the station transform
is unmeasured, or while the grip thresholds are unmeasured.

Handoff: /Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress-retrofit/HANDOFF-ROBOT-TRAINING.md
(copied to docs/r2a-handoff.md). Parts and hashes: parts/r2a/.
"""
