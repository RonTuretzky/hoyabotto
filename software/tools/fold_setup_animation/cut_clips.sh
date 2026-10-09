#!/bin/bash
# cut per-step MP4s from overlaid frames: cut DIR FIRST LAST NAME
set -e
D=/Users/wk/conductor/workspaces/xlerobot-farm/las-vegas-v1/.context/chat-mac/software/docs/img/fold-policy-setup
cut() { ffmpeg -loglevel error -y -framerate 24 -start_number $2 -i $1/o%04d.png -frames:v $(( $3 - $2 + 1 )) -c:v libx264 -pix_fmt yuv420p -crf 23 -movflags +faststart $D/$4.mp4; echo "$4: $(( ($3 - $2 + 1) / 24 )) s"; }
cut /tmp/foldviz/out 176 405 step1-pan-axes
cut /tmp/foldviz/out 406 600 step2-height
cut /tmp/foldviz/out_steps 1 300 step3-square
cut /tmp/foldviz/out_steps 301 720 step4-carton
cut /tmp/foldviz/out_steps 721 900 step4b-tags
cut /tmp/foldviz/out_steps 901 1020 step5-head
cut /tmp/foldviz/out_steps 1021 1140 step6a-floor-tags
cut /tmp/foldviz/out_steps 1141 1260 step8b-start-pose
