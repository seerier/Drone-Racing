#!/bin/bash

python scripts/rsl_rl/play_race.py \
--task Isaac-Quadcopter-Race-v0 \
--num_envs 1 \
--load_run [YYYY-MM-DD_XX-XX-XX] \ # The run directory is in logs/rsl_rl/quadcopter_direct/
--checkpoint best_model.pt \
--headless \
--video \
--video_length 800