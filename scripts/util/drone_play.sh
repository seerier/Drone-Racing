#!/bin/bash

python scripts/rsl_rl/play_race.py \
--task Isaac-Quadcopter-Race-v0 \
--num_envs 1 \
--load_run 2026-03-19_06-11-10 \
--checkpoint best_model.pt \
--headless \
--video \
--video_length 800