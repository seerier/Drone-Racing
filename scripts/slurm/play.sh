#!/bin/bash                                                                                                           
#SBATCH --job-name=quad_race                                                                                          
#SBATCH --time=1:00:00                                   
#SBATCH --gres=gpu:l40:1                                                                                                  
#SBATCH --mem=128G                                                                                                     
#SBATCH --cpus-per-task=16                                                                                            
#SBATCH --output=logs/slurm/slurm_%j.out          

source ~/.bashrc
conda activate /mnt/kostas_home/gxzhao4/conda_envs/drone_racing                  
                                                    
cd /home/gxzhao4/github/ese651_project
export PYTHONPATH=/home/gxzhao4/github/ese651_project:$PYTHONPATH

python scripts/rsl_rl/play_race.py \
--task Isaac-Quadcopter-Race-v0 \
--num_envs 1 \
--load_run 2026-03-19_07-33-40 \
--checkpoint best_model.pt \
--headless \
--video \
--video_length 800 \
--seed 1