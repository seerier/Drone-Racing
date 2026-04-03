#!/bin/bash                                                                                                           
#SBATCH --job-name=quad_race                                                                                          
#SBATCH --time=24:00:00                                   
#SBATCH --gres=gpu:l40:1                                                                                                  
#SBATCH --mem=128G                                                                                                     
#SBATCH --cpus-per-task=16                                                                                            
#SBATCH --output=/home/gxzhao4/github/ese651_project/logs/slurm/slurm_%j.out          

source ~/.bashrc
conda activate /mnt/kostas_home/gxzhao4/conda_envs/drone_racing                  
                                                    
cd /home/gxzhao4/github/ese651_project
export PYTHONPATH=/home/gxzhao4/github/ese651_project:$PYTHONPATH                                                                                        
                                                                                                                    
python scripts/rsl_rl/train_race.py \
--task Isaac-Quadcopter-Race-v0 \
--num_envs 8192 \
--max_iterations 20000 \
--headless \
--logger wandb 