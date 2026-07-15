#!/usr/bin/env python3
"""
Train DUNE (Deep Unfolded Neural Encoder) for the Ackermann Robot.

This generates synthetic training data (random 2D points + CVXPY optimization)
and trains ObsPointNet, a small MLP that maps obstacle points to distance features.

The trained .pth checkpoint is vehicle-specific — it encodes the robot's
collision geometry (0.70m × 0.52m × 0.593m wheelbase) into the distance latent space.

Training is pure computation: no simulation, no real data, no GPU required.
Runtime: ~30-60 minutes on CPU for 200K points × 5000 epochs.

Usage:
    conda activate neupan
    cd /home/young/AckermannRobot-2D/src/neupan_ros2/train
    python train_dune.py
"""

import os
import sys

# Ensure checkpoints save under THIS directory
script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)
if sys.path[0] != os.getcwd():
    sys.path.insert(0, os.getcwd())

from neupan import neupan

if __name__ == '__main__':
    config_file = os.path.join(script_dir, 'train_config.yaml')
    print(f"Loading config: {config_file}")

    neupan_planner = neupan.init_from_yaml(config_file)
    neupan_planner.train_dune()

    print("\n=== Training complete ===")
    print(f"Checkpoints saved under: {script_dir}/model/")

    # Copy best model to the runtime models directory
    import shutil
    model_dir = os.path.join(script_dir, 'model', 'ackermann_robot_default')
    dest_dir = os.path.join(script_dir, '..', 'config', 'robots', 'ackermann_robot', 'models')
    best_model = os.path.join(model_dir, 'model_5000.pth')
    dest_model = os.path.join(dest_dir, 'dune_model_5000.pth')
    if os.path.exists(best_model):
        shutil.copy2(best_model, dest_model)
        print(f"Copied best model to: {dest_model}")
    else:
        print(f"WARNING: Best model not found at {best_model}")
        print("Check training output and copy the best checkpoint manually.")
