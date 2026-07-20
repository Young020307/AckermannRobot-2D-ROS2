# Ackermann Steering Limit Alignment Design

## Problem

The vehicle model and `ros2_control` interface both define `0.52 rad` as the physical limit of each front steering joint. Hybrid A* and NeuPAN currently use the same value as a bicycle-model center steering limit. With a `0.593 m` wheelbase and `0.510 m` front track, a `0.52 rad` center command requires approximately `0.6496 rad` at the inside wheel, so the controller requests motion that the physical joint cannot execute.

This curvature mismatch lets tracking error accumulate. At a Reeds-Shepp gear-change point, NeuPAN can then remain outside its arrival threshold, collapse its reference horizon to the unreachable cusp, and command nearly zero speed indefinitely.

## Selected design

Keep the physical steering-joint and `ros2_control` command limits at `±0.52 rad`. Convert that inside-wheel limit into the bicycle-model center limit:

```text
center_limit = atan(wheelbase / (wheelbase / tan(wheel_limit) + track / 2))
             = 0.430678 rad
```

Use rounded, mutually consistent planner settings:

- NeuPAN runtime center steering limit: `±0.43 rad`;
- NeuPAN runtime initial-path minimum radius: `1.30 m`;
- Hybrid A* minimum turning radius: `1.30 m`;
- NeuPAN training configuration: the same `±0.43 rad` and `1.30 m` values.

Keep reference speed, steering-rate limit, tracking weights, arrival thresholds, wheelbase, and track unchanged. Keep the previously implemented conversion from NeuPAN steering angle to ROS yaw rate.

The DUNE checkpoint encodes collision geometry rather than the vehicle's maximum steering command, so this control-bound correction does not require retraining. Updating the training YAML keeps future training runs consistent.

## Verification

Add geometry consistency tests that read the physical joint limit, `ros2_control` command limits, controller geometry, NeuPAN runtime/training configuration, and Hybrid A* configuration. The tests calculate the center-equivalent limit and verify all planner radii and steering bounds agree.

Rebuild `ackermann_robot`, `hybrid_astar_planner`, and `neupan_ros2`. Gazebo and NeuPAN must be restarted before runtime testing because the robot model and planner configuration are loaded at startup.
