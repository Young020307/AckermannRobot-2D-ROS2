# Ackermann Steering Limit Alignment Design

## Problem

Hybrid A* and NeuPAN model `0.52 rad` as the bicycle-model center steering angle. With a `0.593 m` wheelbase and `0.510 m` front track, the ROS 2 Ackermann controller expands that command to approximately `0.6496 rad` on the inside wheel and `0.4307 rad` on the outside wheel. The XACRO currently limits both steering joints to `0.52 rad`, so the inside joint saturates and the simulated vehicle cannot execute the planners' `1.05 m` minimum-radius path.

The resulting tracking error can prevent NeuPAN from reaching a Reeds-Shepp gear-change point. NeuPAN then holds a zero-length reference horizon at that point and commands nearly zero speed indefinitely.

## Selected design

Treat `0.52 rad` as the center-equivalent steering limit, as requested. Raise both simulated front steering joint limits symmetrically from `±0.52 rad` to `±0.66 rad`. The new limit covers the theoretical `0.6496 rad` inside-wheel requirement with a small numerical margin.

Keep the following unchanged:

- Hybrid A* minimum turning radius: `1.05 m`;
- NeuPAN steering limit: `±0.52 rad`;
- NeuPAN initial-path minimum radius: `1.05 m`;
- controller wheelbase and front track;
- NeuPAN arrival thresholds and tracking weights.

## Verification

Add a geometry consistency test that reads the controller and planner configuration, calculates the required inside-wheel angle, and verifies the XACRO joint limit can represent it. The test will also verify the Hybrid A* and NeuPAN minimum-radius assumptions remain consistent with the center steering limit.

After the test passes, rebuild `ackermann_robot`. A running Gazebo instance must be restarted because joint limits are loaded when the robot model is spawned. Runtime verification will compare `steering_angle_command` and `steer_positions` during a maximum-curvature turn; the inside joint must no longer stop at `0.52 rad`.
