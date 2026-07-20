# NeuPAN Ackermann Command Conversion Design

## Problem

NeuPAN's Ackermann model returns a two-element control vector `[v, steering_angle]`. The ROS 2 `ackermann_steering_controller` consumes `geometry_msgs/Twist`, where `linear.x` is vehicle speed and `angular.z` is vehicle yaw rate. The current ROS adapter copies the steering angle directly into `angular.z`, so the controller applies a second kinematic conversion. This reduces curvature during forward motion and reverses the intended steering direction in some reverse-motion commands.

## Considered approaches

1. Convert in `neupan_node` when constructing the ROS `Twist` message. This is the selected approach because it fixes the semantic mismatch at the producer boundary and keeps `/neupan_cmd_vel` standards-compliant.
2. Convert only in `cmd_vel_mux`. This would couple a generic source selector to NeuPAN and Ackermann vehicle geometry, so it is rejected.
3. Replace the command interface with an Ackermann-specific message/controller. This would broaden the change across launch files and controllers without being necessary for the current bug, so it is rejected.

## Design

Add a pure utility that converts NeuPAN's second control component to yaw rate. For Ackermann kinematics it applies:

```text
yaw_rate = speed * tan(steering_angle) / wheelbase
```

The utility validates that Ackermann wheelbase is positive. It naturally preserves the correct sign for forward and reverse motion. Zero speed produces zero yaw rate. For differential-drive kinematics, where NeuPAN's second control component is already yaw rate, it returns that component unchanged.

`NeupanCore.generate_twist_msg` will continue to publish zero commands when the planner returns no action, requests an emergency stop, or reports arrival. Otherwise it will publish the NeuPAN speed as `linear.x` and the converted yaw rate as `angular.z`, using the kinematics and wheelbase already loaded by NeuPAN's robot model.

No planner weights, speed limits, Hybrid A* parameters, mux behavior, or physical steering limits are changed in this fix.

## Tests

Unit tests will cover:

- forward motion conversion;
- reverse motion conversion, including the sign seen in the reported failure;
- zero speed;
- invalid wheelbase;
- differential-drive passthrough;
- integration of the conversion into `generate_twist_msg` while retaining stop and arrival behavior.

The package's existing tests and Python syntax/compile checks will run after the focused regression tests.
