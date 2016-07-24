UARM_CONTROLLER
======

The function of this node is to translate desired joint positions to servo positions.

It receives messages containing the position desired for joints, and it communicate that to the servo driver

Test
======
```
rostopic pub /uarm/joints_to_controller uarm_msgs/Joints '{angle_r: 0.581789, angle_l: 1.941941, angle_rot: 0.0, angle_hand_rot: 0.0, angle_grip: 0.0}' --once


```

Command for turn off all servos
======
```
rostopic pub /pca9685/servostate_to_controller pca9685_msgs/ServoState '{port_num: 0, servo_rot: 0.5, servo_type: 0}' --once

```
