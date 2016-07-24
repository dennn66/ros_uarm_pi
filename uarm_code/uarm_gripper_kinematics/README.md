UARM_GRIPPER_KINEMATICS
======

The function of this node is to convert desired gripper action to joints position

Test
======

```
rostopic pub /uarm/set_gripper_position uarm_msgs/GripperPosition '{stretch: 25, height: 80, arm_rot: 0.0, hand_rot: 0.0}' --once

rostopic pub /uarm/set_gripper_position uarm_msgs/GripperPosition '{stretch: 10, height: -10, arm_rot: 0.0, hand_rot: 0.0}' --once
rostopic pub /uarm/set_gripper_state std_msgs/Bool '{data: 1}' --once
```

Turn off all servos
======

```
rostopic pub /pca9685/servostate_to_controller pca9685_msgs/ServoState '{port_num: 0, servo_rot: 0.5, servo_type: 0}' --once

```
