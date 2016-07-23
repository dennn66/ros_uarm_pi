UARM_GRIPPER_KINEMATICS
======

The function of this node is to convert desired gripper action to joints position

Test
======

```
rostopic pub /uarm/set_gripper_position uarm_msgs/GripperPosition '{stretch: 10, height: -10, arm_rot: 0.0, hand_rot: 0.0}' --once
rostopic pub /uarm/set_gripper_state std_msgs/Bool '{data: 1}' --once
```
