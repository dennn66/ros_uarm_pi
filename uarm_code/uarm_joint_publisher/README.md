UARM_JOINT_PUBLISHER
====================

They receives data from the topic `/uarm/joints_to_controller`, that contains the position desired for joints, and between the three make possible the visualization of the uarm's model in RVIZ.


Test
======
```
rostopic pub /uarm/joints_to_controller uarm_msgs/Joints '{angle_r: 0.581789, angle_l: 1.941941, angle_rot: 0.0, angle_hand_rot: 0.0, angle_grip: 0.0}' --once
```
