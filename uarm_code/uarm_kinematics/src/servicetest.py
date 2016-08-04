#!/usr/bin/env python

import sys
import rospy
from uarm_msgs.srv import *
from uarm_msgs.msg import *
from sensor_msgs.msg import JointState

def get_ik_client(x, y, z):
    rospy.wait_for_service('/uarm_ik_service/get_ik')
    try:
        get_ik_f = rospy.ServiceProxy('/uarm_ik_service/get_ik', GetUarmIKSolver)
        current_joints = UarmJointsState()
	current_joints.joint[0] = 0
        current_joints.joint[1] = 0
        current_joints.joint[2] = 0
        target_position = UarmPositionState()
        target_position.x = x
        target_position.y = y
        target_position.z = z
        resp1 = get_ik_f(current_joints, target_position)
	print "Status: %s"%resp1
        pub = rospy.Publisher('uarm/uarm_joint_publisher', JointState, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        msg = JointState()
        msg.name = ["base_body_j","body_upper_arm_j","forearm_wrist_j","wrist_palm_j", "fingers_j"]
        msg.position = [resp1.target_joints.joint[0], resp1.target_joints.joint[1], resp1.target_joints.joint[2]-resp1.target_joints.joint[1], 0, 0]
        msg.velocity = [0, 0, 0, 0, 0]
        msg.effort = [0, 0, 0, 0, 0]

        #msg.name.append('test')
        #msg.position.append(0.0)
        #msg.velocity.append(0.0)
        #msg.effort.append(0.0)
        msg.header.frame_id = 'base_link'

        while not rospy.is_shutdown():
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            rate.sleep()
        return resp1.target_joints
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y z]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s, %s, %s"%(x, y, z)
    get_ik_client(x, y, z)

