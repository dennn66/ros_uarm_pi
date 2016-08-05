#include "ros/ros.h"
#include "uarm_msgs/Joints.h"
#include <sensor_msgs/JointState.h>

typedef boost::shared_ptr<uarm_msgs::Joints const> JointsStateConstPtr;

float positions[5];

const std::string  joints[5] = {"base_body_j",
"body_upper_arm_j",
"forearm_wrist_j",
"wrist_palm_j",
"fingers_j"
};

ros::Publisher joint_msg_pub;
sensor_msgs::JointState joint_msg;

void chatterJointsState (const JointsStateConstPtr& state){

// rostopic pub /uarm/target_position uarm_msgs/Joints '{angle_r: 0.581789, angle_l: 1.941941, angle_rot: 0.0, 
// angle_hand_rot: 0.0, angle_grip: 0.0}' --once



	positions[0] = state->angle_rot; //"base_body_j"
        positions[1] = state->angle_l; //"body_upper_arm_j",
        positions[2] = state->angle_r; //"forearm_wrist_j"
        positions[3] = state->angle_hand_rot; //"wrist_palm_j"
        positions[4] = state->angle_grip; //"fingers_j"

	joint_msg.header.stamp = ros::Time::now();

	for (int name=0; name<5; name++){
			joint_msg.name.push_back(joints[name].c_str());
			joint_msg.position.push_back(positions[name]);
	}
	joint_msg_pub.publish(joint_msg);
	joint_msg.name.clear();
	joint_msg.position.clear();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "uarm_joint_publisher");
	ros::NodeHandle node;

	joint_msg_pub = node.advertise<sensor_msgs::JointState>("uarm/uarm_joint_publisher", 1);
//	ros::Rate loop_rate(20);

	ros::Subscriber sub = node.subscribe("uarm/target_position", 1, chatterJointsState);

	ros::spin();

	return 0;
}
