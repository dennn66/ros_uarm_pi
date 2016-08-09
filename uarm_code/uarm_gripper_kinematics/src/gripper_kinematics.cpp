#include "gripper_kinematics.hpp"


Gripper::Gripper(){

        void chatterGripperPosition (const uarm_msgs::GripperPositionConstPtr &gripper_pos);
        void chatterGripperState (const std_msgs::BoolConstPtr &gripper_state);

	gripperState = 1;
	angleR = 0;
	angleL = 0;
	armRot = 0;
	handRot = 0;

	sub_gripper_position = node.subscribe("uarm/set_gripper_position", 10, &Gripper::chatterGripperPosition, this);
        sub_gripper_state = node.subscribe("uarm/set_gripper_state", 10, &Gripper::chatterGripperState, this);

        joint_msg_pub = node.advertise<sensor_msgs::JointState>("uarm/target_joint_positions", 10);

	ros::Duration(1).sleep(); // optional, to make sure no message gets lost
	ROS_INFO("Servo controller is ready...");
}

void Gripper::chatterGripperPosition (const uarm_msgs::GripperPositionConstPtr &gripper_pos){
        ROS_INFO("Recieved uarm position");
	setPosition(gripper_pos->stretch, gripper_pos->height, gripper_pos->arm_rot, gripper_pos->hand_rot);
}

void Gripper::chatterGripperState (const std_msgs::BoolConstPtr &gripper_state){
    if(gripper_state->data)
    {
	ROS_INFO("ServoGripper closed");
    	gripperState = true;
    } else {
	ROS_INFO("ServoGripper openned");
    	gripperState = false;
    }
    publishJoints();
}

void Gripper::setPosition(double _stretch, double _height, double _armRot, double _handRot)
{

	armRot = -_armRot;
	armRot  = constrain(armRot,  ARM_ROTATION_MIN,  ARM_ROTATION_MAX);
	handRot = constrain(_handRot, HAND_ROTATION_MIN, HAND_ROTATION_MAX);
        _stretch = constrain(_stretch, ARM_STRETCH_MIN,   ARM_STRETCH_MAX) + 30 - 35;                // +55, set zero -stretch
        _height  = constrain(_height,  ARM_HEIGHT_MIN,    ARM_HEIGHT_MAX);                      // + gripper height
        ROS_INFO(" _stretch %f",  _stretch);
        ROS_INFO(" _height %f",  _height);

	// angle calculation
	double stretch2height2 = _stretch * _stretch + _height * _height;              // 
        ROS_INFO("sqrt(stretch2height2)  %f mm", sqrt(stretch2height2));

	double angleA = (acos( (ARM_A2B2 - stretch2height2) / ARM_2AB )); // angle between the upper and the lower
        ROS_INFO("angleA rad %f", angleA);

	double angleB = (atan(_height/_stretch)) ;                         // 
        ROS_INFO("angleB rad %f", angleB );

	double angleC = acos((ARM_A2 + stretch2height2 -ARM_B2)/(2 * ARM_A * sqrt(stretch2height2))); // 
        ROS_INFO(" angleC  rad %f", angleC );

	angleR = 3.141592654  - angleA - angleB - angleC;        // 
        ROS_INFO(" angleR  rad %f",  angleR);

	angleL = angleB + angleC;                       // 
        ROS_INFO(" angleL  rad %f",  angleL);

        publishJoints();
}


void Gripper::publishJoints()
{

    sensor_msgs::JointState joint_msg;

    const std::string  joints[5] = {"base_body_j",
        "body_upper_arm_j",
        "forearm_wrist_j",
        "wrist_palm_j",
        "fingers_j"
    };

    double servoR =  (angleR + FIXED_OFFSET_R - 1.570796327);        //
    ROS_INFO(" servoR  rad %f",  servoR);

    double servoL =  (angleL + FIXED_OFFSET_L - 1.570796327);                       //
    ROS_INFO(" servoL  rad %f",  servoL);

    positions[0] = armRot; //"base_body_j"
    positions[1] = servoL; //"body_upper_arm_j",
    positions[2] = servoR; //"forearm_wrist_j"
    positions[3] = handRot; //"wrist_palm_j"
    if(gripperState)
    {
        positions[4] = HAND_ANGLE_CLOSE; //"fingers_j"

    } else {
        positions[4] = HAND_ANGLE_OPEN;  //"fingers_j"
    }

    joint_msg.header.stamp = ros::Time::now();

    for (int name=0; name<5; name++){
        joint_msg.name.push_back(joints[name].c_str());
	joint_msg.position.push_back(positions[name]);
        joint_msg.velocity.push_back(0.05);
    }
    joint_msg_pub.publish(joint_msg);
    joint_msg.name.clear();
    joint_msg.position.clear();
    joint_msg.velocity.clear();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "uarm_gripper_kinematics");
    Gripper g;
    ros::spin();
}


