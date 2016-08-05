

#ifndef GRIPPER_KINEMATICS_HPP_
#define GRIPPER_KINEMATICS_HPP_



#include <ros/ros.h>
#include <uarm_msgs/GripperPosition.h>
#include <uarm_msgs/Joints.h>
#include <std_msgs/Bool.h>

/****************  Macro definitions  ****************/
#define ARM_A                   148    // upper arm
#define ARM_B                   160    // lower arm
#define ARM_2AB                 47360  // 2*A*B
#define ARM_A2                  21904  // A^2
#define ARM_B2                  25600  // B^2
#define ARM_A2B2                47504  // A^2 + B^2
#define ARM_STRETCH_MIN         0
#define ARM_STRETCH_MAX         210
#define ARM_HEIGHT_MIN          -180
#define ARM_HEIGHT_MAX          150
#define ARM_ROTATION_MIN        -1.570796327
#define ARM_ROTATION_MAX        1.570796327
#define HAND_ROTATION_MIN       -1.570796327
#define HAND_ROTATION_MAX       1.570796327
#define HAND_ANGLE_OPEN         -1.134464014
#define HAND_ANGLE_CLOSE        -0.349065851

#define FIXED_OFFSET_L          0.314159265
#define FIXED_OFFSET_R          0.628318531

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

class Gripper {
	public:
		Gripper();

	private:

		ros::NodeHandle node;
		ros::Subscriber sub_gripper_position;
                ros::Subscriber sub_gripper_state;

		ros::Publisher pub_joints_position;

                void chatterGripperPosition (const uarm_msgs::GripperPositionConstPtr &gripper_pos);
                void chatterGripperState (const std_msgs::BoolConstPtr &gripper_state);
                void setPosition(double _stretch, double _height, double _armRot, double _handRot); // 
		void publishJoints();    //

	/*****************  Define variables  *****************/

	bool gripperState;
        double angleR;
        double angleL;
        double armRot;
        double handRot;
};



#endif /* GRIPPER_KINEMATICS_HPP_ */
