#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_



#include <ros/ros.h>
#include <pca9685_msgs/ServoState.h>
#include <uarm_msgs/Joints.h>

#define ARM_ROTATION_MIN        -1.570796327
#define ARM_ROTATION_MAX        1.570796327
#define HAND_ROTATION_MIN       -1.570796327
#define HAND_ROTATION_MAX       1.570796327
#define HAND_ANGLE_OPEN         -1.134464014
#define HAND_ANGLE_CLOSE        -0.349065851
#define ANGLE_PRECISION        0.001
#define ANGLE_STEP        0.001

/*****************  Port definitions  *****************/
#define SERVO_R        1
#define SERVO_L        2
#define SERVO_ROT      3
#define SERVO_HAND     4
#define SERVO_HAND_ROT 5 

#define N_CHANNELS 5

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

class Controller {
	public:
		Controller();
		void controller();

	private:

		ros::NodeHandle node;
		ros::Subscriber sub_joints_position;

		ros::Publisher pub_servo_position;

		int channels[N_CHANNELS];
                double current_position[N_CHANNELS];
                double target_position[N_CHANNELS];
                int servo_type [N_CHANNELS];
                int servo_needs_hold [N_CHANNELS];
                int servo_hold [N_CHANNELS];

                void chatterJointsState (const uarm_msgs::JointsConstPtr &uarm_jnts);
		double getDelta(double current_pos, double target_pos);

/*******************  Servo offset  *******************/
        double offsetL;
        double offsetR;
};



#endif /* CONTROLLER_HPP_ */
