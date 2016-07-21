#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_



#include <ros/ros.h>
#include <pca9685_msgs/ServoState.h>
#include <uarm_msgs/Joints.h>

/****************  Macro definitions  ****************/
#define FIXED_OFFSET_L          0.314159265
#define FIXED_OFFSET_R          0.628318531

#define INIT_POS_L              0.645771823
#define INIT_POS_R              0.436332313
#define CATCH					0x01
#define RELEASE					0x02

/*****************  Port definitions  *****************/
#define SERVO_R        2
#define SERVO_L        1
#define SERVO_ROT      3
#define SERVO_HAND     4
#define SERVO_HAND_ROT 5 

#define N_CHANNELS 5

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

class Controller {
	public:
		Controller();

	private:

		ros::NodeHandle node;
		ros::Subscriber sub_joints_position;

		ros::Publisher pub_servo_position;

		int channels[N_CHANNELS];
                double position[N_CHANNELS];
                int servo_type [N_CHANNELS];

                void chatterJointsState (const uarm_msgs::JointsConstPtr &uarm_jnts);

/*******************  Servo offset  *******************/
	double offsetL;
	double offsetR;
	/*****************  Define variables  *****************/
};



#endif /* CONTROLLER_HPP_ */
