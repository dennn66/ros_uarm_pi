#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_



#include <ros/ros.h>
#include <pca9685_msgs/ServoState.h>
#include <uarm_msgs/Joints.h>
#include <sensor_msgs/JointState.h>

#define ARM_ROTATION_MIN        -1.570796327
#define ARM_ROTATION_MAX        1.570796327
#define HAND_ROTATION_MIN       -1.570796327
#define HAND_ROTATION_MAX       1.570796327
#define HAND_ANGLE_OPEN         -1.134464014
#define HAND_ANGLE_CLOSE        -0.349065851
#define ANGLE_PRECISION        0.01
#define SERVO_L_INIT            1.141
#define SERVO_R_INIT           -0.67


/*****************  Port definitions  *****************/
#define R_PORT        1
#define L_PORT        2
#define ROT_PORT      3
#define HAND_ROT_PORT 5 
#define HAND_PORT     4

#define SERVO_ROT      0
#define SERVO_L        1
#define SERVO_R        2
#define SERVO_HAND_ROT 3
#define SERVO_HAND     4

#define RATE 25

#define N_CHANNELS 5

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

class Controller {
	public:
		Controller();
		void controller();

	private:

		ros::NodeHandle node;
                ros::Subscriber sub_joint_positions;

		ros::Publisher pub_servo_position;
                ros::Publisher joint_msg_pub;

		int channels[N_CHANNELS];
                double current_position[N_CHANNELS];
                double target_position[N_CHANNELS];
                double target_velocity[N_CHANNELS];
                double init_position[N_CHANNELS];
                int servo_type [N_CHANNELS];
                double servo_offset [N_CHANNELS];
                 int servo_needs_hold [N_CHANNELS];
                int servo_hold [N_CHANNELS];

                void chatterTargetJoints (const sensor_msgs::JointStatePtr &target_jnts);
		double getDelta(double current_pos, double target_pos, double target_vel);

/*******************  Servo offset  *******************/
        double offsetL;
        double offsetR;
        int rate;



};



#endif /* CONTROLLER_HPP_ */
