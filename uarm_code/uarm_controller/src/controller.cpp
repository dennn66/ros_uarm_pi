#include "controller.hpp"


Controller::Controller(){

	//Servo channels
	node.param("servoR", channels[0], SERVO_R);
	node.param("servoL", channels[1], SERVO_L);
	node.param("servoRot", channels[2], SERVO_ROT);
	node.param("servoHandRot", channels[3], SERVO_HAND_ROT);
        node.param("servoHand", channels[4], SERVO_HAND);
 
    servo_type[0] =  pca9685_msgs::ServoState::D150A;
    servo_type[1] =  pca9685_msgs::ServoState::D150A;
    servo_type[2] =  pca9685_msgs::ServoState::D150A;
    servo_type[3] =  pca9685_msgs::ServoState::D009A;
    servo_type[4] =  pca9685_msgs::ServoState::D009A;


        node.param("offsetL", offsetL, 0.314696417);
        node.param("offsetR", offsetR, -0.139311164);

	sub_joints_position = node.subscribe("uarm/joints_to_controller", 100, &Controller::chatterJointsState, this);

	pub_servo_position = node.advertise<pca9685_msgs::ServoState>("pca9685/servostate_to_controller", 100);
	ros::Duration(1).sleep(); // optional, to make sure no message gets lost
	ROS_INFO("Servo controller is ready...");
}

void Controller::chatterJointsState (const uarm_msgs::JointsConstPtr &uarm_jnts){
    ROS_INFO("Recieved uarm position");
    pca9685_msgs::ServoState msg;

    position[0] = uarm_jnts->angle_r+offsetR;
    position[1] = uarm_jnts->angle_l+offsetL;
    position[2] = uarm_jnts->angle_rot;
    position[3] = uarm_jnts->angle_hand_rot;
    position[4] = uarm_jnts->angle_grip;
    
 
//    for(int port = 0; port < N_CHANNELS; port++){
    for(int port = 0; port < 2; port++){
        msg.port_num = channels[port];  //SERVO_ROT
        msg.servo_rot = position[port];
        msg.servo_type =  servo_type[port];
        ROS_INFO("Servo %d [%f]",  channels[port], position[port]);
        pub_servo_position.publish(msg);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    Controller c;
    ros::spin();
}


