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

    servo_needs_hold[0] =  1;
    servo_needs_hold[1] =  1;
    servo_needs_hold[2] =  0;
    servo_needs_hold[3] =  0;
    servo_needs_hold[4] =  0;
        node.param("offsetL", offsetL, 0.33);
        node.param("offsetR", offsetR, -0.20);

	    target_position[0] = current_position[0] = 0+offsetR;
	    target_position[1] = current_position[1] = 0+offsetL;
	    target_position[2] = current_position[2] = 0;
	    target_position[3] = current_position[3] = 0;
	    target_position[4] = current_position[4] = HAND_ANGLE_OPEN;


	sub_joints_position = node.subscribe("uarm/target_position", 100, &Controller::chatterJointsState, this);

	pub_servo_position = node.advertise<pca9685_msgs::ServoState>("pca9685/servostate_to_controller", 100);
	ros::Duration(1).sleep(); // optional, to make sure no message gets lost
	ROS_INFO("Servo controller is ready...");
}

void Controller::chatterJointsState (const uarm_msgs::JointsConstPtr &uarm_jnts){
    ROS_INFO("Recieved uarm position");
    pca9685_msgs::ServoState msg;

    target_position[0] = uarm_jnts->angle_r+offsetR;
    target_position[1] = uarm_jnts->angle_l+offsetL;
    target_position[2] = uarm_jnts->angle_rot;
    target_position[3] = uarm_jnts->angle_hand_rot;
    target_position[4] = uarm_jnts->angle_grip;

}

double Controller::getDelta(double current_pos, double target_pos){
	double step;
	
	//#define ANGLE_PRECISION        0.001
        //#define ANGLE_STEP        0.001

	if(current_pos < target_pos){
		if(target_pos - current_pos > ANGLE_PRECISION){
			if(target_pos - current_pos > ANGLE_STEP){
				step = ANGLE_STEP;
			} else {
				step = target_pos - current_pos;
			}
		} else {
				step = 0;
		}
	} else {
		if(current_pos - target_pos > ANGLE_PRECISION){
			if(current_pos - target_pos > ANGLE_STEP){
				step = -ANGLE_STEP;
			} else {
				step = target_pos - current_pos;
			}
		} else {
				step = 0;
		}
	}

	return(step);
}


void Controller::controller (){

    pca9685_msgs::ServoState msg;

	while (node.ok()){
	    for(int port = 0; port < N_CHANNELS; port++){
		msg.port_num = channels[port];  

                double delta;
                delta = getDelta(current_position[port], target_position[port]);
                current_position[port] += delta;
                if(delta != 0) {
	            msg.servo_rot = current_position[port];
		    msg.servo_type =  servo_type[port];
	        } else {
                    if(servo_needs_hold[port] == 0) {
		        msg.servo_rot = current_position[port];
		        msg.servo_type =  0;  // STOP SERVO
                    } else {
	                msg.servo_rot = current_position[port];
		        msg.servo_type =  servo_type[port];
                    }
                }
		ROS_INFO("Servo %d [%f]",  msg.port_num, msg.servo_rot, msg.servo_type);
		pub_servo_position.publish(msg);
	    }
		ros::spinOnce();
		ros::Rate(25).sleep();
	}
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "uarm_controller");
    Controller c;
    c.controller();
    ros::spin();
}


