#include "controller.hpp"


Controller::Controller(){

	//Servo channels
    node.param("servoRot", channels[SERVO_ROT], ROT_PORT);
    node.param("servoL", channels[SERVO_L], L_PORT);
    node.param("servoR", channels[SERVO_R], R_PORT);
    node.param("servoHandRot", channels[SERVO_HAND_ROT], HAND_ROT_PORT);
    node.param("servoHand", channels[SERVO_HAND], HAND_PORT);
    node.param("controllerRate", rate, RATE);


    servo_type[SERVO_ROT] =  pca9685_msgs::ServoState::D150A;
    servo_type[SERVO_L] =  pca9685_msgs::ServoState::D150A;
    servo_type[SERVO_R] =  pca9685_msgs::ServoState::D150A;
    servo_type[SERVO_HAND_ROT] =  pca9685_msgs::ServoState::D009A;
    servo_type[SERVO_HAND] =  pca9685_msgs::ServoState::D009A;

    servo_needs_hold[SERVO_ROT] =  0;
    servo_needs_hold[SERVO_L] =  1;
    servo_needs_hold[SERVO_R] =  1;
    servo_needs_hold[SERVO_HAND_ROT] =  0;
    servo_needs_hold[SERVO_HAND] =  0;

    node.param("offsetRot",  servo_offset[SERVO_ROT], 0.0);
    node.param("offsetL",    servo_offset[SERVO_L], 0.33);
    node.param("offsetR",    servo_offset[SERVO_R], -0.20);
    node.param("offsetHand", servo_offset[SERVO_HAND_ROT], 0.0);
    node.param("offsetGrip", servo_offset[SERVO_HAND], 0.0);

    current_position[SERVO_ROT] = 0;
    current_position[SERVO_L] = SERVO_L_INIT;
    current_position[SERVO_R] = SERVO_R_INIT;
    current_position[SERVO_HAND_ROT] = 0;
    current_position[SERVO_HAND] = HAND_ANGLE_OPEN;

    init_position[SERVO_ROT] = 0;
    init_position[SERVO_L] = SERVO_L_INIT;
    init_position[SERVO_R] = SERVO_R_INIT;
    init_position[SERVO_HAND_ROT] = 0;
    init_position[SERVO_HAND] = HAND_ANGLE_OPEN;

    target_position[SERVO_ROT] = 0;
    target_position[SERVO_L] = SERVO_L_INIT;
    target_position[SERVO_R] = SERVO_R_INIT;
    target_position[SERVO_HAND_ROT] = 0;
    target_position[SERVO_HAND] = HAND_ANGLE_OPEN;

    target_velocity[SERVO_ROT] = 0.1;
    target_velocity[SERVO_L] = 0.1;
    target_velocity[SERVO_R] = 0.1;
    target_velocity[SERVO_HAND_ROT] = 0.1;
    target_velocity[SERVO_HAND] = 0.1;


    sub_joint_positions = node.subscribe("uarm/target_joint_positions", 100, &Controller::chatterTargetJoints, this);

    joint_msg_pub = node.advertise<sensor_msgs::JointState>("uarm/uarm_joint_publisher", 1);

    pub_servo_position = node.advertise<pca9685_msgs::ServoState>("pca9685/servostate_to_controller", 100);
    ros::Duration(1).sleep(); // optional, to make sure no message gets lost
    ROS_INFO("Servo controller is ready...");
}

void Controller::chatterTargetJoints (const sensor_msgs::JointStatePtr &msg){
    ROS_INFO("Recieved uarm position");

    const std::string  joints[ N_CHANNELS] = {
        "base_body_j",
        "body_upper_arm_j",
        "forearm_wrist_j",
        "wrist_palm_j",
        "fingers_j"
    };

    for( int i = 0; i < msg->name.size(); i++)
    {
        for( int j = 0; j < N_CHANNELS; j++)
        {
            if(msg->name[i] == joints[j])
            {
                target_position[j] = msg->position[i];
                target_velocity[j] = msg->velocity[i];
            }
        }
    }
}


double Controller::getDelta(double current_pos, double target_pos,  double maxstep){
    double step;

    step = target_pos - current_pos;
    if((step > -ANGLE_PRECISION) && (step < ANGLE_PRECISION)) step = 0; 
    //ROS_INFO("target_pos [%f] current_pos[%f] step[%f] maxstep[%f] ", target_pos, current_pos, step, maxstep);

    step = constrain(step,-maxstep,maxstep);
    //ROS_INFO("step[%f] ", step);

    return(step);
}


void Controller::controller (){

    const std::string  joints[N_CHANNELS] = {
        "base_body_j",
        "body_upper_arm_j",
        "forearm_wrist_j",
        "wrist_palm_j",
        "fingers_j"
    };
    sensor_msgs::JointState joint_msg;

    pca9685_msgs::ServoState msg;
    static ros::Time last_time =  ros::Time::now();
    double dt = 0;

    while (node.ok()){
        dt = (ros::Time::now() - last_time).toSec();
        last_time = ros::Time::now();

        for(int port = 0; port < N_CHANNELS; port++){
            msg.port_num = channels[port];  
            double delta;
            delta = getDelta(current_position[port], target_position[port], target_velocity[port]*dt);
            current_position[port] += delta;
            if(delta != 0) {
                msg.servo_rot = current_position[port]+servo_offset[port];
                msg.servo_type =  servo_type[port];
            } else {
                if(servo_needs_hold[port] == 0 || 
                     ((current_position[port]-init_position[port] > -ANGLE_PRECISION )&&
                           (current_position[port]-init_position[port] < ANGLE_PRECISION))) {
                    msg.servo_rot = current_position[port]+servo_offset[port];
                    msg.servo_type =  0;  // STOP SERVO
                } else {
                    msg.servo_rot = current_position[port]+servo_offset[port];
                    msg.servo_type =  servo_type[port];
                }
            }
            //ROS_INFO("Servo %d [%f] - %d",  msg.port_num, msg.servo_rot, msg.servo_type);
            pub_servo_position.publish(msg);
        }	

        joint_msg.header.stamp = ros::Time::now();
        for (int name=0; name < N_CHANNELS; name++){
            joint_msg.name.push_back(joints[name].c_str());
            joint_msg.position.push_back(current_position[name]);
        }
        joint_msg_pub.publish(joint_msg);
        joint_msg.name.clear();
        joint_msg.position.clear();

        ros::spinOnce();
        ros::Rate(rate).sleep();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "uarm_controller");
    Controller c;
    c.controller();
    ros::spin();
}


