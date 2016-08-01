#include "uarm_ik_service.hpp"


UarmKinematics::UarmKinematics():	node_private("~"){}

bool UarmKinematics::init() {
	 std::string robot_desc_string;
    // Get URDF XML
    if (!node.getParam("robot_description", robot_desc_string)) {
           ROS_FATAL("Could not load the xml from parameter: robot_description");
           return false;
    }

    // Get Root and Tip From Parameter Server
    node_private.param("root_name", root_name, std::string("uarm_base_link"));
    node_private.param("tip_name", tip_name, std::string("uarm_wrist_link"));

    // Load and Read Models
    if (!loadModel(robot_desc_string)) {
        ROS_FATAL("Could not load models!");
        return false;
    }

    // Get Min and Max joints limits
    node.param("joint_lower_limit", joint_lower_limit, -(KDL::PI/2));
    node.param("joint_upper_limit", joint_upper_limit, KDL::PI/2);
    joint_min.resize(num_joints);
    joint_max.resize(num_joints);
    for (unsigned int i=0; i<num_joints; i++){
    	joint_min(i) = joint_lower_limit;
    	joint_max(i) = joint_upper_limit;
    }
    // Get Solver Parameters
    int maxIterations;
    double epsilon;

    node_private.param("maxIterations", maxIterations, 100);
    node_private.param("epsilon", epsilon, 1e-3);

    // Build Solvers
    fk_solver = new KDL::ChainFkSolverPos_recursive(*chains_ptr);
    ik_solver_vel = new KDL::ChainIkSolverVel_pinv(*chains_ptr);
    ik_solver_pos = new KDL::HP_ChainIkSolverPos_NR_JL(*chains_ptr, joint_min, joint_max,
				  *fk_solver, *ik_solver_vel, maxIterations, epsilon);

	ROS_INFO("Advertising service");
	ik_service = node_private.advertiseService("get_ik",&UarmKinematics::getUarmIKSolver,this);
	ROS_INFO("Ready to client's request...");
	return true;
}

bool UarmKinematics::loadModel(const std::string xml) {
    KDL::Tree tree;
    KDL::Chain chain;
    std::string tip_name_result;

    if (!kdl_parser::treeFromString(xml, tree)) {
        ROS_ERROR("Could not initialize tree object");
        return false;
    }
    ROS_INFO("Construct tree");

    if (!tree.getChain(root_name, tip_name, chain)) {
	ROS_ERROR("Could not initialize chain object");
	return false;
    }
    chains_ptr = new KDL::Chain(chain);
    ROS_INFO("Construct chains");

    return true;
}

bool UarmKinematics::getUarmIKSolver (	uarm_msgs::GetUarmIKSolver::Request &request,
					uarm_msgs::GetUarmIKSolver::Response &response){

	uarm_msgs::UarmPositionState dest_pos;
	//response.target_joints.clear();

	dest_pos = request.target_position;
	KDL::JntArray jnt_pos_in(num_joints);
	KDL::JntArray jnt_pos_out(num_joints);

	//Get initial joints and frame
	for (unsigned int j=0; j < num_joints; j++) {
		jnt_pos_in(j) = request.current_joints.joint[j];
	}
	KDL::Frame F_dest (KDL::Vector(dest_pos.x, dest_pos.y, dest_pos.z));

	//IK solver
	int ik_valid = ik_solver_pos->CartToJnt(jnt_pos_in, F_dest, jnt_pos_out);

	//ROS_ERROR("---: LEG_IK_SOLVER: %i", ik_valid);
	if (ik_valid >= 0) {
		uarm_msgs::UarmJointsState jnt_buf;
		for (unsigned int j=0; j<num_joints; j++) {
				jnt_buf.joint[j] = jnt_pos_out(j);
		}
		response.target_joints = jnt_buf;
		response.error_codes = response.IK_FOUND;
		ROS_INFO("IK Solution found");
	}
	else {
		response.error_codes = response.IK_NOT_FOUND;
		ROS_ERROR("An IK solution could not be found");
		return true;
	}
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "uarm_ik_service");
	UarmKinematics k;
    if (k.init()<0) {
        ROS_ERROR("Could not initialize kinematics node");
        return -1;
    }

    ros::spin();
    return 0;
}
