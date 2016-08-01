

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <eigen3/Eigen/Core>
#include <kdl_parser/kdl_parser.hpp>
#include "chainiksolvervel_pinv.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include "hp_chainiksolverpos_nr_jl.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "uarm_kinematics");
	KDL::Tree tree;
	KDL::Chain chain;
	ros::NodeHandle node;

	KDL::ChainFkSolverPos_recursive* fk_solver;
	KDL::HP_ChainIkSolverPos_NR_JL *ik_solver_pos;
	KDL::ChainIkSolverVel_pinv* ik_solver_vel;

	std::string robot_desc_string;
	node.param("robot_description", robot_desc_string, std::string("robot_description"));
	if (!kdl_parser::treeFromString(robot_desc_string, tree)){
		ROS_ERROR("Failed to construct kdl tree");
		return false;
		}

	if (!tree.getChain("uarm_base_link", "uarm_wrist_link", chain)) {
		ROS_ERROR("Failed to construct kdl chain");
		return false;
	   }
	ROS_INFO("Construct kdl chain");

		unsigned int nj = chain.getNrOfJoints();
		unsigned int js = chain.getNrOfSegments();
		std_msgs::String msg;
		std::stringstream ss;
		ss << "# J: " << nj << "  # S: " << js;
		msg.data = ss.str();
		ROS_INFO("Construct kdl tree");
		ROS_INFO("%s", msg.data.c_str()) ;

	fk_solver = new KDL::ChainFkSolverPos_recursive(chain);
	ik_solver_vel = new KDL::ChainIkSolverVel_pinv(chain);
//	Eigen::MatrixXd matrix_Mx = Eigen::MatrixXd::Identity(6,6);
//	matrix_Mx(3,3) = 0; matrix_Mx(4,4) = 0; matrix_Mx(5,5) = 0;
//	ik_solver_vel -> setWeightTS(matrix_Mx);
	KDL::JntArray joint_min(chain.getNrOfJoints());
	KDL::JntArray joint_max(chain.getNrOfJoints());
	joint_min(0) = -1.57;
	joint_min(1) = -1.57;
	joint_min(2) = -1.57;
	joint_max (0) = 1.57;
	joint_max (1) = 1.57;
	joint_max (2) = 1.57;
	ik_solver_pos = new KDL::HP_ChainIkSolverPos_NR_JL (chain, joint_min, joint_max,
					*fk_solver, *ik_solver_vel, 20, 0.0001);

	KDL::JntArray q_init(chain.getNrOfJoints());
	q_init (0) = 0;
	q_init (1) = 0;
	q_init (2) = 0;
	KDL::JntArray q_out(chain.getNrOfJoints());

	for(int seg_num = 0; seg_num < chain.getNrOfSegments(); seg_num++) {
	    KDL::Segment my_seg = chain.getSegment(seg_num);
	    KDL::Frame my_fr;

	    if (fk_solver->JntToCart(q_init, my_fr) >= 0){

			std_msgs::String msg;
			std::stringstream ss;
			ss << "Seg: "<< seg_num <<"# 0: " << my_fr.p[0] << "  # 1: " << my_fr.p[1] << "  # 2: " << my_fr.p[2];
			msg.data = ss.str();
			ROS_INFO("Construct kdl tree");
			ROS_INFO("%s", msg.data.c_str()) ;
	    }
        }

	double x = 0.085, y = 0.0, z = 0.0133;//x = 0.04, y = 0.063, z = 0.013;//x = 0.076, y = 0.0, z = 0.013;  
	KDL::Frame p_in (KDL::Vector(x, y, z));

	int ik_valid = ik_solver_pos -> CartToJnt(q_init, p_in, q_out);
	if (ik_valid >= 0){
		ROS_INFO("\nJoint 1: %f\nJoint 2: %f\nJoint 3: %f\n", q_out(0),q_out(1),q_out(2));
	}
	else {
		ROS_ERROR("IK not found");
	}

	return 0;
}
