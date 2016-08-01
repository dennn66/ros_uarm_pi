
#ifndef UARM_IK_SERVICE_HPP_
#define UARM_IK_SERVICE_HPP_

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include "chainiksolvervel_pinv.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include "hp_chainiksolverpos_nr_jl.hpp"
#include <uarm_msgs/GetUarmIKSolver.h>

#define NUM_JOINTS 3

class UarmKinematics {
	public:
		UarmKinematics();
		bool init();
	private:
		ros::NodeHandle node, node_private;
		std::string root_name, tip_name;
		double joint_lower_limit, joint_upper_limit;
		const static unsigned int num_joints = NUM_JOINTS;

		KDL::Chain* chains_ptr;
		KDL::JntArray joint_min, joint_max;
		KDL::ChainFkSolverPos_recursive* fk_solver;
		KDL::HP_ChainIkSolverPos_NR_JL* ik_solver_pos;
		KDL::ChainIkSolverVel_pinv* ik_solver_vel;

		ros::ServiceServer ik_service;

		bool loadModel(const std::string xml);
		bool getUarmIKSolver (	uarm_msgs::GetUarmIKSolver::Request &request,
								uarm_msgs::GetUarmIKSolver::Response &response);


};



#endif /* UARM_IK_SERVICE_HPP_ */
