/*
 * Laurent LEQUIEVRE
 * Research Engineer, CNRS (France)
 * laurent.lequievre@uca.fr
 * UMR 6602 - Institut Pascal
 * Alberto DESTRI
 * Master Robotic Student
 * albydex93@gmail.com
 * 
 */

#ifndef JOINT_GROUP_POSITION_CONTROLLER_H
#define JOINT_GROUP_POSITION_CONTROLLER_H

// ROS
#include <controller_interface/controller.h>
#include <urdf/model.h>

// hardware_interface 
#include <hardware_interface/joint_command_interface.h> // contains definition of PositionJointInterface

// msgs Float64MultiArray
#include <std_msgs/Float64MultiArray.h>

// KDL
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarrayacc.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

// Boost
#include <boost/scoped_ptr.hpp>

#define TRACE_ACTIVATED 1

namespace robot_controllers
{
	class CartesianVelocityControl: public controller_interface::Controller<hardware_interface::PositionJointInterface>
	{
		public:
			CartesianVelocityControl();
			~CartesianVelocityControl();
			
			bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n); // Init the controller
			void starting(const ros::Time& time);  // Start the controller
			void stopping(const ros::Time& time);  // Stop the controller
			void update(const ros::Time& time, const ros::Duration& period);  // Update the controller
			
		private:
			ros::NodeHandle nh_;
			int cmd_flag_;  // flag set only to 1 when the controller receive a message to the command topic
			
			std::string robot_namespace_;
			
			// configuration
			int n_joints_ = 4; // the robot arduino has 4 joints
			std::vector<std::string> joint_names_; // vector of joints names : (joint1, joint2, ...)
			
			std::vector<hardware_interface::PositionJointInterface::ResourceHandleType> joint_handles_;
			
			void commandCB_(const std_msgs::Float64MultiArrayConstPtr& msg); // function associate to a subscribe command topic
			
			ros::Subscriber sub_command_;	
			std::vector<double> commands_buffer_;	// the vector of desired joint values

			KDL::Chain kdl_chain_;	// KDL chain construct from URDF
			KDL::JntArrayAcc joint_msr_states_, joint_des_states_;  // joint states (measured and desired)s
			
			boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
			boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
			boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
			boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;
			
			KDL::Frame x_;		//current pose
			KDL::Frame x_des_;	//desired pose

			struct limits_
			{
				KDL::JntArray min;
				KDL::JntArray max;
				KDL::JntArray center;
			} joint_limits_; // KDL structures to store limits min, max, center of each joint

			bool initKDLChain_();
			
			ros::Time last_time, current_time;
	};
}

#endif
