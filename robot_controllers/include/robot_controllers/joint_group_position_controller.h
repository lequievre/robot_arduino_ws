/*
 * Laurent LEQUIEVRE
 * Research Engineer, CNRS (France)
 * laurent.lequievre@uca.fr
 * UMR 6602 - Institut Pascal
 * 
 */

#ifndef JOINT_GROUP_POSITION_CONTROLLER_H
#define JOINT_GROUP_POSITION_CONTROLLER_H

// ROS
#include <controller_interface/controller.h>

// hardware_interface 
#include <hardware_interface/joint_command_interface.h> // contains definition of PositionJointInterface

// msgs Float64MultiArray
#include <std_msgs/Float64MultiArray.h>

#define TRACE_ACTIVATED 1

namespace robot_controllers
{
	class JointGroupPosition: public controller_interface::Controller<hardware_interface::PositionJointInterface>
	{
		public:
			JointGroupPosition();
			~JointGroupPosition();
			
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
	};
}

#endif
