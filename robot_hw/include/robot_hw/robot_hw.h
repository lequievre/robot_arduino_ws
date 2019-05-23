/*
 * Laurent LEQUIEVRE
 * Research Engineer, CNRS (France)
 * laurent.lequievre@uca.fr
 * UMR 6602 - Institut Pascal
 * 
*/

#ifndef ROBOT_HW_H
#define ROBOT_HW_H

// ROS controls HW Interfaces
#include <hardware_interface/robot_hw.h>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

// ROS headers
#include <urdf/model.h>
#include <ros/callback_queue.h>

// Joint State Robot Arduino
#include <sensor_msgs/JointState.h>

// Joint Arm Arduino cmd
#include <std_msgs/Float32MultiArray.h>

namespace robot_hw
{
	
	class RobotHW : public hardware_interface::RobotHW
	{
		public:
			RobotHW();
			virtual ~RobotHW();
			
			bool init();
			void create(std::string urdf_string, const ros::NodeHandle& nh);
			
			void read(ros::Time time, ros::Duration period);  // read 'measurement' joint values
			void write(ros::Time time, ros::Duration period);	// write 'cmd' joint cmd values

		private:
		
			int cpt_ = 0;
			// configuration
			int n_joints_ = 7; // the arduino robot has 6 joints (wheel right, wheel left, 4 joints for the arm + 1 joint for the gripper)
			std::vector<std::string> joint_names_; // vector of joints names : (wheel_right, wheel_left, joint1, ...)
			
			ros::Subscriber	joint_state_arduino_sub_;
			ros::Publisher  joint_cmd_pos_arduino_pub_;
			
			// state and commands
			std::vector<double>
			joint_position_,
			joint_velocity_,
			joint_effort_,
			joint_position_command_,
			joint_velocity_command_,
			joint_effort_command_,
			joint_recv_position,
			joint_recv_velocity,
			joint_recv_effort;
			
			// Hardware interfaces
			hardware_interface::JointStateInterface joint_state_interface_;
			hardware_interface::EffortJointInterface joint_effort_interface_;
			hardware_interface::PositionJointInterface joint_position_interface_;
			
			
			ros::NodeHandle nh_;
			
			ros::CallbackQueue subscriber_queue_;
			
			boost::shared_ptr<ros::AsyncSpinner> subscriber_spinner_;
			
			// Register all interfaces necessary
			void registerInterfaces_();
			
			void jointStateArduinoCallback_(const sensor_msgs::JointStateConstPtr& joint_state);
		
	};
}

#endif
