/*
 * Laurent LEQUIEVRE
 * Research Engineer, CNRS (France)
 * laurent.lequievre@uca.fr
 * UMR 6602 - Institut Pascal
 * 
*/

#include "robot_hw/robot_hw.h"

namespace robot_hw
{
	RobotHW::RobotHW()
	{	
	}

	RobotHW::~RobotHW() 
	{ 
	}
	
	bool RobotHW::init()
	{
		return true;
	}
	
	void RobotHW::read(ros::Time time, ros::Duration period)
	{
		for (unsigned int i=0; i<n_joints_; i++)
		{
			joint_position_[i] = joint_recv_position[i];
			joint_velocity_[i] = joint_recv_velocity[i];
			joint_effort_[i] = joint_recv_effort[i];
		}
	}
	
	void RobotHW::write(ros::Time time, ros::Duration period)
	{
		//joint_position_command_[5] = joint_position_command_[5] + 0.0001;
		
		// create a message Float32MultiArray
		std_msgs::Float32MultiArray msg;
		
		msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
		msg.layout.dim[0].size = 4;
		msg.layout.dim[0].stride = 1;
		msg.layout.dim[0].label = "arm_pos_command";

		// copy in the data
		msg.data.clear();
		msg.data.insert(msg.data.end(), joint_position_command_.begin()+2, joint_position_command_.end());
		
		// publish the command
		joint_cmd_pos_arduino_pub_.publish(msg);
	}
	
	void RobotHW::create(std::string urdf_string, const ros::NodeHandle& nh)
	{
		// Get vector of joint names defined in the file 'robot_hw_interface.yaml'
		joint_names_.clear();
		nh.getParam("joints",joint_names_);
		
		for (size_t i=0; i<joint_names_.size(); ++i)
				ROS_INFO("RobotHW (create) : name of joint %zd = %s", i, joint_names_[i].c_str());
				
		// Resize Vectors
		joint_position_.resize(n_joints_);
		joint_position_command_.resize(n_joints_);
		
		joint_velocity_.resize(n_joints_);
		joint_velocity_command_.resize(n_joints_);
		
		joint_effort_.resize(n_joints_);
		joint_effort_command_.resize(n_joints_);
		
		joint_recv_position.resize(n_joints_);
		joint_recv_velocity.resize(n_joints_);
		joint_recv_effort.resize(n_joints_);
		
		// reset all vectors with default values
		std::fill(joint_position_.begin(), joint_position_.end(), 0.0);
		std::fill(joint_velocity_.begin(), joint_velocity_.end(), 0.0);
		std::fill(joint_effort_.begin(), joint_effort_.end(), 0.0);
		
		std::fill(joint_position_command_.begin(), joint_position_command_.end(), 0.0);  
		std::fill(joint_velocity_command_.begin(), joint_velocity_command_.end(), 0.0);
		std::fill(joint_effort_command_.begin(), joint_effort_command_.end(), 0.0);
		
		std::fill(joint_recv_position.begin(), joint_recv_position.end(), 0.0);
		std::fill(joint_recv_velocity.begin(), joint_recv_velocity.end(), 0.0);
		std::fill(joint_recv_effort.begin(), joint_recv_effort.end(), 0.0);
		
		registerInterfaces_();
		
		// Subscribe to /arduino/joint_states topic
		
		joint_state_arduino_sub_ = nh_.subscribe("/arduino/joint_states", 1, &RobotHW::jointStateArduinoCallback_, this);
		joint_cmd_pos_arduino_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/arduino/cmd_pos", 1);
		
		nh_.setCallbackQueue(&subscriber_queue_);
		
		subscriber_spinner_.reset(new ros::AsyncSpinner(4, &subscriber_queue_));
		subscriber_spinner_->start();
	}
	
	void RobotHW::jointStateArduinoCallback_(const sensor_msgs::JointStateConstPtr& joint_state)
	{
		//cpt_++;
		
		for (unsigned int i=0; i<n_joints_; i++)
		{
			joint_recv_position[i] = joint_state->position[i];
			joint_recv_velocity[i] = joint_state->velocity[i];
			joint_recv_effort[i] = joint_state->effort[i];	
		}
		
		/*if (cpt_ % 100)
		{
			ROS_INFO("RobotHW (jointStateArduinoCallback_) : %f, %f, %f, %f, %f, %f", joint_recv_position[0], joint_recv_position[1], joint_recv_position[2], joint_recv_position[3], joint_recv_position[4], joint_recv_position[5]);
		}*/
	}
	
	// Register all interfaces necessary
	void RobotHW::registerInterfaces_()
	{
		// Initialize values
		for(unsigned int i=0; i < n_joints_; i++)
		{
			joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));
		
			hardware_interface::JointHandle joint_handle_effort;
			joint_handle_effort = hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]),
                                                       &joint_effort_command_[i]);
			joint_effort_interface_.registerHandle(joint_handle_effort);
			
			hardware_interface::JointHandle joint_handle_position;
			joint_handle_position = hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]),
													   &joint_position_command_[i]);
			joint_position_interface_.registerHandle(joint_handle_position);
		
		}
		
		// Register joints state, effort, position interfaces to 'hardware_interface::RobotHW'
		registerInterface(&joint_state_interface_);
		registerInterface(&joint_effort_interface_);
		registerInterface(&joint_position_interface_);
		
	}
}
