/*
 * Laurent LEQUIEVRE
 * Research Engineer, CNRS (France)
 * laurent.lequievre@uca.fr
 * UMR 6602 - Institut Pascal
 * 
 */

#include <robot_controllers/joint_group_position_controller.h>

// For plugin
#include <pluginlib/class_list_macros.h>


namespace robot_controllers
{
	JointGroupPosition::JointGroupPosition()
	{
		#if TRACE_ACTIVATED
			ROS_INFO("JointGroupPosition: constructor !");
		#endif
	}
	
	JointGroupPosition::~JointGroupPosition()
	{
		#if TRACE_ACTIVATED
			ROS_INFO("JointGroupPosition: destructor !");
		#endif
	}
	
	bool JointGroupPosition::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n)
    {
		#if TRACE_ACTIVATED
			ROS_INFO("JointGroupPosition: init !");
		#endif
		
		robot_namespace_ = nh_.getNamespace();
		
		// Get vector of joint names defined in the file 'robot_hw_interface.yaml'
		joint_names_.clear();
		nh_.getParam("joints",joint_names_);
		
		n_joints_ = joint_names_.size()-2;
		
		for (size_t i=0; i<n_joints_; ++i)
		{
				ROS_INFO("RobotHW (create) : name of joint %zd = %s", i, joint_names_[i+2].c_str());
				joint_handles_.push_back(robot->getHandle(joint_names_[i+2]));
		}
		
		commands_buffer_.resize(n_joints_);
		
		for (int i=0; i < n_joints_; i++)
        {
            // set initial desired command values 
            commands_buffer_[i] = 0.0;
        }
        
		
		sub_command_ = nh_.subscribe("joint_group_position/command", 1, &JointGroupPosition::commandCB_, this);
		
		cmd_flag_ = 0;  // set this flag to 0 to not to run the update method
		
		return true;
	}
	
	void JointGroupPosition::commandCB_(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
		#if TRACE_ACTIVATED
			ROS_INFO("JointGroupPosition: commandCB_ !");
		#endif	
		
		if(msg->data.size()!=n_joints_)
	     { 
	       ROS_ERROR_STREAM("JointGroupPosition: Dimension (of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
	       cmd_flag_ = 0;
	       return; 
	     }
	     
	     // clear buffers for initial values
	     commands_buffer_.clear();
	     
	     for (size_t i=0; i<n_joints_; ++i)
	     {
			ROS_DEBUG_STREAM("msg->data[" << i << "]=" << msg->data[i]);
			commands_buffer_.push_back(msg->data[i]);
		 }
	     
	     cmd_flag_ = 1;
	}
	
	void JointGroupPosition::starting(const ros::Time& time)
    {
		#if TRACE_ACTIVATED
			ROS_INFO("JointGroupPosition: starting !");
		#endif
    }
    
    void JointGroupPosition::stopping(const ros::Time& time)
	{
		#if TRACE_ACTIVATED
			ROS_INFO("JointGroupPosition: stopping !");
		#endif
	}
	
	void JointGroupPosition::update(const ros::Time& time, const ros::Duration& period)
	{
		if (cmd_flag_)
		{
			// set control command for joints
			for (int i = 0; i < n_joints_; i++)
				joint_handles_[i].setCommand(commands_buffer_[i]);
		}
	}
}

PLUGINLIB_EXPORT_CLASS(robot_controllers::JointGroupPosition, controller_interface::ControllerBase)
