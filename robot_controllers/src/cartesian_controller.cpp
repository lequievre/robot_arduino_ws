/*
 * Laurent LEQUIEVRE
 * Research Engineer, CNRS (France)
 * laurent.lequievre@uca.fr
 * UMR 6602 - Institut Pascal
 * 
 */

#include <robot_controllers/cartesian_controller.h>

// For plugin
#include <pluginlib/class_list_macros.h>


namespace robot_controllers
{
	CartesianVelocityControl::CartesianVelocityControl()
	{
		#if TRACE_ACTIVATED
			ROS_INFO("CartesianVelocityControl: constructor !");
		#endif
	}
	
	CartesianVelocityControl::~CartesianVelocityControl()
	{
		#if TRACE_ACTIVATED
			ROS_INFO("CartesianVelocityControl: destructor !");
		#endif
	}

	bool CartesianVelocityControl::initKDLChain_()
	{
		std::string robot_description, root_name, tip_name;

		if (!ros::param::search(nh_.getNamespace(),"robot_description", robot_description))
		{
		    ROS_ERROR_STREAM("CartesianVelocityControl: No robot description (URDF) found on parameter server (" << nh_.getNamespace() << "/robot_description)");
		    return false;
		}

		if (!nh_.getParam("cartesian_velocity_controller/root_name", root_name))
		{
		    ROS_ERROR_STREAM("CartesianVelocityControl: No root name found on parameter server (" << nh_.getNamespace() << "/root_name)");
		    return false;
		}


		if (!nh_.getParam("cartesian_velocity_controller/tip_name", tip_name))
		{
		    ROS_ERROR_STREAM("CartesianVelocityControl: No tip name found on parameter server (" << nh_.getNamespace() << "/tip_name)");
		    return false;
		}

		ROS_INFO("cartesian_velocity_controller/root_name = %s",root_name.c_str());
		ROS_INFO("cartesian_velocity_controller/tip_name = %s",tip_name.c_str());

		// Construct an URDF model from the xml string
		std::string xml_string;

		if (nh_.hasParam(robot_description))
		    nh_.getParam(robot_description.c_str(), xml_string);
		else
		{
		    ROS_ERROR("CartesianVelocityControl -> (init) Parameter %s not set, shutting down node...",robot_description.c_str());
		    nh_.shutdown();
		    return false;
		}

		// Get urdf model out of robot_description
		urdf::Model model;
		if (!model.initString(xml_string))
		{
		    ROS_ERROR("CartesianVelocityControl -> (init) Failed to parse urdf file");
		    nh_.shutdown();
		    return false;
		}

		KDL::Tree kdl_tree_;
		// Parse URDF to get the KDL tree
		if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
		{
		    ROS_ERROR("CartesianVelocityControl -> (init) Failed to construct kdl tree");
		    //nh_.shutdown();
		    return false;
		}

		// Populate the KDL chain
		if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
		{
		    ROS_ERROR_STREAM("CartesianVelocityControl -> (init) Failed to get KDL chain from tree: ");
		    ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
		    ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
		    ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
		    ROS_ERROR_STREAM("  The segments are:");

		    KDL::SegmentMap segment_map = kdl_tree_.getSegments();
		    KDL::SegmentMap::iterator it;

		    for( it=segment_map.begin(); it != segment_map.end(); it++ )
		      ROS_ERROR_STREAM( "    " << (*it).first);

		    return false;
		}
		else
		{
		    ROS_INFO_STREAM("CartesianVelocityControl -> (init) The KDL chain from tree is : ");
		    ROS_INFO_STREAM("  " << root_name << " --> " << tip_name);
		    ROS_INFO_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
		    ROS_INFO_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
		    ROS_INFO_STREAM("  The segments are:");

		    KDL::SegmentMap segment_map = kdl_tree_.getSegments();
		    KDL::SegmentMap::iterator it;

		    for( it=segment_map.begin(); it != segment_map.end(); it++ )
		      ROS_INFO_STREAM( "    " << (*it).first);
		}
		
		ROS_INFO("KDL Chain has %d joints !!!", kdl_chain_.getNrOfJoints());

		// Parsing joint limits from urdf model along kdl chain
		boost::shared_ptr<const urdf::Link> link_ = model.getLink(tip_name);	// A Link defined in a URDF structure
		boost::shared_ptr<const urdf::Joint> joint_;  // A Joint defined in a URDF structure
		
		// Resize joints limits arrays with the "KDL chain" number of joints
		joint_limits_.min.resize(kdl_chain_.getNrOfJoints());
		joint_limits_.max.resize(kdl_chain_.getNrOfJoints());
		joint_limits_.center.resize(kdl_chain_.getNrOfJoints());
		
		joint_msr_states_.resize(kdl_chain_.getNrOfJoints());
		joint_des_states_.resize(kdl_chain_.getNrOfJoints());

		int index;
		
		link_ = model.getLink(link_->getParent()->name);
		
		for (int i = 0; i < kdl_chain_.getNrOfJoints() && link_; i++)
		{
		    joint_ = model.getJoint(link_->parent_joint->name);
 
		    ROS_INFO("CartesianVelocityControl -> (init) Getting limits for joint: %s", joint_->name.c_str());
			

		    index = kdl_chain_.getNrOfJoints() - i - 1;

		    joint_limits_.min(index) = joint_->limits->lower;
		    joint_limits_.max(index) = joint_->limits->upper;
		    joint_limits_.center(index) = (joint_limits_.min(index) + joint_limits_.max(index))/2;

		    ROS_INFO("CartesianVelocityControl -> min = %f, max = %f, center = %f", joint_limits_.min(index), joint_limits_.max(index),joint_limits_.center(index));

		    link_ = model.getLink(link_->getParent()->name);
		}
		

		return true;

	}
	
	bool CartesianVelocityControl::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n)
    {
		#if TRACE_ACTIVATED
			ROS_INFO("CartesianVelocityControl: init !");
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
		
		sub_command_ = nh_.subscribe("joint_group_position/command", 1, &CartesianVelocityControl::commandCB_, this);
		
		

		if (!initKDLChain_())
                {
			ROS_ERROR_STREAM("CartesianVelocityControl::init -> NO KDL CHAIN !!");
		}
		else
 		{
			ROS_INFO("CartesianVelocityControl::init -> KDL Chain is loaded !!");
		}
		
		
		// get joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
            joint_des_states_.q(i) = joint_msr_states_.q(i);
        }
        
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
        ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_,joint_limits_.min,joint_limits_.max,*fk_pos_solver_,*ik_vel_solver_));
		
		// computing forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);

        //Desired posture is the current one
        x_des_ = x_;
        
        cmd_flag_ = 0;  // set this flag to 0 to not to run the update method
		
		return true;
	}
	
	void CartesianVelocityControl::commandCB_(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
		#if TRACE_ACTIVATED
			ROS_INFO("CartesianVelocityControl: commandCB_ !");
		#endif	
		
		if(msg->data.size()!=n_joints_)
	     { 
	       ROS_ERROR_STREAM("CartesianVelocityControl: Dimension (of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
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
	
	void CartesianVelocityControl::starting(const ros::Time& time)
    {
		#if TRACE_ACTIVATED
			ROS_INFO("CartesianVelocityControl: starting !");
		#endif
		last_time = ros::Time::now();
    }
    
    void CartesianVelocityControl::stopping(const ros::Time& time)
	{
		#if TRACE_ACTIVATED
			ROS_INFO("CartesianVelocityControl: stopping !");
		#endif
	}
	
	void CartesianVelocityControl::update(const ros::Time& time, const ros::Duration& period)
	{
		
		// get joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        }
        
		// computing forward kinematics
		fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);
		
		current_time = ros::Time::now();
		ros::Duration elapsed_time = current_time - last_time;
		
		
		if (elapsed_time.toSec() >= 1.0)
		{
			ROS_INFO("x= %f, y = %f, z = %f", x_.p.x(), x_.p.y(), x_.p.z());
			ROS_INFO("j1 = %f, j2=%f, j3=%f, j4=%f", joint_msr_states_.q(0), joint_msr_states_.q(1), joint_msr_states_.q(2), joint_msr_states_.q(3));
			last_time = current_time;
		}
		
		/*if (cmd_flag_)
		{
			// set control command for joints
			for (int i = 0; i < n_joints_; i++)
				joint_handles_[i].setCommand(commands_buffer_[i]);
		}*/
		
		
	}
}

PLUGINLIB_EXPORT_CLASS(robot_controllers::CartesianVelocityControl, controller_interface::ControllerBase)
