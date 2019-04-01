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
		
		
		sub_command_ = nh_.subscribe("cartesian_velocity_controller/command", 1, &CartesianVelocityControl::commandCB_, this);
		
		
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
        
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
        ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_,joint_limits_.min,joint_limits_.max,*fk_pos_solver_,*ik_vel_solver_));
		
        
        J_.resize(kdl_chain_.getNrOfJoints());

		
		return true;
	}
	
	void CartesianVelocityControl::commandCB_(const geometry_msgs::TwistConstPtr& msg)
    {
		ROS_INFO("***** START CartesianVelocityControl::command ************");


       ROS_INFO("***** CartesianVelocityControl::command position only ************");
		x_des_ = KDL::Frame(
		KDL::Rotation::RPY(msg->angular.x,
						  msg->angular.y,
						  msg->angular.z),
		KDL::Vector(msg->linear.x,
					msg->linear.y,
					msg->linear.z));
					

        cmd_flag_ = 1;
        
        on_target_ = 0;
        
        ROS_INFO("***** FINISH CartesianVelocityControl::command ************");
	}
	
	void CartesianVelocityControl::starting(const ros::Time& time)
    {
		#if TRACE_ACTIVATED
			ROS_INFO("CartesianVelocityControl: starting !");
		#endif
		last_time = ros::Time::now();
		
		// computing forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);
        
        //Desired posture is the current one
        x_des_ = x_;
        
        cmd_flag_ = 0;  // set this flag to 0 to not to run the update method
        
        on_target_ = 0;
          
    }
    
    void CartesianVelocityControl::stopping(const ros::Time& time)
	{
		#if TRACE_ACTIVATED
			ROS_INFO("CartesianVelocityControl: stopping !");
		#endif
		
		cmd_flag_ = 0;  // set this flag to 0 to not to run the update method
		
		on_target_ = 0;
	}
	
	void CartesianVelocityControl::update(const ros::Time& time, const ros::Duration& period)
	{
		
		// get joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        }
        
        current_time = ros::Time::now();
		ros::Duration elapsed_time = current_time - last_time;
			
		if (elapsed_time.toSec() >= 1.0)
		{
			if (!on_target_)
				ROS_INFO("x= %f, y = %f, z = %f", x_.p.x(), x_.p.y(), x_.p.z());
			else 
				ROS_INFO("ON TARGET !!");
				
			last_time = current_time;
		}

        if (cmd_flag_)
        {
			
            // computing Jacobian
            jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_);

            // computing J_pinv_
            pseudo_inverse_(J_.data, J_pinv_);

            // computing forward kinematics
            fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);
            
			SetToZero(x_err_);
			
            // end-effector position error
            x_err_.vel = x_des_.p - x_.p;   //attention x_des is now a velocity in m/s 

			
            // getting quaternion from rotation matrix
            x_.M.GetQuaternion(quat_curr_.v(0),quat_curr_.v(1),quat_curr_.v(2),quat_curr_.a);
            x_des_.M.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);

            skew_symmetric_(quat_des_.v, skew_);

            for (int i = 0; i < skew_.rows(); i++)
            {
                v_temp_(i) = 0.0;
                for (int k = 0; k < skew_.cols(); k++)
                    v_temp_(i) += skew_(i,k)*(quat_curr_.v(k));
            }

            // end-effector orientation error
            x_err_.rot = quat_curr_.a*quat_des_.v - quat_des_.a*quat_curr_.v - v_temp_;
           

            // computing q_dot
            for (int i = 0; i < J_pinv_.rows(); i++)
            {
                joint_des_states_.qdot(i) = 0.0;
                for (int k = 0; k < J_pinv_.cols(); k++)
                    joint_des_states_.qdot(i) += J_pinv_(i,k)*x_err_(k); //removed scaling factor of .7
          
            }

            // integrating q_dot -> getting q (Euler method)
            for (int i = 0; i < joint_handles_.size(); i++)
                joint_des_states_.q(i) += period.toSec()*joint_des_states_.qdot(i);

            // joint limits saturation
            for (int i =0;  i < joint_handles_.size(); i++)
            {
                if (joint_des_states_.q(i) < joint_limits_.min(i))
                    joint_des_states_.q(i) = joint_limits_.min(i);
                if (joint_des_states_.q(i) > joint_limits_.max(i))
                    joint_des_states_.q(i) = joint_limits_.max(i);
            }

            if (Equal(x_, x_des_, 0.005))
            {
                ROS_INFO("On target");
                on_target_ = 1;
                cmd_flag_ = 0;
            }
            
           /* 
            ROS_INFO("update position desired -> x = %f, y = %f, z = %f", x_des_.p.x(), x_des_.p.y(), x_des_.p.z());
            ROS_INFO("update position calculated -> x = %f, y = %f, z = %f", x_.p.x(), x_.p.y(), x_.p.z());
            ROS_INFO("error translation (des-cal) -> x = %f, y = %f, z = %f", x_err_.vel.x(), x_err_.vel.y(), x_err_.vel.z());
            ROS_INFO("update rotation calculated ->  O = %f, 1 = %f, 2 = %f, 3 = %f, 4 = %f, 5 = %f, 6 = %f, 7 = %f, 8 = %f", x_.M.data[0],x_.M.data[1],x_.M.data[2],x_.M.data[3],x_.M.data[4],x_.M.data[5],x_.M.data[6],x_.M.data[7],x_.M.data[8]);
            ROS_INFO("update rotation desired ->  O = %f, 1 = %f, 2 = %f, 3 = %f, 4 = %f, 5 = %f, 6 = %f, 7 = %f, 8 = %f", x_des_.M.data[0],x_des_.M.data[1],x_des_.M.data[2],x_des_.M.data[3],x_des_.M.data[4],x_des_.M.data[5],x_des_.M.data[6],x_des_.M.data[7],x_des_.M.data[8]);
           */

	   // set controls for joints
           for (int i = 0; i < joint_handles_.size(); i++)
           {
             joint_handles_[i].setCommand(joint_des_states_.q(i));
           }
            
        }

	//ROS_INFO("***** CartesianVelocityControl::update fin ************");
		
		
		
	}
}

PLUGINLIB_EXPORT_CLASS(robot_controllers::CartesianVelocityControl, controller_interface::ControllerBase)
