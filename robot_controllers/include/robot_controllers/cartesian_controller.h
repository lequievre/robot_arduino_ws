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
#include <geometry_msgs/Twist.h>

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

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
using namespace Eigen;

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
			
			int on_target_;
			
			std::string robot_namespace_;
			
			// configuration
			int n_joints_ = 4; // the robot arduino has 4 joints
			std::vector<std::string> joint_names_; // vector of joints names : (joint1, joint2, ...)
			
			std::vector<hardware_interface::PositionJointInterface::ResourceHandleType> joint_handles_;
			
			void commandCB_(const geometry_msgs::TwistConstPtr& msg); // function associate to a subscribe command topic
			
			ros::Subscriber sub_command_;

			KDL::Chain kdl_chain_;	// KDL chain construct from URDF
			KDL::JntArrayAcc joint_msr_states_, joint_des_states_;  // joint states (measured and desired)s
			
			boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
			boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
			boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
			boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;
			
			KDL::Frame x_;		//current pose
			KDL::Frame x_des_;	//desired pose
			
			KDL::Jacobian J_;	//Jacobian
			
			Eigen::MatrixXd J_pinv_;
			Eigen::Matrix<double,3,3> skew_;
			
			KDL::Twist x_err_;

			struct limits_
			{
				KDL::JntArray min;
				KDL::JntArray max;
				KDL::JntArray center;
			} joint_limits_; // KDL structures to store limits min, max, center of each joint

			bool initKDLChain_();
			
			ros::Time last_time, current_time;
			
			struct quaternion_
			{
				KDL::Vector v;
				double a;
			} quat_curr_, quat_des_;

			KDL::Vector v_temp_;
			
			inline void pseudo_inverse_(const Eigen::MatrixXd &M_, Eigen::MatrixXd &M_pinv_,bool damped = true)
			{	
				double lambda_ = damped?0.2:0.0;

				JacobiSVD<MatrixXd> svd(M_, ComputeFullU | ComputeFullV);
				JacobiSVD<MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
				MatrixXd S_ = M_;	// copying the dimensions of M_, its content is not needed.
				S_.setZero();

				for (int i = 0; i < sing_vals_.size(); i++)
					S_(i,i) = (sing_vals_(i))/(sing_vals_(i)*sing_vals_(i) + lambda_*lambda_);

				M_pinv_ = MatrixXd(svd.matrixV()*S_.transpose()*svd.matrixU().transpose());
			}
			
			inline void skew_symmetric_(KDL::Vector &v_, Eigen::Matrix<double,3,3> &skew_mat_)
			{
				skew_mat_ = Eigen::Matrix<double,3,3>::Zero();
				
				skew_mat_(0,1) = -v_(2);
				skew_mat_(0,2) =  v_(1);
				skew_mat_(1,0) =  v_(2);
				skew_mat_(1,2) = -v_(0);
				skew_mat_(2,0) = -v_(1);
				skew_mat_(2,1) =  v_(0);
			}
	};
}

#endif
