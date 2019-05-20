/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <urdf/model.h>

// KDL
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarrayacc.hpp>
#include <kdl_parser/kdl_parser.hpp>

#define NB_JOINTS 5

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

ros::Publisher 	vel_pub;
ros::Publisher 	cmd_pub, cmd_pub_joint1, cmd_pub_joint2, cmd_pub_joint3, cmd_pub_joint4, cmd_pub_joint5;
ros::Subscriber joint_states_sub;

double linear_scale;
double angular_scale;

float  current_position[NB_JOINTS];

interactive_markers::MenuHandler menu_handler;
tf::StampedTransform transform[NB_JOINTS];
geometry_msgs::Quaternion current_rotation[NB_JOINTS];


// Create tf listener
boost::shared_ptr<tf::TransformListener> listener;

struct limits
{
	KDL::JntArray min;
	KDL::JntArray max;
	KDL::JntArray center;
} joint_limits; // KDL structures to store limits min, max, center of each joint

bool getJointsLimitsFromURDF(ros::NodeHandle& nh)
{
		std::string robot_description, root_name, tip_name;
		
		root_name = "link_1";
		tip_name = "link_gripper_left";

		if (!ros::param::search(nh.getNamespace(),"robot_description", robot_description))
		{
		    ROS_ERROR_STREAM("No robot description (URDF) found on parameter server (" << nh.getNamespace() << "/robot_description)");
		    return false;
		}

		ROS_INFO("root_name = %s",root_name.c_str());
		ROS_INFO("tip_name = %s",tip_name.c_str());

		// Construct an URDF model from the xml string
		std::string xml_string;

		if (nh.hasParam(robot_description))
		    nh.getParam(robot_description.c_str(), xml_string);
		else
		{
		    ROS_ERROR("Parameter %s not set ...",robot_description.c_str());
		    return false;
		}

		// Get urdf model out of robot_description
		urdf::Model model;
		if (!model.initString(xml_string))
		{
		    ROS_ERROR("Failed to parse urdf file");
		    return false;
		}

		KDL::Tree kdl_tree;
		// Parse URDF to get the KDL tree
		if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
		{
		    ROS_ERROR("Failed to construct kdl tree");
		    return false;
		}

		KDL::Chain kdl_chain;	// KDL chain construct from URDF
		
		// Populate the KDL chain
		if(!kdl_tree.getChain(root_name, tip_name, kdl_chain))
		{
		    ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
		    ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
		    ROS_ERROR_STREAM("  Tree has " << kdl_tree.getNrOfJoints() << " joints");
		    ROS_ERROR_STREAM("  Tree has " << kdl_tree.getNrOfSegments() << " segments");
		    ROS_ERROR_STREAM("  The segments are:");

		    KDL::SegmentMap segment_map = kdl_tree.getSegments();
		    KDL::SegmentMap::iterator it;

		    for( it=segment_map.begin(); it != segment_map.end(); it++ )
		      ROS_ERROR_STREAM( "    " << (*it).first);

		    return false;
		}
		else
		{
		    ROS_INFO_STREAM("The KDL chain from tree is : ");
		    ROS_INFO_STREAM("  " << root_name << " --> " << tip_name);
		    ROS_INFO_STREAM("  Tree has " << kdl_tree.getNrOfJoints() << " joints");
		    ROS_INFO_STREAM("  Tree has " << kdl_tree.getNrOfSegments() << " segments");
		    ROS_INFO_STREAM("  The segments are:");

		    KDL::SegmentMap segment_map = kdl_tree.getSegments();
		    KDL::SegmentMap::iterator it;

		    for( it=segment_map.begin(); it != segment_map.end(); it++ )
		      ROS_INFO_STREAM( "    " << (*it).first);
		}
		
		ROS_INFO("KDL Chain has %d joints !!!", kdl_chain.getNrOfJoints());

		// Parsing joint limits from urdf model along kdl chain
		boost::shared_ptr<const urdf::Link> link = model.getLink(tip_name);	// A Link defined in a URDF structure
		boost::shared_ptr<const urdf::Joint> joint;  // A Joint defined in a URDF structure
		
		// Resize joints limits arrays with the "KDL chain" number of joints
		joint_limits.min.resize(kdl_chain.getNrOfJoints());
		joint_limits.max.resize(kdl_chain.getNrOfJoints());
		joint_limits.center.resize(kdl_chain.getNrOfJoints());
		
		int index;
		
		//link = model.getLink(link->getParent()->name);
		
		for (int i = 0; i < kdl_chain.getNrOfJoints() && link; i++)
		{
		    joint = model.getJoint(link->parent_joint->name);
 
		    ROS_INFO("Getting limits for joint: %s", joint->name.c_str());
			
		    index = kdl_chain.getNrOfJoints() - i - 1;

		    joint_limits.min(index) = joint->limits->lower;
		    joint_limits.max(index) = joint->limits->upper;
		    joint_limits.center(index) = (joint_limits.min(index) + joint_limits.max(index))/2;

		    ROS_INFO("-> index = %d, min = %f, max = %f, center = %f", index, joint_limits.min(index), joint_limits.max(index),joint_limits.center(index));

		    link = model.getLink(link->getParent()->name);
		}
		

		return true;

}


void updatePoseOfAllMarkers(const ros::TimerEvent&)
{
	   geometry_msgs::Pose p;
	   bool get_transform = false;
	   
	   // Marker joint2
	   while (!get_transform)
	   {
		  try{
		  
				listener->lookupTransform("base_link", "link_3",ros::Time(0), transform[1]);
				get_transform = true;                            
										 
										 }
			catch (tf::TransformException &ex) {
					ROS_INFO("%s",ex.what());
					get_transform = false;
					ros::Duration(1.0).sleep();
			 }
		}
		
        tf::pointTFToMsg(transform[1].getOrigin(), p.position);
        p.orientation =  current_rotation[1];
        //tf::quaternionTFToMsg(transform[1].getRotation(), p.orientation);
        
        server->setPose("joint2_marker", p);
        
        // Marker joint3
       get_transform = false;
	   while (!get_transform)
	   {
		  try{
		  
				listener->lookupTransform("base_link", "link_4",ros::Time(0), transform[2]);
				get_transform = true;                            
										 
										 }
			catch (tf::TransformException &ex) {
					ROS_INFO("%s",ex.what());
					get_transform = false;
					ros::Duration(1.0).sleep();
			 }
		}
  
        tf::pointTFToMsg(transform[2].getOrigin(), p.position);
        p.orientation =  current_rotation[2];
        //tf::quaternionTFToMsg(transform[2].getRotation(), p.orientation);
        
        server->setPose("joint3_marker", p);
        
        // Marker joint4
        get_transform = false;
	   while (!get_transform)
	   {
		  try{
		  
				listener->lookupTransform("base_link", "link_5",ros::Time(0), transform[3]);
				get_transform = true;                            
										 
										 }
			catch (tf::TransformException &ex) {
					ROS_INFO("%s",ex.what());
					get_transform = false;
					ros::Duration(1.0).sleep();
			 }
		}
  
        tf::pointTFToMsg(transform[3].getOrigin(), p.position);
        p.orientation =  current_rotation[3];
        //tf::quaternionTFToMsg(transform[3].getRotation(), p.orientation);
        
        /*tf2::Quaternion quat_tf;
        quat_tf.setRPY( 0, 0, current_position[3] );
        quat_tf.normalize();
        
        geometry_msgs::Quaternion quat_msg;
        tf2::fromMsg(quat_msg, quat_tf);
        
        p.orientation = quat_msg;*/
        
        
        
        server->setPose("joint4_marker", p);
        
        
		// 'commit' changes and send to all clients
		server->applyChanges();
		
		//ROS_INFO("update markers position !");
	
}

void processMenuStopCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  geometry_msgs::Twist vel;
  
  vel.angular.z = 0;
  vel.linear.x = 0;
  
  vel_pub.publish(vel); 
}


void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
	for (int i=0; i<NB_JOINTS; i++)
	{
		current_position[i] = msg->position[i+2];
	}
}

void processJoint1Feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	    std::ostringstream s;
		s << "Feedback from marker '" << feedback->marker_name << "' "
		<< " / control '" << feedback->control_name << "'";
	
		switch ( feedback->event_type )
		{
			case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
			
				// Get Yaw orientation value
				double yaw = tf::getYaw(feedback->pose.orientation);
				
				// Take care about limits
				if (yaw < joint_limits.min(0))
					yaw = joint_limits.min(0);
				else
				{  
					if (yaw > joint_limits.max(0))
						yaw = joint_limits.max(0);
					else
						current_rotation[0] = feedback->pose.orientation;
				}
			
				// Print debug informations
				ROS_INFO_STREAM( s.str() << ": pose changed"
				<< "\nyaw: " << yaw
				<< "\nposition = "
				<< feedback->pose.position.x
				<< ", " << feedback->pose.position.y
				<< ", " << feedback->pose.position.z
				<< "\norientation = "
				<< feedback->pose.orientation.w
				<< ", " << feedback->pose.orientation.x
				<< ", " << feedback->pose.orientation.y
				<< ", " << feedback->pose.orientation.z
				<< "\nframe: " << feedback->header.frame_id
				<< " time: " << feedback->header.stamp.sec << " sec, "
				<< feedback->header.stamp.nsec << " nsec");
			
				// Publish command to dynamixel motor by using a dedicated topic
				std_msgs::Float32 cmd;
				cmd.data = yaw;
				cmd_pub_joint1.publish(cmd);
	}
}

void processJoint2Feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
		std::ostringstream s;
		s << "Feedback from marker '" << feedback->marker_name << "' "
		<< " / control '" << feedback->control_name << "'";
	
		switch ( feedback->event_type )
		{
			case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
			
				// Get Yaw orientation value
				double yaw = tf::getYaw(feedback->pose.orientation);
				
				// Take care about limits
				if (yaw < joint_limits.min(1))
					yaw = joint_limits.min(1);
				else
				{  
					if (yaw > joint_limits.max(1))
						yaw = joint_limits.max(1);
					else
						current_rotation[1] = feedback->pose.orientation;
				}
			
				// Print debug informations
				ROS_INFO_STREAM( s.str() << ": pose changed"
				<< "\nyaw: " << yaw
				<< "\nposition = "
				<< feedback->pose.position.x
				<< ", " << feedback->pose.position.y
				<< ", " << feedback->pose.position.z
				<< "\norientation = "
				<< feedback->pose.orientation.w
				<< ", " << feedback->pose.orientation.x
				<< ", " << feedback->pose.orientation.y
				<< ", " << feedback->pose.orientation.z
				<< "\nframe: " << feedback->header.frame_id
				<< " time: " << feedback->header.stamp.sec << " sec, "
				<< feedback->header.stamp.nsec << " nsec");
			
				// Publish command to dynamixel motor by using a dedicated topic
				std_msgs::Float32 cmd;
				cmd.data = yaw;
				cmd_pub_joint2.publish(cmd);
	}
}

void processJoint3Feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
		std::ostringstream s;
		s << "Feedback from marker '" << feedback->marker_name << "' "
		<< " / control '" << feedback->control_name << "'";
	
		switch ( feedback->event_type )
		{
			case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
			
				// Get Yaw orientation value
				double yaw = tf::getYaw(feedback->pose.orientation);
				
				// Take care about limits
				if (yaw < joint_limits.min(2))
					yaw = joint_limits.min(2);
				else
				{  
					if (yaw > joint_limits.max(2))
						yaw = joint_limits.max(2);
					else
						current_rotation[2] = feedback->pose.orientation;
				}
			
				// Print debug informations
				ROS_INFO_STREAM( s.str() << ": pose changed"
				<< "\nyaw: " << yaw
				<< "\nposition = "
				<< feedback->pose.position.x
				<< ", " << feedback->pose.position.y
				<< ", " << feedback->pose.position.z
				<< "\norientation = "
				<< feedback->pose.orientation.w
				<< ", " << feedback->pose.orientation.x
				<< ", " << feedback->pose.orientation.y
				<< ", " << feedback->pose.orientation.z
				<< "\nframe: " << feedback->header.frame_id
				<< " time: " << feedback->header.stamp.sec << " sec, "
				<< feedback->header.stamp.nsec << " nsec");
			
				// Publish command to dynamixel motor by using a dedicated topic
				std_msgs::Float32 cmd;
				cmd.data = yaw;
				cmd_pub_joint3.publish(cmd);
	}
	   
}

void processJoint4Feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
		std::ostringstream s;
		s << "Feedback from marker '" << feedback->marker_name << "' "
		<< " / control '" << feedback->control_name << "'";
	
		switch ( feedback->event_type )
		{
			case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
			
				// Get Yaw orientation value
				double yaw = tf::getYaw(feedback->pose.orientation);
				
				// Take care about limits
				if (yaw < joint_limits.min(3))
					yaw = joint_limits.min(3);
				else
				{  
					if (yaw > joint_limits.max(3))
						yaw = joint_limits.max(3);
					else
						current_rotation[3] = feedback->pose.orientation;
				}
			
				// Print debug informations
				ROS_INFO_STREAM( s.str() << ": pose changed"
				<< "\nyaw: " << yaw
				<< "\nposition = "
				<< feedback->pose.position.x
				<< ", " << feedback->pose.position.y
				<< ", " << feedback->pose.position.z
				<< "\norientation = "
				<< feedback->pose.orientation.w
				<< ", " << feedback->pose.orientation.x
				<< ", " << feedback->pose.orientation.y
				<< ", " << feedback->pose.orientation.z
				<< "\nframe: " << feedback->header.frame_id
				<< " time: " << feedback->header.stamp.sec << " sec, "
				<< feedback->header.stamp.nsec << " nsec");
			
				// Publish command to dynamixel motor by using a dedicated topic
				std_msgs::Float32 cmd;
				cmd.data = yaw;
				cmd_pub_joint4.publish(cmd);
	}
}

void processJoint5Feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
		std::ostringstream s;
		s << "Feedback from marker '" << feedback->marker_name << "' "
		<< " / control '" << feedback->control_name << "'";
	
		switch ( feedback->event_type )
		{
			case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
			
				// Get Yaw orientation value
				double yaw = tf::getYaw(feedback->pose.orientation);
				
				// Take care about limits
				if (yaw < joint_limits.min(4))
					yaw = joint_limits.min(4);
				else
				{  
					if (yaw > joint_limits.max(4))
						yaw = joint_limits.max(4);
					else
						current_rotation[4] = feedback->pose.orientation;
				}
			
				// Print debug informations
				ROS_INFO_STREAM( s.str() << ": pose changed"
				<< "\nyaw: " << yaw
				<< "\nposition = "
				<< feedback->pose.position.x
				<< ", " << feedback->pose.position.y
				<< ", " << feedback->pose.position.z
				<< "\norientation = "
				<< feedback->pose.orientation.w
				<< ", " << feedback->pose.orientation.x
				<< ", " << feedback->pose.orientation.y
				<< ", " << feedback->pose.orientation.z
				<< "\nframe: " << feedback->header.frame_id
				<< " time: " << feedback->header.stamp.sec << " sec, "
				<< feedback->header.stamp.nsec << " nsec");
			
				// Publish command to dynamixel motor by using a dedicated topic
				std_msgs::Float32 cmd;
				cmd.data = yaw;
				cmd_pub_joint5.publish(cmd);
	}
}

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
		// Handle angular change (yaw is the only direction in which you can rotate)
		double yaw = tf::getYaw(feedback->pose.orientation);

		geometry_msgs::Twist vel;
		vel.angular.z = angular_scale*yaw;
		vel.linear.x = linear_scale*feedback->pose.position.x;

		vel_pub.publish(vel); 
		
	    server->setPose("mobile_marker", geometry_msgs::Pose());

		server->applyChanges();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mobile_marker_node");
  
  ros::NodeHandle nh;
  
  getJointsLimitsFromURDF(nh);
  
  nh.param<double>("linear_scale", linear_scale, 0.1);
  nh.param<double>("angular_scale", angular_scale, 0.1);
  
  ROS_INFO_STREAM( "Initialized with linear_scale = " << linear_scale << ", and angular_scale = " << angular_scale);
	
  vel_pub = nh.advertise<geometry_msgs::Twist>("/arduino/cmd_vel", 1);
  //cmd_pub = nh.advertise<std_msgs::Float32MultiArray>("/arduino/cmd_pos", 1);
  cmd_pub_joint1 = nh.advertise<std_msgs::Float32>("/arduino/cmd_pos_joint1", 1);
  cmd_pub_joint2 = nh.advertise<std_msgs::Float32>("/arduino/cmd_pos_joint2", 1);
  cmd_pub_joint3 = nh.advertise<std_msgs::Float32>("/arduino/cmd_pos_joint3", 1);
  cmd_pub_joint4 = nh.advertise<std_msgs::Float32>("/arduino/cmd_pos_joint4", 1);
  cmd_pub_joint5 = nh.advertise<std_msgs::Float32>("/arduino/cmd_pos_joint5", 1);
  joint_states_sub = nh.subscribe<sensor_msgs::JointState> ("/arduino/joint_states", 1, jointStateCallback);
  
  ros::Duration(1.0).sleep();
  
  bool get_transform = false;
  
  // Create a Maker Server for the Robot
  server.reset( new interactive_markers::InteractiveMarkerServer("mobile_marker_server") );
  
  listener.reset( new tf::TransformListener());
  
  //ros::Duration(0.1).sleep();

  // create an interactive marker for the mobile part
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.header.stamp=ros::Time::now();
  int_marker.name = "mobile_marker";
  int_marker.description = "Mobile Robot Control";
  
  visualization_msgs::InteractiveMarkerControl control;
  
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.always_visible = true;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  
  //control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  control.always_visible = true;
  int_marker.controls.push_back(control);
  
  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server->insert(int_marker, &processFeedback);
  
  // Create Joint 1 Control
  visualization_msgs::InteractiveMarker int_marker_joint1;
  int_marker_joint1.header.frame_id = "base_link";
  int_marker_joint1.header.stamp=ros::Time::now();
  int_marker_joint1.name = "joint1_marker";
  int_marker_joint1.description = "Joint1 Control";
  int_marker_joint1.scale = 0.2;
  
  visualization_msgs::InteractiveMarkerControl controlJoint1;

  tf::Quaternion orien1(0.0, 1.0, 0.0, 1.0);
  orien1.normalize();
  tf::quaternionTFToMsg(orien1, controlJoint1.orientation);
  controlJoint1.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  controlJoint1.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  controlJoint1.always_visible = true;
  
  get_transform = false;
  while (!get_transform)
  {
	  try{
	  
			listener->lookupTransform("base_link", "link_2",ros::Time(0), transform[0]);
			get_transform = true;                            
									 
									 }
		catch (tf::TransformException &ex) {
				ROS_INFO("%s",ex.what());
				get_transform = false;
				ros::Duration(1.0).sleep();
		 }
  }
  
  tf::pointTFToMsg(transform[0].getOrigin(), int_marker_joint1.pose.position);
  tf::quaternionTFToMsg(transform[0].getRotation(), int_marker_joint1.pose.orientation);
  
  //current_rotation[0] = int_marker_joint1.pose.orientation;

  int_marker_joint1.controls.push_back(controlJoint1);
  server->insert(int_marker_joint1, &processJoint1Feedback);
  
  // Create Joint 2 Control
  visualization_msgs::InteractiveMarker int_marker_joint2;
  int_marker_joint2.header.frame_id = "base_link";
  int_marker_joint2.header.stamp=ros::Time::now();
  int_marker_joint2.name = "joint2_marker";
  int_marker_joint2.description = "Joint2 Control";
  int_marker_joint2.scale = 0.1;
  visualization_msgs::InteractiveMarkerControl controlJoint2;

  //tf::Quaternion orien2(0.0, 0.0, 1.0, 1.0);
  tf::Quaternion orien2(0.0, 1.0, 0.0, 1.0);
  orien2.normalize();
  tf::quaternionTFToMsg(orien2, controlJoint2.orientation);
  controlJoint2.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  controlJoint2.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  controlJoint2.always_visible = true;
  
  get_transform = false;
  while (!get_transform)
  {
	  try{
	  
			listener->lookupTransform("base_link", "link_3",ros::Time(0), transform[1]);
			get_transform = true;                            
									 
									 }
		catch (tf::TransformException &ex) {
				ROS_INFO("%s",ex.what());
				get_transform = false;
				ros::Duration(1.0).sleep();
		 }
  }
  
  tf::pointTFToMsg(transform[1].getOrigin(), int_marker_joint2.pose.position);
  tf::quaternionTFToMsg(transform[1].getRotation(), int_marker_joint2.pose.orientation);
  
  //current_rotation[1] = int_marker_joint2.pose.orientation;

  int_marker_joint2.controls.push_back(controlJoint2);
  server->insert(int_marker_joint2, &processJoint2Feedback);
  
  // Create Joint 3 Control
  visualization_msgs::InteractiveMarker int_marker_joint3;
  int_marker_joint3.header.frame_id = "base_link";
  int_marker_joint3.header.stamp=ros::Time::now();
  int_marker_joint3.name = "joint3_marker";
  int_marker_joint3.description = "Joint3 Control";
  int_marker_joint3.scale = 0.1;
  visualization_msgs::InteractiveMarkerControl controlJoint3;

  //tf::Quaternion orien3(0.0, 0.0, 1.0, 1.0);
  tf::Quaternion orien3(0.0, 1.0, 0.0, 1.0);
  orien3.normalize();
  tf::quaternionTFToMsg(orien3, controlJoint3.orientation);
  controlJoint3.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  controlJoint3.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  controlJoint3.always_visible = true;
  
  get_transform = false;
  while (!get_transform)
  {
	  try{
	  
			listener->lookupTransform("base_link", "link_4",ros::Time(0), transform[2]);
			get_transform = true;                            
									 
									 }
		catch (tf::TransformException &ex) {
				ROS_INFO("%s",ex.what());
				get_transform = false;
				ros::Duration(1.0).sleep();
		 }
  }
  
  tf::pointTFToMsg(transform[2].getOrigin(), int_marker_joint3.pose.position);
  tf::quaternionTFToMsg(transform[2].getRotation(), int_marker_joint3.pose.orientation);
  
  //current_rotation[2] = int_marker_joint3.pose.orientation;

  int_marker_joint3.controls.push_back(controlJoint3);
  server->insert(int_marker_joint3, &processJoint3Feedback);
  
  // Create Joint 4 Control
  visualization_msgs::InteractiveMarker int_marker_joint4;
  int_marker_joint4.header.frame_id = "base_link";
  int_marker_joint4.header.stamp=ros::Time::now();
  int_marker_joint4.name = "joint4_marker";
  int_marker_joint4.description = "Joint4 Control";
  int_marker_joint4.scale = 0.1;
  visualization_msgs::InteractiveMarkerControl controlJoint4;

  //tf::Quaternion orien4(0.0, 0.0, 1.0, 1.0);
  tf::Quaternion orien4(0.0, 1.0, 0.0, 1.0);
  orien4.normalize();
  tf::quaternionTFToMsg(orien4, controlJoint4.orientation);
  controlJoint4.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  controlJoint4.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  controlJoint4.always_visible = true;
  
  get_transform = false;
  while (!get_transform)
  {
	  try{
	  
			listener->lookupTransform("base_link", "link_5",ros::Time(0), transform[3]);
			get_transform = true;                            
									 
									 }
		catch (tf::TransformException &ex) {
				ROS_INFO("%s",ex.what());
				get_transform = false;
				ros::Duration(1.0).sleep();
		 }
  }
  
  tf::pointTFToMsg(transform[3].getOrigin(), int_marker_joint4.pose.position);
  tf::quaternionTFToMsg(transform[3].getRotation(), int_marker_joint4.pose.orientation);
  
  //current_rotation[3] = int_marker_joint4.pose.orientation;

  int_marker_joint4.controls.push_back(controlJoint4);
  server->insert(int_marker_joint4, &processJoint4Feedback);
  
  // Create Joint 5 Control
  visualization_msgs::InteractiveMarker int_marker_joint5;
  int_marker_joint5.header.frame_id = "base_link";
  int_marker_joint5.header.stamp=ros::Time::now();
  int_marker_joint5.name = "joint5_marker";
  int_marker_joint5.description = "Joint5 Control";
  int_marker_joint5.scale = 0.1;
  visualization_msgs::InteractiveMarkerControl controlJoint5;

  tf::Quaternion orien5(0.0, 1.0, 0.0, 1.0);
  orien5.normalize();
  tf::quaternionTFToMsg(orien5, controlJoint5.orientation);
  controlJoint5.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  controlJoint5.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  controlJoint5.always_visible = true;
  
  get_transform = false;
  while (!get_transform)
  {
	  try{
	  
			listener->lookupTransform("base_link", "end_effector_link",ros::Time(0), transform[4]);
			get_transform = true;                            
									 
									 }
		catch (tf::TransformException &ex) {
				ROS_INFO("%s",ex.what());
				get_transform = false;
				ros::Duration(1.0).sleep();
		 }
  }
  
  tf::pointTFToMsg(transform[4].getOrigin(), int_marker_joint5.pose.position);
  tf::quaternionTFToMsg(transform[4].getRotation(), int_marker_joint5.pose.orientation);
  
  int_marker_joint5.controls.push_back(controlJoint5);
  server->insert(int_marker_joint5, &processJoint5Feedback);
  
  // Menu insert
  menu_handler.insert("Stop Robot",&processMenuStopCb);
  menu_handler.apply(*server, int_marker.name);

  // 'commit' changes and send to all clients
  server->applyChanges();
  
  // create a timer to update the published transforms
  ros::Timer frame_timer = nh.createTimer(ros::Duration(0.01), updatePoseOfAllMarkers);

  // start the ROS main loop
  ros::spin();
  
  server.reset();

}
