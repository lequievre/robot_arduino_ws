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


// %Tag(fullSource)%
#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_listener.h>

#define NB_JOINTS 4

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

ros::Publisher 	vel_pub;
ros::Publisher 	cmd_pub;
ros::Subscriber joint_states_sub;

double linear_scale;
double angular_scale;

float  current_position[NB_JOINTS];

interactive_markers::MenuHandler menu_handler;
tf::StampedTransform transform[NB_JOINTS];

// Create tf listener
boost::shared_ptr<tf::TransformListener> listener;


void updatePoseOfAllMarkers()
{
	   geometry_msgs::Pose p;
	   bool get_transform = false;
	   
	   // Marker joint2
	   while (!get_transform)
	   {
		  try{
		  
				listener->lookupTransform("link_2", "link_3",ros::Time(0), transform[1]);
				get_transform = true;                            
										 
										 }
			catch (tf::TransformException &ex) {
					ROS_INFO("%s",ex.what());
					get_transform = false;
					ros::Duration(1.0).sleep();
			 }
		}
		
        tf::pointTFToMsg(transform[1].getOrigin(), p.position);
        tf::quaternionTFToMsg(transform[1].getRotation(), p.orientation);
        
        server->setPose("joint2_marker", p);
        
        // Marker joint3
       get_transform = false;
	   while (!get_transform)
	   {
		  try{
		  
				listener->lookupTransform("link_3", "link_4",ros::Time(0), transform[2]);
				get_transform = true;                            
										 
										 }
			catch (tf::TransformException &ex) {
					ROS_INFO("%s",ex.what());
					get_transform = false;
					ros::Duration(1.0).sleep();
			 }
		}
  
        tf::pointTFToMsg(transform[2].getOrigin(), p.position);
        tf::quaternionTFToMsg(transform[2].getRotation(), p.orientation);
        
        server->setPose("joint3_marker", p);
        
        // Marker joint4
        get_transform = false;
	   while (!get_transform)
	   {
		  try{
		  
				listener->lookupTransform("link_4", "endeffector",ros::Time(0), transform[3]);
				get_transform = true;                            
										 
										 }
			catch (tf::TransformException &ex) {
					ROS_INFO("%s",ex.what());
					get_transform = false;
					ros::Duration(1.0).sleep();
			 }
		}
  
        tf::pointTFToMsg(transform[3].getOrigin(), p.position);
        tf::quaternionTFToMsg(transform[3].getRotation(), p.orientation);
        
        server->setPose("joint4_marker", p);
        
        
		server->applyChanges();
	
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
	
	    double yaw = tf::getYaw(feedback->pose.orientation);
	
	    std_msgs::Float32MultiArray cmd;
	    
	    cmd.data.resize(NB_JOINTS);
	    cmd.data[0] = yaw;
	    cmd.data[1] = current_position[1];
	    cmd.data[2] = current_position[2];
	    cmd.data[3] = current_position[3];
	    
	    cmd_pub.publish(cmd); 
	    
	    updatePoseOfAllMarkers();
	   
	   ROS_INFO(" joint1 = %f",yaw);
	   ROS_INFO(" joint1 position x=%f, y=%f, z=%f",feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
}

void processJoint2Feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	
	switch ( feedback->event_type )
	{
		case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:

			double yaw = tf::getYaw(feedback->pose.orientation);
		
			std_msgs::Float32MultiArray cmd;
			
			cmd.data.resize(NB_JOINTS);
			cmd.data[0] = current_position[0];
			cmd.data[1] = yaw;
			cmd.data[2] = current_position[2];
			cmd.data[3] = current_position[3];
			
			cmd_pub.publish(cmd); 
			
			updatePoseOfAllMarkers();
		   
			ROS_INFO(" joint2 = %f",yaw);
			ROS_INFO(" joint2 position x=%f, y=%f, z=%f",feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
			
	    break;
	 }
}

void processJoint3Feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	
	    double yaw = tf::getYaw(feedback->pose.orientation);
	    
	    std_msgs::Float32MultiArray cmd;
	    
	    cmd.data.resize(NB_JOINTS);
	    cmd.data[0] = current_position[0];
	    cmd.data[1] = current_position[1];
	    cmd.data[2] = yaw;
	    cmd.data[3] = current_position[3];
	    
	    cmd_pub.publish(cmd); 
	    
	    updatePoseOfAllMarkers();
	   
	    ROS_INFO(" joint3 = %f",yaw);
	    ROS_INFO(" joint3 position x=%f, y=%f, z=%f",feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
}

void processJoint4Feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	
	    double yaw = tf::getYaw(feedback->pose.orientation);
	
	    std_msgs::Float32MultiArray cmd;
	    
	    cmd.data.resize(NB_JOINTS);
	    cmd.data[0] = current_position[0];
	    cmd.data[1] = current_position[1];
	    cmd.data[2] = current_position[2];
	    cmd.data[3] = yaw;
	    
	    cmd_pub.publish(cmd); 
	    
	    updatePoseOfAllMarkers();
	   
	    ROS_INFO(" joint4 = %f",yaw);
	    ROS_INFO(" joint4 position x=%f, y=%f, z=%f",feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
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
  nh.param<double>("linear_scale", linear_scale, 0.1);
  nh.param<double>("angular_scale", angular_scale, 0.1);
  
  ROS_INFO_STREAM( "Initialized with linear_scale = " << linear_scale << ", and angular_scale = " << angular_scale);
	
  vel_pub = nh.advertise<geometry_msgs::Twist>("/arduino/cmd_vel", 1);
  cmd_pub = nh.advertise<std_msgs::Float32MultiArray>("/arduino/cmd_pos", 1);
  joint_states_sub = nh.subscribe<sensor_msgs::JointState> ("/arduino/joint_states", 1, jointStateCallback);
  
  ros::Duration(1.0).sleep();
  
  bool get_transform = false;
  
  // Create a Maker Server for the Robot
  server.reset( new interactive_markers::InteractiveMarkerServer("mobile_marker_server") );
  
  listener.reset( new tf::TransformListener());

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
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  
  //control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  int_marker.controls.push_back(control);
  
  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server->insert(int_marker, &processFeedback);
  

  // Create Joint 1 Control
  visualization_msgs::InteractiveMarker int_marker_joint1;
  int_marker_joint1.header.frame_id = "link_1";
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
  
  get_transform = false;
  while (!get_transform)
  {
	  try{
	  
			listener->lookupTransform("link_1", "link_2",ros::Time(0), transform[0]);
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

  int_marker_joint1.controls.push_back(controlJoint1);
  server->insert(int_marker_joint1, &processJoint1Feedback);
  
  // Create Joint 2 Control
  visualization_msgs::InteractiveMarker int_marker_joint2;
  int_marker_joint2.header.frame_id = "link_2";
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
  
  get_transform = false;
  while (!get_transform)
  {
	  try{
	  
			listener->lookupTransform("link_2", "link_3",ros::Time(0), transform[1]);
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

  int_marker_joint2.controls.push_back(controlJoint2);
  server->insert(int_marker_joint2, &processJoint2Feedback);
  
  // Create Joint 3 Control
  visualization_msgs::InteractiveMarker int_marker_joint3;
  int_marker_joint3.header.frame_id = "link_3";
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
  
  get_transform = false;
  while (!get_transform)
  {
	  try{
	  
			listener->lookupTransform("link_3", "link_4",ros::Time(0), transform[2]);
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

  int_marker_joint3.controls.push_back(controlJoint3);
  server->insert(int_marker_joint3, &processJoint3Feedback);
  
  // Create Joint 4 Control
  visualization_msgs::InteractiveMarker int_marker_joint4;
  int_marker_joint4.header.frame_id = "link_4";
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
  
  get_transform = false;
  while (!get_transform)
  {
	  try{
	  
			listener->lookupTransform("link_4", "endeffector",ros::Time(0), transform[3]);
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

  int_marker_joint4.controls.push_back(controlJoint4);
  server->insert(int_marker_joint4, &processJoint4Feedback);
  
  
  
  // Menu insert
  menu_handler.insert("Stop Robot",&processMenuStopCb);
  menu_handler.apply(*server, int_marker.name);

  // 'commit' changes and send to all clients
  server->applyChanges();

  // start the ROS main loop
  ros::spin();

}
// %Tag(fullSource)%
