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
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <interactive_markers/menu_handler.h>

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

ros::Publisher vel_pub;

double linear_scale;
double angular_scale;

interactive_markers::MenuHandler menu_handler;


void processMenuStopCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  geometry_msgs::Twist vel;
  
  vel.angular.z = 0;
  vel.linear.x = 0;
  
  vel_pub.publish(vel); 
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


  server.reset( new interactive_markers::InteractiveMarkerServer("mobile_marker_server") );

  // create an interactive marker for our server
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
  
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  //control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  
  menu_handler.insert("Stop Robot",&processMenuStopCb);
   
  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server->insert(int_marker, &processFeedback);
  
  menu_handler.apply(*server, int_marker.name);

  // 'commit' changes and send to all clients
  server->applyChanges();

  // start the ROS main loop
  ros::spin();

}
// %Tag(fullSource)%
