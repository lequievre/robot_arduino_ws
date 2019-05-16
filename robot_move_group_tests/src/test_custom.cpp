/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */

//#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

//#include <rviz_visual_tools/rviz_visual_tools.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

const std::string PLANNING_SCENE_SERVICE = "get_planning_scene";

moveit::core::RobotState getCurrentRobotState(planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_)
{
        // each time the current state is needed
        planning_scene_monitor_->requestPlanningSceneState(PLANNING_SCENE_SERVICE);
        planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);
        ps->getCurrentStateNonConst().update();
        robot_state::RobotState current_state = ps->getCurrentState();
        return current_state;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  
  // Define and start an AsyncSpinner that use only 1 thread to spin callbacks.
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "robot_arm";

  /* This sleep is ONLY to allow Rviz to come up */
  sleep(2.0);
  
  // The :move_group_interface:`MoveGroup` class can be easily 
  // setup using just the name
  // of the group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  // (Optional) Create a publisher for visualizing plans in Rviz.
  //ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  //moveit_msgs::DisplayTrajectory display_trajectory;

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("End Effector Link: %s", move_group.getEndEffectorLink().c_str());
  
  geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
  ROS_INFO("Current pose : x=%f, y=%f, z=%f", current_pose.position.x, current_pose.position.y, current_pose.position.z);
  
  //planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_ =
  //std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description"/*, tf_listener_*/);
  //group.setStartState(getCurrentRobotState(planning_scene_monitor_));
  
  //group.setStartStateToCurrentState();
  //group.setPlanningTime(10.0);
  
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  sleep(5.0);
  
  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();
  
  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the 
  // end-effector.
  geometry_msgs::Pose target_pose1;
 /* target_pose1.orientation.w = 0.726282;
  target_pose1.orientation.x= 4.04423e-07;
  target_pose1.orientation.y = -0.687396;
  target_pose1.orientation.z = 4.81813e-07;*/

  target_pose1.position.x = 0.0;
  target_pose1.position.y = 0.0;
  target_pose1.position.z = 0.45;
  move_group.setPoseTarget(target_pose1);
  
  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group 
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode success = move_group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success == moveit_msgs::MoveItErrorCodes::SUCCESS ? "" : "FAILED");    
    
  if (success == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
		/* Sleep to give Rviz time to visualize the plan. */
		ROS_INFO("Wait 5.0 seconds time to visualize the plan in RVIZ !");
		sleep(5.0);
		
		// Visualizing plans
		// ^^^^^^^^^^^^^^^^^
		// We can also visualize the plan as a line with markers in RViz.
		ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
		visual_tools.publishAxisLabeled(target_pose1, "pose1");
		visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
		visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
		visual_tools.trigger();
		visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  
		/* Execute the plan */
		//group.move();
		move_group.execute(my_plan);
  }
  
  // Wait before to shutdown the node
  // When the node is deleted, it also delete/stop the AsyncSpinner !
  ros::waitForShutdown();
  
  return 0;
}
