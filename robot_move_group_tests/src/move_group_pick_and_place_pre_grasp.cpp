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

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <std_msgs/Float32.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher  cmd_pub_joint5;

  cmd_pub_joint5 = node_handle.advertise<std_msgs::Float32>("/arduino/cmd_pos_joint5", 1);

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "robot_arm";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // Change Default Planner !
  //move_group.setPlannerId("RRTConnect");

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link","/rviz_visual_tools");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 0.65;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  std::vector<double> joint_group_positions;
  //joint_group_positions.resize(4);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::core::RobotStatePtr current_state;
  bool success;

  geometry_msgs::Pose current_pose;
  current_pose = move_group.getCurrentPose().pose;


  ROS_INFO("Current Pose x= %f, y=%f, z=%f", current_pose.position.x, current_pose.position.y, current_pose.position.z);
  ROS_INFO("Current Orientation w=%f, x= %f, y=%f, z=%f", current_pose.orientation.w, current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z);

  std_msgs::Float32 cmd;
  cmd.data = 2.35;
  cmd_pub_joint5.publish(cmd);


  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' to plan a Pre Grasp Target !");


  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  move_group.setPlanningTime(50.0);
  //move_group.setGoalOrientationTolerance(0.1);
  //move_group.setGoalPositionTolerance(0.1);
  move_group.setStartStateToCurrentState();
  move_group.setMaxVelocityScalingFactor(1.0);

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 0.024541;
  target_pose1.orientation.x = 0.000767;
  target_pose1.orientation.y = 0.999699;
  target_pose1.orientation.z = -0.000019;
  target_pose1.position.x = 0.182583;
  target_pose1.position.y = -0.000280;
  target_pose1.position.z = 0.253623;
  	
  move_group.setPoseTarget(target_pose1);

  ROS_INFO("Target Pose1 x= %f, y=%f, z=%f", target_pose1.position.x, target_pose1.position.y, target_pose1.position.z);
  ROS_INFO("Target Orientation1 w=%f, x= %f, y=%f, z=%f", target_pose1.orientation.w, target_pose1.orientation.x, target_pose1.orientation.y, target_pose1.orientation.z);
  
 
  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "axis_target_pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  visual_tools.prompt("Press 'next' to move to a Pre grasp target!");

  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // Moving to a pose goal is similar to the step above
  // except we now use the move() function. Note that
  // the pose goal we had set earlier is still active
  // and so the robot will try to move to that goal. We will
  // not use that function in this tutorial since it is
  // a blocking function and requires a controller to be active
  // and report success on execution of a trajectory.

  /* Uncomment below line when working with a real robot */
  //move_group.move();
  if (success)
  	move_group.execute(my_plan);


  visual_tools.prompt("Press 'next' to plan a cartesian path !");


  move_group.setStartStateToCurrentState();
  move_group.setPlanningTime(50.0);

  geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;

  std::vector<geometry_msgs::Pose> waypoints;
  //waypoints.push_back(target_pose3);

  target_pose3.position.z -= 0.02;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.z -= 0.02;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.z -= 0.02;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.z -= 0.02;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.z -= 0.02;
  waypoints.push_back(target_pose3);  // down


  // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
  // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
  // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
  move_group.setMaxVelocityScalingFactor(0.1);

  // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in Cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.001;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, false);


  // The trajectory needs to be modified so it will include velocities as well.
  // First to create a RobotTrajectory object
  robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "robot_arm");

  // Second get a RobotTrajectory from trajectory
  rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
 
  // Thrid create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  // Fourth compute computeTimeStamps
  success = iptp.computeTimeStamps(rt);
  ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory);

  // Finally plan and execute the trajectory
  my_plan.trajectory_ = trajectory;

  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  
  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();

  
  visual_tools.prompt("Press 'next' to move cartesian path !");
  move_group.execute(my_plan);


  visual_tools.prompt("Press 'next' to close the gripper !"); 

  cmd.data = 0.8;
  cmd_pub_joint5.publish(cmd);

  visual_tools.prompt("Press 'next' to plan an opposite Pre grasp target!");

//Current Pose x= -0.191080, y=0.000293, z=0.233134
//[ INFO] [1559137596.597112715]: Current Orientation w=0.168694, x= -0.000756, y=-0.985668, z=-0.000129

//Current Pose x= -0.175524, y=0.000539, z=0.229133
//[ INFO] [1559140715.255771388]: Current Orientation w=0.173984, x= -0.001511, y=-0.984747, z=-0.000267


  move_group.setStartStateToCurrentState();
  move_group.setPlanningTime(50.0);
  move_group.setMaxVelocityScalingFactor(1.0);

  target_pose1.orientation.w = 0.173984;
  target_pose1.orientation.x = -0.001511;
  target_pose1.orientation.y = -0.984747;
  target_pose1.orientation.z = -0.000267;
  target_pose1.position.x = -0.175524;
  target_pose1.position.y = 0.000539;
  target_pose1.position.z = 0.229133;

  move_group.setPoseTarget(target_pose1);

  ROS_INFO("Target Pose1 x= %f, y=%f, z=%f", target_pose1.position.x, target_pose1.position.y, target_pose1.position.z);
  ROS_INFO("Target Orientation1 w=%f, x= %f, y=%f, z=%f", target_pose1.orientation.w, target_pose1.orientation.x, target_pose1.orientation.y, target_pose1.orientation.z);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "axis_target_pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  visual_tools.prompt("Press 'next' to move to an opposite Pre grasp target!");

   if (success)
  	move_group.execute(my_plan);

  visual_tools.prompt("Press 'next' to plan a cartesian path !");


  move_group.setStartStateToCurrentState();
  move_group.setPlanningTime(50.0);

  target_pose3 = move_group.getCurrentPose().pose;

  waypoints.clear();
  //waypoints.push_back(target_pose3);

  target_pose3.position.z -= 0.02;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.z -= 0.02;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.z -= 0.02;
  waypoints.push_back(target_pose3);  // down


  // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
  // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
  // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
  move_group.setMaxVelocityScalingFactor(0.1);

  // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in Cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  //moveit_msgs::RobotTrajectory trajectory;
  fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, false);
  my_plan.trajectory_  = trajectory;
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();

  
  visual_tools.prompt("Press 'next' to move cartesian path !");
  move_group.execute(my_plan);

  visual_tools.prompt("Press 'next' to open the gripper !"); 

  cmd.data = 2.35;
  cmd_pub_joint5.publish(cmd);

  visual_tools.prompt("Press 'next' to finish !");

 
  ros::waitForShutdown();

  return 0;
}
