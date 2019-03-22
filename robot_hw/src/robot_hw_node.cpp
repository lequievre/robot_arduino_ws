/*
 * Laurent LEQUIEVRE
 * Research Engineer, CNRS (France)
 * laurent.lequievre@uca.fr
 * UMR 6602 - Institut Pascal
 * 
*/

// ROS headers
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <signal.h>

#include <unistd.h>

#include "robot_hw/robot_hw.h"

bool g_quit = false;

void quitRequested(int sig)
{
  g_quit = true;
}

int main( int argc, char** argv )
{
	// initialize ROS
	ros::init(argc, argv, "robot_hw_node", ros::init_options::NoSigintHandler);
	
	// ros spinner
	ros::AsyncSpinner spinner(4);
	spinner.start();

	// custom signal handlers
	signal(SIGTERM, quitRequested);
	signal(SIGINT, quitRequested);
	signal(SIGHUP, quitRequested);

	// create a node
	ros::NodeHandle my_robot_nh;
	
	robot_hw::RobotHW my_robot_hw;
	std::string urdf_string = "";
	
	my_robot_hw.create(urdf_string, my_robot_nh);
	
	if (!my_robot_hw.init())
	{
		ROS_FATAL_NAMED("robot_hw::RobotHW","robot_hw_node -> Could not initialize robot real interface");
		return -1;
	}
	
	//the controller manager
	controller_manager::ControllerManager manager(&my_robot_hw, my_robot_nh);
	
	ros::Rate loop_rate(50);
	
	ros::Time last_time = ros::Time::now();
	
	// run as fast as possible
	while( !g_quit ) 
	{
		//usleep(100000);  // in microseconds --> 100 ms
		
		loop_rate.sleep();
		
		ros::Time current_time = ros::Time::now();
		ros::Duration elapsed_time = current_time - last_time;
		last_time = current_time;
		
		// read the state from the lwr robot
		my_robot_hw.read(current_time, elapsed_time);
		
		// update the controllers
		manager.update(current_time, elapsed_time);
		
		// write the command to the lwr
		my_robot_hw.write(current_time, elapsed_time);
		
	}

	spinner.stop();
	
	ROS_INFO("The node robot_hw_node was killed!");
	
	return 0;
}

