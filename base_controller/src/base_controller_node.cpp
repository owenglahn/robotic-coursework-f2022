#include <ros/ros.h>
#include "base_controller/BaseController.hpp"

int main(int argc, char** argv) {

	// Initialize ROS
	ros::init(argc, argv, "base_controller");

	// Add a node handle
	ros::NodeHandle nodeHandle("~");

	// specify the frequency to 10HZ
	ros::Rate loopRate(10) ;

	// Make an instance of your ROS package
	base_controller::BaseController BaseController(nodeHandle);


	while ( ros::ok() ) {

		// the callbacks function will automatically be called.
		ros::spinOnce() ;

		// call the update function
		BaseController.update_function() ;

		// sleep for any time remaining to the publish rate
		loopRate.sleep() ;

	}

	return 0;
}
