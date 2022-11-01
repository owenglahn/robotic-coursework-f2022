#include "highlevel_controller/CubicPolyController.hpp" 	// include your ROS class
#include <ros/ros.h>

int main(int argc, char** argv) {

	// Initialize ROS
	ros::init(argc, argv, "highlevel_controller");

	// Add a node handle
	ros::NodeHandle nodeHandle("~");

	// specify the frequency to 10HZ
	ros::Rate loopRate(10) ;

	// Make an instance of your ROS package
	highlevel_controller::CubicPolyController HighLevelController(nodeHandle);


	while ( ros::ok() ) {

		// the callbacks function will automatically be called.
		ros::spinOnce() ;

		// call the update function
		HighLevelController.update_function() ;

		// sleep for any time remaining to the publish rate
		loopRate.sleep() ;

	}

	return 0;
}
