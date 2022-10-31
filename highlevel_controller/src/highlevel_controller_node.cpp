#include <ros/ros.h>
#include "highlevel_controller/HighLevelController.hpp" 	// include your ROS class

int main(int argc, char** argv) {

	// Initialize ROS
	ros::init(argc, argv, "highlevel_controller");

	// Add a node handle
	ros::NodeHandle nodeHandle("~");

	// specify the frequency to 10HZ
	ros::Rate loopRate(1000) ;

	// Make an instance of your ROS package
	highlevel_controller::HighLevelController HighLevelController(nodeHandle);


	while ( ros::ok() ) {

		// the callbacks function will automatically be called.
		ros::spinOnce() ;

		// call the update function
		HighLevelController.update() ;

		// sleep for any time remaining to the publish rate
		loopRate.sleep() ;

	}

	return 0;
}
