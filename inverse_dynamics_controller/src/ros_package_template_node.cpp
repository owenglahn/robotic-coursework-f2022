#include <ros/ros.h>
#include "ros_package_template/RosPackageTemplate.hpp" 	// include your ROS class

int main(int argc, char** argv) {

	// Initialize ROS
	ros::init(argc, argv, "ros_package_template");

	// Add a node handle
	ros::NodeHandle nodeHandle("~");

	// specify the frequency to 10HZ
	ros::Rate loopRate(1000) ;

	// Make an instance of your ROS package
	ros_package_template::RosPackageTemplate rosPackageTemplate(nodeHandle);


	while ( ros::ok() ) {

		// the callbacks function will automatically be called.
		ros::spinOnce() ;

		// call the update function
		rosPackageTemplate.update() ;

		// sleep for any time remaining to the publish rate
		loopRate.sleep() ;

	}

	return 0;
}
