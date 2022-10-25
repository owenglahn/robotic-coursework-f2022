#include "inverse_dynamics_controller/InverseDynamicsController.hpp" 	// include your ROS class
#include <ros/ros.h>

int main(int argc, char** argv) {

	// Initialize ROS
	ros::init(argc, argv, "inverse_dynamics_controller");

	std::cout << "initialized ros" << std::endl;
	// Add a node handle
	ros::NodeHandle nodeHandle;

	ros::Rate loopRate(5);
	std::cout << "set loop rate" << std::endl;
	// Make an instance of your ROS package
	inverse_dynamics_controller::InverseDynamicsController 
		inverseDynamicsController(nodeHandle);
	std::cout << "created controller" << std::endl;


	while ( ros::ok() ) {

		// the callbacks function will automatically be called.
		ros::spinOnce() ;
		// call the update function
		inverseDynamicsController.update() ;

		// sleep for any time remaining to the publish rate
		loopRate.sleep() ;

	}

	return 0;
}
