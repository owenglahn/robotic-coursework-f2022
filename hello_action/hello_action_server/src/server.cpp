#include <ros/ros.h>
#include <hello_action_server/ActionServer.hpp>

int main(int argc, char** argv) {

	ros::init(argc, argv, "action_server");
	ros::NodeHandle node_handle ;

	ActionServer action_server (node_handle) ;

	ros::Rate loop_rate (500) ;
	while ( ros::ok() ) {
		ros::spinOnce();
		action_server.update() ;
		loop_rate.sleep() ;
	}

	return 0;
}
