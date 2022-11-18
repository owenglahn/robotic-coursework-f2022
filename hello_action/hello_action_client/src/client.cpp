//#include <chores/DoDishesAction.h> // Note: "Action" is appended
#include <ros/ros.h>
#include <hello_action_client/ActionClient.hpp>
//#include <actionlib/client/simple_action_client.h>
//#include <hello_action/MoveToAction.h>

//typedef actionlib::SimpleActionClient<chores::DoDishesAction> Client;



int main(int argc, char** argv) {

	ros::init(argc, argv, "action_client") ;
	ros::NodeHandle node_handle ;

	ActionClient action_client(node_handle) ;

	ros::Rate loop_rate(500) ;

	while ( ros::ok() ) {
		ros::spinOnce();
		action_client.update() ;
		loop_rate.sleep() ;
	}

	return 0;
}
