
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <hello_action_type/MoveToAction.h>


// Note: "Action" is not appended to DoDishes here
void go_to_home_configuration(
		const hello_action_type::MoveToGoalConstPtr& goal,
		actionlib::SimpleActionServer<hello_action_type::MoveToAction>* action_server) {

	ROS_INFO_STREAM("[Server] callback function") ;
	ROS_INFO_STREAM("[Server] goal: x=" << goal->x << ", y=" << goal->y << ", z= " << goal->z) ;
	// Do lots of awesome groundbreaking robot stuff here

	action_server->setSucceeded() ;

	ROS_INFO("[Server] Action is succesfully done") ;
}


int main(int argc, char** argv) {

	ros::init(argc, argv, "action_server");
	ros::NodeHandle node_handle ;

	actionlib::SimpleActionServer<hello_action_type::MoveToAction> server(node_handle, "go_to_home_configuration", boost::bind(&go_to_home_configuration, _1, &server), false) ;

	ROS_INFO("[Server] Action server is ready") ;

	server.start();

	while ( ros::ok() ) {
		ros::spinOnce();
	}

	return 0;
}
