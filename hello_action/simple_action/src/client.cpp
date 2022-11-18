#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <hello_action_type/MoveToAction.h>


int main(int argc, char** argv) {

	ros::init(argc, argv, "action_client") ;
	ros::NodeHandle node_handle ;

	actionlib::SimpleActionClient<hello_action_type::MoveToAction> client("go_to_home_configuration", true); // true -> don't need ros::spin()

	ROS_INFO("[Client] Action client is ready. Wait for the server...") ;

	client.waitForServer();

	ROS_INFO("[Client] Server is ready. Send the action") ;

	hello_action_type::MoveToGoal action_goal ;


	action_goal.x = 0.5 ;
	action_goal.y = 0.0 ;
	action_goal.z = 0.3 ;

	// Fill in goal here
	client.sendGoal(action_goal);

	ROS_INFO("[Client] Sent action goal. Waiting for the results...") ;
	client.waitForResult( ros::Duration(5.0) );

	if ( client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ) {
		printf("[Client] Yay! The end-effector reaches the goal position");
	}

	printf("[Client] Current State: %s\n", client.getState().toString().c_str());


	return 0;
}
