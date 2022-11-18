#include "hello_action_client/ActionClient.hpp"


ActionClient::ActionClient(ros::NodeHandle& node_handle) : 	node_handle_(node_handle),
															move_to_action_client_ ("/move_to_action", true) {

	if ( !readParameters() ) {
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}

	target_translation_ 	= Eigen::MatrixXd::Random(NUM_TARGETS_,3) ; // instead of random numbers, read it from YAML
	feedback_topic_name_ 	= "/move_to_action/feedback" ; // instead of providing a string here, read it from YAML


	// prepare all publishers, subscribers, servers, clients, etc
//	feedback_subscriber_ 	= node_handle_.subscribe(feedback_topic_name_, 1, 	&ActionClient::feedbackCallback, this);

	ROS_INFO_STREAM("Successfully launched node.") ;
}

ActionClient::~ActionClient() {

}

bool ActionClient::readParameters() {
	if ( !node_handle_.getParam("subscriber_topic", subscriberTopic_ )) {
		return false ;
	}
	return true;
}

void ActionClient::update() {

	ROS_INFO_STREAM("[ActionClient::update] Action client is ready. Wait for the server...") ;

	//boost::function fun_done = boost::bind(&ActionClient::doneCallback, this, _1, _2) ;

	for ( int counter = 0 ; counter < NUM_TARGETS_ ; counter++ ) {

		// Fill in goal here
		action_goal_.x = target_translation_(counter, 0) ;
		action_goal_.y = target_translation_(counter, 1) ;
		action_goal_.z = target_translation_(counter, 2) ;

		// wait for the server
		move_to_action_client_.waitForServer();

		move_to_action_client_.sendGoal(action_goal_,
				boost::bind(&ActionClient::doneCallback, this, _1, _2),
				boost::bind(&ActionClient::activeCallback, this),
				boost::bind(&ActionClient::feedbackCallback, this, _1) ) ;

		ROS_INFO_STREAM("[ActionClient::update] Sent action goal. Waiting for the results...") ;
		move_to_action_client_.waitForResult( ros::Duration(30.0) );

		if ( move_to_action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ) {
			ROS_INFO_STREAM("[ActionClient::update] Yay! The end-effector reaches the goal position") ;
		}
	}


	ROS_INFO_STREAM("[ActionClient::doneCallback] Done, shotdown") ;
	ros::shutdown();
}


void ActionClient::doneCallback(const actionlib::SimpleClientGoalState& state, const hello_action_type::MoveToResultConstPtr& result) {
	ROS_INFO("[ActionClient::doneCallback] Finished in state [%s]", state.toString().c_str());
}

void ActionClient::activeCallback() {
	ROS_INFO_STREAM("[ActionClient::activeCallback] Action has become active");
}


void ActionClient::feedbackCallback(const hello_action_type::MoveToFeedbackConstPtr& feedback) {
	ROS_DEBUG_STREAM("[ActionClient::feedbackCallback]" << feedback->distance) ;
}
