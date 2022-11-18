#include "hello_action_server/ActionServer.hpp"


ActionServer::ActionServer(ros::NodeHandle& node_handle) : 	node_handle_(node_handle),
															move_to_action_server_ (node_handle_, "/move_to_action", boost::bind(&ActionServer::move_to_ballback, this, _1 ), false),
															loop_rate_(500)
															{
														//	move_to_action_server_ (node_handle_, "go_to_home_configuration", false) {
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
		ros::console::notifyLoggerLevelsChanged();
	}

	if ( !readParameters() ) {
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}

	// prepare all publishers, subscribers, servers, clients, etc
	feedback_subscriber_ 	= node_handle_.subscribe("/gazebo/model_states", 1, &ActionServer::feedbackCallback, this) ;

	move_to_action_server_.start();

	ROS_INFO_STREAM("[ActionServer::ActionServer] action server is ready");

}

ActionServer::~ActionServer() {

}


bool ActionServer::readParameters() {
	/*
	if ( !node_handle_.getParam("subscriber_topic", subscriberTopic_ )) {
		return false ;
	}
	*/
	return true;
}

void ActionServer::update() {
	/*
	 *  do something. this could be your cubic polynomial or potential field
	 */

	move_to_feedback_.distance = move_to_feedback_.distance * 0.9 ;
	cubicPolyPlanner_.update();
}

void ActionServer::feedbackCallback(const gazebo_msgs::ModelStates& fbk_pose_msg) {

	if ( move_to_action_server_.isActive() ) {

		ROS_DEBUG_STREAM("[ActionServer::feedbackCallback] pose= "
				<< fbk_pose_msg.pose[1].position.x << ", "
				<< fbk_pose_msg.pose[1].position.y << ", "
				<< fbk_pose_msg.pose[1].position.z ) ;
	}
}



void ActionServer::move_to_ballback(const hello_action_type::MoveToGoalConstPtr& goal) {

	ROS_INFO_STREAM("[ActionServer::move_to_ballback] goal: x=" << goal->x << ", y=" << goal->y << ", z= " << goal->z) ;

	move_to_feedback_.distance = 0.5 ;

	while (move_to_feedback_.distance > 0.001 ) {

		// publish the feedback. the client will be able to read it
		move_to_action_server_.publishFeedback(move_to_feedback_) ;

		ROS_DEBUG_STREAM("[ActionServer::move_to_ballback] feedback=" << move_to_feedback_.distance) ;

		loop_rate_.sleep() ;
	}

	move_to_action_server_.setSucceeded() ;

	ROS_INFO_STREAM("[ActionServer::move_to_ballback] Action is succesfully done") ;

}
