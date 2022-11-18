#pragma once

#include <Eigen/Dense>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <hello_action_type/MoveToAction.h>




class ActionClient {

	public:
		/*!
		* Constructor.
		* @param node_handle_ the ROS node handle.
		*/
		ActionClient(ros::NodeHandle& node_handle);

		/*!
		 * Destructor.
		 */
		virtual ~ActionClient();

		/*
		 * A function handling necessary actions in every loop
		 */
		void update() ;

	protected:

		void activeCallback() ;
		void feedbackCallback(const hello_action_type::MoveToFeedbackConstPtr& feedback) ;
		void doneCallback(const actionlib::SimpleClientGoalState& state, const hello_action_type::MoveToResultConstPtr& result) ;

	private:
		/*!
		* Reads and verifies the ROS parameters.
		* @return true if successful.
		*/
		bool readParameters();

		//! ROS node handle.
		ros::NodeHandle& node_handle_;

		//! ROS topic subscriber.
		ros::Subscriber feedback_subscriber_ ;

		//! ROS topic name to subscribe to.
		std::string feedback_topic_name_ ;

		//! ROS service server.
		ros::ServiceServer serviceServer_;

		hello_action_type::MoveToGoal action_goal_ ;

		actionlib::SimpleActionClient<hello_action_type::MoveToAction> move_to_action_client_ ; //("go_to_home_configuration", true); // true -> don't need ros::spin()

		int NUM_TARGETS_ = 3 ;
		Eigen::MatrixXd target_translation_ ;

		int counter_ = 0 ;
		//! Algorithm computation object.
		// Algorithm algorithm_;
};


