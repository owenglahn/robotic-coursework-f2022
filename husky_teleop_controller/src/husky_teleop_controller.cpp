#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

class SubscribeAndPublish {
public: 
	SubscribeAndPublish() {
		pub = node_handle
			.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 2); 
		sub = node_handle
			.subscribe("teleop/cmd", 1, &SubscribeAndPublish::call_back_function, this);
	}

	void call_back_function ( const std_msgs::String& msg) {
		ROS_INFO( "subscriber: %s", msg.data.c_str() ) ;
		
		geometry_msgs::Twist message;
		if (strcmp(msg.data.c_str(), "i") == 0) { 
			message.linear.x = 1;
		} else if (strcmp(msg.data.c_str(), "o") == 0) {
			message.linear.x = 1;
			message.angular.z = -1;
		} else if (strcmp(msg.data.c_str(), "u") == 0) {
			message.linear.x = 1;
			message.angular.z = 1;
		}	
		pub.publish(message);
		ROS_INFO( "to robot: %f, %f", message.linear.x, message.angular.z ) ;
	}
private:
	ros::NodeHandle node_handle;
	ros::Subscriber sub;
	ros::Publisher pub;
};

int main(int argc, char **argv) {

	// initialize ROS
	ros::init(argc, argv, "subscriber_node") ;

	SubscribeAndPublish sap;

	// specify the frequency to 10HZ
	ros::Rate loopRate(10) ;

	// ros::ok() returns false when the program is finished (e.g., when you do Ctrl-C)
	while ( ros::ok() ) {

		// process callbacks and will not return until the node has been shutdown
		ros::spinOnce() ;

		// sleep for any time remaining to the publish rate
		loopRate.sleep() ;
	}

	return 0 ;
}
