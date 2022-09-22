#include <ros/ros.h>
#include <std_msgs/String.h>
#include "cubic_polynomial_planner/my_service.h"

bool call_back(cubic_polynomial_planner::my_service::Request &req, 
				cubic_polynomial_planner::my_service::Response &res) {
	ROS_INFO("Server: x=%ld, y=%ld, z=%ld, T=%ld", (long int) req.x, (long int) req.y, 
		(long int) req.z, (long int) req.T);
}

int main(int argc, char **argv) {

	// initialize ROS
	ros::init(argc, argv, "publisher_node") ;

	// create a node handle
	ros::NodeHandle node_handle ;

	// advertise the topic "my_topic" 
	ros::Publisher publisher = node_handle.advertise<std_msgs::String>("my_topic", 1) ;

	// specify the frequency to 10HZ
	ros::Rate loopRate(10) ;

	// make a string message
	std_msgs::String message ;

	int	counter = 0 ;

	// ros::ok() returns false when the program is finished (e.g., when you do Ctrl-C)
	while ( ros::ok() ) {	

		// change the content of the message 
		message.data = "hello world " + std::to_string(counter) ;

		// output the message to the screen
		ROS_INFO_STREAM("publisher: " << message.data) ;

		// publish the message
		publisher.publish(message) ;

		// call all the callbacks waiting to be called 
		// ( in this case, nothing )
		ros::spinOnce() ;

		// sleep for any time remaining to the publish rate
		loopRate.sleep() ;
		counter++ ;
	}
	return 0 ;
}
