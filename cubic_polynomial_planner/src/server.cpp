#include <ros/ros.h>
#include <std_msgs/String.h>
#include "cubic_polynomial_planner/my_service.h"

class Server {
public:
	Server() {
		// advertise service	
		this.service = node_handle
			.advertiseService("my_service", &Server::call_back, this);	
	}

	bool call_back(cubic_polynomial_planner::my_service::Request &req, 
					cubic_polynomial_planner::my_service::Response &res) {
		ROS_INFO("Server: x=%ld, y=%ld, z=%ld, T=%ld", (long int) req.x, (long int) req.y, 
			(long int) req.z, (long int) req.T);
		this.x = (long int) req.x;
		this.y = (long int) req.y;
		this.z = (long int) req.x;
		this.T = (long int) req.T;
		res.result = "";
		return true;
	}

	int[] getPosition() {
		int toReturn[4];
		toReturn[0] = this.x;
		toReturn[1] = this.y;
		toReturn[2] = this.z;
		toReturn[3] = this.T;
		return toReturn;
	}
private:
	ros::NodeHandle node_handle;
	ros::ServiceServer service; 
	long int x;
	long int y;
	long int z;
	long int T;	
};

int main(int argc, char **argv) {

	// initialize ROS
	ros::init(argc, argv, "publisher_node") ;
	
	// create server
	Server server;

	// specify the frequency to 10HZ
	ros::Rate loopRate(10) ;

	// ros::ok() returns false when the program is finished (e.g., when you do Ctrl-C)
	while ( ros::ok() ) {
		// call all the callbacks waiting to be called 
		// ( in this case, nothing )
		ros::spin() ;

		// sleep for any time remaining to the publish rate
		loopRate.sleep() ;
		counter++ ;
	}
	return 0 ;
}
