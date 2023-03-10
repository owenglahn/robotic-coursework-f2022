#include <ros/ros.h>
#include <std_msgs/String.h>
#include "cubic_polynomial_planner/move_robot.h"
#include "gazebo_msgs/GetModelState.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Dense>
#include <cmath>

class CubicPolyServer {
public:
	CubicPolyServer(std::string adv_service, std::string read_service, std::string write_topic) {
		// advertise service	
		this -> service = node_handle
			.advertiseService(adv_service, &CubicPolyServer::call_back, this);	
		this -> model_state_client = node_handle
			.serviceClient<gazebo_msgs::GetModelState>(read_service);
		this -> pub = node_handle
			.advertise<geometry_msgs::PoseStamped>(write_topic, 1);
	}
	
	Eigen::Vector3d get_position(ros::Duration t) {
		double s_t = 3/pow(target_time.toSec(), 2) * pow(t.toSec(), 2) 
			- 2/pow(target_time.toSec(), 3) * pow(t.toSec(), 3);
		ROS_INFO("time scale=%f", s_t);
		return pose_to_vector(start_pose) + s_t * (target_pos - pose_to_vector(start_pose));
	}
	
	Eigen::Vector3d pose_to_vector(geometry_msgs::Pose pose) {
		Eigen::Vector3d toReturn;
		toReturn[0] = pose.position.x;
		toReturn[1] = pose.position.y;
		toReturn[2] = pose.position.z;
		return toReturn;
	}

	void update_function() {
		Eigen::Vector3d current_pos = get_position(duration);
		current_pose.position.x = current_pos[0];
		current_pose.position.y = current_pos[1];
		current_pose.position.z = current_pos[2];
		ROS_INFO("pose: x=%f, y=%f, z=%f", current_pos[0], current_pos[1], current_pos[2]);
		geometry_msgs::PoseStamped toPublish;
		toPublish.pose = current_pose;
		this -> pub.publish(toPublish);
	}

	bool call_back(cubic_polynomial_planner::move_robot::Request &req, 
					cubic_polynomial_planner::move_robot::Response &res) {
		gazebo_msgs::GetModelState srv;
		srv.request.model_name = getenv("MODEL_NAME");
		srv.request.relative_entity_name = getenv("RELATIVE_ENTITY_NAME");

		if (this -> model_state_client.call(srv)) {
			start_pose = srv.response.pose;
			current_pose.orientation = start_pose.orientation;
			current_pose.position = start_pose.position;
			ROS_INFO("Start position: x=%f, y=%f, z=%f", 
				start_pose.position.x, start_pose.position.y, start_pose.position.z);
		} else {
			ROS_ERROR("Failed to call service /gazebo/get_model_state");
			return 1;
		}
		
		this -> target_pos[0] = (double) req.x;
		this -> target_pos[1] = (double) req.y;
		this -> target_pos[2] = (double) req.z;
		this -> target_time.sec = (int) req.T;

		ROS_INFO("Moving to: x=%f, y=%f, z=%f in T=%f seconds", 
			(double) req.x, (double) req.y, (double) req.z, (double) req.T);
		this -> ros_start_time = ros::Time::now();
		this -> duration = ros::Time::now() - this -> ros_start_time;
		while (duration.toSec() < target_time.toSec()) {
			ROS_INFO("duration=%f", duration.toSec());
			this -> update_function();
			duration = ros::Time::now() - ros_start_time; 
		}
		res.result = "Success";
		return true;
	}

 private:
 	ros::NodeHandle node_handle;
 	ros::ServiceServer service;
	ros::ServiceClient model_state_client;
	ros::Time ros_start_time;
	ros::Duration duration;
	ros::Publisher pub;
	Eigen::Vector3d target_pos;
	geometry_msgs::Pose start_pose;
	geometry_msgs::Pose current_pose;
	ros::Duration target_time;
 };

int main(int argc, char **argv) {

	// initialize ROS
	ros::init(argc, argv, "move_robot_node") ;
	
	// create server
	CubicPolyServer server("/cubic_polynomial_planner/move_robot", 
		"/gazebo/get_model_state", "/firefly/command/pose");

	// specify the frequency to 10HZ
	ros::Rate loopRate(10) ;

	ros::spin();
	return 0 ;
}
