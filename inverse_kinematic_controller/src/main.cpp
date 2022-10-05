#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include "server.cpp"
#include <iostream>
#include <ros/ros.h>


int main(int argc, char** argv) {
   ros::init(argc, argv, "inverse_kinematic_controller_node");

   std::string adv_service = "/cubic_polynomial_planner/move_robot";
   std::string read_topic = "/gen3/joint_states";
   std::string pub_topic = "/gen3/joint_group_position_controller/command";

   Server server(adv_service, read_topic, pub_topic);

   ros::Rate loopRate(10);
   ros::spin();
}