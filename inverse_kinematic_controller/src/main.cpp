#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include "inverse_kinematic_controller/move_robot.h"
#include "cubic_poly_planner.cpp"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <ros/ros.h>

class Server {
public:
    Server(std::string adv_service, std::string read_service, std::string write_topic) {
        start = true;
        this -> service = node_handle.advertiseService("/cubic_polynomial_planner/move_robot", &Server::call_back, this);
        this -> subscriber = node_handle.subscribe<sensor_msgs::JointState>("/gen3/joint_states", 2, 
            &Server::update_joint_position, this);
        this -> publisher = node_handle.advertise<std_msgs::Float64MultiArray>(write_topic, 2);
        urdf_file_name = 
            "/home/oglahn/comp514/ros_kortex/kortex_description/urdf/gen3.urdf";
        pinocchio::urdf::buildModel(urdf_file_name, model, false);
        data = pinocchio::Data(model);

        dim_joints = model.nq;
        jacobian = Eigen::MatrixXd::Zero(6, dim_joints);
        pseudo_inv_jacobian = Eigen::MatrixXd::Zero(dim_joints, 3);
        joint_pos = Eigen::VectorXd::Zero(dim_joints); 
        joint_vel = Eigen::VectorXd::Zero(dim_joints);
        task_ref = Eigen::Vector3d::Zero();
        task_ref_dot = Eigen::Vector3d::Zero();
        task_fbk = Eigen::Vector3d::Zero();
        target_pos = Eigen::Vector3d::Zero();
        t = ros::Duration(0);
        T = ros::Duration(0);
    }

    // call back for joint state
    void update_joint_position(sensor_msgs::JointState joint_state) {
        const int JOINT_ID = 7;
        double dt = 0.002;

        for (int i = 0; i < dim_joints; i++) {
         joint_vel[i] = joint_state.velocity[i];
         joint_pos[i] = joint_state.position[i];
        } 

        pinocchio::forwardKinematics(model, data, joint_pos, joint_vel);							// forward kinematics
        task_fbk = data.oMi[JOINT_ID].translation();
        std::cout << "task_fbk\n" << task_fbk << std::endl;
        if (start) {
            pinocchio::SE3 pose_now = data.oMi[JOINT_ID];
            effector_start = pose_now.translation();
            std::cout << "effector_start\n" << effector_start << std::endl;
            start = false;
        }

        pinocchio::computeAllTerms(model, data, joint_pos, joint_vel);
        pinocchio::getJointJacobian(model, data, JOINT_ID, 
            pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian);
        
        pseudo_inv_jacobian = 
            jacobian.block(0, 0, 3, dim_joints).completeOrthogonalDecomposition().pseudoInverse();
    }

    void update_function() {
        t = ros::Time::now() - start_time;
        if (t.toSec() <= T.toSec()) {
            task_ref = get_position(t, T, effector_start, target_pos);
        }
        task_ref_dot = (task_ref - task_fbk) / .01;
        std::cout << "task_ref\n" << task_ref << std::endl;
        std::cout << "task_ref_dot\n" << task_ref_dot << std::endl;

        std_msgs::Float64MultiArray new_joint_pos;
        new_joint_pos.layout.dim.push_back(std_msgs::MultiArrayDimension());
        new_joint_pos.layout.dim[0].size = dim_joints;
        new_joint_pos.layout.dim[0].stride = 1;
        new_joint_pos.layout.dim[0].label = "joints";

        Eigen::VectorXd joint_ref = joint_pos + .01 * pseudo_inv_jacobian * task_ref_dot;
        std::cout << "joint_ref\n" << joint_ref << std::endl;
        new_joint_pos.data.insert(new_joint_pos.data.end(), joint_ref.data(), joint_ref.data()
            + dim_joints);
        publisher.publish(new_joint_pos);
    }

    bool call_back(inverse_kinematic_controller::move_robot::Request &req, 
        inverse_kinematic_controller::move_robot::Response &res) {
        ROS_INFO("Call back ");
        T = ros::Duration(req.T);
        start_time = ros::Time::now();
        t = ros::Duration(0);
        
        target_pos[0] = req.x;
        target_pos[1] = req.y;
        target_pos[2] = req.z;

        start = true;
        return true;
    }
private:
    ros::NodeHandle node_handle;
    ros::Publisher publisher;
    ros::Subscriber subscriber;
    pinocchio::Model model;
    pinocchio::Data data;
    Eigen::VectorXd joint_pos;
    Eigen::VectorXd joint_vel;
    Eigen::Vector3d target_pos;
    Eigen::Vector3d task_ref;
    Eigen::Vector3d task_ref_dot;
    Eigen::Vector3d task_fbk;
    ros::Time start_time;
    ros::Duration t;
    ros::Duration T;
    Eigen::MatrixXd jacobian;
    Eigen::MatrixXd pseudo_inv_jacobian;
    bool start;
    Eigen::Vector3d effector_start;
    int dim_joints; 
    std::string urdf_file_name;
    ros::ServiceServer service;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "inverse_kinematic_controller_node");

    std::string adv_service = "/cubic_polynomial_planner/move_robot";
    std::string read_topic = "/gen3/joint_states";
    std::string pub_topic = "/gen3/joint_group_position_controller/command";

    Server server(adv_service, read_topic, pub_topic);
    ros::Rate loopRate(10);

    while(ros::ok()) {
        ros::spinOnce();
        server.update_function();
        loopRate.sleep();
    }
}