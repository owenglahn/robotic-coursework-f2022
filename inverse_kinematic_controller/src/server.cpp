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
        ROS_INFO("Advertising service "); 
        node_handle.advertiseService("/cubic_polynomial_planner/move_robot", &Server::call_back, this);
        subscriber = node_handle.subscribe<sensor_msgs::JointState>(read_service, 2, 
            &Server::update_joint_position, this);
        publisher = node_handle.advertise<std_msgs::Float64MultiArray>(write_topic, 2);
        urdf_file_name = 
            "/home/oglahn/comp514/ros_kortex/kortex_description/urdf/gen3.urdf";
    }

    // call back for joint state
    void update_joint_position(sensor_msgs::JointState joint_state) {

        pinocchio::Model model;
        pinocchio::urdf::buildModel(urdf_file_name, model, false);
        pinocchio::Data data(model);	
        const int JOINT_ID = 7;
        double dt = 0.002;

        dim_joints = model.nq;

        joint_pos = Eigen::VectorXd::Random(dim_joints); 
        joint_vel = Eigen::VectorXd::Random(dim_joints);
        joint_pos[0] = joint_state.position[0];
        joint_pos[1] = joint_state.position[1];
        joint_pos[2] = joint_state.position[2];

        joint_vel[0] = joint_state.velocity[0];
        joint_vel[1] = joint_state.velocity[1];
        joint_vel[2] = joint_state.velocity[2];

        pinocchio::forwardKinematics(model, data, joint_pos, joint_vel);							// forward kinematics
        if (start) {
            pinocchio::SE3 pose_now = data.oMi[JOINT_ID];
            effector_start = pose_now.translation();
            start = false;
        }

        Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6,dim_joints);
        pinocchio::computeAllTerms(model, data, joint_pos, joint_vel);
        pinocchio::getJointJacobian(model, data, JOINT_ID, 
            pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian);
        
        pseudo_inv_jacobian = pseudo_inverse(jacobian);
    }

    Eigen::MatrixXd pseudo_inverse(Eigen::MatrixXd matrix) {
        return matrix.transpose() * (matrix * matrix.transpose()).inverse();
    }
    
    void update_function(ros::Duration t, ros::Duration T, Eigen::Vector3d target_pos) {
        Eigen::Vector3d effector_pos = get_position(t, T, effector_start, target_pos);

        std_msgs::Float64MultiArray new_joint_pos;
        Eigen::VectorXd product = pseudo_inv_jacobian * effector_pos; 
        for (int i = 0; i < dim_joints; i++) {
            new_joint_pos.data[i] = product[i];
        }
        publisher.publish(new_joint_pos);
    }

    bool call_back(inverse_kinematic_controller::move_robot::Request &req, 
        inverse_kinematic_controller::move_robot::Response &res) {
        ros::Duration T(req.T);
        start_time = ros::Time::now();
        ros::Duration t(0);
        
        Eigen::Vector3d target_pos;
        target_pos[0] = req.x;
        target_pos[1] = req.y;
        target_pos[2] = req.z;

        while (t.toSec() < T.toSec()) {
            update_function(t, T, target_pos);
            t = ros::Time::now() - start_time;
        }
        return true;
    }
private:
    ros::NodeHandle node_handle;
    ros::Publisher publisher;
    ros::Subscriber subscriber;
    Eigen::VectorXd joint_pos;
    Eigen::VectorXd joint_vel;
    ros::Time start_time;
    pinocchio::Model model;
    Eigen::MatrixXd jacobian;
    Eigen::MatrixXd pseudo_inv_jacobian;
    bool start;
    Eigen::Vector3d effector_start;
    int dim_joints; 
    std::string urdf_file_name;
};