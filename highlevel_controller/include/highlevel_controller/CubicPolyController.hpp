#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/math/rpy.hpp>
#include "highlevel_controller/cubic_poly_planner.hpp"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <eigen3/Eigen/Dense>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/GripperCommandGoal.h>
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>

namespace highlevel_controller {

enum ArmState {
    MOVING, PRE_GRASP, GRASP, SECOND_MOVE, RELEASE
};

class CubicPolyController {
public:
    CubicPolyController(ros::NodeHandle nodeHandle);
    ~CubicPolyController();
    bool readParams();

    // call back for joint state
    void update_joint_position(sensor_msgs::JointState joint_state);

    void update_function();

private:
    ros::NodeHandle node_handle;
    std::string pubName_;
    std::string subName_;
    ros::Publisher publisher;
    ros::Subscriber subscriber;
    pinocchio::Model model;
    pinocchio::Data data;
    Eigen::VectorXd joint_pos;
    Eigen::VectorXd joint_vel;
    Eigen::VectorXd* target_pos = nullptr;
    Eigen::VectorXd task_ref;
    Eigen::VectorXd task_ref_dot;
    Eigen::VectorXd task_fbk;
    ros::Time start_time;
    ros::Duration t;
    ros::Duration T;
    Eigen::MatrixXd jacobian;
    Eigen::MatrixXd pseudo_inv_jacobian;
    bool start;
    Eigen::VectorXd effector_start;
    int dim_joints; 
    std::string urdf_file_name;
    std::vector<Eigen::VectorXd> target_objs;
    ArmState state;
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction>* actionClient;
    std::string actionName;
};


}