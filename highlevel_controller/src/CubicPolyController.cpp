#include "highlevel_controller/CubicPolyController.hpp"
namespace highlevel_controller {

CubicPolyController::CubicPolyController(ros::NodeHandle nodeHandle) {
    node_handle = nodeHandle;
    start = true;
    if (!readParams()) {
        ROS_ERROR("Unable to read params.");
        ros::requestShutdown();
    }
    this -> subscriber = node_handle.subscribe<sensor_msgs::JointState>(subName_, 2, 
        &CubicPolyController::update_joint_position, this);
    this -> publisher = node_handle.advertise<std_msgs::Float64MultiArray>(pubName_, 2);
    pinocchio::urdf::buildModel(urdf_file_name, model, false);
    data = pinocchio::Data(model);

    dim_joints = model.nq;
    jacobian = Eigen::MatrixXd::Zero(6, dim_joints);
    pseudo_inv_jacobian = Eigen::MatrixXd::Zero(dim_joints, 3);
    joint_pos = Eigen::VectorXd::Zero(dim_joints); 
    joint_vel = Eigen::VectorXd::Zero(dim_joints);
    task_ref = Eigen::VectorXd::Zero(6);
    task_ref_dot = Eigen::VectorXd::Zero(6);
    task_fbk = Eigen::VectorXd::Zero(6);
    target_pos = Eigen::VectorXd::Zero(6);
    t = ros::Duration(0);
    T = ros::Duration(0);
}

bool CubicPolyController::readParams() {
    std::vector<double> paramOrientation;
    std::vector<double> target;
    if (!node_handle.getParam("orientation", paramOrientation) ||
        !node_handle.getParam("target", target) || 
        !node_handle.getParam("sub_topic", subName_) || 
        !node_handle.getParam("pub_topic", pubName_) || 
        !node_handle.getParam("urdf_filename", urdf_file_name)) {
        return false;
    }
    int i = 0;
    for (double el: paramOrientation) { 
        task_ref[3+i++] = el;
    }
    i=0;
    for (double el: target) {
        target_pos[i++] = el;
    }
    return true;
}

// call back for joint state
void CubicPolyController::update_joint_position(sensor_msgs::JointState joint_state) {
    const int JOINT_ID = 7;

    for (int i = 0; i < dim_joints; i++) {
        joint_vel[i] = joint_state.velocity[i];
        joint_pos[i] = joint_state.position[i];
    } 

    pinocchio::forwardKinematics(model, data, joint_pos, joint_vel);							// forward kinematics
    task_fbk.head<3>() = data.oMi[JOINT_ID].translation();
    task_fbk.tail<3>() = pinocchio::rpy::matrixToRpy(data.oMi[JOINT_ID].rotation()); 
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

void CubicPolyController::update_function() {
    t = ros::Time::now() - start_time;
    if (t.toSec() <= T.toSec()) {
        task_ref.head<3>() = get_position(t, T, effector_start, target_pos);
    }
    task_ref_dot = (task_ref - task_fbk) / .01;
    std::cout << "task_ref\n" << task_ref << std::endl;
    std::cout << "task_ref_dot\n" << task_ref_dot << std::endl;

    std_msgs::Float64MultiArray new_joint_pos;
    new_joint_pos.layout.dim.push_back(std_msgs::MultiArrayDimension());
    new_joint_pos.layout.dim[0].size = dim_joints;
    new_joint_pos.layout.dim[0].stride = 1;
    new_joint_pos.layout.dim[0].label = "joints";

    Eigen::VectorXd joint_ref = joint_pos + .01 * pseudo_inv_jacobian * task_ref_dot.head<3>();
    std::cout << "joint_ref\n" << joint_ref << std::endl;
    new_joint_pos.data.insert(new_joint_pos.data.end(), joint_ref.data(), joint_ref.data()
        + dim_joints);
    publisher.publish(new_joint_pos);
}
}