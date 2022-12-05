#include "highlevel_controller/CubicPolyController.hpp"
namespace highlevel_controller {

CubicPolyController::CubicPolyController(ros::NodeHandle nodeHandle) {
    node_handle = nodeHandle;
    start = true;
    state = ArmState::MOVING;
    task_ref = Eigen::VectorXd::Zero(6);
    for (int i = 0; i < 3; i++) {
        target_objs.push_back(Eigen::VectorXd::Zero(6));
    }
    T = ros::Duration(1);
    if (!readParams()) {
        ROS_ERROR("Unable to read params.");
        ros::requestShutdown();
    }
    std::cout << "Action server: " << actionName << std::endl;
    std::cout << "Read params" << std::endl;
    this -> target_pos = &(target_objs.front());
    this -> start_time = ros::Time::now();
    std::cout << "Target pos at init:\n" << *target_pos << std::endl;
    this -> subscriber = node_handle.subscribe<sensor_msgs::JointState>(subName_, 2, 
        &CubicPolyController::update_joint_position, this);
    this -> publisher = node_handle.advertise<std_msgs::Float64MultiArray>(pubName_, 2);
    this -> actionClient = new actionlib::SimpleActionClient<control_msgs::GripperCommandAction>
        (actionName, true);
    std::cout << "Building model" << std::endl;
    pinocchio::urdf::buildModel(urdf_file_name, model, false);
    std::cout << "Built model" << std::endl;
    data = pinocchio::Data(model);

    dim_joints = model.nq;
    effector_start = Eigen::VectorXd::Zero(6);
    jacobian = Eigen::MatrixXd::Zero(6, dim_joints);
    pseudo_inv_jacobian = Eigen::MatrixXd::Zero(dim_joints, 6);
    joint_pos = Eigen::VectorXd::Zero(dim_joints); 
    joint_vel = Eigen::VectorXd::Zero(dim_joints);
    task_ref_dot = Eigen::VectorXd::Zero(6);
    task_fbk = Eigen::VectorXd::Zero(6);
    t = ros::Duration(0);
    this -> actionClient -> waitForServer();
    std::cout << "Finished init" << std::endl;
}

bool CubicPolyController::readParams() {
    std::vector<double> paramOrientation;
    std::vector<double> target1;
    std::vector<double> target2;
    std::vector<double> target3;
    int tar_time; 
    if (!node_handle.getParam("orientation", paramOrientation) ||
        !node_handle.getParam("sub_topic", subName_) || 
        !node_handle.getParam("pub_topic", pubName_) || 
        !node_handle.getParam("target1", target1) || 
        !node_handle.getParam("target2", target2) ||
        !node_handle.getParam("target3", target3) || 
        !node_handle.getParam("urdf_file_name", urdf_file_name) ||
        !node_handle.getParam("action", actionName) ||
        !node_handle.getParam("target_time", tar_time)) { 
        return false;
    }
    int i = 0;
    for (double el: paramOrientation) { 
        task_ref[3+i++] = el;
    }
    i=0;
    for (double el: target1) {
        target_objs[0][i++] = el;
    }
    i=0;
    for (double el: target2) {
        target_objs[1][i++] = el;
    }
    i=0;
    for (double el: target3) {
        target_objs[2][i++] = el;
    }
    T = ros::Duration(tar_time);
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
        start_time = ros::Time::now();
        pinocchio::SE3 pose_now = data.oMi[JOINT_ID];
        effector_start.head<3>() = pose_now.translation();
        std::cout << "effector_start\n" << effector_start << std::endl;
        start = false;
    }

    pinocchio::computeAllTerms(model, data, joint_pos, joint_vel);
    pinocchio::getJointJacobian(model, data, JOINT_ID, 
        pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian);
    
    pseudo_inv_jacobian = 
        jacobian.completeOrthogonalDecomposition().pseudoInverse();
}

void CubicPolyController::update_function() {
    t = ros::Time::now() - start_time;
    std::cout << "Arm state: " << this -> state << std::endl;
    if (state == ArmState::MOVING ) {
        if  (t.toSec() <= T.toSec() || start) {
            std::cout << "Target pos:\n" << *target_pos << std::endl;
            task_ref = get_position(t, T, effector_start, *target_pos);
        } else {
            control_msgs::GripperCommandGoal squeeze;
            if (target_pos == &(target_objs[0])){
                squeeze.command.max_effort = 1.0;
                squeeze.command.position = 0.0;
                actionClient -> sendGoal(squeeze);
                actionClient -> waitForResult(ros::Duration(5.0));
                target_pos++;
                start_time = ros::Time::now();
                t = ros::Duration(0);
            } else {
                state = ArmState::PRE_GRASP;
                squeeze.command.max_effort = 1.0;
                squeeze.command.position = 0.6;
                actionClient -> sendGoal(squeeze);
                actionClient -> waitForResult(ros::Duration(5.0));
            }
        }
    } else if (state == ArmState::PRE_GRASP) {
        if ((actionClient -> getState()) == actionlib::SimpleClientGoalState::SUCCEEDED) {
            std::cout << "ACTION SUCCEEDED" << std::endl;
            state = ArmState::GRASP;
            start_time = ros::Time::now();
            t = ros::Duration(0);
            target_pos++;
        }
    } else if (state == ArmState::GRASP) {
        if (t.toSec() <= T.toSec()) {
            task_ref = get_position(t, T, effector_start, *target_pos);
        } else {
            state = ArmState::RELEASE;
            control_msgs::GripperCommandGoal squeeze;
            squeeze.command.max_effort = 1.0;
            squeeze.command.position = 0.0;
            actionClient -> sendGoal(squeeze);
            actionClient -> waitForResult(ros::Duration(5.0));
        }
    }

    task_ref_dot = (task_ref - task_fbk) / .01;
    std::cout << "effector_start\n" << effector_start << std::endl;
    std::cout << "target_pos\n" << target_pos << std::endl;
    std::cout << "task_ref\n" << task_ref << std::endl;
    std::cout << "task_ref_dot\n" << task_ref_dot << std::endl;

    std_msgs::Float64MultiArray new_joint_pos;
    new_joint_pos.layout.dim.push_back(std_msgs::MultiArrayDimension());
    new_joint_pos.layout.dim[0].size = dim_joints;
    new_joint_pos.layout.dim[0].stride = 1;
    new_joint_pos.layout.dim[0].label = "joints";

    Eigen::VectorXd joint_ref = joint_pos + .01 * pseudo_inv_jacobian * task_ref_dot;
    new_joint_pos.data.insert(new_joint_pos.data.end(), joint_ref.data(), joint_ref.data()
        + dim_joints);
    publisher.publish(new_joint_pos);
}

CubicPolyController::~CubicPolyController() {
    delete this -> actionClient;
}
}