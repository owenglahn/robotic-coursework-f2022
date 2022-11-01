#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <iostream>
#include "inverse_dynamics_controller/PotentialField.hpp"
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <ros/ros.h>

namespace inverse_dynamics_controller {

PotentialField::PotentialField(const std::string urdfFileName, double k_attr,
	double k_task, double k_joint, double d_attr, double d_task, double d_joint) {
	pinocchio::urdf::buildModel(urdfFileName, model, false);
	this -> data = new pinocchio::Data(model);
	this -> dim_joints = model.nq;
	this -> joint_pos = Eigen::VectorXd::Zero(dim_joints);
	this -> joint_vel = Eigen::VectorXd::Zero(dim_joints);
	this -> k_attr = k_attr;
	this -> k_task = k_task;
	this -> k_joint = k_joint;
	this -> d_attr = d_attr;
	this -> d_task = d_task;
	this -> d_joint = d_joint;
	this -> jacobian = Eigen::MatrixXd::Zero(6, dim_joints);
	this -> jacobian_dot = Eigen::MatrixXd::Zero(6, dim_joints);
	this -> task_ref_pos = Eigen::Vector3d::Zero();
	this -> task_fbk_pos = Eigen::Vector3d::Zero();
	this -> task_fbk_dot = Eigen::Vector3d::Zero();
	this -> task_ref_dot = Eigen::Vector3d::Zero();
	this -> task_ref_acc = Eigen::Vector3d::Zero();
	this -> joint_ref_pos = Eigen::VectorXd::Zero(dim_joints);
	this -> joint_ref_dot = Eigen::VectorXd::Zero(dim_joints);
	this -> joint_ref_acc = Eigen::VectorXd::Zero(dim_joints);
	this -> joint_tar = Eigen::VectorXd::Zero(dim_joints);
	this -> joint_tar << 0.0, 0.0, 0.0, -1.57, 0.0, 0.75, 1.57;
	const int JOINT_ID = 7;
	pinocchio::forwardKinematics(model, *data, joint_tar, joint_vel);
	pinocchio::SE3 pose_now = data -> oMi[JOINT_ID];
	this -> target_pos = pose_now.translation();
}

PotentialField::~PotentialField() {
	delete data;
}

void scale_task_dot(Eigen::Vector3d& task_dot) {
	double task_velo = std::pow(task_dot[0], 2) + std::pow(task_dot[1], 2) + 
		std::pow(task_dot[2], 2);
	if (task_velo > 0.5) {
		double scale = sqrt(0.25 / (std::pow(task_dot[0], 2) 
			+ std::pow(task_dot[1], 2) + std::pow(task_dot[2], 2)));
		task_dot *= scale;
	}
}

Eigen::MatrixXd pseudo_inverse(Eigen::MatrixXd matrix) {
	return matrix.completeOrthogonalDecomposition().pseudoInverse();
	// return matrix.transpose() * (matrix * matrix.transpose()).inverse();
}

Eigen::Vector3d PotentialField::get_task_cmd_acc() {
	return task_ref_acc + d_task * (task_ref_dot - task_fbk_dot) + 
		k_task * (task_ref_pos - task_fbk_pos);
}

Eigen::MatrixXd PotentialField::get_alpha() {
	return (inverse_dynamics_controller::pseudo_inverse(jacobian.transpose())
		* data -> M * inverse_dynamics_controller::pseudo_inverse(jacobian))
		.block(0,0,3,3);
}
Eigen::Vector3d PotentialField::get_n(const Eigen::MatrixXd& alpha) {
	return inverse_dynamics_controller::pseudo_inverse(jacobian.transpose()).block(0,0,3,dim_joints)
		* data -> nle - alpha * jacobian_dot.block(0,0,3,dim_joints) * joint_vel;
}
Eigen::Vector3d PotentialField::get_F_cmd(const Eigen::MatrixXd& alpha, 
	const Eigen::Vector3d& x_acc_cmd, const Eigen::Vector3d& n) {
	return alpha * x_acc_cmd + n;
}
Eigen::VectorXd PotentialField::get_task_torque(const Eigen::Vector3d& F_cmd) {
	return jacobian.transpose().block(0,0,dim_joints, 3) * F_cmd;
}

Eigen::VectorXd PotentialField::get_task_torque_all_terms() {
	Eigen::Vector3d task_cmd_acc = get_task_cmd_acc();
	Eigen::MatrixXd alpha = this -> get_alpha();
	Eigen::Vector3d n = this -> get_n(alpha);
	Eigen::Vector3d F = this -> get_F_cmd(alpha, task_cmd_acc, n);
	return get_task_torque(F);
}

Eigen::VectorXd PotentialField::get_joint_cmd_acc() {
	return joint_ref_acc + d_joint * (joint_ref_dot - joint_vel) + 
		k_joint * (joint_ref_pos - joint_pos);
}

Eigen::VectorXd PotentialField::get_joint_torque(const Eigen::VectorXd& joint_cmd_acc) {
	return data -> M * joint_cmd_acc + data -> nle;
}
Eigen::VectorXd PotentialField::get_joint_torque_all_terms() {

	Eigen::VectorXd joint_cmd_acc = this -> get_joint_cmd_acc();
	return this -> get_joint_torque(joint_cmd_acc); 
}

Eigen::MatrixXd PotentialField::get_P() {
	return Eigen::MatrixXd::Identity(dim_joints, dim_joints) - jacobian.transpose() * 
		(jacobian * (data -> M).inverse() * jacobian.transpose()).inverse() * 
		jacobian * (data -> M).inverse();
}

void PotentialField::update() {
	const int JOINT_ID = 7;
	const float dt = 0.005;

	pinocchio::forwardKinematics(model, *data, joint_pos, joint_vel);
	pinocchio::SE3 pose_now = data -> oMi[JOINT_ID];
	task_fbk_pos = pose_now.translation();

	pinocchio::computeAllTerms(model, *data, joint_pos, joint_vel) ;
	pinocchio::getJointJacobian(model, *data, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian) ;

	pinocchio::computeJointJacobiansTimeVariation(model, *data, joint_pos, joint_vel );
	pinocchio::getJointJacobianTimeVariation(model, *data, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_dot);

	std::cout << "task_fbk_pos\n" << task_fbk_pos << std::endl;
	task_ref_dot = k_attr * (target_pos - task_fbk_pos);
	std::cout << "task_ref_dot\n" << task_ref_dot << std::endl;
	inverse_dynamics_controller::scale_task_dot(task_ref_dot);

	task_ref_pos = task_fbk_pos + task_ref_dot * dt;
	// task_fbk_dot = jacobian.block(0, 0, 3, dim_joints) * joint_vel;
	task_fbk_dot = jacobian_dot.block<3, 1>(0, 6);
	task_ref_acc = (1/dt) * (task_ref_dot - task_fbk_dot);
}

void PotentialField::update_joint_refs() {
	const float dt = 0.005;
	joint_ref_dot = k_attr * (joint_tar - joint_pos);
	joint_ref_pos = joint_pos + joint_ref_dot * dt;
	joint_ref_acc = 1/dt * (joint_ref_dot - joint_vel);
}

void PotentialField::update_joints(const sensor_msgs::JointState& joint_state) {
	for (int i = 0; i < dim_joints; i++) {
		joint_pos[i] = joint_state.position[i];
		joint_vel[i] = joint_state.velocity[i];
	}
	std::cout << "joint_vel\n" << joint_vel << std::endl;
}

} /* namespace */
