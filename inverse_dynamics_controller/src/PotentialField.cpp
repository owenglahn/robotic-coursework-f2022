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

namespace inverse_dynamics_controller {

PotentialField::PotentialField(const std::string urdfFileName) {
	pinocchio::urdf::buildModel(urdfFileName, model, false);
	data = new pinocchio::Data(model);
	dim_joints = model.nq;
	joint_pos = Eigen::VectorXd::Zero(dim_joints);
	joint_vel = Eigen::VectorXd::Zero(dim_joints);
	this -> K = .0001 * Eigen::MatrixXd::Identity(3,3);
	this -> K_joint = .0001 * Eigen::MatrixXd::Identity(dim_joints, dim_joints);
	this -> D = .0001 * Eigen::MatrixXd::Identity(3,3);
	this -> D_joint = .0001 * Eigen::MatrixXd::Identity(dim_joints, dim_joints);
	jacobian = Eigen::MatrixXd::Zero(6, dim_joints);
	jacobian_dot = Eigen::MatrixXd::Zero(6, dim_joints);
	task_ref_pos = Eigen::Vector3d::Zero();
	target_pos = Eigen::Vector3d::Zero();
	task_fbk_pos = Eigen::Vector3d::Zero();
	task_fbk_dot = Eigen::Vector3d::Zero();
	task_ref_dot = Eigen::Vector3d::Zero();
	task_ref_acc = Eigen::Vector3d::Zero();
	joint_ref_pos = Eigen::VectorXd::Zero(dim_joints);
	joint_ref_dot = Eigen::VectorXd::Zero(dim_joints);
	joint_ref_acc = Eigen::VectorXd::Zero(dim_joints);
	joint_tar = Eigen::VectorXd::Zero(dim_joints);
	joint_tar << 0.0, 0.0, 0.0, -1.57, 0.0, 0.75, 1.57;
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
	return matrix.transpose() * (matrix * matrix.transpose()).inverse();
}

Eigen::Vector3d PotentialField::get_task_cmd_acc() {
	return task_ref_acc + D * (task_ref_dot - task_fbk_dot) + K * (task_ref_pos - task_fbk_pos);
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
	return joint_ref_acc + D_joint * (joint_ref_dot - joint_vel) + 
		K_joint * (joint_ref_pos - joint_pos);
}

Eigen::VectorXd PotentialField::get_joint_torque(const Eigen::VectorXd& joint_cmd_acc) {
	std::cout << "M: " << data -> M << std::endl;
	return data -> M * joint_cmd_acc + data -> nle;
}

Eigen::VectorXd PotentialField::get_joint_torque_all_terms() {
	Eigen::VectorXd joint_cmd_acc = this -> get_joint_cmd_acc();
	std::cout << "joint cmd acc: " << joint_cmd_acc << std::endl;
	return this -> get_joint_torque(joint_cmd_acc); 
}

Eigen::MatrixXd PotentialField::get_P() {
	std::cout << "M inverse: " << (data -> M).inverse()<< std::endl;
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

	std::cout << "Computing all terms" << std::endl;
	pinocchio::computeAllTerms(model, *data, joint_pos, joint_vel) ;
	pinocchio::getJointJacobian(model, *data, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian) ;

	jacobian_dot = Eigen::MatrixXd::Zero(6,dim_joints);
	pinocchio::computeJointJacobiansTimeVariation(model, *data, joint_pos, joint_vel );
	pinocchio::getJointJacobianTimeVariation(model, *data, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_dot);

	K = .0001 * Eigen::MatrixXd::Identity(3,3);

	task_ref_dot = K * (target_pos - task_fbk_pos);
	inverse_dynamics_controller::scale_task_dot(task_ref_dot);

	std::cout << "Got task ref vel" << std::endl;
	task_ref_pos = task_fbk_pos + task_ref_dot * dt;
	task_fbk_dot = jacobian.block(0, 0, 3, dim_joints) * joint_vel;
	task_ref_acc = (1/dt) * (task_ref_dot - task_fbk_dot);
	std::cout<< "Finished PotentialField update" << std::endl;
}

void PotentialField::update_joint_refs() {
	const float dt = 0.005;
	joint_ref_dot = K_joint * (joint_tar - joint_pos);
	joint_ref_pos = joint_pos + joint_ref_dot * dt;
	joint_ref_acc = 1/dt * (joint_ref_dot - joint_vel);
	std::cout << "finished update joint refs" << std::endl;
}

void PotentialField::update_joints(const sensor_msgs::JointState& joint_state) {
	for (int i = 0; i < dim_joints; i++) {
		joint_pos[i] = joint_state.position[i];
		joint_vel[i] = joint_state.velocity[i];
	}
	std::cout << "finished update joint pos and vel" << std::endl;
}

} /* namespace */
