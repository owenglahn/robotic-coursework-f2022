#include "ros_package_template/Algorithm.hpp"
#include <Eigen/Dense>

namespace ros_package_template {

Algorithm::Algorithm() {
	pinocchio::urdf::buildModel(urdf_file_name, model, false);
	pinocchio::Data data(model);
	dim_joints = model.nq;
	joint_pos = Eigen::VectorXd::Zero(dim_joints);
	joint_vel = Eigen::VectorXd::Zero(dim_joints);
	this -> K = Eigen::MatrixXd::Random(3, 3);
	this -> D = Eigen::MatrixXd::Random(3, 3);
	jacobian = Eigen::MatrixXd::Random(6, dim_joints);
	jacobian_dot = Eigen::MatrixXd::Random(6, dim_joints);
	target_pos = Eigen::Vector3d::Zero(3);
}

Algorithm::~Algorithm() {

}

Eigen::Vector3d get_position(ros::Duration t, ros::Duration target_time,
	Eigen::Vector3d start_pos, Eigen::Vector3d target_pos) {
	double s_t = 3/pow(target_time.toSec(), 2) * pow(t.toSec(), 2) 
		- 2/pow(target_time.toSec(), 3) * pow(t.toSec(), 3);
	return start_pos + s_t * (target_pos - start_pos);
}

Eigen::Vector3d get_velocity(ros::Duration t, ros::Duration target_time,
	Eigen::Vector3d start_pos, Eigen::Vector3d target_pos) {
	double s_tvelo = 6/pow(target_time.toSec(), 2) * t.toSec() 
		- 6/pow(target_time.toSec(), 3) * pow(t.toSec(), 2);
	return s_tvelo * (target_pos - start_pos);
}

void Algorithm::update() {
	const int JOINT_ID = 7;
	pinocchio::forwardKinematics(model, data, joint_pos, joint_vel);
	pinocchio::SE3 pose_now = data.oMi[JOINT_ID];
	task_fbk_pos = pose_now.translation();

	jacobian = Eigen::MatrixXd::Zero(6,dim_joints) ;
	pinocchio::computeAllTerms(model, data, joint_pos, joint_vel) ;
	pinocchio::getJointJacobian(model, data, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_local_world) ;

	jacobian_dot = Eigen::MatrixXd::Zero(6,dim_joints);
	pinocchio::computeJointJacobiansTimeVariation(model, data, joint_pos, joint_vel );
	pinocchio::getJointJacobianTimeVariation(model, data, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_dot);

	Eigen::Vector3d current_velocity = get_velocity();
	K(0, 0) = ;
}

} /* namespace */
