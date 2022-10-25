#include <Eigen/Dense>

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