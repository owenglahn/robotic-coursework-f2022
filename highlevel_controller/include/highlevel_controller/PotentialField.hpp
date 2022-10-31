#pragma once

#include <iostream>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/multibody/model.hpp>
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>

namespace inverse_dynamics_controller {

// const double DT = 0.005;
/*!
 * Class containing the algorithmic part of the package.
 */
class PotentialField {

	public:
	Eigen::Vector3d target_pos;

	/*!
	* Constructor.
	*/
	PotentialField(const std::string urdfFileName, double k_attr, double k_task, double k_joint, 
		double d_attr, double d_task, double d_joint);

	/*!
	* Destructor.
	*/
	virtual ~PotentialField();

	/*!
	* a function that runs some algorithms
	*/
	void update();
	void update_joints(const sensor_msgs::JointState& joint_state);
	void update_joint_refs();
	Eigen::Vector3d get_task_cmd_acc();
	Eigen::MatrixXd get_alpha();
	Eigen::Vector3d get_n(const Eigen::MatrixXd& alpha);
	Eigen::Vector3d get_F_cmd(const Eigen::MatrixXd& alpha, const Eigen::Vector3d& x_acc_cmd, 
		const Eigen::Vector3d& n);
	Eigen::VectorXd get_task_torque(const Eigen::Vector3d& F_cmd);
	Eigen::VectorXd get_task_torque_all_terms();

	Eigen::VectorXd get_joint_cmd_acc(); 
	Eigen::VectorXd get_joint_torque(const Eigen::VectorXd& joint_cmd_acc);
	Eigen::VectorXd get_joint_torque_all_terms();
	Eigen::MatrixXd get_P();

	private:
	Eigen::VectorXd joint_pos;
	Eigen::VectorXd joint_vel;
	Eigen::MatrixXd jacobian;
	Eigen::MatrixXd jacobian_dot;
	double k_attr;
	double k_task;
	double k_joint;
	double d_attr;
	double d_task;
	double d_joint;
	pinocchio::Model model;
	int dim_joints;
	Eigen::Vector3d task_ref_pos;
	Eigen::Vector3d task_fbk_pos;
	Eigen::Vector3d task_fbk_dot;
	Eigen::Vector3d task_ref_dot;
	Eigen::Vector3d task_ref_acc;
	pinocchio::Data *data;

	Eigen::VectorXd joint_ref_pos;
	Eigen::VectorXd joint_ref_dot;
	Eigen::VectorXd joint_ref_acc;
	Eigen::VectorXd joint_tar;
};

} /* namespace */
