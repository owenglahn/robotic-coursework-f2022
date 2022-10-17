#pragma once
#include <pinocchio/multibody/model.hpp>
#include <Eigen/Dense>

namespace ros_package_template {

/*!
 * Class containing the algorithmic part of the package.
 */
class Algorithm {

	public:
	std::string urdfFileName;

	/*!
	* Constructor.
	*/
	Algorithm();

	/*!
	* Destructor.
	*/
	virtual ~Algorithm();

	/*!
	* a function that runs some algorithms
	*/
	void update ();


	private:
	Eigen::VectorXd joint_pos;
	Eigen::VectorXd joint_vel;
	Eigen::Vector3d task_pos;
	Eigen::MatrixXd jacobian;
	Eigen::MatrixXd jacobian_dot;
	Eigen::MatrixXd K;
	Eigen::MatrixXd D;
	pinocchio::Model model;
	int dim_joints;
	Eigen::Vector3d task_fbk_pos;
	Eigen::Vector3d target_pos;
	ros::Duration t;
};

} /* namespace */
