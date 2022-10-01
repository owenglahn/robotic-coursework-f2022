#include <iostream>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>

int main() {

	// change this to your directory
	std::string urdf_file_name = "/home/hsiuchin/code/COMP514/robots/ros_kortex/kortex_description/urdf/gen3.urdf" ;

	pinocchio::Model model;													// create a model object
	pinocchio::urdf::buildModel(urdf_file_name, model, false);				// read the URDF file
	pinocchio::Data data(model); 											// create the data structure for the calculations
	const int JOINT_ID = 7;
	double dt = 0.002 ;

	int dim_joints = model.nq ;

	std::cout << "model.nq() = " << model.nq << std::endl ;
	std::cout << "model.nv() = " << model.nv << std::endl ;
	std::cout << "model.njoints() = " << model.njoints  << std::endl ;

	Eigen::VectorXd joint_pos = Eigen::VectorXd::Random(dim_joints) ; // pinocchio::neutral(model) ; // neutral is the default configuration
	Eigen::VectorXd joint_vel = Eigen::VectorXd::Random(dim_joints) ;
	Eigen::VectorXd joint_acc = Eigen::VectorXd::Zero(dim_joints) ;
	std::cout << "joint_pos = " << joint_pos.transpose() << std::endl ;
	std::cout << "joint_vel = " << joint_vel.transpose() << std::endl ;
	std::cout << "joint_acc = " << joint_acc.transpose() << std::endl ;

	// TEST FORWARD KINEMATICS
	pinocchio::forwardKinematics(model, data, joint_pos, joint_vel );							// forward kinematics
	pinocchio::SE3 pose_now = data.oMi[JOINT_ID]  ;												// end-effector pose
	std::cout << "current end-effector pose = \n" << pose_now << std::endl ;
	std::cout << "current end-effector translation = \n" << pose_now.translation() << std::endl ;

	// *************** Jacobian in the local frame *******************************************
	Eigen::MatrixXd jacobian_local = Eigen::MatrixXd::Zero(6,dim_joints) ;
	pinocchio::computeJointJacobian(model, data, joint_pos, JOINT_ID, jacobian_local);

	// *************** Jacobian in the world frame *******************************************
	Eigen::MatrixXd jacobian_local_world = Eigen::MatrixXd::Zero(6,dim_joints) ;
	pinocchio::computeAllTerms(model, data, joint_pos, joint_vel) ;
	pinocchio::getJointJacobian(model, data, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_local_world) ;


	Eigen::MatrixXd jacobian_dot = Eigen::MatrixXd::Zero(6,dim_joints) ;
	pinocchio::computeJointJacobiansTimeVariation(model, data, joint_pos, joint_vel );
	pinocchio::getJointJacobianTimeVariation(model, data, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_dot) ;

	std::cout << "jacobian_dot = \n" << jacobian_dot << std::endl ;
	std::cout << "data_.M = \n" << data.M  << std::endl ;
	std::cout << "data_.nle = \n" << data.nle << std::endl ;


	// Integrate the joint velocity to get the next position
	Eigen::VectorXd joint_pos_next = joint_pos + joint_vel * dt ;
	pinocchio::forwardKinematics(model, data, joint_pos_next, joint_vel );
	pinocchio::SE3 pose_next = data.oMi[JOINT_ID]  ;
	std::cout << " next end-effector pose= \n" << pose_next << std::endl ;

	std::cout << "\n Use jacobian in the local frame \n" ;
	pinocchio::SE3 pose_err = pose_now.actInv(pose_next) ;
	std::cout << "pose_now.actInv(pose_next) " << (pinocchio::log6(pose_err).toVector() /dt).transpose() << std::endl ;
	std::cout << "jacobian_local*bqdot =" << (jacobian_local * joint_vel).transpose() << std::endl ;


	std::cout << "\n Use jacobian in the world frame \n" ;
	Eigen::VectorXd ee_vel = (pose_next.translation() - pose_now.translation()) / dt ;		// the end-effector velocity by differentiating the positions
	std::cout << "Integrated velocity = " << ee_vel.transpose() << std::endl ;
	std::cout << "jacobian_local_world*bqdot = " << (jacobian_local_world * joint_vel).transpose() << std::endl ;


	return 0 ;
}
