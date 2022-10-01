/*
 * test.cpp
 *
 *  Created on: Sep. 13, 2021
 *      Author: hsiuchin
 */


#include <iostream>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>


void skew(const Eigen::Vector3d & vec, Eigen::Matrix3d & S) {
	S(0,0) =  0 ;
	S(0,1) = -vec(2) ;
	S(0,2) =  vec(1) ;
	S(1,0) =  vec(2) ;
	S(1,1) =  0 ;
	S(1,2) = -vec(0) ;
	S(2,0) = -vec(1) ;
	S(2,1) =  vec(0) ;
	S(2,2) =  0 ;
}



void twistLocalToWorld( const Eigen::VectorXd & r, const Eigen::MatrixXd & R, const Eigen::VectorXd & twist_local, Eigen::VectorXd & twist_world) {

	Eigen::Matrix3d SR = Eigen::Matrix3d::Zero() ;
	skew(r, SR) ;

	twist_world.head<3>() = SR* R * twist_local.tail<3>() + R*twist_local.head<3>() ;
	twist_world.tail<3>() = R * twist_local.tail<3>() ;
}

int main() {

	// change this to your directory
	std::string urdf_file_name = "/home/hsiuchin/Teaching/2021-Fall-597-ROS/startup-code/robot_model/urdf/j2n7s300.urdf" ;

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


	// Integrate the joint velocity to get the next position
	Eigen::VectorXd joint_pos_next = joint_pos + joint_vel * dt ;
	pinocchio::forwardKinematics(model, data, joint_pos_next, joint_vel );
	pinocchio::SE3 pose_next = data.oMi[JOINT_ID]  ;
	std::cout << " next end-effector pose= \n" << pose_next << std::endl ;

	std::cout << "\n Use jacobian in the local frame \n" ;
	pinocchio::SE3 pose_err = pose_now.actInv(pose_next) ;
	Eigen::VectorXd pose_err_vec_local = (pinocchio::log6(pose_err).toVector() /dt) ;
	Eigen::VectorXd pose_err_vec_world = Eigen::VectorXd::Zero(6) ;
	twistLocalToWorld( data.oMf[JOINT_ID].translation(), pose_now.rotation(), pose_err_vec_local, pose_err_vec_world) ;
	std::cout << "pose_err_vec_local =" << pose_err_vec_local.transpose() << std::endl ;
	std::cout << "pose_err_vec_world =" << pose_err_vec_world.transpose() << std::endl ;
	std::cout << "pose err computed manually" << (pinocchio::log6(pose_now.inverse() * pose_next).toVector() /dt).transpose() << std::endl  ;






	updateFramePlacement(model,data, JOINT_ID) ;
	std::cout << "oMi " <<  data.oMi[JOINT_ID]   <<std::endl ;
	std::cout << "oMf " <<  data.oMf[JOINT_ID]   <<std::endl ;


	Eigen::VectorXd twist_local = jacobian_local * joint_vel ;
	Eigen::VectorXd twist_world = Eigen::VectorXd::Zero(6) ;

	twistLocalToWorld( data.oMf[JOINT_ID].translation(), pose_now.rotation(), twist_local, twist_world) ;

	std::cout << "twist_local =" << twist_local.transpose() << std::endl ;
	std::cout << "twist_world =" << twist_world.transpose() << std::endl ;

	std::cout << "\n Use jacobian in the world frame \n" ;
	Eigen::VectorXd ee_vel = (pose_next.translation() - pose_now.translation()) / dt ;		// the end-effector velocity by differentiating the positions
	std::cout << "Integrated pos err = " << ee_vel.transpose() << std::endl ;
	std::cout << "jacobian_local_world*bqdot = " << (jacobian_local_world * joint_vel).transpose() << std::endl ;



	return 0 ;
}
