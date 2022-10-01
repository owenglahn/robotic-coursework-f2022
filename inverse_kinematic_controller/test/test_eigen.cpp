/*
 * test-eigen.cpp
 *
 *  Check if Eigen3 is correctly linked
 *      Author: hsiuchin
 */

#include <iostream>
#include <Eigen/Dense>

int main() {

	Eigen::VectorXd v = Eigen::VectorXd::Zero(4,1) ;

	std::cout << "v = \n"  << v << std::endl ;
	std::cout << "v transpose = \n" << v.transpose() << std::endl ;

	std::cout << "third element = \n" << v(2) << std::endl ;
	std::cout << "first two elements = \n" << v.head(2) << std::endl ;
	std::cout << "last two elements = \n" << v.tail(2) << std::endl ;

	// matrices with different initial values
	Eigen::MatrixXd M = Eigen::MatrixXd::Random(2,2) ;
	Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6,7) ;
	Eigen::MatrixXd B = Eigen::MatrixXd::Identity(4,4) ;

	std::cout << "M = \n" << M << std::endl ;
	std::cout << "J = \n" << J << std::endl ;
	std::cout << "B = \n" << B << std::endl ;

	std::cout << "M inverse = \n" << M.inverse() << std::endl ;

	// access part of the matrix
	std::cout << "top 3 rows \n" << J.topRows(3) << std::endl ;
	std::cout << "top 3 by 7 elements \n" << J.block<3,7>(0,0) << std::endl ;


	return 0 ;

}
