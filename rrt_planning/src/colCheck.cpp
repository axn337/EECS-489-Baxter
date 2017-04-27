#include<ros/ros.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

//Eigen Vector3d has a .dot member function

//Matrix<float, 7, 1>

void check_for_collisions(Eigen::Matrix<float, 4, 4> box_1, Eigen::Matrix<float, 4, 4> box_2, Eigen::Matrix<float, 3, 1> extent_1, Eigen::Matrix<float, 3, 1> extent_2){
	Eigen::Matrix<float, 3, 1> Pa;
	Pa(0,0) = box_1(0, 3);
	Pa(1,0) = box_1(1, 3);
	Pa(2,0) = box_1(2, 3);

	Eigen::Matrix<float, 3, 1> Ax;
	Eigen::Matrix<float, 3, 1> Ay;
	Eigen::Matrix<float, 3, 1> Az;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "example_eigen_plane_fit"); //node name
	ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
	return 0;
}
