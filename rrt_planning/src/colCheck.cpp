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
	//Pa(0,0) = 0.0;//box_1(0, 3);
	//Pa(1,0) = 1.0;//box_1(1, 3);
	Pa(2,0) = 2.0;//box_1(2, 3);
}
