#include<ros/ros.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <stdlib.h>     /* abs */

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

//Eigen Vector3d has a .dot member function

//Matrix<float, 7, 1>

Eigen::Matrix<float, 4, 4> custom_cum_prod(Eigen::Matrix<float, 4, 4> input_mat, int num_start, int num_end){//TODO: change datatype of input_mat if it is not appropriate
	//TODO: whatever 'g_temp = eye(4,4);' did
	for(int i = num_start; i <= num_end){//this searches from num_start to num_end inclusive
		//TODO: the multiplication step, remembering that c++ indexing starts from 0
	}
}

bool check_for_collisions(Eigen::Matrix<float, 4, 4> box_1, Eigen::Matrix<float, 4, 4> box_2, Eigen::Matrix<float, 3, 1> extent_1, Eigen::Matrix<float, 3, 1> extent_2){

	bool collideCheck = true;

	//A
	Eigen::Vector3d Pa;
	Pa(0) = box_1(0, 3);
	Pa(1) = box_1(1, 3);
	Pa(2) = box_1(2, 3);

	Eigen::Vector3d Ax;
	Ax(0) = box_1(0, 0);
	Ax(1) = box_1(1, 0);
	Ax(2) = box_1(2, 0);

	Eigen::Vector3d Ay;
	Ay(0) = box_1(0, 1);
	Ay(1) = box_1(1, 1);
	Ay(2) = box_1(2, 1);

	Eigen::Vector3d Az;
	Az(0) = box_1(0, 2);
	Az(1) = box_1(1, 2);
	Az(2) = box_1(2, 2);

	int Wa = extent_1(0,0);
	int Ha = extent_1(1,0);
	int Da = extent_1(2,0);

	//B
	Eigen::Vector3d Pb;
	Pb(0) = box_2(0, 3);
	Pb(1) = box_2(1, 3);
	Pb(2) = box_2(2, 3);

	Eigen::Vector3d Bx;
	Bx(0) = box_2(0, 0);
	Bx(1) = box_2(1, 0);
	Bx(2) = box_2(2, 0);

	Eigen::Vector3d By;
	By(0) = box_2(0, 1);
	By(1) = box_2(1, 1);
	By(2) = box_2(2, 1);

	Eigen::Vector3d Bz;
	Bz(0) = box_2(0, 2);
	Bz(1) = box_2(1, 2);
	Bz(2) = box_2(2, 2);

	double Wb = extent_2(0,0);
	double Hb = extent_2(1,0);
	double Db = extent_2(2,0);

	Eigen::Vector3d T = Pb - Pa;//not sure on the syntax of this one

	double Rxx = Ax.dot(Bx);
	double Rxy = Ax.dot(By);
	double Rxz = Ax.dot(Bz);
	double Ryy = Ay.dot(By);
	double Ryz = Ay.dot(Bz);
	double Rzz = Az.dot(Bz);
	
	//Case 1
	if (abs(T.dot(Ax)) > Wa + abs(Wb * Rxx) + abs(Hb * Rxy) + abs(Db * Rxz)){
		collideCheck = False;
		return collideCheck;
	}
	//Case 2
	if (abs(T.dot(Ay)) > Ha + abs(Wb * Rxy) + abs(Hb * Ryy) + abs(Db * Ryz)){
		collideCheck = False;
		return collideCheck;
	}
	//Case 3
	if (abs(T.dot(Az)) > Wa + abs(Wb * Rxz) + abs(Hb * Ryz) + abs(Db * Rzz)){
		collideCheck = False;
		return collideCheck;
	}
	//Case 4
	if (abs(T.dot(Bx)) > Wa + abs(Wb * Rxx) + abs(Hb * Rxy) + abs(Da * Rxz)){
		collideCheck = False;
		return collideCheck;
	}
	//Case 5
	if (abs(T.dot(By)) > Wa + abs(Wb * Rxy) + abs(Hb * Ryy) + abs(Da * Ryz)){
		collideCheck = False;
		return collideCheck;
	}
	//Case 6
	if (abs(T.dot(Bz)) > Wa + abs(Wb * Rxz) + abs(Hb * Ryz) + abs(Da * Rzz)){
		collideCheck = False;
		return collideCheck;
	}
	//Case 7
	if (abs(T.dot(Az)*Rxy - T.dot(Ay)*Rxz) > abs(Ha*Rxz)+abs(Da*Rxy)+abs(Hb*Rxz)+abs(Db*Rxy)){
		collideCheck = False;
		return collideCheck;
	}
	//Case 8
	if (abs(T.dot(Az)*Ryy - T.dot(Ay)*Ryz) > abs(Ha*Ryz)+abs(Da*Ryy)+abs(Wb*Rxz)+abs(Db*Rxx)){
		collideCheck = False;
		return collideCheck;
	}

	//Case 9
	if (abs(T.dot(Az)*Ryz - T.dot(Ay)*Rzz) > abs(Ha*Rzz)+abs(Da*Ryz)+abs(Wb*Rxy)+abs(Hb*Rxx)){
		collideCheck = False;
		return collideCheck;
	}

	//Case 10
	if (abs(T.dot(Ax)*Rxz - T.dot(Az)*Rxx) > abs(Wa*Rxx)+abs(Da*Rxx)+abs(Hb*Ryz)+abs(Db*Ryy)){
		collideCheck = False;
		return collideCheck;
	}

	//Case 11
	if (abs(T.dot(Ax)*Ryz - T.dot(Az)*Rxy) > abs(Wa*Rxy)+abs(Da*Rxy)+abs(Wb*Ryz)+abs(Db*Rxy)){
		collideCheck = False;
		return collideCheck;
	}

	//Case 12
	if (abs(T.dot(Ax)*Rzz - T.dot(Az)*Rxz) > abs(Wa*Rxz)+abs(Da*Rxz)+abs(Wb*Ryy)+abs(Hb*Rxy)){
		collideCheck = False;
		return collideCheck;
	}

	//Case 13
	if (abs(T.dot(Ay)*Rxx - T.dot(Ax)*Rxy) > abs(Wa*Rxx)+abs(Ha*Rxx)+abs(Hb*Rzz)+abs(Db*Ryz)){
		collideCheck = False;
		return collideCheck;
	}

	//Case 14
	if (abs(T.dot(Ay)*Rxy - T.dot(Ax)*Ryy) > abs(Wa*Rxy)+abs(Ha*Rxy)+abs(Wb*Rzz)+abs(Db*Rxz)){
		collideCheck = False;
		return collideCheck;
	}

	//Case 15
	if (abs(T.dot(Ay)*Rxz - T.dot(Ax)*Ryz) > abs(Wa*Rxz)+abs(Ha*Rxz)+abs(Wb*Ryz)+abs(Hb*Rxz)){
		collideCheck = False;
		return collideCheck;
	}


	ROS_WARN("Collision_detected");
	return collideCheck;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "example_eigen_plane_fit"); //node name
	ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
	return 0;
}
