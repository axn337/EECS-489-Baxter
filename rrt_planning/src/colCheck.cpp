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

#include <math.h>

//Eigen Vector3d has a .dot member function

//Matrix<float, 7, 1>

Eigen::Matrix<float,4,4> forward_kinematics(std::vec<double> angles) {
	double T1 = angels[0];
	double T2 = angles[1];
	double T3 = angles[2];
	double T4 = angles[3];
	double T5 = angles[4];
	double T6 = angles[5];
	double T7 = angles[6];
	
	Eigen::Matrix<float,4,4> joint1 << 	 cos(T1), -1.0*sin(T1),   0,   0,
										 sin(T1),      cos(T1),   0,   0,
										       0,            0, 1.0,   0,
										       0,            0,   0, 1.0;
	
	Eigen::Matrix<float,4,4> joint2 << 	 1.0,            0,       0,                                    0,
										   0,      cos(T2), sin(T2), 0.069 - 0.27*sin(T2) - 0.069*cos(T2),
										   0, -1.0*sin(T2), cos(T2),  0.069*sin(T2) - 0.27*cos(T2) + 0.27,
										   0,            0,       0,                                  1.0;
										
	Eigen::Matrix<float,4,4> joint3 << 	      cos(T3),   0, sin(T3),       -0.27*sin(T3),
										            0, 1.0,       0,                   0,
										 -1.0*sin(T3),   0, cos(T3), 0.27 - 0.27*cos(T3),
										            0,   0,       0,                 1.0;		

	Eigen::Matrix<float,4,4> joint4 << 	 1.0,            0,       0,                                     0,
										   0,      cos(T4), sin(T4), 0.433 - 0.201*sin(T4) - 0.433*cos(T4),
										   0, -1.0*sin(T4), cos(T4), 0.433*sin(T4) - 0.201*cos(T4) + 0.201,
										   0,            0,       0,                                   1.0;

	Eigen::Matrix<float,4,4> joint5 << 	      cos(T5),   0, sin(T5),        -0.201*sin(T5),
										            0, 1.0,       0,                     0,
										 -1.0*sin(T5),   0, cos(T5), 0.201 - 0.201*cos(T5),
										            0,   0,       0,                   1.0;
													
	Eigen::Matrix<float,4,4> joint6 <<	 1.0,            0,       0,                                     0,
										   0,      cos(T6), sin(T6), 0.808 - 0.191*sin(T6) - 0.808*cos(T6),
										   0, -1.0*sin(T6), cos(T6), 0.808*sin(T6) - 0.191*cos(T6) + 0.191,
										   0,            0,       0,                                   1.0;

	Eigen::Matrix<float,4,4> joint7 <<	      cos(T7),   0, sin(T7),        -0.191*sin(T7),
										            0, 1.0,       0,                     0,
										 -1.0*sin(T7),   0, cos(T7), 0.191 - 0.191*cos(T7),
										            0,   0,       0,                   1.0;
	
	//Box 1
	Eigen::Matrix<float,4,4> box1 << 	1, 0, 0, 0,
										0, 1, 0, 0,
										0, 0, 1, 0.27035/2,
										0, 0, 0, 1;
	Eigen::Matrix<float,3,1> extent_1 << .060, 0.060, 0.27035/2;
	
	//Box 2
	Eigen::Matrix<float,4,4> box2 <<	1, 0, 0, 0,
										0, 1, 0, 0.069+0.36445/2,
										0, 0, 1, 0.27035,
										0, 0, 0, 1;
	Eigen::Matrix<float,3,1> extent_2 << 0.06, 0.360/2, 0.06;
	

	//Box 3
	Eigen::Matrix<float,4,4> box3 <<	1, 0, 0, 0,
										0, 1, 0, 0.069+0.36435+0.37429/2,
										0, 0, 1, 0.27035-0.069,
										0, 0, 0, 1;
	Eigen::Matrix<float,3,1> extent_3 << 0.06, 0.370/2, 0.06;
	
	//Obstacles:
	Eigen::Matrix<float,4,4> obst1 << 1, 0, 0, 0,
									  0, 1, 0, 0.069+0.36435+0.37429+.229525/2,
									  0, 0, 1, 0.27035-0.069-0.01,
									  0, 0, 0, 1;
									  
	Eigen::Matrix<float,3,1> Eobst1 << 0.2,10,10;
	
	//Check self collision:
	
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
