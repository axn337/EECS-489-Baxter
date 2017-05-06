//RRT_Baxter.cpp
//Ammar Nahari axn337
//04/23/2017
//THis code performs rapidly exploring random tree algorithm (RRT) on a rethink baxter robot.

#include <ros/ros.h>

//basic include statements for interfacing with ROS and Baxter
#include <algorithm>    // std::find
#include "baxter_core_msgs/JointCommand.h"
//#include <baxter_core_msgs/SEAJointState.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <cmath>
#include <complex>

#include <fstream>
#include <iostream>
#include <iomanip>
#include <map>
#include <math.h>


#include <std_msgs/Float64.h>
#include <string>
#include <sstream>
#include <string>
#include <stdlib.h>     /* abs1 */
#include <valarray>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>




//global variables:----------------------------------------------------------------------

std::vector<std::vector<double> > g_vertices_set; //2-D vectors contains the verticies in the free work-space
//std::vector<std::vector<std::vector<double> > > g_edges_set; //3-dimension vector that contains the set of collision free edges
int g_number_of_samples=10000;//select number of samples in free workspace
double g_max_edge_length=0.05;//TODO: select value

std::vector<double> g_initial_vertex;//initial vertex setting up //g_initial_vertex
std::vector<double> g_trgt_vertex;//TODO: put real values
double trgt_zone=0.01 ;
bool g_trgt_reached= false;


//RRT_Functions-------------------------------------------------------------------------------
std::vector<double> RandomAssignment(){
	//a function that assignes a random variable to rand_vertex
	//TODO: fill up with a random generator code

	double generator;
	std::vector<double> rand_vertex;

	double range_from[]={-1.7016,-2.147,-3.0541,-0.05, -3.059,-1.5707,-3.059};//TODO: change the range for each joint seperatly
    double range_to[]= {  1.7016, 1.047, 3.0541, 2.618, 3.059, 2.094,  3.059};

	generator= ((double) rand() / (RAND_MAX));
	if(generator>0.7){
		rand_vertex=g_trgt_vertex; 
	}
	else{
		for(int i=0; i<7; ++i){
			generator= ((double) rand() / (RAND_MAX));
			rand_vertex.push_back((range_to[i]-range_from[i])*generator+range_from[i]);
		}
	}	
	return rand_vertex;

}
double v_distance(std::vector<double> vertex_1, std::vector<double> vertex_2){
	//a function that takes two verteces and return the distance between them
	double dist=0;

	for(int i=0; i<7; i++){
		dist+=pow((vertex_1[i]-vertex_2[i]),2);
	}
	
	return sqrt(dist);
}
double NearestVertex(std::vector<double> vertex){
	//TODO: fill up
	//A function that chooses the nearest node in the verteces set

	std::vector<double> nearest= g_vertices_set[0];//Temporary assumbtion in seek of comparison
	double nearest_index=0;
	
	for(int i=1; i<g_vertices_set.size(); i++){ //TODO: check g_vertices_set.size, 2-D vector
		
	//ROS_WARN("i eval: %d", i);

		//double current_dist = v_distance(vertex,g_vertices_set[i]);
		//double initial_dist = v_distance(vertex,g_vertices_set[0]);
		//ROS_INFO("Current dist: %f, Initial dist: %f",current_dist, initial_dist);

		if(v_distance(vertex,g_vertices_set[i])<v_distance(vertex,nearest)){
			nearest=g_vertices_set[i]; //replace nearest with the new shortest in distance vertex 
			nearest_index= (double)i;
			
		}
	}
	ROS_INFO("i nearest: %f", nearest_index);
	return nearest_index;
}



std::vector<double> Steer(double nearest_index, std::vector<double> rand_vertex){
	//returns a node in the same direction  of the random_vertex, but with a maximum distance from the nearest vertex

	//TODO: Check calculation
	std::vector<double> nearest_vertex= g_vertices_set[nearest_index];

	double current_dist=0;// distance btw nearest_vertex and rand_vertex

	current_dist=v_distance(nearest_vertex,rand_vertex);

	double dist_ratio= g_max_edge_length/current_dist; // ratio btw maximum edge lenght and current_dist
	std::vector<double> steered_vertex; 

	if(dist_ratio>1){
//		rand_vertex.push_back(g_index_counter);
//		++g_index_counter;
		return rand_vertex;
	}
	else{
		std::vector<double> difference_vector;
		for(int i=0; i<7; ++i){
			//calculate coordinates in 7 dimensions
			difference_vector.push_back(rand_vertex[i] - nearest_vertex[i]);
			difference_vector[i] *= dist_ratio;
		}
		for(int i=0; i<7; i++){
			steered_vertex.push_back(nearest_vertex[i] + difference_vector[i]);
		}
		
//		steered_vertex.push_back(g_index_counter);
//		++g_index_counter;
		return steered_vertex;
	}
}
//collision check function ------------------------------------------------
/*
bool obstacle_free_vertex(std::vector<double> new_vertex){
	//The purpose of this function is to return true if new_vertex is in obsticle free ws
	//and false otherwise
	
	//TODO: fill up
	//get code from Tony
	return true;

}*/
double abs1(double inputNum) {
	if (inputNum<0) {
		inputNum = -inputNum;
		return inputNum;
	}
	return inputNum;
}

bool check_for_collisions(Eigen::Matrix<double, 4, 4> box_1, Eigen::Matrix<double, 4, 4> box_2, Eigen::Matrix<double, 3, 1> extent_1, Eigen::Matrix<double, 3, 1> extent_2){

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

	double Wa = extent_1(0);
	double Ha = extent_1(1);
	double Da = extent_1(2);

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

	double Wb = extent_2(0);
	double Hb = extent_2(1);
	double Db = extent_2(2);

	//Eigen::Vector3d T = Pb - Pa;//not sure on the syntax of this one
	Eigen::Vector3d T;
	double T1; double T2; double T3;
	T1 = Pb(0) - Pa(0);
	T2 = Pb(1) - Pa(1);
	T3 = Pb(2) - Pa(2);
	T << T1, T2, T3;

	double Rxx = Ax.dot(Bx);
	double Rxy = Ax.dot(By);
	double Rxz = Ax.dot(Bz);
	double Ryy = Ay.dot(By);
	double Ryz = Ay.dot(Bz);
	double Rzz = Az.dot(Bz);

	//ROS_INFO("Debuggin: Ha: %f Wb: %f Rxy: %f Hb: %f Ryy: %f Db: %f Ryz: %f", Ha, Wb, Rxy, Hb, Ryy, Db, Ryz);
	//ROS_INFO("MORE DEBUG: Pb: %f,%f,%f, Pa: %f,%f,%f", Pb(0), Pb(1), Pb(2), Pa(0), Pa(1), Pa(2));
	//ROS_INFO("Debug Ay: %f,%f,%f",Ay(0),Ay(1),Ay(2));
	//ROS_INFO("Debug T: %f,%f,%f", T(0),T(1),T(2));

	//double tempNum = Wb*Rxy;
	//double tempCheck1 = Ha + abs1(tempNum) + abs1(Hb * Ryy) + abs1(Db * Ryz);
	//double tempCheck2 = abs1(T.dot(Ay));
	//ROS_WARN("Check case 2: %f > %f ?", tempCheck2, tempCheck1);
	
	//Case 1
	if (abs1(T.dot(Ax)) > Wa + abs1(Wb * Rxx) + abs1(Hb * Rxy) + abs1(Db * Rxz)){
		collideCheck = false;
		//ROS_INFO("CASE 1 GOOD");
		return collideCheck;
	}
	//Case 2
	if (abs1(T.dot(Ay)) > Ha + abs1(Wb * Rxy) + abs1(Hb * Ryy) + abs1(Db * Ryz)){
		collideCheck = false;
		//ROS_INFO("CASE 2 GOOD");
		return collideCheck;
	}
	//Case 3
	if (abs1(T.dot(Az)) > Da + abs1(Wb * Rxz) + abs1(Hb * Ryz) + abs1(Db * Rzz)){
		collideCheck = false;
		//ROS_INFO("CASE 3 GOOD");
		return collideCheck;
	}
	//Case 4
	if (abs1(T.dot(Bx)) > Wb + abs1(Wa * Rxx) + abs1(Ha * Rxy) + abs1(Da * Rxz)){
		collideCheck = false;
		//ROS_INFO("CASE 4 GOOD");
		return collideCheck;
	}
	//Case 5
	if (abs1(T.dot(By)) > Hb + abs1(Wa * Rxy) + abs1(Ha * Ryy) + abs1(Da * Ryz)){
		collideCheck = false;
		//ROS_INFO("CASE 5 GOOD");
		return collideCheck;
	}
	//Case 6
	if (abs1(T.dot(Bz)) > Db + abs1(Wa * Rxz) + abs1(Ha * Ryz) + abs1(Da * Rzz)){
		collideCheck = false;
		//ROS_INFO("CASE 6 GOOD");
		return collideCheck;
	}
	//Case 7
	if (abs1(T.dot(Az)*Rxy - T.dot(Ay)*Rxz) > abs1(Ha*Rxz)+abs1(Da*Rxy)+abs1(Hb*Rxz)+abs1(Db*Rxy)){
		collideCheck = false;
		//ROS_INFO("CASE 7 GOOD");
		return collideCheck;
	}
	//Case 8
	if (abs1(T.dot(Az)*Ryy - T.dot(Ay)*Ryz) > abs1(Ha*Ryz)+abs1(Da*Ryy)+abs1(Wb*Rxz)+abs1(Db*Rxx)){
		collideCheck = false;
		//ROS_INFO("CASE 8 GOOD");
		return collideCheck;
	}

	//Case 9
	if (abs1(T.dot(Az)*Ryz - T.dot(Ay)*Rzz) > abs1(Ha*Rzz)+abs1(Da*Ryz)+abs1(Wb*Rxy)+abs1(Hb*Rxx)){
		collideCheck = false;
		//ROS_INFO("CASE 9 GOOD");
		return collideCheck;
	}

	//Case 10
	if (abs1(T.dot(Ax)*Rxz - T.dot(Az)*Rxx) > abs1(Wa*Rxz)+abs1(Da*Rxx)+abs1(Hb*Ryz)+abs1(Db*Ryy)){
		collideCheck = false;
		//ROS_INFO("CASE 10 GOOD");
		return collideCheck;
	}

	//Case 11
	if (abs1(T.dot(Ax)*Ryz - T.dot(Az)*Rxy) > abs1(Wa*Ryz)+abs1(Da*Rxy)+abs1(Wb*Ryz)+abs1(Db*Rxy)){
		collideCheck = false;
		//ROS_INFO("CASE 11 GOOD");
		return collideCheck;
	}

	//Case 12
	if (abs1(T.dot(Ax)*Rzz - T.dot(Az)*Rxz) > abs1(Wa*Rzz)+abs1(Da*Rxz)+abs1(Wb*Ryy)+abs1(Hb*Rxy)){
		collideCheck = false;
		//ROS_INFO("CASE 12 GOOD");
		return collideCheck;
	}

	//Case 13
	if (abs1(T.dot(Ay)*Rxx - T.dot(Ax)*Rxy) > abs1(Wa*Rxy)+abs1(Ha*Rxx)+abs1(Hb*Rzz)+abs1(Db*Ryz)){
		collideCheck = false;
		//ROS_INFO("CASE 13 GOOD");
		return collideCheck;
	}

	//Case 14
	if (abs1(T.dot(Ay)*Rxy - T.dot(Ax)*Ryy) > abs1(Wa*Ryy)+abs1(Ha*Rxy)+abs1(Wb*Rzz)+abs1(Db*Rxz)){
		collideCheck = false;
		//ROS_INFO("CASE 14 GOOD");
		return collideCheck;
	}

	//Case 15
	if (abs1(T.dot(Ay)*Rxz - T.dot(Ax)*Ryz) > abs1(Wa*Ryz)+abs1(Ha*Rxz)+abs1(Wb*Ryz)+abs1(Hb*Rxz)){
		collideCheck = false;
		//ROS_INFO("CASE 15 GOOD");
		return collideCheck;
	}


	//ROS_WARN("Collision Detected (in function)");
	return collideCheck;
}

bool obstacle_free_vertex(std::vector<double> angles) {
	double T1 = angles[0];
	double T2 = angles[1];
	double T3 = angles[2];
	double T4 = angles[3];
	double T5 = angles[4];
	double T6 = angles[5];
	double T7 = angles[6];
	
	Eigen::Matrix<double,4,4> joint1; 
				 joint1	<< 	cos(T1), -1.0*sin(T1),   0,   0,
							sin(T1),      cos(T1),   0,   0,
							0,            0, 1.0,   0,
							0,            0,   0, 1.0;
	
	
	Eigen::Matrix<double,4,4> joint2;
				 joint2 << 	1.0,            0,       0,                                  0,
							0,      cos(T2), sin(T2), 0.069 - 0.27*sin(T2) - 0.069*cos(T2),
							0, -1.0*sin(T2), cos(T2),  0.069*sin(T2) - 0.27*cos(T2) + 0.27,
							0,            0,       0,                                  1.0;
										
	Eigen::Matrix<double,4,4> joint3; 
				 joint3	<< 	cos(T3),   0, sin(T3),       -0.27*sin(T3),
							0, 1.0,       0,                   0,
							-1.0*sin(T3),   0, cos(T3), 0.27 - 0.27*cos(T3),
							0,   0,       0,                 1.0;		

	Eigen::Matrix<double,4,4> joint4; 
				 joint4	<< 	1.0,            0,       0,                                     0,
							0,      cos(T4), sin(T4), 0.433 - 0.201*sin(T4) - 0.433*cos(T4),
							0, -1.0*sin(T4), cos(T4), 0.433*sin(T4) - 0.201*cos(T4) + 0.201,
							0,            0,       0,                                   1.0;

	Eigen::Matrix<double,4,4> joint5; 
				 joint5	<< 	cos(T5),   0, sin(T5),        -0.201*sin(T5),
							0, 1.0,       0,                     0,
							-1.0*sin(T5),   0, cos(T5), 0.201 - 0.201*cos(T5),
							0,   0,       0,                   1.0;
													
	Eigen::Matrix<double,4,4> joint6; 
				 joint6	<<	1.0,            0,       0,                                     0,
							0,      cos(T6), sin(T6), 0.808 - 0.191*sin(T6) - 0.808*cos(T6),
							0, -1.0*sin(T6), cos(T6), 0.808*sin(T6) - 0.191*cos(T6) + 0.191,
							0,            0,       0,                                   1.0;

	Eigen::Matrix<double,4,4> joint7; 
				 joint7	<<	cos(T7),   0, sin(T7),        -0.191*sin(T7),
							0, 1.0,       0,                     0,
							-1.0*sin(T7),   0, cos(T7), 0.191 - 0.191*cos(T7),
							0,   0,       0,                   1.0;
													
										
	
	//Box 1
	Eigen::Matrix<double,4,4> box1;
				 box1	 << 1, 0, 0, 0,
							0, 1, 0, 0,
							0, 0, 1, 0.27035/2,
							0, 0, 0, 1;
	Eigen::Matrix<double,3,1> extent_1;
				//extent_1 << .060, 0.060, 0.27035/2-0.05;
				extent_1 << 0.0, 0.0, 0.0;
	
	//Box 2
	Eigen::Matrix<double,4,4> box2;
				 box2	 <<	1, 0, 0, 0,
							0, 1, 0, 0.069+0.36445/2,
							0, 0, 1, 0.27035,
							0, 0, 0, 1;
	Eigen::Matrix<double,3,1> extent_2;
				extent_2 << 0.06, 0.27/2-0.05, 0.06;
				//extent_2 << 0.0, 0.0, 0.0;
	

	//Box 3
	Eigen::Matrix<double,4,4> box3;
				 box3	 <<	1, 0, 0, 0,
							0, 1, 0, 0.069+0.36435+0.37429/2,
							0, 0, 1, 0.27035-0.069,
							0, 0, 0, 1;
	Eigen::Matrix<double,3,1> extent_3;
				extent_3 << 0.06, 0.370/2-0.05, 0.06;
				//extent_3 << 0.0, 0.0, 0.0;
	
	//Box 4
	Eigen::Matrix<double,4,4> box4;
				 box4	 << 	1, 0, 0, 0,
								0, 1, 0, 0.069+0.36435+0.37429+.229525/2,
								0, 0, 1, 0.27035-0.069-0.01,
								0, 0, 0, 1;
	Eigen::Matrix<double,3,1> extent_4;
				extent_4 << 0.06, 0.20/2-0.05, 0.06;
				//extent_4 << 0.0, 0.0, 0.0;
	
	//Obstacles:
	Eigen::Matrix<double,4,4> obst1; 
				 obst1	<< 	1, 0, 0, 0,
							0, 1, 0, 20,
							0, 0, 1, 0,
							0, 0, 0, 1;
									  
	Eigen::Matrix<double,3,1> Eobst1;
				 Eobst1	 << 0.5, 0.01, 0.05;
				//Eobst1 << 0.0, 0.0, 0.0;
	
	//Check self collision:
	Eigen::Matrix<double,4,4> rotatedBox1;
	Eigen::Matrix<double,4,4> rotatedBox2;
	Eigen::Matrix<double,4,4> rotatedBox3;
	Eigen::Matrix<double,4,4> rotatedBox4;

	rotatedBox1 = joint1*box1;
	rotatedBox2 = joint1*joint2*joint3*box2;
	rotatedBox3 = joint1*joint2*joint3*joint4*joint5*box3;
	rotatedBox4 = joint1*joint2*joint3*joint4*joint5*joint6*joint7*box4;
	
	std::vector< Eigen::Matrix<double,4,4> > rotatedArms;
	rotatedArms.push_back(rotatedBox1);
	rotatedArms.push_back(rotatedBox2);
	rotatedArms.push_back(rotatedBox3);
	rotatedArms.push_back(rotatedBox4);

	std::vector< Eigen::Matrix<double,3,1> > armExtents;
	armExtents.push_back(extent_1);
	armExtents.push_back(extent_2);
	armExtents.push_back(extent_3);
	armExtents.push_back(extent_4);
	
	bool leave = false;
	for (int i = 0; i < 4; i ++) {
		Eigen::Matrix<double,4,4> selfBlock;
		selfBlock << rotatedArms[i];
		//selfBlock << rotatedArms.block<4,4,1>(0,0,i);
		Eigen::Matrix<double,3,1> selfExtent;
		selfExtent << armExtents[i];
		//selfExtent << armExtents.block<3,1,1>(0,0,i);
		for (int i1 = i+1; i1 < 4; i1 ++) {
			Eigen::Matrix<double,4,4> checkingBlock;
			checkingBlock << rotatedArms[i1];
			//checkingBlock << rotatedArms.block<4,4>(0,0,i1);
			Eigen::Matrix<double,3,1> checkingExtent;
			checkingExtent << armExtents[i1];
			//checkingExtent << armExtents.block<3,1>(0,0,i1);
			leave = check_for_collisions(selfBlock, checkingBlock, selfExtent, checkingExtent);
			if (leave == true) {
				//ROS_WARN("BLOCK %d AND %d",i,i1);
				ROS_WARN("Box %d and %d", i, i1);
				ROS_WARN("Self Collision Detected");
				return false;
			}
		}
		
		//Collision check:
		leave = check_for_collisions(selfBlock, obst1, selfExtent, Eobst1);
		if (leave == true) {
			ROS_WARN("Box %d",i);
			ROS_WARN("Obstacle collision detected");
			return false;
		}
	}
	return true;
	
}

std::vector<double> vectorAdd(std::vector<double> vect1, std::vector<double> vect2, int op) {
	std::vector<double> outputVect;
	for (int i = 0; i < vect1.size(); i++) {
		if (op > 0) {
			outputVect.push_back(vect1[i] - vect2[i]);
		}
		else {
			outputVect.push_back(vect1[i] + vect2[i]);
		}
	}
	return outputVect;
}

bool obstacle_free_edge(std::vector<double> new_vertex,double nearest_index){
	//The purpose of this function is to return true if the edge is in obsticle free ws
	//and false otherwise
	//TODO: fill up
	//get code from Tony
	int incrementNum = 10;
	
	std::vector<double> nearest_vertex= g_vertices_set[(int) nearest_index];
	double current_dist = 0;// distance btw nearest_vertex and rand_vertex
	current_dist=v_distance(nearest_vertex,new_vertex);
	//std::vector<double> directionVect = (new_vertex - nearest_vertex)/current_dist;
	std::vector<double> directionVect = vectorAdd(new_vertex, nearest_vertex, -1);
	for (int i = 0; i < directionVect.size(); i ++) {
		directionVect[i] = directionVect[i]/current_dist;
	}

	//ROS_INFO("Path check: Curent vertex: %f,%f,%f,%f,%f,%f,%f",new_vertex[0],new_vertex[1],new_vertex[2],new_vertex[3],new_vertex[4],new_vertex[5],new_vertex[6]);
	//ROS_INFO("Target Vertex: %f,%f,%f,%f,%f,%f,%f",nearest_vertex[0],nearest_vertex[1],nearest_vertex[2],nearest_vertex[3],nearest_vertex[4],nearest_vertex[5],nearest_vertex[6]);
	//ROS_INFO("Direction: %f,%f,%f,%f,%f,%f,%f",directionVect[0],directionVect[1],directionVect[2],directionVect[3],directionVect[4],directionVect[5],directionVect[6]);
	
	double incrementDist = current_dist/incrementNum;

	for (int i = 1; i <= incrementNum; i ++) {
		//std::vector<double> testing_vertex = nearest_vertex + directionVect*(incrementDist*i);
		//ROS_INFO("Path evaluating point %f", i);
		std::vector<double> currentDistance;
		for (int i1 = 0; i1 < directionVect.size(); i1 ++) {
			currentDistance.push_back(directionVect[i1]*incrementDist*i);
		}
		
		std::vector<double> testing_vertex;
		testing_vertex = vectorAdd(nearest_vertex, currentDistance, 1);

		//ROS_INFO("Test current point: %f,%f,%f,%f,%f,%f,%f",testing_vertex[0],testing_vertex[1],testing_vertex[2],testing_vertex[3],testing_vertex[4],testing_vertex[5],testing_vertex[6]);
		
		bool collisionFree = obstacle_free_vertex(testing_vertex);
		
		if (collisionFree == false) {
			ROS_WARN("Collided Path");
			ROS_INFO(" ");

			return false;
		}
		//ROS_INFO("Free test point");
	}

	ROS_INFO("Free Path");
	ROS_INFO("Added: %f,%f,%f,%f,%f,%f,%f",new_vertex[0],new_vertex[1],new_vertex[2],new_vertex[3],new_vertex[4],new_vertex[5],new_vertex[6]);
	ROS_INFO("Connected to: %f", nearest_index);
	ROS_INFO(" ");
	return true;
}
//finding_path -----------------------------------------------------------------------------

std::vector<std::vector<double> > FindPath(){
	
	//A function that finds the parent of each vertex starting from the target and moving towards the initial node, and construct a path accordingly 
	//ROS_INFO("trying to find the path");

	std::vector<std::vector<double> > path;
	int index;

	//construct reversed path from target to initial vertex

	double i=g_vertices_set.size()-1;
	int j=1;

	do{	
		path.push_back(g_vertices_set[i]); //adds the parent vertex to the path

		ROS_INFO("%d- element number %f added to path ",j,i);
		
		i=  g_vertices_set[i][7];
		j++;

	
	}while(i>-1);

	

	// reverse back the path so that it goes from initial to target vertices
	std::reverse(path.begin(),path.end());

	//ROS_INFO("Path generated with %d vertices", path.size());


	return path;

} 

void MoveToTRGT(std::vector<std::vector<double> > bath){
	//excecute the movement from current pose to target pose
	
}


//Main function-------------------------------------------------------------------------------

int main(int argc, char **argv){
	//basic ROS setup for communications with baxter
	ros::init(argc, argv, "RRT_Baxter");
	ros::NodeHandle n;
	ros::Publisher action_publisher = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command",1);


	std::vector<double> rand_vertex; //generate random vertex 
	//std::vector<double> nearest_vertex; //holds the value of the nearest vertex
	double nearest_index;
	std::vector<double> new_vertex; // steer the vertex with a Max distance
	std::vector<std::vector<double> > path;// contains a set of nodesfrom the goal to the initial pose
	ros::Time start = ros::Time::now();
	ros::Rate loop_rate(80);
  	baxter_core_msgs::JointCommand cmd;

	cmd.mode = baxter_core_msgs::JointCommand::POSITION_MODE;

	cmd.names.push_back("left_s0");
	cmd.names.push_back("left_s1");
	cmd.names.push_back("left_e0");
	cmd.names.push_back("left_e1");
	cmd.names.push_back("left_w0");
	cmd.names.push_back("left_w1");
	cmd.names.push_back("left_w2");

	cmd.command.resize(cmd.names.size());

	//command an initial motion to 0, then wait 5 seconds
	cmd.command[0] = 0;
	cmd.command[1] = 0;
	cmd.command[2] = 0;
	cmd.command[3] = 0;
	cmd.command[4] = 0;
	cmd.command[5] = 0;
	cmd.command[6] = 0;
	
	ROS_INFO("moving to initial pose");

	while((ros::Time::now() - start) < ros::Duration(2)) {
			action_publisher.publish(cmd);
			ros::spinOnce();
			loop_rate.sleep();

	}

	//spicify initial and trgt vertix and add it to the vertices set
	ros::Duration(1);
	
	//ROS_INFO("calculating RRT");

	for(int i=0;i<7;i++){
		g_initial_vertex.push_back(1.0);
		g_trgt_vertex.push_back(0.0);
	}

	bool initial_check = obstacle_free_vertex(g_initial_vertex);
	bool target_check = obstacle_free_vertex(g_trgt_vertex);

	g_initial_vertex.push_back(-1);

	if (initial_check != true) {
		ROS_WARN("INITIAL COLLIDE");
		return 0;
	}
	if (target_check != true) {
		ROS_WARN("TARGET COLLIDE");
		return 0;
	}


 	//double g_initial[]={0,0,0,0,0,0,0,-1};
	//double g_trgt[]={1,1,1,1,1,1,1};

	//g_initial_vertex (g_initial, g_initial + sizeof(g_initial) / sizeof(double) );//initial vertex setting up //g_initial_vertex
	//g_trgt_vertex (g_initial, g_initial + sizeof(g_initial) / sizeof(double) );
	g_vertices_set.push_back(g_initial_vertex); // add initial vertex to the vertices set

	int i=1;

	double shortest_dist = 99999.9;

	std::vector<double> nearest_vertex_temp;
	std::vector<double> nearest_vertex;
	std::vector<double> direction_vector;

	do{

		rand_vertex=RandomAssignment(); // assign a random variable to rand_vertex

		nearest_index=NearestVertex(rand_vertex); // look for the nearest vertex and its index in g_vertices_set

		//Get nearest vertex:
		nearest_vertex_temp = g_vertices_set[(int) nearest_index];
		for (int i = 0; i < 7; i++) {
			nearest_vertex.push_back(nearest_vertex_temp[i]);
		}

		new_vertex=Steer(nearest_index, rand_vertex); //steer to the maximum edge legnth

		direction_vector = vectorAdd(new_vertex,nearest_vertex,-1);
		double direction_distance = v_distance(new_vertex, nearest_vertex);
		ROS_INFO("direction vector (pre-norm): %f,%f,%f,%f,%f,%f,%f",direction_vector[0],direction_vector[1],direction_vector[2],direction_vector[3],direction_vector[4],direction_vector[5],direction_vector[6]);
		for (int i = 0; i < 7; i++) {
			direction_vector[i] = direction_vector[i]/direction_distance;
		}
		ROS_INFO("direction distance: %f", direction_distance);
		ROS_INFO("direction vector: %f,%f,%f,%f,%f,%f,%f",direction_vector[0],direction_vector[1],direction_vector[2],direction_vector[3],direction_vector[4],direction_vector[5],direction_vector[6]);

		bool obstacle_free_check = obstacle_free_edge(new_vertex, nearest_index);
		int counter = 0;
		while (obstacle_free_check != true) {
			if (counter == 5) {
				break;
			}
			direction_distance = direction_distance/2;
			for (int i = 0; i < 7; i++) {
				new_vertex[i] = nearest_vertex[i] + direction_vector[i]*direction_distance;
			}

			ROS_INFO("New half distance vertex: %f,%f,%f,%f,%f,%f,%f", new_vertex[0],new_vertex[1],new_vertex[2],new_vertex[3],new_vertex[4],new_vertex[5],new_vertex[6]);
			obstacle_free_check = obstacle_free_edge(new_vertex, nearest_index);

			counter = counter + 1;
		}

		//if(obstacle_free_vertex(new_vertex)&&obstacle_free_edge(new_vertex,nearest_index)){
		if(obstacle_free_edge(new_vertex,nearest_index)){
			g_vertices_set.push_back(new_vertex); //add new_vertex to the vertices set
			g_vertices_set[i].push_back(nearest_index); //add the index of the parent
			//g_edges_set[0].push_back(new_vertex); //add the new edge to the edges set.
			//g_edges_set[1].push_back(nearest_vertex); 

			//optional: AddGraph(new_vertex,nearest_vertex) a function that visually present the RRT process
			//ROS_INFO("New node= %f %f %f %f %f %f %f %f",g_vertices_set[i][0],g_vertices_set[i][1],g_vertices_set[i][2],g_vertices_set[i][3],g_vertices_set[i][4],g_vertices_set[i][5],g_vertices_set[i][6],g_vertices_set[i][7]);

			i++; // increase the counter only when the new edge and vertex are added. If not repeat the loop.
			ROS_INFO("-------------------Point Added--------------------------");
			ROS_INFO("Number of points on graph: %d", i);

			double distance_to_goal = v_distance(new_vertex, g_trgt_vertex);
			if (shortest_dist > distance_to_goal) {
				shortest_dist = distance_to_goal;
			}
			ROS_INFO("Current distance %f ", distance_to_goal);
			ROS_INFO("Shortest distance %f ", shortest_dist);

			if(distance_to_goal <= g_max_edge_length) {
				ROS_INFO("***************IN MOVE RANGE**********************");
				if(obstacle_free_edge(g_trgt_vertex,i-1)){

					g_trgt_reached = true;

					g_vertices_set.push_back(g_trgt_vertex);
					g_vertices_set[i].push_back(i-1);
					ROS_INFO("RRT reached goal");
					ros::Duration(1);
					break;
				}
			}
		}
//		else{
//			--g_index_counter;
//		}
		//ROS_INFO("%d %d",g_vertices_set.size(),g_vertices_set[i].size());


		if(v_distance(new_vertex, g_trgt_vertex)<trgt_zone){
			g_trgt_reached=true; 
				

			ROS_INFO("RRT reached goal");
			ros::Duration(1);
			break;
		}



	}while(i<g_number_of_samples);

	//ROS_INFO("number of vertices in vertices_set =%d",g_vertices_set.size());

	ros::Duration(1);
	if(g_trgt_reached==false){
		ROS_INFO("unable to connect to goal");
		return 0;
	}
	
	ROS_INFO("finding path..");
	//plan the path and store it in "path"
	path= FindPath( );
	ros::Duration(1);

	//path movement execution-------------------------
	ROS_INFO("moving to goal");
	ros::Duration(3);

	for(int j=1; j<path.size()-1; j++){
		//ROS_INFO("path node %d= %f %f %f %f %f %f %f",j,path[j][0],path[j][1],path[j][2],path[j][3],path[j][4],path[j][5],path[j][6]);
		
		for(int k=0; k<path[j].size()-1;k++){ //to avoid parent index		
			//ROS_WARN("k=%d",k);
			cmd.command[k]=path[j][k];
		}
		//ROS_INFO("visit location number %d in path",j+1);
	

		start = ros::Time::now();
		while((ros::Time::now() - start) < ros::Duration(0.05)) {
			action_publisher.publish(cmd);
			//ros::spinOnce();
			loop_rate.sleep();

		}
	}
	

	//MoveToTRGT(bath);




	return 0;
}
