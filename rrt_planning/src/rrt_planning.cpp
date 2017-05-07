//RRT_Baxter.cpp
//Ammar Nahari axn337
//04/23/2017
//THis code performs rapidly exploring random tree algorithm (RRT) on a rethink baxter robot.


//basic include statements for interfacing with ROS and Baxter
#include <algorithm>    // std::find
#include "baxter_core_msgs/JointCommand.h"
//#include <baxter_core_msgs/SEAJointState.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <cmath>
#include <complex>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <map>
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <string>
#include <sstream>
#include <string>
#include <stdlib.h>     /* abs */
#include <valarray>
#include <vector>




//global variables:----------------------------------------------------------------------

std::vector<std::vector<double> > g_vertices_set; //2-D vectors contains the verticies in the free work-space
//std::vector<std::vector<std::vector<double> > > g_edges_set; //3-dimension vector that contains the set of collision free edges
int g_number_of_samples=10000;//select number of samples in free workspace
double g_max_edge_length=0.01;//TODO: select value

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

	double range_from[]={-1.5,-1.5,-1.5,-1.5-1.5,-1.5,-1.5};//TODO: change the range for each joint seperatly
    double range_to[]= {1.5,1.5,1.5,1.5,1.5,1.5,1.5};

	generator= ((double) rand() / (RAND_MAX));
	if(generator>0.9){
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
		

		if(v_distance(vertex,g_vertices_set[i])<v_distance(vertex,nearest)){
			nearest=g_vertices_set[i]; //replace nearest with the new shortest in distance vertex 
			nearest_index= (double)i;
		}
	}
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

bool obstacle_free_vertex(std::vector<double> new_vertex){
	//The purpose of this function is to return true if new_vertex is in obsticle free ws
	//and false otherwise
	
	//TODO: fill up
	//get code from Tony
	return true;

}


bool obstacle_free_edge(std::vector<double> new_vertex,double nearest_index){

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

		//ROS_INFO("%d- element number %f added to path ",j,i);
		
		i=  g_vertices_set[i][7];
		j++;

	
	}while(i>-1);

	

	// reverse back the path so that it goes from initial to target vertices
	std::reverse(path.begin(),path.end());

	ROS_INFO("Path generated with %d vertices", path.size());


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
	cmd.command[0] = 1.2;
	cmd.command[1] = 1.2;
	cmd.command[2] = 1.2;
	cmd.command[3] = 1.2;
	cmd.command[4] = 1.2;
	cmd.command[5] = 1.2;
	cmd.command[6] = 1.2;
	
	ROS_INFO("moving to initial pose");

	while((ros::Time::now() - start) < ros::Duration(1)) {
			action_publisher.publish(cmd);
			ros::spinOnce();
			loop_rate.sleep();

	}

	//spicify initial and trgt vertix and add it to the vertices set
	ros::Duration(1);
	
	ROS_INFO("calculating RRT");

	for(int i=0;i<7;i++){
		g_initial_vertex.push_back(1.2);
		g_trgt_vertex.push_back(0.0);
	}
	g_initial_vertex.push_back(-1);
 	//double g_initial[]={0,0,0,0,0,0,0,-1};
	//double g_trgt[]={1,1,1,1,1,1,1};

	//g_initial_vertex (g_initial, g_initial + sizeof(g_initial) / sizeof(double) );//initial vertex setting up //g_initial_vertex
	//g_trgt_vertex (g_initial, g_initial + sizeof(g_initial) / sizeof(double) );
	g_vertices_set.push_back(g_initial_vertex); // add initial vertex to the vertices set

	int i=1;
	do{

		rand_vertex=RandomAssignment(); // assign a random variable to rand_vertex

		nearest_index=NearestVertex(rand_vertex); // look for the nearest vertex and its index in g_vertices_set

		new_vertex=Steer(nearest_index, rand_vertex); //steer to the maximum edge legnth

		if(obstacle_free_vertex(new_vertex)&&obstacle_free_edge(new_vertex,nearest_index)){
			g_vertices_set.push_back(new_vertex); //add new_vertex to the vertices set
			g_vertices_set[i].push_back(nearest_index); //add the index of the parent
			//g_edges_set[0].push_back(new_vertex); //add the new edge to the edges set.
			//g_edges_set[1].push_back(nearest_vertex); 

			//optional: AddGraph(new_vertex,nearest_vertex) a function that visually present the RRT process
			//ROS_INFO("New node= %f %f %f %f %f %f %f %f",g_vertices_set[i][0],g_vertices_set[i][1],g_vertices_set[i][2],g_vertices_set[i][3],g_vertices_set[i][4],g_vertices_set[i][5],g_vertices_set[i][6],g_vertices_set[i][7]);

			i++; // increase the counter only when the new edge and vertex are added. If not repeat the loop.
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

	ROS_INFO("number of vertices in vertices_set =%d",g_vertices_set.size());

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
