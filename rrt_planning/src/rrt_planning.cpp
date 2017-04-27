//RRT_Baxter.cpp
//Ammar Nahari axn337
//04/23/2017
//THis code performs rapidly exploring random tree algorithm (RRT) on a rethink baxter robot.


//basic include statements for interfacing with ROS and Baxter
#include<ros/ros.h>
#include "baxter_core_msgs/JointCommand.h"
#include<std_msgs/Float64.h>
#include<baxter_core_msgs/SEAJointState.h>

//include statements for special functionality used
#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <random>
#include <cmath>
#include <algorithm>    // std::find
#include <vector>       // std::vector


//global variables:----------------------------------------------------------------------

std::vector<std::vector<double> > g_vertices_set; //2-D vectors contains the verticies in the free work-space
//std::vector<std::vector<std::vector<double> > > g_edges_set; //3-dimension vector that contains the set of collision free edges
int g_number_of_samples=10000;//select number of samples in free workspace
double g_max_edge_length=0.5;//TODO: select value
std::std::vector<double> g_vertices_set[0]={0,0,0,0,0,0,0,-1};//initial vertex setting up //g_initial_vertex
std::std::vector<double> g_trgt_vertex={1,1,1,1,1,1,1};//TODO: put real values
double trgt_zone=0.2 
bool g_trgt_reached= false;
double g_index_counter=1;


//RRT_Functions-------------------------------------------------------------------------------
void RandomAssignment(std::vector<double> rand_vertex){
	//a function that assignes a random variable to rand_vertex
	//TODO: fill up with a random generator code

	const int range_from  = -1.5;//TODO: change the range for each joint seperatly
    const int range_to    = 1.5;

    std::random_device                  rand_dev;
    std::mt19937                        generator(rand_dev());
    std::uniform_int_distribution<int>  distr(range_from, range_to);
 
	for(int i=1; i<8; ++i){
		if(generator>.95){
			rand_vertex.push_back(g_trgt_vertex);
		}
		else{
			rand_vertex.push_back(distr(generator));
		}
	}
}

std::vector<double>,double NearestVertex(std::vector<double> v){
	//TODO: fill up
	//A function that chooses the nearest node in the verteces set

	std::vector<double> nearest= g_vertices_set(0);//Temporary assumbtion in seek of comparison
	double nearest_index=0;
	for(int i=0; i<g_vertices_set.size; ++i){ //TODO: check g_vertices_set.size, 2-D vector

		if(distance(v,g_vertices_set(i))<nearest){
			nearest=g_vertices_set(i) //replace nearest with the new shortest in distance vertex 
			nearest_index= (double)i;
		}
	}
	return nearest,nearest_index;
}

double distance(std::vector<double> vertex_1, std::vector<double> vertex_2){
	//a function that takes two verteces and return the distance between them
	double dist=0;

	for(int i=0; i<7; ++i){
		dist+=pow(vertex_1(i)-vertex_2(i),2)
	}
	return sqrt(dist);
}

std::vector<double> Steer(std::vector<double> nearest_vertex, std::vector<double> rand_vertex){
	//returns a node in the same direction  of the random_vertex, but with a maximum distance from the nearest vertex

	//TODO: Check calculation
	double current_dist=distance(nearest_vertex,rand_vertex);// distance btw nearest_vertex and rand_vertex
	double dist_ratio= g_max_edge_length/current_dist; // ratio btw maximum edge lenght and current_dist
	std::vector<double> steered_vertex; 

	if(dist_ratio>1){
//		rand_vertex.push_back(g_index_counter);
//		++g_index_counter;
		return rand_vertex;
	}
	else{
		for(int i=0; i<7; ++i){
			//calculate coordinates in 7 dimensions
			steered_vertex.push_back((1-dist_ratio)*nearest_vertex(i)+dist_ratio*rand_vertex(i));
		}
		
//		steered_vertex.push_back(g_index_counter);
//		++g_index_counter;
		return steered_vertex;
	}
}


bool obstacle_free_vertex(std::vector<double> new_vertex){
	//The purpose of this function is to return true if new_vertex is in obsticle free ws
	//and false otherwise
	//TODO: fill up
	//get code from Tony
	return true;

}

bool obstacle_free_edge(std::vector<double> new_vertex,std::vector<double> nearest_vertex){
	//The purpose of this function is to return true if the edge is in obsticle free ws
	//and false otherwise
	//TODO: fill up
	//get code from Tony
	return true;

}
//finding_path -----------------------------------------------------------------------------

std::vector<std::vector<double> > FindPath(void){
	
	//A function that finds the parent of each vertex starting from the target and moving towards the initial node, and construct a path accordingly 

	std::vector<std::vector<double> > path;
	int index;

	//construct reversed path from target to initial vertex

	int i=g_vertices_set.size;

	do{	
		path.push_back(g_vertices_set[i]); //adds the parent vertex to the path

		i = (int) g_vertices_set[i][7];

	
	}while(i>-1)

	// reverse back the path so that it goes from initial to target vertices
	std::reverse(path.begin(),path.end());

	ROS_INFO("Path generated with %i vertices", i);

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


	std::std::vector<double> rand_vertex; //generate random vertex 
	std::std::vector<double> nearest_vertex; //holds the value of the nearest vertex
	double nearest_index;
	std::std::vector<double> new_vertex; // steer the vertex with a Max distance
	std::std::vector<double> path;// contains a set of nodesfrom the goal to the initial pose

	

	//spicify initial vertix and add it to the vertices set
	//std::std::vector<double> initial_vertex=(0,0,0,0,0,0,0);//TODO: put real values

	g_vertices_set.push_back(initial_vertex); // add initial vertex to the vertices set

	int i=1;
	do{
		RandomAssignment(rand_vertex); // assign a random variable to rand_vertex
		(nearest_vertex,nearest_index)=NearestVertex(rand_vertex); // look for the nearest vertex and its index in g_vertices_set
		new_vertex=Steer(nearest_vertex, rand_vertex); //steer to the maximum edge legnth
		
		if(obstacle_free_vertex(new_vertex)&&obstacle_free_edge(new_vertex,nearest_vertex)){
			g_vertices_set.push_back(new_vertex); //add new_vertex to the vertices set
			g_vertices_set[i].push_back(nearest_index); //add the index of the parent
			//g_edges_set[0].push_back(new_vertex); //add the new edge to the edges set.
			//g_edges_set[1].push_back(nearest_vertex); 


			
			//optional: AddGraph(new_vertex,nearest_vertex) a function that visually present the RRT process

			i++; // increase the counter only when the new edge and vertex are added. If not repeat the loop.
		}
//		else{
//			--g_index_counter;
//		}

		ROS_INFO("number of particles accepted=%f",g_index_counter+1);

		if(distance(new_vertex, g_trgt_vertex)<trgt_zone){
			g_trgt_reached=true; 
			ROS_INFO("RRT reached goal");
			break;
		}

	}while(i<g_number_of_samples)

	if(g_trgt_reached==false){
		ROS_INFO("unable to connect to goal");
		return 0;
	}

	//plan the path and store it in "path"
	path= FindPath();

	//path movement execution-------------------------

	ros::Rate loop_rate(100);
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


	for(int j=0; j<path.size; j++){
		
		for(int k=0; k<path[j].size-1;k++){
			cmd.command[k]=path[j][k];
		}
		action_publisher.publish(cmd);
		loop_rate.sleep();
		ros::spinOnce();
		//TODO: we need to wait until the execution of the movement
	}

	//MoveToTRGT(bath);




	return 0;
}