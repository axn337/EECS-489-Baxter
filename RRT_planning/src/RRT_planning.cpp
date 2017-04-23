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


//global variables:----------------------------------------------------------------------

std::vector<std::vector<double> > g_vertices_set; //2-D vectors contains the verticies in the free work-space
std::vector<std::vector<std::vector<double> > > g_edges_set; //3-dimension vector that contains the set of collision free edges
int g_number_of_samples=10000;//select number of samples in free workspace
double g_max_edge_length=0.5;//TODO: select value

//Functions-------------------------------------------------------------------------------
void RandomAssignment(std::vector<double> rand_vertex){
	//a function that assignes a random variable to rand_vertex
	//TODO: fill up with a random generator code

	const int range_from  = -1;//TODO:change the range
    const int range_to    = 1;

    std::random_device                  rand_dev;
    std::mt19937                        generator(rand_dev());
    std::uniform_int_distribution<int>  distr(range_from, range_to);
 
	for(int i=0; i<7; ++i){
		rand_vertex.push_back(distr(generator));
	}
}

std::vector<double> NearestVertex(std::vector<double> v){
	//TODO: fill up
	//A function that chooses the nearest node in the verteces set

	std::vector<double> nearest= g_vertices_set(0);//Temporary assumbtion in seek of comparison

	for(int i=1; i<g_vertices_set.size; ++i){

		if(distance(v,g_vertices_set(i))<nearest){
			nearest=g_vertices_set(i) //replace nearest with the new shortest in distance vertex 
		}
	}
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
		return rand_vertex;
	}
	else{
		for(int i=0; i<7; ++i){
			//calculate coordinates in 7 dimensions
			steered_vertex.push_back((1-dist_ratio)*nearest_vertex(i)+dist_ratio*rand_vertex(i));
		}
		return steered_vertex;
	}
}


bool obstacle_free_vertex(std::vector<double> new_vertex){
	//The purpose of this function is to return true if new_vertex is in obsticle free ws
	//and false otherwise
	//TODO: fill up
	//get code from Tony

}

bool obstacle_free_edge(std::vector<double> new_vertex,std::vector<double> nearest_vertex){
	//The purpose of this function is to return true if the edge is in obsticle free ws
	//and false otherwise
	//TODO: fill up
	//get code from Tony
}

//Main function-------------------------------------------------------------------------------

int main(int argc, char **argv){
	//basic ROS setup for communications with baxter
	ros::init(argc, argv, "RRT_Baxter");
	ros::NodeHandle n;

	std::std::vector<double> rand_vertex; //generate random vertex 
	std::std::vector<double> nearest_vertex; //holds the value of the nearest vertex
	std::std::vector<double> new_vertex; // steer the vertex with a Max distance

	

	//spicify initial vertix and add it to the vertices set
	std::std::vector<double> initial_vertex=(0,0,0,0,0,0,0);//TODO: put real values

	g_vertices_set.push_back(initial_vertex); // add initial vertex to the vertices set

	int i=0;
	do{
		RandomAssignment(rand_vertex); // assign a random variable to rand_vertex
		nearest_vertex=NearestVertex(rand_vertex); // look for the nearest vertex 
		new_vertex=Steer(nearest_vertex, rand_vertex); //steer to the maximum edge legnth
		
		if(obstacle_free_vertex(new_vertex)&&obstacle_free_edge(new_vertex,nearest_vertex)){
			g_vertices_set.push_back(new_vertex); //add new_vertex to the vertices set
			g_edges_set[0].push_back(new_vertex); //add the new edge to the edges set.
			g_edges_set[1].push_back(nearest_vertex); 
			
			//optional: AddGraph(new_vertex,nearest_vertex) a function that visually present the RRT process

			i++; // increase the counter only when the new edge and vertex are added. If not repeat the loop.
		}

	}while(i<g_number_of_samples)

	return 0;
}