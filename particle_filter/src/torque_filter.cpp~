#include<ros/ros.h>

//num_joints = 7?? TODO: verify this
//int g_num_particles will be 1000 to start
//current_particles will be an array of 1000 arrays of torques (each with length of num_joints)

for (int i =0; i < num_particles; i++){
	//
}

int g_num_particles = 1000;
std::vector<std::vector<double>> current_particles (g_num_particles, std::vector<double> (7, 0.0));
	//initializes a set of particles which all have 0s for every torque

void particle_filter(double previous_particles[][], /* TODO: some_action_type action, */ /* TODO: sensor_data_type sensor_data , */ double current_particles[][]&, /*TODO: verify & usage*/){
	//calculates new particles, places them in the container "current_particles"
	
	for(int m = 0; m < g_num_particles; m++){
		//sample a state by:
			//selecting from the previous list of particles at random
			//feeding this and the chosen action to state_transition_sampler
	}

	/* when I resample, keep a running sum of the weights of particles selected. when 
	done resampling, divide by number of particles and check against threshold for collision
	detection*/
}

std::vector<double> state_transition_sampler(/* TODO: some_action_type action, */ std::vector<double> prev_state){
	//use the rejection sampling technique to get a new sample state
}
