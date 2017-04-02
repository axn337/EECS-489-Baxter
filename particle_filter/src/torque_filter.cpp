#include<ros/ros.h>
#include <random>

//int g_num_particles will be 1000 to start
//current_particles will be an array of 1000 arrays of torques (each with length of num_joints)

int g_num_particles = 1000;

double g_importance_threshold = 0.2;

/*
useful stuff to copy/paste:
--------------------------
std::vector<std::vector<double>> current_particles;
*/

void particle_filter(std::vector<std::vector<double>> &previous_particles, std::vector<double> action, std::vector<double> sensor_data , std::vector<std::vector<double>> &current_particles){
	//calculates new particles, places them in the container "current_particles"

	//a uniform distribution to sample from
	std::uniform_int_distribution<int> distribution(0,g_num_particles - 1);

	//some variable declarations for later
	int random_particle_index = 0;
	std::vector<double> prev_state;
	std::vector<double> sampled_state;
	double importance_factor = 1.0;

	//create a container for candidate particles and associated likelihoods given sensor data
	std::vector<std::vector<double>> weighted_candidate_particles;
	std::vector<double> importance_factors;
	//positions 0-6 are the features of the state associated with the particle, while position 7 is the importance factor of that particle
	
	for(int m = 0; m < g_num_particles; m++){

		//select a random particle from the previous set
		random_particle_index = distribution(generator);
		prev_state = previous_particles[random_particle_index];

		//use the sampled particle to generate a new state
		sampled_state = state_transition_sampler(action, prev_state);

		//set the likelihood of the particle, given the sensor data
		importance_factor = importance_factor(sensor_data, sampled_state);
		
		//add newly generated particle to the list of those being considered
		weighted_candidate_particles.push_back(sampled_state);
		importance_factors.push_back(importance_factor);

	}
	
	double sum_of_importances = 0.0;

	for(int m = 0; m < g_num_particles; m++){
		sum_of_importances += importance_factors[m];
	}

	double cumulative_sum = 0.0;

	for(int m = 0; m < g_num_particles; m++){
		importance_factors[m] /= sum_of_importances;
		cumulative_sum += importance_factors[m];
		importance_factors[m] = cumulative_sum;
	}

	if ((sum_of_importances / g_num_particles) < g_importance_threshold){
		//collision inferred
		ROS_INFO("possible collision detected");
	}

	for(int m = 0; m < g_num_particles; m++){
		//resampling step

		//generate random number from 0 to 1
		r = ((double) rand() / (RAND_MAX))
		
		for(int m = 0; m < g_num_particles; m++){
			//find the first particle with the cumulative sum larger than r
			if(importance_factors[m] > r){
				//add new particle to output set
				current_particles.push_back(weighted_candidate_particles[m]);
				break;
			}
		}
	}

}

std::vector<double> state_transition_sampler(std::vector<double> action, std::vector<double> prev_state){
	//use the rejection sampling technique to get a new sample state
}

double importance_factor(std::vector<double> sensor_data, std::vector<double> state){
	//calculates and returns the probability density of sensor_data if the robot is in state
	
}
