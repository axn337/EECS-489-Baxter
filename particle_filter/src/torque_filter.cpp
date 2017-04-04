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

//useful global variables:
int g_num_particles = 1000;//the number of particles maintained by the filter

double g_importance_threshold = 0.0002;
//a tunable parameter to determine how sensitive to collsions baxter is.
//higher makes baxter more likely to report a collision

//the most recent set of torques reported by baxter
std::vector<double> g_sensor_data ;

//a global variable for collision reporting
bool g_collision_detected = false;

//this is ROS code to maintain the latest data from Baxter
void torque_callback(const baxter_core_msgs::SEAJointState message_holder){
	g_sensor_data.clear();
	for(int i =0; i < 7; i++){
		g_sensor_data.push_back(message_holder.gravity_only[i]);
	}
}

//returns the gaussian density at x for a gaussian with mean m and standard deviation s
float normal_pdf(float x, float m, float s) {
	static const float inv_sqrt_2pi = 0.3989422804014327;
	float a = (x - m)/s;
	return inv_sqrt_2pi/s*std::exp(-0.5f * a * a);
}

//rejection sampler
double normal_distribution_sampler(float mean, float standard_dev) {
	
	double max_prob = (double) normal_pdf(mean, mean, standard_dev);
	double range = -3*standard_dev + 3*standard_dev;
	
	double r1 = ((double) rand() / (RAND_MAX)) * max_prob;
	double r2 = (double) -3*standard_dev + ((double) rand() / (RAND_MAX))*range;
	double check_prob = (double) normal_pdf(r2, mean, standard_dev);
	int counter = 1;
	
	while (r1 > check_prob) {
		r1 = ((double) rand() / (RAND_MAX)) * max_prob;
		r2 = (double)-3*standard_dev + ((double) rand() / (RAND_MAX))*range;
		check_prob = (double) normal_pdf(r2, mean, standard_dev);
		counter += 1;
		
		if (counter > 10000) {
			ROS_WARN("Normal sampler running too long");
			return 0.0;
		}
	}
	
	return r2;
}

//samples a new state, given an old state and an action taken
//for this application, the output is actually independent of the previous state
//this function would be more meaningful if it had been practical to model the torques on a moving Baxter
std::vector<double> state_transition_sampler(std::vector<double> action, std::vector<double> prev_state){
	//use the rejection sampling technique to get a new sample state
	double T1 = action[0];
	double T2 = action[1];
	double T3 = action[2];
	double T4 = action[3];
	double T5 = action[4];
	double T6 = action[5];
	double T7 = action[6];
	
	std::vector<double> error_vect;

	for (int i=0;i<7;i++) {
		error_vect.push_back(normal_distribution_sampler(0.0, 0.01));
	}
	
	T1 += error_vect[0];
	T2 += error_vect[1];
	T3 += error_vect[2];
	T4 += error_vect[3];
	T5 += error_vect[4];
	T6 += error_vect[5];
	T7 += error_vect[6];
	
	std::vector<double> N;
	N.push_back(0);
	

	N.push_back(28.6*sin(T2) - 10.2*cos(T2) - 0.989*cos(T7)*(sin(T6)*(cos(T2)*cos(T4) - 1.0*cos(T3)*sin(T2)*sin(T4)) + cos(T6)*(cos(T5)*(cos(T2)*sin(T4) + cos(T3)*cos(T4)*sin(T2)) - 1.0*sin(T2)*sin(T3)*sin(T5))) - 0.00509*cos(T7)*(sin(T5)*(cos(T2)*sin(T4) + cos(T3)*cos(T4)*sin(T2)) + cos(T5)*sin(T2)*sin(T3)) - 0.00509*sin(T7)*(sin(T6)*(cos(T2)*cos(T4) - 1.0*cos(T3)*sin(T2)*sin(T4)) + cos(T6)*(cos(T5)*(cos(T2)*sin(T4) + cos(T3)*cos(T4)*sin(T2)) - 1.0*sin(T2)*sin(T3)*sin(T5))) + 63.5*cos(T2)*(0.433*cos(T4) + 0.201*sin(T4) - 0.433) - 21.1*(cos(T5)*(cos(T2)*sin(T4) + cos(T3)*cos(T4)*sin(T2)) - 1.0*sin(T2)*sin(T3)*sin(T5))*(0.808*sin(T6) - 0.191*cos(T6) + 0.191) - 0.0273*sin(T7)*(sin(T5)*(cos(T2)*sin(T4) + cos(T3)*cos(T4)*sin(T2)) + cos(T5)*sin(T2)*sin(T3)) + 106.0*sin(T2)*(0.27*cos(T3) - 0.27) + 43.2*(0.201*cos(T5) - 0.201)*(cos(T2)*sin(T4) + cos(T3)*cos(T4)*sin(T2)) - 19.1*cos(T6)*(cos(T2)*cos(T4) - 1.0*cos(T3)*sin(T2)*sin(T4)) - 2.91*sin(T6)*(cos(T2)*cos(T4) - 1.0*cos(T3)*sin(T2)*sin(T4)) + 5.32*(0.191*cos(T7) - 0.191)*(sin(T6)*(cos(T2)*cos(T4) - 1.0*cos(T3)*sin(T2)*sin(T4)) + cos(T6)*(cos(T5)*(cos(T2)*sin(T4) + cos(T3)*cos(T4)*sin(T2)) - 1.0*sin(T2)*sin(T3)*sin(T5))) - 24.3*cos(T2)*cos(T4) - 2.91*cos(T6)*(cos(T5)*(cos(T2)*sin(T4) + cos(T3)*cos(T4)*sin(T2)) - 1.0*sin(T2)*sin(T3)*sin(T5)) - 5.06*cos(T3)*sin(T2) - 4.05*cos(T2)*sin(T4) + 19.1*sin(T6)*(cos(T5)*(cos(T2)*sin(T4) + cos(T3)*cos(T4)*sin(T2)) - 1.0*sin(T2)*sin(T3)*sin(T5)) - 4.47*cos(T5)*(cos(T2)*sin(T4) + cos(T3)*cos(T4)*sin(T2)) + 0.171*sin(T2)*sin(T3) + 21.1*(cos(T2)*cos(T4) - 1.0*cos(T3)*sin(T2)*sin(T4))*(0.808*cos(T6) + 0.191*sin(T6) - 0.808) - 0.196*sin(T5)*(cos(T2)*sin(T4) + cos(T3)*cos(T4)*sin(T2)) - 4.05*cos(T3)*cos(T4)*sin(T2) + 24.3*cos(T3)*sin(T2)*sin(T4) - 0.196*cos(T5)*sin(T2)*sin(T3) - 4.21*sin(T2)*sin(T3)*sin(T5) - 63.5*cos(T3)*sin(T2)*(0.433*sin(T4) - 0.201*cos(T4) + 0.201));


	N.push_back(0.00509*cos(T7)*(cos(T2)*cos(T3)*cos(T5) - 1.0*cos(T2)*cos(T4)*sin(T3)*sin(T5)) - 0.00509*sin(T7)*(cos(T6)*(cos(T2)*cos(T3)*sin(T5) + cos(T2)*cos(T4)*cos(T5)*sin(T3)) - 1.0*cos(T2)*sin(T3)*sin(T4)*sin(T6)) + 0.0273*sin(T7)*(cos(T2)*cos(T3)*cos(T5) - 1.0*cos(T2)*cos(T4)*sin(T3)*sin(T5)) - 2.91*cos(T6)*(cos(T2)*cos(T3)*sin(T5) + cos(T2)*cos(T4)*cos(T5)*sin(T3)) + 19.1*sin(T6)*(cos(T2)*cos(T3)*sin(T5) + cos(T2)*cos(T4)*cos(T5)*sin(T3)) - 21.1*(cos(T2)*cos(T3)*sin(T5) + cos(T2)*cos(T4)*cos(T5)*sin(T3))*(0.808*sin(T6) - 0.191*cos(T6) + 0.191) + 5.32*(0.191*cos(T7) - 0.191)*(cos(T6)*(cos(T2)*cos(T3)*sin(T5) + cos(T2)*cos(T4)*cos(T5)*sin(T3)) - 1.0*cos(T2)*sin(T3)*sin(T4)*sin(T6)) - 0.171*cos(T2)*cos(T3) + 23.5*cos(T2)*sin(T3) - 0.989*cos(T7)*(cos(T6)*(cos(T2)*cos(T3)*sin(T5) + cos(T2)*cos(T4)*cos(T5)*sin(T3)) - 1.0*cos(T2)*sin(T3)*sin(T4)*sin(T6)) + 0.196*cos(T2)*cos(T3)*cos(T5) - 4.05*cos(T2)*cos(T4)*sin(T3) + 4.21*cos(T2)*cos(T3)*sin(T5) + 24.3*cos(T2)*sin(T3)*sin(T4) - 63.5*cos(T2)*sin(T3)*(0.433*sin(T4) - 0.201*cos(T4) + 0.201) + 43.2*cos(T2)*cos(T4)*sin(T3)*(0.201*cos(T5) - 0.201) - 4.47*cos(T2)*cos(T4)*cos(T5)*sin(T3) - 0.196*cos(T2)*cos(T4)*sin(T3)*sin(T5) + 19.1*cos(T2)*cos(T6)*sin(T3)*sin(T4) + 2.91*cos(T2)*sin(T3)*sin(T4)*sin(T6) - 21.1*cos(T2)*sin(T3)*sin(T4)*(0.808*cos(T6) + 0.191*sin(T6) - 0.808));

	N.push_back(0.989*cos(T7)*(sin(T6)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) - 1.0*cos(T5)*cos(T6)*(cos(T4)*sin(T2) + cos(T2)*cos(T3)*sin(T4))) - 5.32*(0.191*cos(T7) - 0.191)*(sin(T6)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) - 1.0*cos(T5)*cos(T6)*(cos(T4)*sin(T2) + cos(T2)*cos(T3)*sin(T4))) + 0.00509*sin(T7)*(sin(T6)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) - 1.0*cos(T5)*cos(T6)*(cos(T4)*sin(T2) + cos(T2)*cos(T3)*sin(T4))) + 43.2*(0.201*cos(T5) - 0.201)*(cos(T4)*sin(T2) + cos(T2)*cos(T3)*sin(T4)) + 19.1*cos(T6)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) + 2.91*sin(T6)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) + 63.5*sin(T2)*(0.201*cos(T4) - 0.433*sin(T4)) - 4.05*cos(T4)*sin(T2) - 4.47*cos(T5)*(cos(T4)*sin(T2) + cos(T2)*cos(T3)*sin(T4)) + 24.3*sin(T2)*sin(T4) - 21.1*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4))*(0.808*cos(T6) + 0.191*sin(T6) - 0.808) - 0.196*sin(T5)*(cos(T4)*sin(T2) + cos(T2)*cos(T3)*sin(T4)) + 63.5*cos(T2)*cos(T3)*(0.433*cos(T4) + 0.201*sin(T4)) - 24.3*cos(T2)*cos(T3)*cos(T4) - 4.05*cos(T2)*cos(T3)*sin(T4) - 2.91*cos(T5)*cos(T6)*(cos(T4)*sin(T2) + cos(T2)*cos(T3)*sin(T4)) + 19.1*cos(T5)*sin(T6)*(cos(T4)*sin(T2) + cos(T2)*cos(T3)*sin(T4)) - 0.00509*cos(T7)*sin(T5)*(cos(T4)*sin(T2) + cos(T2)*cos(T3)*sin(T4)) - 0.0273*sin(T5)*sin(T7)*(cos(T4)*sin(T2) + cos(T2)*cos(T3)*sin(T4)) - 21.1*cos(T5)*(cos(T4)*sin(T2) + cos(T2)*cos(T3)*sin(T4))*(0.808*sin(T6) - 0.191*cos(T6) + 0.191));

	N.push_back(2.91*cos(T6)*(sin(T5)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) - 1.0*cos(T2)*cos(T5)*sin(T3)) - 19.1*sin(T6)*(sin(T5)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) - 1.0*cos(T2)*cos(T5)*sin(T3)) - 0.196*cos(T5)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) - 4.21*sin(T5)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) - 0.00509*cos(T7)*(cos(T5)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) + cos(T2)*sin(T3)*sin(T5)) + 21.1*(sin(T5)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) - 1.0*cos(T2)*cos(T5)*sin(T3))*(0.808*sin(T6) - 0.191*cos(T6) + 0.191) - 0.0273*sin(T7)*(cos(T5)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) + cos(T2)*sin(T3)*sin(T5)) + 0.989*cos(T6)*cos(T7)*(sin(T5)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) - 1.0*cos(T2)*cos(T5)*sin(T3)) + 0.00509*cos(T6)*sin(T7)*(sin(T5)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) - 1.0*cos(T2)*cos(T5)*sin(T3)) + 4.21*cos(T2)*cos(T5)*sin(T3) - 0.196*cos(T2)*sin(T3)*sin(T5) - 5.32*cos(T6)*(sin(T5)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) - 1.0*cos(T2)*cos(T5)*sin(T3))*(0.191*cos(T7) - 0.191));

	N.push_back(0.00509*sin(T7)*(sin(T6)*(cos(T5)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) + cos(T2)*sin(T3)*sin(T5)) - 1.0*cos(T6)*(cos(T4)*sin(T2) + cos(T2)*cos(T3)*sin(T4))) - 21.1*(0.808*cos(T6) + 0.191*sin(T6))*(cos(T5)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) + cos(T2)*sin(T3)*sin(T5)) - 5.32*(0.191*cos(T7) - 0.191)*(sin(T6)*(cos(T5)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) + cos(T2)*sin(T3)*sin(T5)) - 1.0*cos(T6)*(cos(T4)*sin(T2) + cos(T2)*cos(T3)*sin(T4))) + 19.1*cos(T6)*(cos(T5)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) + cos(T2)*sin(T3)*sin(T5)) + 21.1*(0.191*cos(T6) - 0.808*sin(T6))*(cos(T4)*sin(T2) + cos(T2)*cos(T3)*sin(T4)) + 2.91*sin(T6)*(cos(T5)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) + cos(T2)*sin(T3)*sin(T5)) - 2.91*cos(T6)*(cos(T4)*sin(T2) + cos(T2)*cos(T3)*sin(T4)) + 0.989*cos(T7)*(sin(T6)*(cos(T5)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) + cos(T2)*sin(T3)*sin(T5)) - 1.0*cos(T6)*(cos(T4)*sin(T2) + cos(T2)*cos(T3)*sin(T4))) + 19.1*sin(T6)*(cos(T4)*sin(T2) + cos(T2)*cos(T3)*sin(T4)));

	N.push_back(0.00509*sin(T7)*(sin(T5)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) - 1.0*cos(T2)*cos(T5)*sin(T3)) - 0.0273*sin(T7)*(cos(T6)*(cos(T5)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) + cos(T2)*sin(T3)*sin(T5)) + sin(T6)*(cos(T4)*sin(T2) + cos(T2)*cos(T3)*sin(T4))) - 0.0273*cos(T7)*(sin(T5)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) - 1.0*cos(T2)*cos(T5)*sin(T3)) - 0.00509*cos(T7)*(cos(T6)*(cos(T5)*(sin(T2)*sin(T4) - 1.0*cos(T2)*cos(T3)*cos(T4)) + cos(T2)*sin(T3)*sin(T5)) + sin(T6)*(cos(T4)*sin(T2) + cos(T2)*cos(T3)*sin(T4))));
	

	return N;
}

//calculates probability of sensor_data, given that the roboti is in "state"
double importance_factor(std::vector<double> sensor_data, std::vector<double> state){
	//calculates and returns the probability density of sensor_data if the robot is in state
	
	double standard_dev = 1;
	double mean = 0.0;
	double weight = 1;
	for (int i=0;i<7;i++) {
		double error = sensor_data[i] - state[i];
		//ROS_INFO("sensor data %i = %f", i, sensor_data[i]);
		//ROS_INFO("state %i = %f", i, state[i]);
		double prob = normal_pdf(error,mean,standard_dev);
		//ROS_INFO("prob = %f", prob);
		if (i>=0) {
		weight *= prob;
		}
	}
	//ROS_INFO("returning a weight of %f", weight);
	return weight;
}

//the primary particle filter function. modelled after the pseudocode presented in lecture
void particle_filter(std::vector<std::vector<double> > &previous_particles, std::vector<double> action, std::vector<double> sensor_data , std::vector<std::vector<double> > &current_particles){
	//calculates new particles, places them in the container "current_particles"

	//some variable declarations for later
	int random_particle_index = 0;
	std::vector<double> prev_state;
	std::vector<double> sampled_state;
	double importance_factor_var = 1.0;

	//create a container for candidate particles and associated likelihoods given sensor data
	std::vector<std::vector<double> > weighted_candidate_particles;
	std::vector<double> importance_factors;
	//positions 0-6 are the features of the state associated with the particle, while position 7 is the importance factor of that particle
	
	//generate new particles
	for(int m = 0; m < g_num_particles; m++){

		//select a random particle from the previous set
		int random_particle_index = g_num_particles * ((double) rand() / (RAND_MAX));
		prev_state = previous_particles[random_particle_index];

		//use the sampled particle to generate a new state
		sampled_state = state_transition_sampler(action, prev_state);

		//set the likelihood of the particle, given the sensor data
		importance_factor_var = importance_factor(sensor_data, sampled_state);
		
		//add newly generated particle to the list of those being considered
		weighted_candidate_particles.push_back(sampled_state);
		importance_factors.push_back(importance_factor_var);

	}
	
	//perform some calculations to check for consistency between actual torques and predicted torques
	double sum_of_importances = 0.0;
	for(int m = 0; m < g_num_particles; m++){
		sum_of_importances += importance_factors[m];
	}

	//perform collision detection based on the average likelihood of an obeservation given some sensor data
	if ((sum_of_importances / g_num_particles) < g_importance_threshold){
		//collision inferred
		ROS_INFO("possible collision detected");
		ROS_WARN("average importance factor when collision triggered: %f", (sum_of_importances / g_num_particles));
		g_collision_detected = true;
	}
	
	//set up a useful index for the resampling step
	double cumulative_sum = 0.0;
	for(int m = 0; m < g_num_particles; m++){
		importance_factors[m] /= sum_of_importances;
		cumulative_sum += importance_factors[m];
		importance_factors[m] = cumulative_sum;
	}

	//resampling step
	for(int m = 0; m < g_num_particles; m++){
		
		//generate random number from 0 to 1
		double r = ((double) rand() / (RAND_MAX));
		
		for(int m = 0; m < g_num_particles; m++){
			//find the first particle with the cumulative sum larger than r
			if(importance_factors[m] > r){
				//add new particle to output set
				current_particles.push_back(weighted_candidate_particles[m]);
				break;
			}
		}
	}
	
	//ROS_INFO("Paticle Filter Location 5");
}

int main(int argc, char **argv){
	ros::init(argc, argv, "particle_filter");
	ros::NodeHandle n;
	ros::Subscriber effort_subscriber = n.subscribe("/robot/limb/left/gravity_compensation_torques", 1, torque_callback);

	ros::Publisher action_publisher = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command",1);
	
	//ROS_INFO("Location 1");

	std::vector<std::vector<double> > current_particles;
	std::vector<std::vector<double> > previous_particles;
	
	//ROS_INFO("Location 2");

	ros::Rate loop_rate(100);
  	baxter_core_msgs::JointCommand cmd;
	cmd.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
	
	//ROS_INFO("Location 2.1");
	
	cmd.names.push_back("left_s0");
	cmd.names.push_back("left_s1");
	cmd.names.push_back("left_e0");
	cmd.names.push_back("left_e1");
	cmd.names.push_back("left_w0");
	cmd.names.push_back("left_w1");
	cmd.names.push_back("left_w2");
      
	cmd.command.resize(cmd.names.size());
      
	//ROS_INFO("Location 2.15");
	
	//command an initial motion to 0, then wait
	cmd.command[0] = 0;
	cmd.command[1] = 0;
	cmd.command[2] = 0;
	cmd.command[3] = 0;
	cmd.command[4] = 0;
	cmd.command[5] = 0;
	cmd.command[6] = 0;
	
	std::cout<<cmd<<std::endl;

	//ROS_INFO("Location 2.2");
	
	ros::Time begin = ros::Time::now();
	
	while((ros::Time::now() - begin).toSec() < 5.0){
		action_publisher.publish(cmd);
		loop_rate.sleep();
		ros::spinOnce();
	}
	
	//ROS_INFO("Location 3");
	
	//initialize particles to match latest sensor data
	for (int m = 0; m < g_num_particles; m++){
		current_particles.push_back(g_sensor_data);
	}

	double current_joint_angle=0;

	std::vector<double> last_action;
	
	//ROS_INFO("Location 4");
	
	int counter = 0;
	
	while(ros::ok()){
		
		counter++;
		
		current_joint_angle += 0.05;
		
		last_action.clear();

		//command next action
		
		for(int i = 0; i < 7; i++){
			if(i != 2){
				//cmd.command[i] = current_joint_angle;
				if(!g_collision_detected){
					double r = 0.6*(((double) rand() / (RAND_MAX)) - 0.5);//random joint angle from -0.5 to 0.5
					cmd.command[i] = r;
				}
			}
	  	}
	  	
    	ros::Time begin = ros::Time::now();
	
		while((ros::Time::now() - begin).toSec() < 4.0){
			action_publisher.publish(cmd);
			loop_rate.sleep();
			ros::spinOnce();
			
		}

		//ROS_INFO("Location 5");

		//convert action data type for use by particle_filter`
		for(int i = 0; i < 7; i++){
			last_action.push_back(cmd.command[i]);
		}
		
		//ROS_INFO("Location 5.1");
		
		//ROS_INFO("Location 5.2");

		//update particles
		previous_particles = current_particles;
		//ROS_INFO("Location 5.3");
		current_particles.clear();
		//ROS_INFO("Location 5.4");
		particle_filter(previous_particles, last_action, g_sensor_data, current_particles);
		
		//ROS_INFO("Location 6");

		//check for if a collision was detected
			//if so, warn and break loop
		if(g_collision_detected){
			ROS_WARN("collision detected. breaking out of loop");
			break;
		}
		if(counter > 10){
			break;
		}
		
	}
	
	return 0;
}
