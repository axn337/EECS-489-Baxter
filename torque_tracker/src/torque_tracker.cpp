#include "ros/ros.h"
#include "baxter_core_msgs/JointCommand.h"
#include <sensor_msgs/JointState.h>

double tourque_exp[]; //expected torque

void myCallback(const sensor_msgs::JointState& laser_scan) 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "torque_tracker");
  ros::NodeHandle n;
  ros::Publisher left_cmd_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1);
  ros::Subscriber torque_subscriber = n.subscribe<sensor_msgs/JointState>("/robot/joint_states", 1,maCallback);

   publish at at least 5 Hz, or else Baxter switches back to Position mode and holds position
  ros::Rate loop_rate(100);
  baxter_core_msgs::JointCommand cmd;
  sensor_msgs::JointState states;
  
  states.mode= sensor_msgs::JointState::effort
  
  states.names.push_back("left_s0");
  states.names.push_back("left_s1");
  states.names.push_back("left_e0");
  states.names.push_back("left_e1");
  states.names.push_back("left_w0");
  states.names.push_back("left_w1");
  states.names.push_back("left_w2");
  
  states.effort.resize(states.names.size());
  
  // command in position mode
  cmd.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
  
  // command joints in the order shown in baxter_interface
  cmd.names.push_back("left_s0");
  cmd.names.push_back("left_s1");
  cmd.names.push_back("left_e0");
  cmd.names.push_back("left_e1");
  cmd.names.push_back("left_w0");
  cmd.names.push_back("left_w1");
  cmd.names.push_back("left_w2");
  
  // set your calculated positions
  cmd.command.resize(cmd.names.size());
  
  
  //initial torque readings
  for(size_t i = 0; i < states.names.size(); i++){
    states.effort[i] = 0.0; // get torque reading
    ROS_INFO("Torque for joint %d = %d", i, states.effort[i]);
	}
  
  //fisrt pose
  
  for(size_t i = 0; i < cmd.names.size(); i++){
    cmd.command[i] = 0.0;
  //calculate expected torque
  torque_exp[i] = 0;//torque position function
  }
  
  // get torque readings
  for(size_t i = 0; i < states.names.size(); i++){
    states.effort[i] = 0.0; //actual value
    ROS_INFO("for joint %d: actual Torque  = %d, expected torque = %d", i, states.effort[i], torque_exp[i]);
	}
  
  //second pose
  
  for(size_t i = 0; i < cmd.names.size(); i++){
    cmd.command[i] = 0.0;
  //calculate expected torque
  torque_exp[i] = 0;//torque position function
  }
  
  // get torque readings
  for(size_t i = 0; i < states.names.size(); i++){
    states.effort[i] = 0.0; //actual value
    ROS_INFO("for joint %d: actual Torque  = %d, expected torque = %d", i, states.effort[i], torque_exp[i]);
	}
 
  
  
  std::cout<<cmd<<std::endl;
  while(ros::ok()){
    //update cmd.command commands here
    left_cmd_pub.publish(cmd);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
