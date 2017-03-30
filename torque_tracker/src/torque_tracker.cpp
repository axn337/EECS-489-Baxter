#include "ros/ros.h"
#include "baxter_core_msgs/JointCommand.h"
#include <sensor_msgs/JointState.h>

double tourque_exp[7]; //expected torques
double torque_real[7]; //torque values from sensors
double torque_err[7];  // error 
double theta[7]; //position set


void myCallback(const sensor_msgs::JointState& joint_torque){ 
	
	ROS_INFO("torques recieved");
}

double Torque1(double theta[]){
 //calculate the torque value for s1
 // check with tony for the corosponding torque

double t = 0.167*sin(theta[2]) - 26.1*cos(theta[2]) - 8.14*cos(theta[2])*cos(theta[4]) + 0.932*cos(theta[3])*sin(theta[2]) + 0.227*cos(theta[2])*sin(theta[4]) - 0.588*sin(theta[2])*sin(theta[3]) - 1.35*cos(theta[2])*cos(theta[4])*cos(theta[6]) + 0.227*cos(theta[3])*cos(theta[4])*sin(theta[2]) - 2.68*cos(theta[2])*cos(theta[5])*sin(theta[4]) - 0.0948*cos(theta[2])*cos(theta[4])*sin(theta[6]) + 8.14*cos(theta[3])*sin(theta[2])*sin(theta[4]) - 1.06*cos(theta[5])*sin(theta[2])*sin(theta[3]) - 1.06*cos(theta[2])*sin(theta[4])*sin(theta[5]) + 2.68*sin(theta[2])*sin(theta[3])*sin(theta[5]) - 2.68*cos(theta[3])*cos(theta[4])*cos(theta[5])*sin(theta[2]) - 0.0948*cos(theta[2])*cos(theta[5])*cos(theta[6])*sin(theta[4]) - 0.00509*cos(theta[2])*cos(theta[4])*cos(theta[6])*sin(theta[6]) - 1.06*cos(theta[3])*cos(theta[4])*sin(theta[2])*sin(theta[5]) + 1.35*cos(theta[3])*cos(theta[6])*sin(theta[2])*sin(theta[4]) + 1.35*cos(theta[2])*cos(theta[5])*sin(theta[4])*sin(theta[6]) + 0.355*cos(theta[5])*cos(theta[6])*sin(theta[2])*sin(theta[3]) + 0.355*cos(theta[2])*cos(theta[6])*sin(theta[4])*sin(theta[5]) + 0.355*cos(theta[2])*cos(theta[4])*sin(theta[6])*sin(theta[6]) + 0.0948*cos(theta[3])*sin(theta[2])*sin(theta[4])*sin(theta[6]) + 0.0948*cos(theta[6])*sin(theta[2])*sin(theta[3])*sin(theta[5]) + 0.00509*cos(theta[5])*sin(theta[2])*sin(theta[3])*sin(theta[6]) + 0.00509*cos(theta[2])*sin(theta[4])*sin(theta[5])*sin(theta[6]) - 1.35*sin(theta[2])*sin(theta[3])*sin(theta[5])*sin(theta[6]) - 0.0948*cos(theta[3])*cos(theta[4])*cos(theta[5])*cos(theta[6])*sin(theta[2]) - 0.00509*cos(theta[2])*cos(theta[5])*cos(theta[6])*cos(theta[6])*sin(theta[4]) + 1.35*cos(theta[3])*cos(theta[4])*cos(theta[5])*sin(theta[2])*sin(theta[6]) + 0.355*cos(theta[3])*cos(theta[4])*cos(theta[6])*sin(theta[2])*sin(theta[5]) + 0.355*cos(theta[2])*cos(theta[5])*cos(theta[6])*sin(theta[4])*sin(theta[6]) + 0.00509*cos(theta[3])*cos(theta[4])*sin(theta[2])*sin(theta[5])*sin(theta[6]) + 0.00509*cos(theta[3])*cos(theta[6])*sin(theta[2])*sin(theta[4])*sin(theta[6]) + 0.00509*cos(theta[6])*cos(theta[6])*sin(theta[2])*sin(theta[3])*sin(theta[5]) - 0.355*cos(theta[3])*sin(theta[2])*sin(theta[4])*sin(theta[6])*sin(theta[6]) - 0.355*cos(theta[6])*sin(theta[2])*sin(theta[3])*sin(theta[5])*sin(theta[6]) - 0.00509*cos(theta[3])*cos(theta[4])*cos(theta[5])*cos(theta[6])*cos(theta[6])*sin(theta[2]) + 0.355*cos(theta[3])*cos(theta[4])*cos(theta[5])*cos(theta[6])*sin(theta[2])*sin(theta[6]);

return t;

}

double Torque2(double theta[]){

 //calculate the torque value for e0
 // check with tony for the corosponding torque
double t = 0.588*cos(theta[2])*cos(theta[3]) + 0.932*cos(theta[2])*sin(theta[3]) + 1.06*cos(theta[2])*cos(theta[3])*cos(theta[5]) + 0.227*cos(theta[2])*cos(theta[4])*sin(theta[3]) - 2.68*cos(theta[2])*cos(theta[3])*sin(theta[5]) + 8.14*cos(theta[2])*sin(theta[3])*sin(theta[4]) - 0.355*cos(theta[2])*cos(theta[3])*cos(theta[5])*cos(theta[6]) - 2.68*cos(theta[2])*cos(theta[4])*cos(theta[5])*sin(theta[3]) - 0.0948*cos(theta[2])*cos(theta[3])*cos(theta[6])*sin(theta[5]) - 0.00509*cos(theta[2])*cos(theta[3])*cos(theta[5])*sin(theta[6]) - 1.06*cos(theta[2])*cos(theta[4])*sin(theta[3])*sin(theta[5]) + 1.35*cos(theta[2])*cos(theta[6])*sin(theta[3])*sin(theta[4]) + 1.35*cos(theta[2])*cos(theta[3])*sin(theta[5])*sin(theta[6]) + 0.0948*cos(theta[2])*sin(theta[3])*sin(theta[4])*sin(theta[6]) - 0.0948*cos(theta[2])*cos(theta[4])*cos(theta[5])*cos(theta[6])*sin(theta[3]) - 0.00509*cos(theta[2])*cos(theta[3])*cos(theta[6])*cos(theta[6])*sin(theta[5]) + 1.35*cos(theta[2])*cos(theta[4])*cos(theta[5])*sin(theta[3])*sin(theta[6]) + 0.355*cos(theta[2])*cos(theta[4])*cos(theta[6])*sin(theta[3])*sin(theta[5]) + 0.355*cos(theta[2])*cos(theta[3])*cos(theta[6])*sin(theta[5])*sin(theta[6]) + 0.00509*cos(theta[2])*cos(theta[4])*sin(theta[3])*sin(theta[5])*sin(theta[6]) + 0.00509*cos(theta[2])*cos(theta[6])*sin(theta[3])*sin(theta[4])*sin(theta[6]) - 0.355*cos(theta[2])*sin(theta[3])*sin(theta[4])*sin(theta[6])*sin(theta[6]) - 0.00509*cos(theta[2])*cos(theta[4])*cos(theta[5])*cos(theta[6])*cos(theta[6])*sin(theta[3]) + 0.355*cos(theta[2])*cos(theta[4])*cos(theta[5])*cos(theta[6])*sin(theta[3])*sin(theta[6]);
return t;
}

double Torque3(double theta[]){
 //calculate the torque value for e1
 // check with tony for the corosponding torque
double t = 0.227*cos(theta[4])*sin(theta[2]) + 8.14*sin(theta[2])*sin(theta[4]) - 8.14*cos(theta[2])*cos(theta[3])*cos(theta[4]) + 0.227*cos(theta[2])*cos(theta[3])*sin(theta[4]) - 2.68*cos(theta[4])*cos(theta[5])*sin(theta[2]) - 1.06*cos(theta[4])*sin(theta[2])*sin(theta[5]) + 1.35*cos(theta[6])*sin(theta[2])*sin(theta[4]) + 0.0948*sin(theta[2])*sin(theta[4])*sin(theta[6]) - 1.35*cos(theta[2])*cos(theta[3])*cos(theta[4])*cos(theta[6]) - 2.68*cos(theta[2])*cos(theta[3])*cos(theta[5])*sin(theta[4]) - 0.0948*cos(theta[2])*cos(theta[3])*cos(theta[4])*sin(theta[6]) - 0.0948*cos(theta[4])*cos(theta[5])*cos(theta[6])*sin(theta[2]) - 1.06*cos(theta[2])*cos(theta[3])*sin(theta[4])*sin(theta[5]) + 1.35*cos(theta[4])*cos(theta[5])*sin(theta[2])*sin(theta[6]) + 0.355*cos(theta[4])*cos(theta[6])*sin(theta[2])*sin(theta[5]) + 0.00509*cos(theta[4])*sin(theta[2])*sin(theta[5])*sin(theta[6]) + 0.00509*cos(theta[6])*sin(theta[2])*sin(theta[4])*sin(theta[6]) - 0.355*sin(theta[2])*sin(theta[4])*sin(theta[6])*sin(theta[6]) - 0.0948*cos(theta[2])*cos(theta[3])*cos(theta[5])*cos(theta[6])*sin(theta[4]) - 0.00509*cos(theta[2])*cos(theta[3])*cos(theta[4])*cos(theta[6])*sin(theta[6]) - 0.00509*cos(theta[4])*cos(theta[5])*cos(theta[6])*cos(theta[6])*sin(theta[2]) + 1.35*cos(theta[2])*cos(theta[3])*cos(theta[5])*sin(theta[4])*sin(theta[6]) + 0.355*cos(theta[2])*cos(theta[3])*cos(theta[6])*sin(theta[4])*sin(theta[5]) + 0.355*cos(theta[2])*cos(theta[3])*cos(theta[4])*sin(theta[6])*sin(theta[6]) + 0.355*cos(theta[4])*cos(theta[5])*cos(theta[6])*sin(theta[2])*sin(theta[6]) + 0.00509*cos(theta[2])*cos(theta[3])*sin(theta[4])*sin(theta[5])*sin(theta[6]) - 0.00509*cos(theta[2])*cos(theta[3])*cos(theta[5])*cos(theta[6])*cos(theta[6])*sin(theta[4]) + 0.355*cos(theta[2])*cos(theta[3])*cos(theta[5])*cos(theta[6])*sin(theta[4])*sin(theta[6]);
return t;
}

double Torque4(double theta[]){
 //calculate the torque value for w0
 // check with tony for the corosponding torque
double t = 2.68*sin(theta[2])*sin(theta[4])*sin(theta[5]) - 1.06*cos(theta[2])*sin(theta[3])*sin(theta[5]) - 1.06*cos(theta[5])*sin(theta[2])*sin(theta[4]) - 2.68*cos(theta[2])*cos(theta[5])*sin(theta[3]) + 1.06*cos(theta[2])*cos(theta[3])*cos(theta[4])*cos(theta[5]) - 2.68*cos(theta[2])*cos(theta[3])*cos(theta[4])*sin(theta[5]) - 0.0948*cos(theta[2])*cos(theta[5])*cos(theta[6])*sin(theta[3]) + 1.35*cos(theta[2])*cos(theta[5])*sin(theta[3])*sin(theta[6]) + 0.355*cos(theta[2])*cos(theta[6])*sin(theta[3])*sin(theta[5]) + 0.355*cos(theta[5])*cos(theta[6])*sin(theta[2])*sin(theta[4]) + 0.00509*cos(theta[2])*sin(theta[3])*sin(theta[5])*sin(theta[6]) + 0.0948*cos(theta[6])*sin(theta[2])*sin(theta[4])*sin(theta[5]) + 0.00509*cos(theta[5])*sin(theta[2])*sin(theta[4])*sin(theta[6]) - 1.35*sin(theta[2])*sin(theta[4])*sin(theta[5])*sin(theta[6]) - 0.0948*cos(theta[2])*cos(theta[3])*cos(theta[4])*cos(theta[6])*sin(theta[5]) - 0.00509*cos(theta[2])*cos(theta[3])*cos(theta[4])*cos(theta[5])*sin(theta[6]) - 0.00509*cos(theta[2])*cos(theta[5])*cos(theta[6])*cos(theta[6])*sin(theta[3]) + 1.35*cos(theta[2])*cos(theta[3])*cos(theta[4])*sin(theta[5])*sin(theta[6]) + 0.355*cos(theta[2])*cos(theta[5])*cos(theta[6])*sin(theta[3])*sin(theta[6]) + 0.00509*cos(theta[6])*cos(theta[6])*sin(theta[2])*sin(theta[4])*sin(theta[5]) - 0.355*cos(theta[6])*sin(theta[2])*sin(theta[4])*sin(theta[5])*sin(theta[6]) - 0.355*cos(theta[2])*cos(theta[3])*cos(theta[4])*cos(theta[5])*cos(theta[6]) - 0.00509*cos(theta[2])*cos(theta[3])*cos(theta[4])*cos(theta[6])*cos(theta[6])*sin(theta[5]) + 0.355*cos(theta[2])*cos(theta[3])*cos(theta[4])*cos(theta[6])*sin(theta[5])*sin(theta[6]);
return t;
}

double Torque5(double theta[]){
 //calculate the torque value for w1
 // check with tony for the corosponding torque
double t = 1.35*cos(theta[4])*sin(theta[2])*sin(theta[6]) - 0.0948*cos(theta[4])*cos(theta[6])*sin(theta[2]) - 0.0948*cos(theta[2])*cos(theta[3])*cos(theta[6])*sin(theta[4]) - 0.00509*cos(theta[4])*cos(theta[6])*cos(theta[6])*sin(theta[2]) + 1.35*cos(theta[2])*cos(theta[3])*sin(theta[4])*sin(theta[6]) + 1.35*cos(theta[2])*cos(theta[6])*sin(theta[3])*sin(theta[5]) + 1.35*cos(theta[5])*cos(theta[6])*sin(theta[2])*sin(theta[4]) + 0.355*cos(theta[4])*cos(theta[6])*sin(theta[2])*sin(theta[6]) + 0.0948*cos(theta[2])*sin(theta[3])*sin(theta[5])*sin(theta[6]) + 0.0948*cos(theta[5])*sin(theta[2])*sin(theta[4])*sin(theta[6]) - 0.0948*cos(theta[2])*cos(theta[3])*cos(theta[4])*cos(theta[5])*sin(theta[6]) - 0.00509*cos(theta[2])*cos(theta[3])*cos(theta[6])*cos(theta[6])*sin(theta[4]) + 0.355*cos(theta[2])*cos(theta[3])*cos(theta[6])*sin(theta[4])*sin(theta[6]) + 0.00509*cos(theta[2])*cos(theta[6])*sin(theta[3])*sin(theta[5])*sin(theta[6]) + 0.00509*cos(theta[5])*cos(theta[6])*sin(theta[2])*sin(theta[4])*sin(theta[6]) - 0.355*cos(theta[2])*sin(theta[3])*sin(theta[5])*sin(theta[6])*sin(theta[6]) - 0.355*cos(theta[5])*sin(theta[2])*sin(theta[4])*sin(theta[6])*sin(theta[6]) - 1.35*cos(theta[2])*cos(theta[3])*cos(theta[4])*cos(theta[5])*cos(theta[6]) - 0.00509*cos(theta[2])*cos(theta[3])*cos(theta[4])*cos(theta[5])*cos(theta[6])*sin(theta[6]) + 0.355*cos(theta[2])*cos(theta[3])*cos(theta[4])*cos(theta[5])*sin(theta[6])*sin(theta[6]);
return t;
}


double Torque6(double theta[]){
 //calculate the torque value for w2
 // check with tony for the corosponding torque
double t=0.355*cos(theta[2])*cos(theta[5])*sin(theta[3])*sin(theta[6])-0.00509*cos(theta[2])*cos(theta[5])*cos(theta[6])*sin(theta[3])+0.355*cos(theta[4])*cos(theta[6])*sin(theta[2])*sin(theta[6])+0.00509*cos(theta[6])*sin(theta[2])*sin(theta[4])*sin(theta[5])+0.00509*cos(theta[4])*sin(theta[2])*sin(theta[6])*sin(theta[6])-0.355*sin(theta[2])*sin(theta[4])*sin(theta[5])*sin(theta[6])-0.00509*cos(theta[2])*cos(theta[3])*cos(theta[4])*cos(theta[6])*sin(theta[5])+0.355*cos(theta[2])*cos(theta[3])*cos(theta[4])*sin(theta[5])*sin(theta[6])+0.355*cos(theta[2])*cos(theta[3])*cos(theta[6])*sin(theta[4])*sin(theta[6])+0.355*cos(theta[2])*cos(theta[6])*cos(theta[6])*sin(theta[3])*sin(theta[5])+0.355*cos(theta[5])*cos(theta[6])*cos(theta[6])*sin(theta[2])*sin(theta[4])+0.00509*cos(theta[2])*cos(theta[3])*sin(theta[4])*sin(theta[6])*sin(theta[6])+0.00509*cos(theta[2])*cos(theta[6])*sin(theta[3])*sin(theta[5])*sin(theta[6])+0.00509*cos(theta[5])*cos(theta[6])*sin(theta[2])*sin(theta[4])*sin(theta[6])-0.355*cos(theta[2])*cos(theta[3])*cos(theta[4])*cos(theta[5])*cos(theta[6])*cos(theta[6])-0.00509*cos(theta[2])*cos(theta[3])*cos(theta[4])*cos(theta[5])*cos(theta[6])*sin(theta[6]);
return t;
}

/*/double update_torque_exp(double theta[]){
	//a function that updates the torque values according to the "Torque" functions gven theta
	double t[7];
	t[0]=0;
	t[1]=Torque1(theta[]);
	t[2]=Torque2(theta[]);
	t[3]=Torque3(theta[]);
	t[4]=Torque4(theta[]);
	t[5]=Torque5(theta[]);
	t[6]=Torque6(theta[]);
	return t;
}/*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "torque_tracker");
  ros::NodeHandle n;
  ros::Publisher left_cmd_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1);
  ros::Subscriber torque_subscriber = n.subscribe<sensor_msgs::JointState>("/robot/joint_states",1,myCallback);

  // publish at at least 5 Hz, or else Baxter switches back to Position mode and holds position
  ros::Rate loop_rate(100);
  baxter_core_msgs::JointCommand cmd;
  sensor_msgs::JointState states;
  
  /*/states.mode= sensor_msgs::JointState::effort;
  
  states.names.push_back("left_s0");
  states.names.push_back("left_s1");
  states.names.push_back("left_e0");
  states.names.push_back("left_e1");
  states.names.push_back("left_w0");
  states.names.push_back("left_w1");
  states.names.push_back("left_w2");
  
  states.effort.resize(states.names.size());/*/
  
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
  for(int i = 0; i < 7; i++){
    torque_real[i] =states.effort[i] ; // get torque reading
    ROS_INFO("Torque for joint %d = %f", i, torque_real[i]);
	}
  
  //fisrt pose------------------------------------------------------------------------------
  
  double theta[]={0,0,0,0,0,0,0}; // theta[] = [s0,s1,eo,e1,w0,w1,w2]
  
  //position command
  for(int i = 0; i < 7; i++){
    cmd.command[i] = theta[i];
  }

  //calculate expected torque
  double torque_exp[] = {0,Torque1(theta),Torque2(theta),Torque3(theta),Torque4(theta),Torque5(theta),Torque6(theta)};

  sleep(5);//wiat for movment completion

  // get torque readings from baxter and calculate the error
  for(int i = 0; i < 7; i++){
  //The function calls in main() pass the name of the array, exam_scores, as an argument because the name of an array in an expression evaluates to a pointer to the array. In other words, the expression, exam_scores, is a pointer to (the first element of) the array, exam_scores[].The function calls in main() pass the name of the array, exam_scores, as an argument because the name of an array in an expression evaluates to a pointer to the array. In other words, the expression, exam_scores, is a pointer to (the first element of) the array, exam_scores[].
    torque_real[i] = states.effort[i]; //actual value
    torque_err[i]= torque_real[i] - torque_exp[i];    // calculate torque error
    ROS_INFO("for joint %d: actual Torque  = %f, expected torque = %f, Torque Error = %f", i, torque_real[i], torque_exp[i],torque_err[i]);
	}

  
  //second pose---------------------------------------------------------------------------
  
  //theta[]={0.25,0.25,0.25,0.25,0.25,0.25,0.25}; // theta[] = [s0,s1,eo,e1,w0,w1,w2]
  
  
  //third pose---------------------------------------------------------------------------
  
  //theta[]={0.5,0.5,0.5,0.5,0.5,0.5,0.5}; // theta[] = [s0,s1,eo,e1,w0,w1,w2]
  

  //fourth pose---------------------------------------------------------------------------
  
  //theta[]={0.75,0.75,0.75,0.75,0.75,0.75,0.75}; // theta[] = [s0,s1,eo,e1,w0,w1,w2]
  
 


  //----------------------------------------------------------------------------------------
  std::cout<<cmd<<std::endl;
  while(ros::ok()){
    //update cmd.command commands here
    left_cmd_pub.publish(cmd);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
