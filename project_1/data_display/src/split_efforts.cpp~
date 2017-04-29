#include<ros/ros.h> 
#include<std_msgs/Float64.h>
#include<baxter_core_msgs/SEAJointState.h>

std_msgs::Float64 g_joint_0_effort;
std_msgs::Float64 g_joint_1_effort;
std_msgs::Float64 g_joint_2_effort;
std_msgs::Float64 g_joint_3_effort;
std_msgs::Float64 g_joint_4_effort;
std_msgs::Float64 g_joint_5_effort;
std_msgs::Float64 g_joint_6_effort;

void splitCallback(const baxter_core_msgs::SEAJointState message_holder){
	//dig out individual efforts, update global variables
	g_joint_0_effort.data = message_holder.actual_effort[0];
	g_joint_1_effort.data = message_holder.actual_effort[1];
	g_joint_2_effort.data = message_holder.actual_effort[2];
	g_joint_3_effort.data = message_holder.actual_effort[3];
	g_joint_4_effort.data = message_holder.actual_effort[4];
	g_joint_5_effort.data = message_holder.actual_effort[5];
	g_joint_6_effort.data = message_holder.actual_effort[6];
}

int main(int argc, char **argv){
	//
	ros::init(argc, argv, "effort_splitter");
	ros::NodeHandle n;
	ros::Subscriber effort_subscriber = n.subscribe("/robot/limb/left/gravity_compensation_torques", 1, splitCallback);
	
	//make one publisher for each joint
	ros::Publisher effort_publisher_0 = n.advertise<std_msgs::Float64>("effort0", 1);
	ros::Publisher effort_publisher_1 = n.advertise<std_msgs::Float64>("effort1", 1);
	ros::Publisher effort_publisher_2 = n.advertise<std_msgs::Float64>("effort2", 1);
	ros::Publisher effort_publisher_3 = n.advertise<std_msgs::Float64>("effort3", 1);
	ros::Publisher effort_publisher_4 = n.advertise<std_msgs::Float64>("effort4", 1);
	ros::Publisher effort_publisher_5 = n.advertise<std_msgs::Float64>("effort5", 1);
	ros::Publisher effort_publisher_6 = n.advertise<std_msgs::Float64>("effort6", 1);

	while(ros::ok()){
		//
		effort_publisher_0.publish(g_joint_0_effort);
		ROS_INFO("Joint effforts: %f,%f,%f,%f,%f,%f,%f", g_joint_0_effort, g_joint_1_effort, g_joint_2_effort, g_joint_3_effort, g_joint_4_effort, g_joint_5_effort, g_joint_6_effort);
		effort_publisher_1.publish(g_joint_1_effort);
		effort_publisher_2.publish(g_joint_2_effort);
		effort_publisher_3.publish(g_joint_3_effort);
		effort_publisher_4.publish(g_joint_4_effort);
		effort_publisher_5.publish(g_joint_5_effort);
		effort_publisher_6.publish(g_joint_6_effort);
		ros::spinOnce();
	}

	return 0;
}
