#include <c++/5.4.0/iostream>
#include <c++/5.4.0/sstream>
#include <c++/5.4.0/string>
#include <c++/5.4.0/stdexcept>
#include <c++/5.4.0/vector>
#include <glob.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include "recogfile.h"
#include "tts.h"



#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <initializer_list>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/Int8.h"
//{7.236,1.150,-3.4/2}  {6.283,3.786,3.5/2}  {6.244,5.285,3.5}
double location[10][3]={{2.657,-0.112,0},{5.683,0.078,0},{4.374,1.259,3.5/2},{3.591,4.709,0},{4.540,4.210,-3.5/2},
	{4.574,1.259,-3.4/2},{7.236,1.050,-3.5/2},{7.830,1.085,-3.5},{6.383,3.786,3.5/2},{6.244,5.285,3.5}};

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int main(int argc, char *argv[])
{
	move_base_msgs::MoveBaseGoal goal;
	ros::init(argc, argv, "indoor_navigation");
	ros::NodeHandle n;	
	ros::Publisher pub = n.advertise<std_msgs::Int8>("take_photo", 1);
	MoveBaseClient ac("move_base", true);
	ac.waitForServer(ros::Duration(5.0));
	ROS_INFO("move_base action server come up");
	std_msgs::Int8 take_photo;
	take_photo.data=1;
	int N=10;

	//for(int i=0;i<N;i++)
	//{
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = 5.713;
	goal.target_pose.pose.position.y =  0.078;
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw( 0);
	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("succeeded");
	else ROS_INFO("failed");
	sleep(5);
		
	pub.publish(take_photo);
	cout<<"开始拍照"<<endl;
	
	sleep(2);
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = 7.266;
	goal.target_pose.pose.position.y =  3.021;
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(3.4/2);
	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("succeeded");
	else ROS_INFO("failed");
	
	pub.publish(take_photo);
	cout<<"开始拍照"<<endl;
	
	
	sleep(2);
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = 6.283;
	goal.target_pose.pose.position.y =  3.786;
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(3.4);
	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("succeeded");
	else ROS_INFO("failed");
	
	pub.publish(take_photo);
	cout<<"开始拍照"<<endl;
	sleep(2);
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = 6.244;
	goal.target_pose.pose.position.y =  5.285;
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(3.4);
	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("succeeded");
	else ROS_INFO("failed");
	
	pub.publish(take_photo);
	cout<<"开始拍照"<<endl;
	
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = 7.266;
	goal.target_pose.pose.position.y =  3.021;
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(3.4/2);
	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("succeeded");
	else ROS_INFO("failed");
	
	return 0;
}
	
