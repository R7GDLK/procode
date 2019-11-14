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
double location[8][3]={{2.541,-0.081,0},{4.247,-0.2,0},{5.564,-0.065,0},{6.421,0.668,0},{7.210,0.920,-3.14/2},
	{6.22,3.6},{6.439,5.200,3.14},{6.414,5.913,3.14/2}};

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
void move()
{
		ros::NodeHandle nh;
		int rate=100;
		int flag=0;
		ros::Rate loop_rate(rate);
		move_base_msgs::MoveBaseGoal goal;
		double goal_angle,angle_duration;
		int ticks;
		geometry_msgs::Twist vel;
		ros::Publisher v_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 50, true);
		float linear_speed,goal_distance,linear_duration;
		goal_distance = 2.0;
		vel.linear.x = 0.3;
		linear_duration = goal_distance /vel.linear.x;
		ticks = int(linear_duration * rate);
		vel.angular.z = 0;
		for(int j=0;j<ticks;j++)	
		{
		  v_pub.publish(vel);
		  loop_rate.sleep();

		}
			vel.linear.x = 0;
			vel.angular.z = 0;	
			v_pub.publish(vel);
					

}


int main(int argc, char *argv[])
{
	move_base_msgs::MoveBaseGoal goal;
	ros::init(argc, argv, "indoor_navigation");
	ros::NodeHandle n;	
	ros::Publisher pub = n.advertise<std_msgs::Int8>("take_photo", 1);
	MoveBaseClient ac("move_base", true);
	ac.waitForServer(ros::Duration(5.0));
	ROS_INFO("move_base action server come up");
	cout<<"导航节点开始运行，等待发送导航目标点"<<endl;
	std_msgs::Int8 take_photo;
	take_photo.data=1;
	move();
	
	/*goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = location[0][0];
	goal.target_pose.pose.position.y =  location[0][1];
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw( location[0][2]);
	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("succeeded");
	else ROS_INFO("failed");
	
	sleep(5);
	/*pub.publish(take_photo);
	cout<<"开始拍照"<<endl;
	sleep(10);*/
	
	/*goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = location[1][0];
	goal.target_pose.pose.position.y =  location[1][1];
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw( location[1][2]);
	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("succeeded");
	else ROS_INFO("failed");
	
	sleep(5);
	/*pub.publish(take_photo);
	cout<<"开始拍照"<<endl;
	sleep(10);*/
	
	/*goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = location[2][0];
	goal.target_pose.pose.position.y =  location[2][1];
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw( location[2][2]);
	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("succeeded");
	else ROS_INFO("failed");
	
	sleep(5);
	/*pub.publish(take_photo);
	cout<<"开始拍照"<<endl;
	sleep(10);*/
	
	/*goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = location[3][0];
	goal.target_pose.pose.position.y =  location[3][1];
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw( location[3][2]);
	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("succeeded");
	else ROS_INFO("failed");
	sleep(1);*/
	
   /*goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = location[4][0];
	goal.target_pose.pose.position.y =  location[4][1];
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw( location[4][2]);
	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("succeeded");
	else ROS_INFO("failed");
	sleep(1);
		goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = location[5][0];
	goal.target_pose.pose.position.y =  location[5][1];
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw( location[5][2]);
	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("succeeded");
	else ROS_INFO("failed");
	sleep(1);
		goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = location[6][0];
	goal.target_pose.pose.position.y =  location[6][1];
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw( location[6][2]);
	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("succeeded");
	else ROS_INFO("failed");
	sleep(1);
	/*goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = location[7][0];
	goal.target_pose.pose.position.y =  location[7][1];
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw( location[7][2]);
	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("succeeded");
	else ROS_INFO("failed");
	sleep(1);*/
		
	return 0;
}
	
