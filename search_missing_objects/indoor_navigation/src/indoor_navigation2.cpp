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
#include <turtlesim/Spawn.h>
#include <initializer_list>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/Int8.h"
double location[8][3]={{2.541,-0.081,0},{4.247,-0.2,0},{5.564,-0.065,0},{6.421,0.668,0},{7.210,0.920,-3.14/2},
	{6.22,3.6},{6.439,5.200,3.14},{6.414,5.913,3.14/2}};

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void AddNewWaypoint(float &x,float &y)
{    /*获取当前机器人在map当中的坐标*/
    
    tf::TransformListener listener;
    tf::StampedTransform transform;
    listener.waitForTransform("/map","/base_link",  ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform("/map","/base_link", ros::Time(0), transform);

    x = transform.getOrigin().x(); //当前位置的x坐标
    y = transform.getOrigin().y(); //当前位置的y坐标
}




int flag=0;
void msg_Callback(const std_msgs::String::ConstPtr& msg)
{
	flag=1;
}
int main(int argc, char *argv[])
{
	const char* login_params = "appid = 5caf019f, work_dir = .";//5aa8ea00  5c7a2954

	MSPLogin(NULL, NULL, login_params);
	move_base_msgs::MoveBaseGoal goal;
	ros::init(argc, argv, "indoor_navigation");
	ros::NodeHandle n;	
	//MoveBaseClient ac("move_base", true);
	//ac.waitForServer(ros::Duration(5.0));
	//ROS_INFO("move_base action server come up");
	ros::Subscriber sub_voice = n.subscribe<std_msgs::String>("pose_info", 10, msg_Callback);
	float x,y;
	ros::Rate loop_rate(50);
	while(ros::ok())
	{
	
	if(flag==1)
	{
	/*AddNewWaypoint(x,y);
	
	sleep(10);
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = 1;
	goal.target_pose.pose.position.y = 1 ;
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw( 0);
	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("succeeded");
	else ROS_INFO("failed");
	
	sleep(3);*/
	cout<<"识别成功"<<endl;
	tts("follow me");
	
	/*goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = x;
	goal.target_pose.pose.position.y = y ;
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("succeeded");
	else ROS_INFO("failed");*/
	
	break;
	
	}
	else
		ros::spinOnce();

		
	}
	MSPLogout(); 
	return 0;
}
	
