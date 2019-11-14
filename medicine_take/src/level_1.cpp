//ROS related
#include <ros/ros.h>
#include <std_msgs/String.h>

//Simple Action Client
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_tutorials/FibonacciAction.h>

//Sending Goals to the Navigation Stack
/*#include <move_base_msgs/MoveBaseAction.h>*/
#include <actionlib/client/simple_action_client.h>

//tf listener
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

//c++ related
#include <bits/stdc++.h>

//voice related
#include "recogfile.h"
#include "tts.h"

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

vector<string> split(const string& s, const string& sep);

string class_name="";
void objectCallback(const std_msgs::String::ConstPtr& msg){
    class_name=msg->data;
}

int stage1=0;
string prescription;
void codeCallback(const std_msgs::String::ConstPtr& msg){
    if(stage1==0){
		if(msg->data.length()>1)
		{
            prescription=msg->data.c_str();
			stage1=1;
		}
	}
}

int main(int argc,char **argv){
    const char* login_params = "appid = 5caf019f, work_dir = .";//5aa8ea00  
	MSPLogout();
	MSPLogin(NULL, NULL, login_params);

    ros::init(argc,argv,"level_one");
    ros::NodeHandle nh;
    MoveBaseClient ac("move_base", true);
    	ac.waitForServer(ros::Duration(1.0));
	ROS_INFO("move_base action server come up");
    ros::Subscriber object_sub = nh.subscribe("/darknet_ros/class_name", 1, objectCallback);
    ros::Subscriber code_sub=nh.subscribe("/zbar_opencv/code_info",1,codeCallback);
    

    std_msgs::String report_msg;
    ros::Publisher report_pub = nh.advertise<std_msgs::String>("/darknet_ros/report_switch", 1);

    std_msgs::String medicine_msg;
    ros::Publisher medicine_pub = nh.advertise<std_msgs::String>("/next_medicine", 1);
    

    vector<string> medicine_order;

    //wait for the action server to come up\
   
   /* while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }*/

    /*=============test(二维码读取)=================*/
	ROS_INFO("======strat======");	
	
    ros::Rate loop_rate(1.0);
	while(ros::ok()){
		if(stage1==1){
			ROS_INFO("=====Prescription received in level_1_node. =====");
            report_msg.data=prescription;
            report_pub.publish(report_msg);
			ROS_INFO("the prescription is : %s",prescription.c_str());
			prescription+=";";
            medicine_order=split(prescription,";");
            break;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

    //we'll send a goal to the robot 
    //OR use a node to find human


    
    move_base_msgs::MoveBaseGoal goal;

 
    
        /*
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = 3.708;
        goal.target_pose.pose.position.y = -5.848;
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        //Robot orientation
        goal.target_pose.pose.orientation.z = 0.999;
        goal.target_pose.pose.orientation.w = -0.041;
        ROS_INFO("Sending goal");
        ac.sendGoal(goal);
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Arrived at the target point");
        else
            ROS_INFO("The base failed to move for some reason");
        */



    
    
    tts("It is time to take medicine, please take medicine now, let me take you to the medicine site");



    goal.target_pose.header.frame_id = "map";//"base_link"
    goal.target_pose.header.stamp = ros::Time:: now();
    //Desk position
    goal.target_pose.pose.position.x = 6.05;
    goal.target_pose.pose.position.y = 0.673;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    //Robot orientation
    goal.target_pose.pose.orientation.z = 0.059; 
    goal.target_pose.pose.orientation.w = 0.998;
    
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Arrived at the target point");
    else
        ROS_INFO("The base failed to move for some reason");
    tts("It is time to take medicine, please take medicine now");
    sleep(10);
    tts("Dear old person, when you are taking medicine, please put the medicine in a sutiable distance");
    tts("The items can placed at a distance of 50 centimeter from my camera.");
   

    string pre="~~~";
    int time=0;
    int time_count=0;
    for(int i=0;i<medicine_order.size();i++){
        ROS_INFO("====READY====");
        sleep(1.0);
        ros::spinOnce();
        string temp;
        temp=class_name;
        
        if(class_name.size()==0){
            i--;
            time ++;
            if(time >= 70){
                tts("You do not take enough meicine, please go back");
                time_count++;
                sleep(5);
                if(time_count>=3){
                    time = -1000;
                }
            }
            continue;
        }
        time = 0;
        if(temp.compare(medicine_order.at(i))==0){
            
            //record
            report_msg.data=medicine_order.at(i);
            report_pub.publish(report_msg);

            ROS_INFO("Take right medicine:%s",medicine_order.at(i).c_str());
            //tts("take right medicine");
            pre=temp;

            //update the medicine what old person take now
            if(i<medicine_order.size()-1)
                medicine_msg.data = medicine_order.at(i+1);
            else
                medicine_msg.data = "You have taken medicine";
            medicine_pub.publish(medicine_msg);

            
        }else{
            
            if(temp!=pre){
                ROS_INFO("Take wrong medicine:%s",temp.c_str());
                tts("Take wrong medicine");
                pre=temp;
            }
            i--;
        }

        
    }
    tts("The old person has taken medicine");

    tts("Competition complete");
    report_msg.data="end";
    report_pub.publish(report_msg);
    /*观察老人是否按规定顺序吃药
    medicine_v.push_back("person");
    medicine_v.push_back("bottle");
    ros::Rate loop_rate(0.2);
    std::cout<<medicine_v.size()<<std::endl;
    for(int i=0;i<=medicine_v.size();i++){
        ROS_INFO("====READY====");
        sleep(10.0);
        ROS_INFO("====START====");
        
        ros::spinOnce();
        ROS_INFO("class_name is:%s,medicine_v.at(i) is:%s",class_name.c_str(),medicine_v.at(i).c_str());
        if(class_name.compare(medicine_v.at(i))==0){
            ROS_INFO("right");
        }else{
            ROS_INFO("Wrong");
            i--;
        }
    }
    ============(not complete)===========*/
    MSPLogout(); 
}

vector<string> split(const string& s, const string& sep)
{
 vector<string> v; 
    string::size_type pos1, pos2;
    pos2 = s.find(sep);
    pos1 = 0;
    while(string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2-pos1));
        
        pos1 = pos2 + sep.size();
        pos2 = s.find(sep, pos1);
    }
    if(pos1 != s.length())
        v.push_back(s.substr(pos1));
    return v;
}
/* 
// split 用法
int main()
{
 string s="medicine a;medicine b;medicine c;medicine d";
 s+=";"; 
  vector<string> ss=split(s,";");
  for(int i=0;i<ss.size();i++)
   cout<<ss[i]<<endl;
 return 0;
}
//判断吃错药的算法
int main()
{
 string s="a;b;c;d";
 s+=";"; 
  vector<string> ss=split(s,";");
  string pre="~~~~";
 for(int i=0;i<ss.size();i++){
//  int flag=true;//false表示出现了错误 
   string temp;
   cin>>temp;
   if(temp==ss[i]){
    cout<<"pass"<<endl;
    pre=temp;
  }else{
   if(temp!=pre){
    cout<<"not pass"<<endl;
    pre=temp;
   }
   i--;
  }
 }
 return 0;
}
*/