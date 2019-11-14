//ROS related
#include <ros/ros.h>
#include <std_msgs/String.h>

//c++ related
#include <fstream>
#include <string>
#include <initializer_list>
#include <iostream>
#include <vector>

//voice related
#include "recogfile.h"
#include "tts.h"

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

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int stage1=0;
int stage2=0;
std_msgs::String response_str;
std::string class_name;

vector<string> split(const string& s, const string& sep);
unsigned int ques_find_or(string str_org, initializer_list<string> lst);
int voice_recog();
std::string last_medicine(string s,std::vector<string> v){
	std::string result="";
	for(int i=0;i<v.size();i++){
		if(s.compare(v.at(i))==0){
			result=v.at(i-1);
		}
	}
	return result;

}

void codeCallback(const std_msgs::String::ConstPtr& msg){
    if(stage1==0){
		response_str.data=msg->data;
		if(response_str.data.length()>1)
		{
			ROS_INFO("prescription:%s",response_str.data.c_str());
			stage1=1;
		}
	}
}
void classCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("In chat_node classCallback:%s",msg->data.c_str());
	class_name=msg->data.c_str();
}

std::string next_medicine;
void medicineCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("In chat_node medicineCallback:%s",msg->data.c_str());
	next_medicine=msg->data.c_str();
}




int main(int argc,char **argv){
    const char* login_params = "appid = 5caf019f, work_dir = .";//5aa8ea00  
	MSPLogout();
	MSPLogin(NULL, NULL, login_params);

    ros::init(argc, argv,"chat");
    ros::NodeHandle nh;
	
/*    ros::Publisher response_pub=nh.advertise<std_msgs::String>("/response",1);*/
//	ros::Subscriber reconizer_sub=nh.subscribe("/reconizer/output",1,reconizerCallback);
	ros::Subscriber code_sub=nh.subscribe("/zbar_opencv/code_info",1,codeCallback);
	ros::Subscriber class_sub = nh.subscribe("/darknet_ros/class_name", 1, classCallback);
	ros::Subscriber medicine_sub = nh.subscribe("/next_medicine", 1, medicineCallback);

	std_msgs::String tips_msg;
	ros::Publisher tip_pub = nh.advertise<std_msgs::String>("/tip_switch", 1);
	std_msgs::String follow_switch_msg;
	ros::Publisher follow_pub = nh.advertise<std_msgs::String>("/follow_switch", 1);
	
	ros::Rate loop_rate(1.0);
MoveBaseClient ac("move_base", true);
	/*=============test(二维码读取测试)===============================test end(pass)=======*/
	ROS_INFO("======strat======");	
	std::string prescription;
	while(ros::ok()){
		if(stage1==1){
			ROS_INFO("Recive perscription");
			prescription=response_str.data;
			/*
			std_msgs::String received;
			received.data="Prescription received . ";
			received.data+=response_str.data;
			ROS_INFO("%s",received.data.c_str());
			response_pub.publish(received);
			*/
			tts("prescription received.");
			
			std::string s="the prescription is : ";
			s+=prescription;

			tts(s.c_str());
			break;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	std::vector<string> medicine_order;
	prescription+=";";
    medicine_order=split(prescription,";");
	next_medicine = medicine_order.at(0);
	/*=============test(物品询问)===============================test end(pass)============*/
	string final_tts;
	while(ros::ok()){
		int number=voice_recog();
		switch(number){
			case 1:
				ros::spinOnce();
				final_tts="The prescription is "+prescription;
				tts(final_tts.c_str());
				break;
			case 2:
				
			case 3:
			
			case 4:

			case 5:
				ros::spinOnce();
				final_tts="The next medicine is "+next_medicine;
				tts(next_medicine.c_str());
				tts("You can check the picture on the screen");
				//image_show
				tips_msg.data=next_medicine;
				tip_pub.publish(tips_msg);
				ROS_INFO("========next mdicine=======\n%s",tips_msg.data.c_str());
				
				break;
			case 6:

			case 7:

			case 8:

			case 9:
				ros::spinOnce();
				final_tts="The medicine on your hand is "+class_name;
				tts(final_tts.c_str());
				break;

			

		}
	}
	

	/*=============test(语音识别测试)================
	ROS_INFO("here");
	while(1){
		ROS_INFO("HERE2");
		std::string s=recog();
		ROS_INFO("HERE3");
		ROS_INFO("%s",s.c_str());
		tts("hello");
	}
	==============test end(pass)=======*/

	

    MSPLogout(); 
    return 0;
}


unsigned int ques_find_or(string str_org, initializer_list<string> lst)
{
	unsigned int find_num = 0;
	for (auto i = lst.begin(); i != lst.end(); i++)
	{
		if (str_org.find(*i) != str_org.npos)
		{
			cout<<*i<<endl;
			return 1;
		}
	}
	return 0;
}

int voice_recog()
{
	string str_raw;
	
    cout<<"======== recognizing ========"<<endl;
    str_raw = recog();
    if((str_raw.empty()||str_raw.size()<10)) return 0;
    
    if(ques_find_or(str_raw, {"script"})) return 1;
	if(ques_find_or(str_raw, {"medicine should I"})) return 2;
    if(ques_find_or(str_raw, {"drug should I"})) return 3;
    if(ques_find_or(str_raw, {"one should I"})) return 4;
	if(ques_find_or(str_raw, {"next"})) return 5;
	if(ques_find_or(str_raw, {"medicine in my hand"})) return 6;
    if(ques_find_or(str_raw, {"drug in my hand"})) return 7;
    if(ques_find_or(str_raw, {"one in my hand"})) return 8;
	if(ques_find_or(str_raw, {"is this"})) return 9;
	if(ques_find_or(str_raw,{"Follow","follow"})) return 10; 
	if(ques_find_or(str_raw,{"ready","Ready","leave","Leave"})) return 11; 
	return 0;
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
unsigned int find_quest(string quest_str)
{
	if(ques_find_or(quest_str, {"medicine"})) return 1;
	
}
*/

/*
void answer( int quest_i)
{
	switch (quest_i)
	{
		case 1:tts("can you help me take the medicine? Yes, I can."); 
		  txtwrite("你好，我叫艾米丽！");break;
	}
}

int Answering()
{
	int label_i;
	int num=0;
	string str;
	string tempstr;
	//tts("You can ask me a question clearly and loudly.");
	while(1)
	{
		std::cout<<"======== recognizing ========"<<std::endl;
		str = recog()cog(;
		if((str.empty()||str.size()<10))
		{
			continue;
		}
		label_i=find_quest(str);
		if((label_i>0)&&(label_i<10))
		{
				//cout << "问题序号：" << label_i << endl;
				answer(label_i);
				num++;
				if (num<10) continue;
				else break;
		}
		
	}
	return 0;
}
*/