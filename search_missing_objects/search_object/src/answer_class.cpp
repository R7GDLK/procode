#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include "recogfile.h"
#include "tts.h"
#include "ros/ros.h"
#include <string.h>
#include <string>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
using namespace std;
//int Arr[3]={1,3,1};
int Arr[3]={0};
void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr& array);
int N=3;
int sum=0;
string obj_name={};
string obj_kind[3]={"drink","snack","food"};
int ques_find_or(string str_org, initializer_list<string> lst)
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
int main(int argc, char **argv)
{
/*========科大讯飞初始化=======================================================================*/

	const char* login_params= "appid = 5caf019f, work_dir = .";//登录参数,appid与msc库绑定,请勿随意改动  5c7a2954
	MSPLogin(NULL, NULL, login_params);//第一个参数是用户名，第二个参数是密码，第三个参数是登录参数，用户名和密码可在http://open.voicecloud.cn注册获取
	ros::init(argc, argv, "arraySubscriber1");
	ros::NodeHandle n;	
	ros::Rate loop_rate(1.0);
	ros::Subscriber sub3 = n.subscribe("class_num", 100, arrayCallback);
	cout<<"回答问题节点开始运行，等待提问"<<endl;
	while(sum==0)
	ros::spinOnce();
	cout<<"收到物品数量信息"<<endl;
	for(int j= 0; j < N; j++)
	{
	//	cout<<Arr[j]<<endl;
	}
	int command_cout=0;
	while (1)
    {
        std::cout<<"**************************************************************"<<std::endl;
        std::cout<<"This is command_test "<<command_cout<<std::endl;
        command_cout++;
        std::string get_result=recog();
        if(get_result.empty()||get_result.size()<20)
         {
			 //tts("speak aloud");
             continue;
         }
       int speices;
       
	   if( ques_find_or(get_result,{"drink"}) )  
	   {
		speices=0; 
		string answer="The number of drink in the room is "+to_string(Arr[speices]);
		const char* p = answer.data();//string型转*char型，tts()入口参数为*char型
        tts(p);//语音复述一遍
        ROS_INFO(p);
		   
		}
	   else if(ques_find_or(get_result,{"snack"})) 
	   {
		 speices=1; 
		string answer="The number of snacks in the room is "+to_string(Arr[speices]);
		const char* p = answer.data();//string型转*char型，tts()入口参数为*char型
        tts(p);//语音复述一遍
        ROS_INFO(p);
		} 
	   else if(ques_find_or(get_result,{"food"}))  
	   {
		   speices=2; 
		 string answer="The number of food in the room is "+to_string(Arr[speices]);
		const char* p = answer.data();//string型转*char型，tts()入口参数为*char型
        tts(p);//语音复述一遍
        ROS_INFO(p);
		}

	   else continue;
	   
	   }
   }
	



void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
{

	int i = 0;
	// print all the remaining numbers
	for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		Arr[i] = *it;
		i++;
	}
	for(int j= 0; j < N; j++)
	{
		sum+=Arr[j];
	}
	return;
}
