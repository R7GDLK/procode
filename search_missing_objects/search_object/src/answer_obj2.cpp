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
int Arr[50]={0};
void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr& array);
int N=10;
int sum=0;
string obj_name={};
string obj_kind[10]={"vita_tea","biscuit","potato_chips","crisps","cake","chocolate","lemon_tea","coke_cola","snickers","chew_gum"};
string place[5]={"on the dining table","on the teapoy","on the sideboard","near the TV","in the room"};
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
	MSPLogout();
	MSPLogin(NULL, NULL, login_params);//第一个参数是用户名，第二个参数是密码，第三个参数是登录参数，用户名和密码可在http://open.voicecloud.cn注册获取
	ros::init(argc, argv, "arraySubscriber");
	ros::NodeHandle n;	
	ros::Rate loop_rate(1.0);
	ros::Subscriber sub3 = n.subscribe("obj_num", 100, arrayCallback);
	cout<<"回答问题节点开始运行，等待提问"<<endl;
	while (sum==0)
	ros::spinOnce();
	cout<<"收到物品数量信息"<<endl;
	for(int j= 0; j < N; j++)
	{
		cout<<Arr[j]<<endl;
	}
	int command_cout=0;
	tts("I have remembered the number of all kinds of objects.you can ask me questions.");
	while (1)
    {
        std::cout<<"**************************************************************"<<std::endl;
        std::cout<<"This is command_test "<<command_cout<<std::endl;
        command_cout++;
        std::string get_result=recog();
        if(get_result.empty()||get_result.size()<10)
         {
			 //tts("speak aloud");
             continue;
         }
       int speices=-1;
        int position=-1;
       if( ques_find_or(get_result,{"where","Where"})) 
       {
				   if( ques_find_or(get_result,{"vita","Vita","party","white"}))   speices=0;
				   if(ques_find_or(get_result,{"biscuit","bis","Biscuit","Bis"}))  speices=1;
				    if(ques_find_or(get_result,{"potato","chips","Potato","Chips"}))    speices=2;
				    if(ques_find_or(get_result,{"crisps","cr","Cr"}))  speices=3;
				    if(ques_find_or(get_result,{"cake","Cake"}))    speices=4;
				    if(ques_find_or(get_result,{"Chocolate","chocolate","late","Late"}))  speices=5;
				    if(ques_find_or(get_result,{"lemon","Lemon"}))   speices=6;
				    if(ques_find_or(get_result,{"coke","cola","coca","Coke","Cola","Coca"}))  speices=7;
				    if(ques_find_or(get_result,{"snickers","inker","sni","kers"}))   speices=8;
				    if(ques_find_or(get_result,{"chew","gum","to come","gun","to go on"}))    speices=9;
				    if(speices!=-1)
				     {
						 for(int k=0;k<50;k=k+10)
						 {
							 if(Arr[speices+k]!=0)
							 {
								 position=k/10;
								string object_kind=obj_kind[speices];
								string object_place=place[position];
								string answer="The "+object_kind+" is "+object_place;
								const char* p = answer.data();//string型转*char型，tts()入口参数为*char型
								tts(p);//语音复述一遍
								ROS_INFO(p);
								 
							}
							 
						}
					}
					else continue;
				   
		}
		
		else
		{


				   if( ques_find_or(get_result,{"dining","table","Dining","Table"}) ) position=0;
				   if( ques_find_or(get_result,{"teapoy","poy","Teapoy","Poy"}) ) position=1;
				   if( ques_find_or(get_result,{"side","board","Side","Board"}) ) position=2;
				   if( ques_find_or(get_result,{"tv","TV","t v","T V"}) ) position=3;
				   if( ques_find_or(get_result,{"room","Room"}) ) position=4;
				   if( ques_find_or(get_result,{"vita","Vita","party","white"}))   speices=0;
				   if(ques_find_or(get_result,{"biscuit","bis","Biscuit","Bis"}))  speices=1;
				    if(ques_find_or(get_result,{"potato","chips","Potato","Chips"}))    speices=2;
				    if(ques_find_or(get_result,{"crisps","cr","Cr"}))  speices=3;
				    if(ques_find_or(get_result,{"cake","Cake"}))    speices=4;
				    if(ques_find_or(get_result,{"Chocolate","chocolate","late","Late"}))  speices=5;
				    if(ques_find_or(get_result,{"lemon","Lemon"}))   speices=6;
				    if(ques_find_or(get_result,{"coke","cola","coca","Coke","Cola","Coca"}))  speices=7;
				    if(ques_find_or(get_result,{"snickers","inker","sni","kers"}))   speices=8;
				    if(ques_find_or(get_result,{"chew","gum","to come","gun","to go on"}))    speices=9;
				   if (position!=-1&&speices!=-1)
				   {
					  string object_kind=obj_kind[speices];
					string object_place=place[position];
					string answer="The number of "+object_kind+object_place+to_string(Arr[position*10+speices]);
					const char* p = answer.data();//string型转*char型，tts()入口参数为*char型
					tts(p);//语音复述一遍
					ROS_INFO(p);
					}
					else continue;
			       
       }
       
       
       
       
	   /*if( ques_find_or(get_result,{"vita","Vita","party","white"}))  
	   {
		speices=0; 
		string answer="The number of vita tea in the room is "+to_string(Arr[speices]);
		printf("%s\n",answer);
		const char* p = answer.data();//string型转*char型，tts()入口参数为*char型
        tts(p);//语音复述一遍
		ROS_INFO(p);
		}
	   else if(ques_find_or(get_result,{"biscuit","bis","Biscuit","Bis"})) 
	   {
		 speices=1; 
		string answer="The number of biscuit in the room is "+to_string(Arr[speices]);
		const char* p = answer.data();//string型转*char型，tts()入口参数为*char型
        tts(p);//语音复述一遍
        ROS_INFO(p);
		} 
	   else if(ques_find_or(get_result,{"potato","chips","Potato","Chips"}))  
	   {
		   speices=2; 
		 string answer="The number of potato chips in the room is "+to_string(Arr[speices]);
		const char* p = answer.data();//string型转*char型，tts()入口参数为*char型
        tts(p);//语音复述一遍
        ROS_INFO(p);
		}
	   else if(ques_find_or(get_result,{"crisps","cr","Cr"}))  
	   {
		   speices=3; 
		 string answer="The number of crisps in the room is "+to_string(Arr[speices]);
		const char* p = answer.data();//string型转*char型，tts()入口参数为*char型
        tts(p);//语音复述一遍
        ROS_INFO(p);
		}	   
		else if(ques_find_or(get_result,{"cake","Cake"}))  
	   {
		   speices=4; 
		 string answer="The number of cake in the room is "+to_string(Arr[speices]);
		const char* p = answer.data();//string型转*char型，tts()入口参数为*char型
        tts(p);//语音复述一遍
        ROS_INFO(p);
		}	   
		else if(ques_find_or(get_result,{"Chocolate","chocolate","late","Late"}))  
	   {
		   speices=5; 
		 string answer="The number of chocolate in the room is  "+to_string(Arr[speices]);
		const char* p = answer.data();//string型转*char型，tts()入口参数为*char型
        tts(p);//语音复述一遍
        ROS_INFO(p);
		}	   
		else if(ques_find_or(get_result,{"lemon","Lemon"}))  
	   {
		  speices=6; 
		string answer="The number of lemon_tea in the room is "+to_string(Arr[speices]);
		const char* p = answer.data();//string型转*char型，tts()入口参数为*char型
        tts(p);//语音复述一遍
        ROS_INFO(p);
		}	   
		else if(ques_find_or(get_result,{"coke","cola","coca","Coke","Cola","Coca"}))  
	   {
		   speices=7; 
		 string answer="The number of coke cola in the room is "+to_string(Arr[speices]);
		const char* p = answer.data();//string型转*char型，tts()入口参数为*char型
        tts(p);//语音复述一遍
        ROS_INFO(p);
		}
		else if(ques_find_or(get_result,{"snickers","inker","sni","kers"}))  
	   {
		   speices=8; 
		 string answer="The number of sinkers in the room is "+to_string(Arr[speices]);
		const char* p = answer.data();//string型转*char型，tts()入口参数为*char型
        tts(p);//语音复述一遍
        ROS_INFO(p);
		}
		else if(ques_find_or(get_result,{"chew","gum","to come","gun","to go on"}))  
	   {
		  speices=9; 
		 string answer="The number of chew gum in the room is "+to_string(Arr[speices]);
		const char* p = answer.data();//string型转*char型，tts()入口参数为*char型
        tts(p);//语音复述一遍
        ROS_INFO(p);
		}
		
	   else continue;
		}*/
	   

	  // }

	   
        
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
