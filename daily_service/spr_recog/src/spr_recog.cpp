#include "ros/ros.h"
#include <stdlib.h>
#include <c++/5.4.0/iostream>
#include <c++/5.4.0/string>
#include <c++/5.4.0/stdexcept>
#include <c++/5.4.0/vector>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "recogfile.h"
#include "tts.h"


using namespace cv;
using namespace std;



void txtwrite(string str){
	fstream dataFile;
	dataFile.open("//home//robot//catkin_ws//src//IJCAI//daily_service//spr_recog//src//result//theSpeechResult.txt", ios::app);
	if(!dataFile){
		cout<<"open src File Error opening"<<endl;
		//return -1;
	}
	cout<< str <<endl;
	dataFile<< str <<endl;
}

unsigned int find_or(string str_org, initializer_list<string> lst)
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

int find_command(string str)
{
	if(find_or(str, {"tar","ollow me","ust follow","ut follow"})) return 1;
	if(find_or(str, {"top","top follow"})) return 0;
	return -1;
}

void follow_response(int i)
{
	switch (i)
	{
		case 1:tts("OK, I will follow you.");cout << "state : follow" << endl;break;
		case 0:tts("OK, follow stop.");cout << "state : stop" << endl; break;
	}
}

/*int AnswerQuestions()
{
	int label_i;
	int num = 0;
	string str;
	while(1)
	{
		std::cout<<"===== recognizing ====="<<std::endl;
		str = recog();
		if((str.empty()||str.size()<10))
		{
			continue;
		}
		label_i=find_quest(str);
		if((label_i>0)&&(label_i<10))
		{
				cout << "问题序号：" << label_i << endl;
				answer(label_i);
				num++;
				if (num<10) continue;
				else break;
		}
		//~ if(label_i==0) {
		    //~ string str1="..."+str;
			//~ const char *p=str1.data();
			//~ tts(p);
			//~ tts("next question");
			//~ num++;
			//~ if (num<5) continue;
			//~ else break;
		//~ }
	}
	return 0;
}*/





int main(int argc, char *argv[])
{
	const char* login_params = "appid = 5caf019f, work_dir = .";//5aa8ea00  5c7a2954

	MSPLogin(NULL, NULL, login_params);

	ros::init(argc, argv, "spr_recog");
	ros::NodeHandle nh;

	ros::Rate loop_rate(50);
	std_msgs::String follow_state;
	ros::Publisher voice_pub = nh.advertise<std_msgs::String>("follow_switch", 1);

	int flag=0;
	string str;
	cout<<"准备就绪"<<endl;
	tts("Good morning, please stand in front of me.");
	tts("I will follow you when you start to walk.Please walk slowly.");
	while(ros::ok())
	{
		
		std::cout<<"======== recognizing ========"<<std::endl;
		str = recog();
		if((str.empty()||str.size()<10))
		{
			continue;
		}
		flag = find_command(str);
		follow_response(flag);
		
		//~ if(cmd == 1) {follow_allow8.data = 1; voice_pub.publish(follow_allow8);}
		//~ if(cmd == 2) {follow_allow8.data = 0; voice_pub.publish(follow_allow8);}
		
		if(flag == 1) {follow_state.data = "follow"; voice_pub.publish(follow_state);}
		if(flag == 0) {follow_state.data = "stop"; voice_pub.publish(follow_state);}
		
		ros::spinOnce();
		loop_rate.sleep();

	}
			
	MSPLogout(); 
	return 0;
}

