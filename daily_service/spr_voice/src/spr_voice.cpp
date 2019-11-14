#include <c++/5.4.0/iostream>
#include <c++/5.4.0/string>
#include <c++/5.4.0/vector>
#include <stdlib.h>
#include <string.h>
#include <fstream>

#include "recogfile.h"
#include "tts.h"

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include "viewer.hpp"
#include "APITools.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace cv;
using namespace std;

void txtwrite(string str){
	fstream dataFile;
	dataFile.open("/home/robot/catkin_ws/src/IJCAI/daily_service/spr_voice/src/result/theSpeechResult.txt", ios::app);
	if(!dataFile){
		cout<<"open src File Error opening"<<endl;
		//return -1;
	}
	cout<< str <<endl;
	dataFile<< str <<endl;
}

int cant_follow = 0;
int take_photo = 0;
int sit_reply = 0;

void chatterCallback(const std_msgs::String::ConstPtr& msg){
	
	if(msg->data == "cant") cant_follow = 1;
	
	if(msg->data == "falling") take_photo = 1;
	
	if(msg->data == "sitting") sit_reply = 1;
	
}

int main(int argc, char *argv[])
{
	const char* login_params = "appid = 5caf019f, work_dir = .";//5aa8ea00  5c7a2954
	MSPLogout();
	MSPLogin(NULL, NULL, login_params);
	
	ros::init(argc, argv, "spr_voice");
	ros::NodeHandle hm;
	//image_transport::ImageTransport it(hm);
	//image_transport::Publisher pub = it.advertise("camera/image", 1);
	ros::Subscriber sub = hm.subscribe("pose_info", 10, chatterCallback);
	ros::Subscriber distance_sub = hm.subscribe("follow_distance", 2, chatterCallback);
	ros::Rate loop_rate(50);
	std::string ns = K2_DEFAULT_NS;
	std::string topicColor = K2_TOPIC_HD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
	std::string topicDepth = K2_TOPIC_HD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
	bool useExact = true;
	bool useCompressed = false;
	Receiver::Mode mode = Receiver::IMAGE;
	topicColor = "/" + ns + topicColor;
	topicDepth = "/" + ns + topicDepth;
	Receiver receiver(topicColor, topicDepth, useExact, useCompressed);
	receiver.start(mode);
	Mat color, depth;
	//tts("I'm ready for testing.");
	cout<<"准备就绪"<<endl;
	while (ros::ok())
	{
		if(cant_follow==1)
		{
			tts("I can't follow you, please walk slowly.");
			txtwrite("I can't follow you, please walk slowly.");
			sleep(1);
			break;
		}
		if(take_photo==1)
		{
			cout<<"识别成功"<<endl;
			tts("Waring!");tts("The elder falls down! ");
			receiver.imageViewer(color, depth, 1);
			tts("I must call at 911 to ask for people's help and take the photo at the scene.");
			txtwrite("Waring, warning! The elder falls down! I must call at 123456789 to ask for people's help and take the photo at the scene.");
			imwrite("/home/robot/catkin_ws/src/IJCAI/daily_service/spr_voice/src/result/theRecord.jpg",color);
			cout<<"拍照成功"<<endl;
			//~ sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color).toImageMsg();
			//~ pub.publish(msg);
			//~ cout<<"发布成功"<<endl;
			break;
		}
		if(sit_reply==1)
		{
			sleep(1);
			tts("hello, I see that you sit down. What can I help you do?");
			txtwrite("hello, I see that you sit down. What can I help you do?");
					//follow中接收pose_info并前进
			break;
		}
		ros::spinOnce();
	}
	
	MSPLogout(); 
	return 0;
}

