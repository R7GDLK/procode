#include "ros/ros.h"
#include <stdlib.h>
#include <string.h>
#include <c++/5.4.0/iostream>
#include <c++/5.4.0/sstream>
#include <c++/5.4.0/string>
#include <c++/5.4.0/stdexcept>
#include <c++/5.4.0/vector>
#include <glob.h>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include "viewer.hpp"
#include <stdio.h>
#include <opencv2/dnn.hpp>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#define camID 0
#define FACENUM 5
using namespace cv;
using namespace std;
using namespace cv::dnn;

const size_t inWidth = 300;
const size_t inHeight = 300;
const float inScaleFactor = 0.007843f;
const float meanVal = 127.5;

//===============================================================================================================================
int take_photo=0;
void arrayCallback(const std_msgs::Int8::ConstPtr& take_photo_allow)
{
	take_photo=take_photo_allow->data;
	
}

int main(int argc, char *argv[])
{
/*===========================ROS节点等初始化======================================================================*/
	ros::init(argc, argv, "image_publisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("camera/image", 1);
	ros::Subscriber sub = nh.subscribe("take_photo", 1, arrayCallback);
	ros::Rate loop_rate(5);
	
	std::string ns = K2_DEFAULT_NS;
	std::string topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
	std::string topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
	bool useExact = true;
	bool useCompressed = false;
	Receiver::Mode mode = Receiver::IMAGE;
	topicColor = "/" + ns + topicColor;
	topicDepth = "/" + ns + topicDepth;
	Receiver receiver(topicColor, topicDepth, useExact, useCompressed);
	receiver.start(mode);
	Mat color, depth;
	cout<<"拍照节点开始运行，等待拍照并发送"<<endl;
	while (ros::ok())
	{
		if(take_photo==1)
		{
			for(int k=1;k<=10;k++)
			receiver.imageViewer(color, depth, 1);
			cout<<"拍照成功"<<endl;
	
	
			/* cv::VideoCapture cap(0);  
				if(!cap.isOpened())   
				{  
					ROS_INFO("can not opencv video device\n");  
					return 1;  
				}  
				for(int k=1;k<=10;k++)
						cap >> color;
	         */
	
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color).toImageMsg();
    pub.publish(msg);
    cout<<"发送成功"<<endl;
    cout<<" "<<endl;
    take_photo=0;
	}

	
	
    ros::spinOnce();
    loop_rate.sleep();
    
    

	}
 
}

