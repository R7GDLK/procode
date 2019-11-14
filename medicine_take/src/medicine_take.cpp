//ROS related
#include <ros/ros.h>
#include <std_msgs/String.h>

//Simple Action Client
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_tutorials/FibonacciAction.h>

//tf listener
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

//c++ related
#include <string>



#include <string>
#include <stdlib.h>
#include <string.h>

#include "recogfile.h"
#include "tts.h"


using namespace std;


string getnum(int num)
{
    switch(num)
    {
        case 0:return "zero";break;
        case 1:return "one";break;
        case 2:return "two";break;
        case 3:return "three";break;
        case 4:return "four";break;
        case 5:return "five";break;
        case 6:return "six";break;
        case 7:return "seven";break;
        case 8:return "eight";break;
        case 9:return "nine";break;
		case 10:return "ten";break;
        default :return "";break;
    }
}




unsigned int ques_find_or(string str_org, initializer_list<string> lst)
{
	unsigned int find_num = 0;
	for (auto i = lst.begin(); i != lst.end(); i++)
	{
		if (str_org.find(*i) != str_org.npos)
		{
			//cout<<*i<<endl;
			return 1;
		}
	}
	return 0;
}

int spr_recog_m()
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
	if(ques_find_or(str_raw, {"this"})) return 9;
	return 0;
}












/*
void code_infoCallback(const std_msgs::String::ConstPtr& msg){
//msg是识别到的QRcode的信息
}
*/
std::string class_name;
void objectCallback(const std_msgs::String::ConstPtr& msg){
    class_name=msg->data;
}

std::string prescription;
std::vector<std::string> medicine_v;

int medicine_order[10]={0};

void AddNewWaypoint(float &x,float &y)
{    /*获取当前机器人在map当中的坐标*/
    
    tf::TransformListener listener;
    tf::StampedTransform transform;
    listener.waitForTransform("/map","/base_link",  ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform("/map","/base_link", ros::Time(0), transform);

    x = transform.getOrigin().x(); //当前位置的x坐标
    y = transform.getOrigin().y(); //当前位置的y坐标
}



int main(int argc, char **argv){
    ros::init(argc,argv,"medicine_take");
    ros::NodeHandle nh;
    const char* login_params = "appid = 5caf019f, work_dir = .";//5aa8ea00  
	MSPLogout();
	
	MSPLogin(NULL, NULL, login_params);
    //Obtaining a prescription( process in another node )

    
    // //语音节点
    // ros::Publisher response_pub = nh.advertise<std_msgs::String>("response",1);
    // std_msgs::String response_msg;

    // //物品识别节点
    // ros::Subscriber object_sub = nh.subscribe("/darknet_ros/class_name", 1, objectCallback);
    while(ros::ok){
        int label = spr_recog_m();

        ROS_INFO("label:%d",label);
        switch(label){
            case 0: continue;
        }
        
    }
    /*=========test=======
    //观察老人是否按规定顺序吃药
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
    pub(stop);
    ==========test end(complete a half)===*/


    /*=================test==============
    //拍摄到物品以后 就会给出提示， 提示老人正在吃药。
    ros::Rate loop_rate(1.0);
    ROS_INFO("======5s=========");
    sleep(5);
    ROS_INFO("======START======");
    for(int i;i<=10;i++){
        if(class_name.length()>1){
            std_msgs::String notice;
            notice.data="Old man is taking medicine";
            response_pub.publish(notice);
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    ==================test end(pass)=======*/


    // find the old person
    //Reminding the old person to take medicine
    /*1.look for the old person*/
        /*1.1 the old person is not near to the medicine site
        /**
            1.测量medicine_site的x、y坐标，假设为x0和y0
            double x0,y0;
            x0=0.0;
            y0=0.0;
            2.获取机器人当前的坐标信息。设为x1和y1.
            float x1,y1;
            AddNewWayPoint(x1,y1);
            3.对比二者之间的偏差。超过一定的 阈值 则判断为老人未吃药,否则进入1.2
            if(x1-x0 >= N && y1-y0 >= N){
                response_msg.data="I have detected you do not take the medicine, please take the medicine";
                response_pub.publish(response_msg);
                //tts("I have detected you do not take the medicine, please take the medicine");
            }
            4.发出语音提醒老人吃药,并告诉老人"我将会跟随你到药物旁边"
            tts("I will follow you to apporach to the medicine site");
            //open the follow node
            //..
            //close the follow node
        **/
        /*1.2 no medicine on the front table or in hand
        /** 
            1、提醒老人伸出手进行拍照，查看老人手中是否有药。
            tts("please reach out your hand, I will help you take the right medicine")

            2、若老人手中无药，则导航至桌子旁边拍照。查看桌子上是否有药
            sub(...);
            if(NO_MEDICINE_HAND){
                //Sending Goals to the Navigation Stack
                //example in APPS/simple_navigation_goals
            }
            3、若桌子上无药，则提醒老人吃药
            if(NO_MEDICINE_DESK){
                response_msg.data="please take medicine";
                response_pub.publish(response_msg);
                //tts("please take medicine");
            }
            4、在步骤2或3当中识别到药。则发出语音"老人正在吃药"
            if(DETECTE_MEDICINE){
                response_msg.data="The old man is taking medicine now";
                response_pub.publish(response_msg);
                //tts("The old man is taking medicine now")
            }
        **/

    /*2.check whether the old person take medicine*/
        /*2.1 ask-for-help*/
        /**
            将语音交互节点开启，并且语音交互节点当中已经保存了二维码当中的信息
        **/
        /*2.2 mistake*/
        /**
            level 1  当中没有
            level 2  当中开启跟随
    */

    MSPLogout(); 
	return 0;
}
