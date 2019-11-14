#include<bits/stdc++.h>
#include<ros/ros.h>
#include<std_msgs/String.h>
using namespace std;
int main(int argc,char** argv){
    ros::init(argc, argv, "me_test");
    ros::NodeHandle nh;
    ros::Publisher pub=nh.advertise<std_msgs::String>("/darknet_ros/report_switch",1);///darknet_ros/report_switch

    std_msgs::String s;

    while(getline(cin,s.data)){
        cout<<s.data<<endl;
        pub.publish(s);
    }

}