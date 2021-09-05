#include "ros/ros.h"
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <iomanip>
using namespace std;

void subCallback(const nav_msgs::Odometry &msg)
{
   Eigen::Vector3d way_points;
   way_points(0)  = msg.pose.pose.position.x;
   way_points(1)  = msg.pose.pose.position.y;
   way_points(2)  = msg.pose.pose.position.z;


//    cout<< "waypoint:" << way_points(0)<< endl;
   std::ofstream out("/home/rui/Desktop/t.txt",std::ios::app);
   if (out.fail())
    {
        cout<< "ERROR In Write Data"<<endl;
    }
   //<<fixed<<setprecision(2)的作用就是将输出结果转化为小数点后两位 
   out<<fixed<<setprecision(2)<<way_points(0)<<"\t"<<way_points(1)<<"\t"<<way_points(2)<<std::endl;
//    out<<fixed<<setprecision(2)<<way_points(0)<<"\t\t"<<way_points(1)<<"\t\t"<<way_points(2)<<std::endl; 
   // out<<fixed<<setprecision(2)<<x<<" "<<y<<" "<<z<<std::endl;
   out.close();
}
 
int main(int argc, char **argv){

    ros::init(argc,argv,"subscrible_01"); //初始化节点
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/airsim_node_fast/drone_1/odom_local_ned", 1000, subCallback);
    
    ros::spin ();
    return 0;
}
