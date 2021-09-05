#include<ros/ros.h>  
#include<pcl/point_cloud.h>  
#include<pcl_conversions/pcl_conversions.h>  
#include<sensor_msgs/PointCloud2.h>  
#include<pcl/io/pcd_io.h>  
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <cassert>
#include <string>

using namespace std;


bool FileExist(const char* FileName)
{
    struct stat my_stat;
    return (stat(FileName, &my_stat) == 0);
}


string readTxt(string file)
{
    ifstream infile; 
    infile.open(file.data());
    assert(infile.is_open());
    string s;
    while(getline(infile,s))
    {
        return s;
    }
    infile.close();             //关闭文件输入流 
}


void cloudCB(const sensor_msgs::PointCloud2 &input)  
{  

  if (FileExist("/home/rui/Desktop/data/path.tmp"))
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;  
    pcl::fromROSMsg(input, cloud);//从ROS类型消息转为PCL类型消息
    string path = readTxt("/home/rui/Desktop/data/path.tmp");
//  pcl::io::savePCDFileASCII ("/home/shuning/catkin_ws/src/imgpcl/data/write_pcd_test.pcd", cloud);
    pcl::io::savePCDFileASCII (path, cloud);
  }  

}  



main (int argc, char **argv)  
{  
  ros::init (argc, argv, "pcl_write");  
  ros::NodeHandle nh;  
//   ros::Subscriber bat_sub = nh.subscribe("/sdf_map/occupancy_inflate", 10, cloudCB);//接收点云  
  ros::Subscriber bat_sub = nh.subscribe("/sdf_map/occupancy_all", 10, cloudCB);//接收点云  
  ros::spin();  
  return 0;  
}