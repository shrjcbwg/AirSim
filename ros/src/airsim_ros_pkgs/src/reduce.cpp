#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>


ros::Publisher pub;
float leftSize;
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
 // 声明存储原始数据与滤波后的数据的点云的格式  
 // Container for original & filtered data
  // pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; //原始的点云的数据格式
  // pcl::PCLPointCloud2 cloud_filtered;//存储滤波后的数据格式

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredT (new pcl::PointCloud<pcl::PointXYZ>);

  // Convert to PCL data type。转化为PCL中的点云的数据格式
  // pcl_conversions::toPCL(*cloud_msg, *cloud);
  pcl::fromROSMsg(*cloud_msg,latest_cloud);
  ROS_INFO_STREAM("The Size of original cloud is:" << latest_cloud.points.size());
  // pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  // ROS_INFO_STREAM("The Size of filtered cloud is:" << cloudPtr.)
  // Perform the actual filtering进行一个滤波处理
  // pcl::VoxelGrid<pcl::PCLPointCloud2> sor; //创建滤波对象
  // sor.setInputCloud (cloudPtr);  //设置输入的滤波，将需要过滤的点云给滤波对象
  // sor.setLeafSize (leftSize, leftSize, leftSize);  //设置滤波时创建的体素大小为1cm立方体
  // sor.filter (cloud_filtered);//执行滤波处理，存储输出cloud_filtered
  // pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  // // build the filter
  // outrem.setInputCloud(latest_cloud.makeShared());
  // outrem.setRadiusSearch(0.4);
  // outrem.setMinNeighborsInRadius(2);
  // // apply filter
  // outrem.filter(*cloud_filtered);


  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (latest_cloud.makeShared());
  // 设置平均距离估计的最近邻居的数量K
  sor.setMeanK (50);
  // 设置标准差阈值系数，值越小，滤波效果越强
  sor.setStddevMulThresh (1.0);
  // sor.setNegative(True); 滤波器取反
  // 执行过滤
  sor.filter (*cloud_filtered);
 

  ROS_INFO_STREAM("The Size of filtered cloud is:" << cloud_filtered->points.size());

  sensor_msgs::PointCloud2 map_cloud;
  pcl::toROSMsg(*cloud_filtered, map_cloud);  //搞成消息
  map_cloud.header.stamp = ros::Time::now();
  map_cloud.header.frame_id = "/map"; 
  pub.publish(map_cloud);  //加上时间戳和frameid发布出来
  // Convert to ROS data type。// 再将滤波后的点云的数据格式转换为ROS下的数据格式发布出去
  // sensor_msgs::PointCloud2 output;//声明的输出的点云的格式
  // pcl_conversions::moveFromPCL(cloud_filtered, output);//第一个参数是输入，后面的是输出
 
  // // Publish the data
  // pub.publish (output);
}
 
int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");//声明节点的名称
  ros::NodeHandle nh;
  nh.getParam("leftSize",leftSize);
 
  // Create a ROS subscriber for the input point cloud
// 为接受点云数据创建一个订阅节点
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/airsim_node/drone_1/points", 1, cloud_cb);
 
  // Create a ROS publisher for the output point cloud
//创建ROS的发布节点
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/airsim_node/drone_1/points_reduced", 1);
 
  // Spin
  // 回调
  ros::spin ();
  return 0;
}