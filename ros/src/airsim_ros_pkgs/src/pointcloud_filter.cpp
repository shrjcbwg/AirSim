#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Pose.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>


using namespace std;
using namespace Eigen;


string target_frame_id;
string cloud_topic;
float leaf_size;
float depth;
int Neighbors_numb;
double td_threshold;

class SegMapROSWraper  
{
private:
  ros::NodeHandle m_nh;  
  ros::Publisher m_globalcloudPub;  
  ros::Subscriber odom_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> *m_pointCloudSub;  
  tf::MessageFilter<sensor_msgs::PointCloud2> *m_tfPointCloudSub; 
  tf::TransformListener m_tfListener;  

  

public:
  SegMapROSWraper()
      : m_nh("~")  
  {

      m_nh.getParam("target_frame_id", target_frame_id);
      m_nh.getParam("cloud_topic", cloud_topic);
      m_nh.getParam("leaf_size", leaf_size);
      m_nh.getParam("Neighbors_numb", Neighbors_numb);
      m_nh.getParam("td_threshold", td_threshold);
      m_nh.getParam("depth", depth);

      m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(m_nh,cloud_topic, 100);  
      m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2>(*m_pointCloudSub, m_tfListener, target_frame_id, 100); 
      m_tfPointCloudSub->registerCallback(boost::bind(&SegMapROSWraper::insertCloudCallback, this, _1));  
      m_globalcloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("/airsim_node/drone_1/points_filtered", 1);  


  }

  ~SegMapROSWraper()
  {

      delete m_pointCloudSub;
      delete m_tfPointCloudSub;

  }
  

  void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud)  
  {
      pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr pc_global (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredT (new pcl::PointCloud<pcl::PointXYZ>);

      pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
      pcl::PassThrough<pcl::PointXYZ> pass;
      pcl::fromROSMsg(*cloud, *pc);
      pcl::VoxelGrid<pcl::PointXYZ> VG;


      tf::StampedTransform sensorToWorldTf;  
      try
      {
          m_tfListener.lookupTransform(target_frame_id, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
      }
      catch (tf::TransformException &ex)
      {
          ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
          return;
      }

      // ROS_ERROR_STREAM("The Size cloud is: " << pc->points.size());
      // ROS_ERROR_STREAM("leaf_size is: " << leaf_size);
      pass.setInputCloud (pc);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.0, depth);
      pass.filter (*cloud_filteredT);

      // Create the filtering object
      pcl::VoxelGrid<pcl::PointXYZ> sor;
      sor.setInputCloud (cloud_filteredT);
      sor.setLeafSize (leaf_size, leaf_size, leaf_size);
      sor.filter (*cloud_filtered);


      // outrem.setInputCloud(cloud_filteredT);
      // outrem.setRadiusSearch(0.1);
      // outrem.setMinNeighborsInRadius(2);
      // // apply filter
      // outrem.filter(*cloud_filtered);

//     //   ROS_INFO_STREAM("The Size of orginal cloud is:" << cloud_filteredT->points.size());
      // ROS_ERROR_STREAM("The Size of orginal cloud is: " << cloud_filtered->points.size());
  //     pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  //     sor.setInputCloud (cloud_filteredT);
  // // 设置平均距离估计的最近邻居的数量K
  //     sor.setMeanK (Neighbors_numb);
  // // 设置标准差阈值系数，值越小，滤波效果越强
  //     sor.setStddevMulThresh (td_threshold);
  // // sor.setNegative(True); 滤波器取反
  // // 执行过滤
  //     sor.filter (*cloud_filtered);
  //   //   ROS_INFO_STREAM("The Size of filtered cloud is:" << cloud_filtered->points.size());
  //     ROS_ERROR_STREAM("The Size of filtered cloud is: " << cloud_filtered->points.size());

      Eigen::Matrix4f sensorToWorld;
      pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);  
      pcl::transformPointCloud(*cloud_filtered, *pc_global, sensorToWorld);  


      // VG.setInputCloud (pc_global);
	    // VG.setLeafSize (leaf_size, leaf_size, leaf_size);
	    // VG.filter(*cloud_filtered);
      // std::cerr << "PointCloud after filtering: " << pc_global->width * pc_global->height  
      //   << " data points (" << pcl::getFieldsList(*pc_global) << ")."; 

      // std::cout<< sensorToWorld <<std::endl;
      sensor_msgs::PointCloud2 map_cloud;
      pcl::toROSMsg(*pc_global, map_cloud); 
      map_cloud.header.stamp = ros::Time::now();
      map_cloud.header.frame_id = target_frame_id; 
      m_globalcloudPub.publish(map_cloud); 
  }
};


int main(int argc, char** argv) {

  ros::init(argc, argv, "filter"); 

  SegMapROSWraper SM;

  ros::spin();
  return 0;
}