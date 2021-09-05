#include <airsim_ros_wrapper.h>
#include <boost/make_shared.hpp>
#include "common/AirSimSettings.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

// std::unique_ptr<msr::airlib::RpcLibClientBase> airsim_client_ = nullptr;
std::unique_ptr<msr::airlib::RpcLibClientBase> airsim_client_ = std::unique_ptr<msr::airlib::RpcLibClientBase>(new msr::airlib::MultirotorRpcLibClient("192.168.3.69"));

void vel_cmd_world_frame_cb(const airsim_ros_pkgs::VelCmd::ConstPtr& msg)
{
    float vel_cmd_duration_ = 0.05f; // todo rosparam
    
    // std::lock_guard<std::mutex> guard(drone_control_mutex_);
    
    static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->moveByVelocityAsync(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z, vel_cmd_duration_, msr::airlib::DrivetrainType::ForwardOnly, true, 'drone_1');

}



 
int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "vel_cmd_control");//声明节点的名称
  ros::NodeHandle nh;


 

  ros::Subscriber vel_cmd_world_frame_sub = nh.subscribe<airsim_ros_pkgs::VelCmd>("/airsim_node/drone_1/vel_cmd_world_frame", 1, vel_cmd_world_frame_cb);

 

  ros::spin ();
}



   
