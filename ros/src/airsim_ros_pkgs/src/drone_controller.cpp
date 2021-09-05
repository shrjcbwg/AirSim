#include "ros/ros.h"
#include <airsim_ros_pkgs/VelCmd.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <Eigen/Dense>
#include "pd_position_controller_simple.h"

class drone_controller
{  
public:
  drone_controller()
  vel_cmd = airsim_ros_pkgs::VelCmd();
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;

  drone_controller()  
  {  
    //Topic you want to publish  
    pub_vel_cmd = n_.advertise<airsim_ros_pkgs::VelCmd>("/vel_cmd_body_frame", 1);  
    sub_pos_cmd = n_.subscribe("/position_cmd", 50, cmdCallback); 
  }  

  void cmdCallback( quadrotor_msgs::PositionCommand msg)  

  {  
    ROS_INFO("I heard cmd: [%d]", msg.trajectory_id.c_str()); 
    //.... do something with the input and generate the output...
    position.x = msg.position.x;
    position.y = msg.position.y;
    position.z = msg.position.z;

    velocity.x = msg.position.x;
    vel_cmd.

  }  

  void compute_control_cmd()
{
    curr_error_.x = target_position_.x - curr_position_.x;
    curr_error_.y = target_position_.y - curr_position_.y;
    curr_error_.z = target_position_.z - curr_position_.z;
    curr_error_.yaw = math_common::angular_dist(curr_position_.yaw, target_position_.yaw);

    double p_term_x = params_.kp_x * curr_error_.x;
    double p_term_y = params_.kp_y * curr_error_.y;
    double p_term_z = params_.kp_z * curr_error_.z;
    double p_term_yaw = params_.kp_yaw * curr_error_.yaw;

    double d_term_x = params_.kd_x * prev_error_.x;
    double d_term_y = params_.kd_y * prev_error_.y;
    double d_term_z = params_.kd_z * prev_error_.z;
    double d_term_yaw = params_.kp_yaw * prev_error_.yaw;

    prev_error_ = curr_error_;

    vel_cmd_.twist.linear.x = p_term_x + d_term_x;
    vel_cmd_.twist.linear.y = p_term_y + d_term_y;
    vel_cmd_.twist.linear.z = p_term_z + d_term_z;
    vel_cmd_.twist.angular.z = p_term_yaw + d_term_yaw; // todo
}

void enforce_dynamic_constraints()
{
    double vel_norm_horz = sqrt((vel_cmd_.twist.linear.x * vel_cmd_.twist.linear.x) 
                            + (vel_cmd_.twist.linear.y * vel_cmd_.twist.linear.y));

    if (vel_norm_horz > constraints_.max_vel_horz_abs)
    {
        vel_cmd_.twist.linear.x = (vel_cmd_.twist.linear.x / vel_norm_horz) * constraints_.max_vel_horz_abs; 
        vel_cmd_.twist.linear.y = (vel_cmd_.twist.linear.y / vel_norm_horz) * constraints_.max_vel_horz_abs; 
    }

    if (std::fabs(vel_cmd_.twist.linear.z) > constraints_.max_vel_vert_abs)
    {
        // todo just add a sgn funciton in common utils? return double to be safe. 
        // template <typename T> double sgn(T val) { return (T(0) < val) - (val < T(0)); }
        vel_cmd_.twist.linear.z = (vel_cmd_.twist.linear.z / std::fabs(vel_cmd_.twist.linear.z)) * constraints_.max_vel_vert_abs; 
    }
    // todo yaw limits
    if (std::fabs(vel_cmd_.twist.linear.z) > constraints_.max_yaw_rate_degree)
    {
        // todo just add a sgn funciton in common utils? return double to be safe. 
        // template <typename T> double sgn(T val) { return (T(0) < val) - (val < T(0)); }
        vel_cmd_.twist.linear.z = (vel_cmd_.twist.linear.z / std::fabs(vel_cmd_.twist.linear.z)) * constraints_.max_yaw_rate_degree;
    }

}

void publish_control_cmd()
{
    airsim_vel_cmd_world_frame_pub_.publish(vel_cmd_);
}

private:  
  ros::NodeHandle n_;   
  ros::Publisher pub_vel_cmd;  
  ros::Subscriber sub_pos_cmd;
};//End of class SubscribeAndPublish  

int main(int argc, char **argv)  
{  
  //Initiate ROS  
  ros::init(argc, argv, "subscribe_and_publish");  

  //Create an object of class SubscribeAndPublish that will take care of everything  
  SubscribeAndPublish test;  
  //ros::spin();
  ros::MultiThreadedSpinner s(2);  //多线程
  ros::spin(s);  

  return 0;  
}  

void SubscribeAndPublish::callback2(const std_msgs::String::ConstPtr& msg2)
{
  ROS_INFO("I heard: [%s]", msg2->data.c_str());
  ros::Rate loop_rate(0.5);//block chatterCallback2() 0.5Hz
  loop_rate.sleep();
}
