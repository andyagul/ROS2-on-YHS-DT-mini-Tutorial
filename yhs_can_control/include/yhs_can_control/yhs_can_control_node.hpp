#ifndef __YHS_CANCONTROL_NODE_H__
#define __YHS_CANCONTROL_NODE_H__

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>


#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"

#include "yhs_can_interfaces/msg/io_cmd.hpp"
#include "yhs_can_interfaces/msg/ctrl_cmd.hpp"
#include "yhs_can_interfaces/msg/free_ctrl_cmd.hpp"
#include "yhs_can_interfaces/msg/chassis_info_fb.hpp"

#define READ_PARAM(TYPE, NAME, VAR, VALUE) VAR = VALUE; \
       	node->declare_parameter<TYPE>(NAME, VAR); \
       	node->get_parameter(NAME, VAR);

namespace yhs {
class CanControl
{

public:
	CanControl(rclcpp::Node::SharedPtr);
	~CanControl();
	
	bool run();
	void stop();
	
private:

	rclcpp::Node::SharedPtr node_;

	std::string if_name_;
	bool use_tf_;
  int can_socket_;
  std::thread thread_;
  std::mutex mutex_;
  
  double imu_roll_, imu_pitch_, imu_yaw_;
  
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  rclcpp::Subscription<yhs_can_interfaces::msg::IoCmd>::SharedPtr io_cmd_subscriber_;
  rclcpp::Subscription<yhs_can_interfaces::msg::CtrlCmd>::SharedPtr ctrl_cmd_subscriber_;
  rclcpp::Subscription<yhs_can_interfaces::msg::FreeCtrlCmd>::SharedPtr free_ctrl_cmd_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber_;
  
  rclcpp::Publisher<yhs_can_interfaces::msg::ChassisInfoFb>::SharedPtr chassis_info_fb_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  
  void io_cmd_callback(const yhs_can_interfaces::msg::IoCmd::SharedPtr io_cmd_msg);
  
  void ctrl_cmd_callback(const yhs_can_interfaces::msg::CtrlCmd::SharedPtr ctrl_cmd_msg);
  
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_msg);
  
  void free_ctrl_cmd_callback(const yhs_can_interfaces::msg::FreeCtrlCmd::SharedPtr free_ctrl_cmd_msg);
  
  bool wait_for_can_frame();
  
  void can_data_recv_callback();
  
  void imu_data_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  
  void publish_odom(const double linear_vel, const double angular_vel);
  
};

}


#endif

