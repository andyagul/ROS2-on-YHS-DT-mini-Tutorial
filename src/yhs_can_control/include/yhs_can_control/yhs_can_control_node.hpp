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

// TYPE ：参数的数据类型。
// NAME ：参数的名称。
// VAR ：用于存储参数值的变量。
// VALUE ：如果参数不存在时的默认值。
// 该宏定义的作用是先将默认值 VALUE 赋给变量 VAR，然后在节点 node 中声明参数 NAME，并从参数服务器中获取参数 NAME 的值，存储到变量 VAR 中。
// 这样可以方便地读取参数，并在参数不存在时使用默认值。
// 这个宏定义的使用依赖于节点 node 的声明和初始化，以及参数服务器的配置。在使用时需要确保节点已经正确初始化，并且参数服务器中存在相应的参数。
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

	rclcpp::Node::SharedPtr node_; //等价于 rclcpp::shared_ptr<rclcpp::Node> node_;

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

