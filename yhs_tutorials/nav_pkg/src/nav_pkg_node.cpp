#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"


int main(int argc, char * argv[]) {

  rclcpp::init(argc, argv); 
  auto node = std::make_shared<rclcpp::Node>("nav_pkg_node");

  bool goal_done = false;

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";//base_link基于机器人本身做坐标；map基于全局坐标
  pose.header.stamp = node->now();
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 0.0;
  pose.pose.orientation.w = 1.0;

  //订阅导航执行状态
  rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::GoalStatusMessage>::SharedPtr navigation_goal_status_sub;
  navigation_goal_status_sub = node->create_subscription<action_msgs::msg::GoalStatusArray>(
    "navigate_to_pose/_action/status",
    rclcpp::SystemDefaultsQoS(),
    [&](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
      if(msg->status_list.back().status == 2){
        RCLCPP_INFO(node->get_logger(), "导航中。。。");
      }
      else if(msg->status_list.back().status == 4){
        RCLCPP_INFO(node->get_logger(), "导航完成。");
        goal_done = true;
      }
    });

//使用action屏蔽此定义，使用topic去掉屏蔽
//#define USE_TOPIC_PUB

#if defined(USE_TOPIC_PUB) 
  //创建“goal_pose”发布器
  auto navigation_publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);

  //发布导航点
  sleep(2);
  pose.header.stamp = node->now();
  navigation_publisher->publish(pose);

#else
  //创建“navigate_to_pose“action客户端
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client =
    rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    node,
    "navigate_to_pose");

  //等待接连服务器
  auto is_action_server_ready = navigation_action_client->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      node->get_logger(),
      "连接 navigate_to_pose 服务器失败！");
    return 0;
  }

  //发送导航点请求
  nav2_msgs::action::NavigateToPose::Goal navigation_goal = nav2_msgs::action::NavigateToPose::Goal();
  navigation_goal.pose = pose;
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  auto future_goal_handle = navigation_action_client->async_send_goal(navigation_goal, send_goal_options);
  std::chrono::milliseconds server_timeout(1000);
  if (rclcpp::spin_until_future_complete(node, future_goal_handle, server_timeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "发送导航点成功失败！");
    return 0;
  }

  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_goal_handle = future_goal_handle.get();
  if (!navigation_goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "导航点被服务器拒绝！");
    return 0;
  }
#endif


  std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  rclcpp::WallRate loop_rate(2);
  while(rclcpp::ok() && !goal_done) {

    try {

      geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer->lookupTransform("map", "base_link",tf2::TimePointZero);

      RCLCPP_INFO(node->get_logger(), "机器人实时坐标(x,y,yaw)=>(%.3f,%.3f,%.3f)",transform_stamped.transform.translation.x,\
      transform_stamped.transform.translation.y,tf2::getYaw(transform_stamped.transform.rotation));

    } catch (tf2::TransformException &ex) {
//      RCLCPP_ERROR(node->get_logger(), "Tf_listener Exception: %s", ex.what());
    }

    rclcpp::spin_some(node); 
    loop_rate.sleep(); 
  }

#ifndef USE_TOPIC_PUB
  auto status = navigation_goal_handle->get_status();
  if ((status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
    status == action_msgs::msg::GoalStatus::STATUS_EXECUTING) && !goal_done)
  {
    auto future_cancel = navigation_action_client->async_cancel_goal(navigation_goal_handle);

    RCLCPP_INFO(node->get_logger(), "导航取消。");
    sleep(2);
    navigation_goal_handle.reset();
  }
#endif

  rclcpp::shutdown();
  return 0;
}