#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

int main(int argc, char * argv[]) {

  rclcpp::init(argc, argv); 
  auto node = std::make_shared<rclcpp::Node>("vel_pub_node");

  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  rclcpp::WallRate loop_rate(100);
  while (rclcpp::ok()) {  

    geometry_msgs::msg::Twist vel_cmd;
    vel_cmd.linear.x = 0.1;  
    vel_cmd.angular.z = 0.1; 

    publisher->publish(vel_cmd);  

    loop_rate.sleep(); 
  }

  rclcpp::shutdown();
  return 0;
}
