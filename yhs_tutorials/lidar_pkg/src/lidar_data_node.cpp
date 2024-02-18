#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {

    std::vector<float> ranges = scan_msg->ranges;

    for (size_t i = 0; i < ranges.size(); ++i) {
        std::cout << "Point " << i << ": " << ranges[i] << std::endl;
    }
}

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("lidar_data_node");

    auto subscription = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), scanCallback);

    rclcpp::spin(node);

    rclcpp::shutdown();
    
    return 0;
}
