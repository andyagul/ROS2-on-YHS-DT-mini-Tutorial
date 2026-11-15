#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>

void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {

    pcl::PointCloud<pcl::PointXYZ> pointCloudIn;

    pcl::fromROSMsg(*cloud_msg , pointCloudIn);
    int cloudSize = pointCloudIn.points.size();
    for(int i=0;i<cloudSize;i++)
    {
        std::cout << "Point " << i << ": (" << std::fixed << std::setprecision(3) 
        << pointCloudIn.points[i].x << ", " 
        << pointCloudIn.points[i].y << ", " 
        << pointCloudIn.points[i].z << ")" 
        << std::endl;
    }  

}

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pc_node");

    RCLCPP_INFO(node->get_logger(),"pc_node start");

    auto subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/points", 10, pointCloudCallback);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
