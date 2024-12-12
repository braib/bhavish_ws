#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

class LaserScanFilterNode : public rclcpp::Node
{
public:
    LaserScanFilterNode() : Node("laser_scan_filter_node")
    {
        // Subscriber to listen to /scan topic (LaserScan data)
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScanFilterNode::scan_callback, this, std::placeholders::_1));

        // Publisher to publish filtered data to /filtered_scan
        filtered_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/filtered_scan", 10);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Create a new LaserScan message for the filtered data
        sensor_msgs::msg::LaserScan filtered_scan = *msg;

        // Calculate the indices corresponding to 0 and 120 degrees in the laser scan data
        int start_index = static_cast<int>((0.0 - msg->angle_min) / msg->angle_increment);
        int end_index = static_cast<int>((2.0 * M_PI / 3.0 - msg->angle_min) / msg->angle_increment);

        // Filter the laser scan data to only include the range of 0 to 120 degrees
        filtered_scan.ranges = std::vector<float>(
            msg->ranges.begin() + start_index,
            msg->ranges.begin() + end_index);

        // Filter the angle information as well
        filtered_scan.angle_min = 0.0;
        filtered_scan.angle_max = 2.0 * M_PI / 3.0;
        filtered_scan.angle_increment = (filtered_scan.angle_max - filtered_scan.angle_min) / filtered_scan.ranges.size();

        // Publish the filtered laser scan data
        filtered_scan_publisher_->publish(filtered_scan);
    }

    // ROS2 subscription and publisher
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_scan_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
