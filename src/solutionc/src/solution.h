#ifndef SOLUTION__SOLUTION_H_
#define SOLUTION__SOLUTION_H_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>

class Solution : public rclcpp::Node
{
public:
    explicit Solution(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~Solution();  // Proper destructor

private:
    void angle_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_subscriber_;
    image_transport::Subscriber sub_;  // image subscriber via image_transport
    float curr_angle = 0.0;

    const std::string OPENCV_WINDOW = "Image window";  // move here for scope
};

#endif  // SOLUTION__SOLUTION_H_
