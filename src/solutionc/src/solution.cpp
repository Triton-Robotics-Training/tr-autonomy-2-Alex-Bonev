#include "solution.h"

#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



Solution::Solution(const rclcpp::NodeOptions & options)
: Node("solution", options)
{
    // Publisher to desired_angle
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("desired_angle", rclcpp::QoS(10));
    // Subscribe to current_angle (Float32)
    angle_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "current_angle",
        rclcpp::QoS(10),
        std::bind(&Solution::angle_callback, this, std::placeholders::_1));

    // Subscribe to robot camera images via image_transport
    sub_ = image_transport::create_subscription(
        this,
        "robotcam",
        std::bind(&Solution::image_callback, this, std::placeholders::_1),
        "raw",
        rmw_qos_profile_sensor_data);

    cv::namedWindow(OPENCV_WINDOW);
    RCLCPP_INFO(this->get_logger(), "Solution node initialized");
}

void Solution::angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received angle: %f", msg->data);

    curr_angle = msg->data;
}

void Solution::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    } catch (cv_bridge::Exception & e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

    cv::Scalar lower_bound(0, 23, 30);
    cv::Scalar upper_bound(0, 255, 255);

    cv::Mat mask;
    cv::inRange(hsv_image, lower_bound, upper_bound, mask);

    cv::imshow(OPENCV_WINDOW, mask);
    cv::waitKey(3);

    int firstWhitePixel = -1;
    int lastWhitePixel = -1;

    for ( int col = 0 ; col < mask.cols ; col++ )
    {
      uchar mask_pixel = mask.at<uchar>(mask.rows/2, col);
      if (mask_pixel > 0)
      {
        if (firstWhitePixel == -1)
        {
          firstWhitePixel = col;
        }
        lastWhitePixel = col;
      }
    }

    int image_center = mask.cols / 2;

    int white_pixel_center = (firstWhitePixel+lastWhitePixel)/2;

    RCLCPP_INFO(this->get_logger(), "White pixel center: %d", white_pixel_center);

    float desired_angle = curr_angle;

    if (white_pixel_center != -1){
        int difference = white_pixel_center - image_center;

        if (difference > 10) {
            desired_angle -= 0.1;
        } else if (difference < -10) {
            desired_angle += 0.1;
        }
    } else {
        desired_angle += 0.4;
    }

    RCLCPP_INFO(this->get_logger(), "Publishing desired angle: %f", desired_angle);

    

    auto message = std_msgs::msg::Float32();
    message.data = static_cast<float>(desired_angle);
    publisher_->publish(message);
}

Solution::~Solution()
{
    cv::destroyWindow(OPENCV_WINDOW);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Solution>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}