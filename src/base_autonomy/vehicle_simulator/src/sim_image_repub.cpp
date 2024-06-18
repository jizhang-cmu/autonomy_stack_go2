#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

std::string camera_in_topic = "/camera/image/compressed";
std::string sem_in_topic = "/camera/semantic_image/compressed";
std::string depth_in_topic = "/camera/depth/image_raw";

std::string camera_raw_out_topic = "/camera/image";
std::string sem_raw_out_topic = "/camera/semantic_image";
std::string depth_raw_out_topic = "/camera/depth/image";

// rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_sem;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth;

void repub_handler(const sensor_msgs::msg::CompressedImage::SharedPtr msg, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), 1);

    if (frame.empty()){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to decode image.");
        return; 
    }
    std_msgs::msg::Header header = msg->header;
    sensor_msgs::msg::Image::SharedPtr ros_image_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
    publisher->publish(*ros_image_msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("sim_image_repub");

    node->declare_parameter<std::string>("camera_in_topic", camera_in_topic);
    node->declare_parameter<std::string>("sem_in_topic", sem_in_topic);
    node->declare_parameter<std::string>("depth_in_topic", depth_in_topic);
    node->declare_parameter<std::string>("camera_raw_out_topic", camera_raw_out_topic);
    node->declare_parameter<std::string>("sem_raw_out_topic", sem_raw_out_topic);
    node->declare_parameter<std::string>("depth_raw_out_topic", depth_raw_out_topic);

    node->get_parameter("camera_in_topic", camera_in_topic);
    node->get_parameter("sem_in_topic", sem_in_topic);
    node->get_parameter("depth_in_topic", depth_in_topic);
    node->get_parameter("camera_raw_out_topic", camera_raw_out_topic);
    node->get_parameter("sem_raw_out_topic", sem_raw_out_topic);
    node->get_parameter("depth_raw_out_topic", depth_raw_out_topic);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image = node->create_publisher<sensor_msgs::msg::Image>(camera_raw_out_topic, 10);
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_sem = node->create_publisher<sensor_msgs::msg::Image>(sem_raw_out_topic, 10);
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth = node->create_publisher<sensor_msgs::msg::Image>(depth_raw_out_topic, 10);

    auto sub_image = node->create_subscription<sensor_msgs::msg::CompressedImage>(camera_in_topic, 10, [pub_image](const sensor_msgs::msg::CompressedImage::SharedPtr msg){repub_handler(msg, pub_image);});
    auto sub_sem = node->create_subscription<sensor_msgs::msg::CompressedImage>(sem_in_topic, 10, [pub_sem](const sensor_msgs::msg::CompressedImage::SharedPtr msg){repub_handler(msg, pub_sem);});
    auto sub_depth = node->create_subscription<sensor_msgs::msg::CompressedImage>(depth_in_topic, 10, [pub_depth](const sensor_msgs::msg::CompressedImage::SharedPtr msg){repub_handler(msg, pub_depth);});

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}