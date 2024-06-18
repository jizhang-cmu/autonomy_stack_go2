#include <iostream>
#include <chrono>
#include <vector>
#include <fstream>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

int state = -1;

std::vector<sensor_msgs::msg::Imu> imu_static;
std::vector<sensor_msgs::msg::Imu> imu_rotation_positive_z;
// std::vector<sensor_msgs::msg::Imu> imu_rotation_negative_z;

double acc_bias_x = 0;
double acc_bias_y = 0;
double acc_bias_z = 0;

double ang_bias_x = 0;
double ang_bias_y = 0;
double ang_bias_z = 0;

double ang_z2x_proj = 0;
double ang_z2y_proj = 0;

void imu_handler(const sensor_msgs::msg::Imu::ConstSharedPtr msg_in)
{
    double theta = 15.1 / 180 * 3.1415926; // Convert it to z-up

    double x = msg_in->angular_velocity.x;
    double y = -msg_in->angular_velocity.y;
    double z = -msg_in->angular_velocity.z;

    double x2 = x * cos(theta) - z * sin(theta);
    double y2 = y;
    double z2 = x * sin(theta) + z * cos(theta);

    double acc_x = msg_in->linear_acceleration.x;
    double acc_y = -msg_in->linear_acceleration.y;
    double acc_z = -msg_in->linear_acceleration.z;

    double acc_x2 = acc_x * cos(theta) - acc_z * sin(theta);
    double acc_y2 = acc_y;
    double acc_z2 = acc_x * sin(theta) + acc_z * cos(theta);

    sensor_msgs::msg::Imu msg_store = *msg_in;
    msg_store.angular_velocity.x = x2;
    msg_store.angular_velocity.y = y2;
    msg_store.angular_velocity.z = z2;
    msg_store.linear_acceleration.x = acc_x2;
    msg_store.linear_acceleration.y = acc_y2;
    msg_store.linear_acceleration.z = acc_z2;
    
    if (state == 1){
        imu_static.push_back(msg_store);
    }
    else if (state == 2){
        imu_rotation_positive_z.push_back(msg_store);
    }
}

void serialize_to_file(){
    const char* homeDir = getenv("HOME");
    std::string file_path = std::string(homeDir) + "/Desktop/imu_calib_data.yaml";
    std::ofstream file;
    file.open(file_path, std::ios::out);
    file << "acc_bias_x: " << acc_bias_x << std::endl;
    file << "acc_bias_y: " << acc_bias_y << std::endl;
    file << "acc_bias_z: " << acc_bias_z << std::endl;
    file << "ang_bias_x: " << ang_bias_x << std::endl;
    file << "ang_bias_y: " << ang_bias_y << std::endl;
    file << "ang_bias_z: " << ang_bias_z << std::endl;
    file << "ang_z2x_proj: " << ang_z2x_proj << std::endl;
    file << "ang_z2y_proj: " << ang_z2y_proj << std::endl;
    file.close();
}

void estimate_bias(){
    // Constant bias from static
    int static_count = 0;
    int positive_rot_count = 0;
    int negative_rot_count = 0;

    double ang_rot_x_mean_positive = 0;
    double ang_rot_y_mean_positive = 0;
    double ang_rot_z_mean_positive = 0;

    double ang_rot_x_mean_negative = 0;
    double ang_rot_y_mean_negative = 0;
    double ang_rot_z_mean_negative = 0;
    
    std::cout << "static size:" << imu_static.size() << std::endl;
    std::cout << "pos size:" << imu_rotation_positive_z.size() << std::endl;
    // std::cout << "neg size:" << imu_rotation_negative_z.size() << std::endl;

    for (auto imu_data: imu_static){
        static_count += 1;

        acc_bias_x += imu_data.linear_acceleration.x;
        acc_bias_y += imu_data.linear_acceleration.y;
        acc_bias_z += imu_data.linear_acceleration.z;

        ang_bias_x += imu_data.angular_velocity.x;
        ang_bias_y += imu_data.angular_velocity.y;
        ang_bias_z += imu_data.angular_velocity.z;
    }
    acc_bias_x /= imu_static.size();
    acc_bias_y /= imu_static.size();
    acc_bias_z /= imu_static.size();
    ang_bias_x /= imu_static.size();
    ang_bias_y /= imu_static.size();
    ang_bias_z /= imu_static.size();
    
    acc_bias_z -= 9.81;

    std::cout << "acc_bias_x: " << acc_bias_x << std::endl;
    std::cout << "acc_bias_y: " << acc_bias_y << std::endl;
    std::cout << "acc_bias_z: " << acc_bias_z << std::endl;
    std::cout << "ang_bias_x: " << ang_bias_x << std::endl;
    std::cout << "ang_bias_y: " << ang_bias_y << std::endl;
    std::cout << "ang_bias_z: " << ang_bias_z << std::endl;

    for (auto imu_data: imu_rotation_positive_z){
        positive_rot_count += 1;
        ang_rot_x_mean_positive += imu_data.angular_velocity.x;
        ang_rot_y_mean_positive += imu_data.angular_velocity.y;
        ang_rot_z_mean_positive += imu_data.angular_velocity.z;
    }
    ang_rot_x_mean_positive = ang_rot_x_mean_positive / imu_rotation_positive_z.size() - ang_bias_x;
    ang_rot_y_mean_positive = ang_rot_y_mean_positive / imu_rotation_positive_z.size() - ang_bias_y;
    ang_rot_z_mean_positive = ang_rot_z_mean_positive / imu_rotation_positive_z.size() - ang_bias_z;

    double positive_comp_x = -ang_rot_x_mean_positive / ang_rot_z_mean_positive;
    double positive_comp_y = -ang_rot_y_mean_positive / ang_rot_z_mean_positive;
    std::cout << "positive_comp_x: " << positive_comp_x << std::endl;
    std::cout << "positive_comp_y: " << positive_comp_y << std::endl;
    
    ang_z2x_proj = positive_comp_x;
    ang_z2y_proj = positive_comp_y;
}

/**
 * State machine:
 * 0-2: step to adjust (0)
 * 2-15: purely static for calibrating bias (1), start collecting data from 3s
 * 15-35: z-axis: 80 deg/s (2)
 * 35-37: write file
 * exit 
*/
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("calibrate_imu");

    auto pubGo2Request = nh->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
    auto pubSpeed = nh->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 5); // Debug purpose

    auto subImu = nh->create_subscription<sensor_msgs::msg::Imu>("/utlidar/imu", 300,  imu_handler);

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = "vehicle";

    rclcpp::Rate rate(100);
    bool status = rclcpp::ok();
    auto beginning = std::chrono::system_clock::now();

    unitree_api::msg::Request req;
    SportClient sport_req;

    bool file_written = false;
    
    double ang_vel = 1.396;

    while (status){
    	rclcpp::spin_some(nh);
    	
        auto current = std::chrono::system_clock::now();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(current - beginning).count();

        if (seconds < 2){
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.linear.y = 0;
            cmd_vel.twist.angular.z = 0;
            pubSpeed->publish(cmd_vel);

            sport_req.Move(req, 0, 0, 0);
            pubGo2Request->publish(req);
            
            state = 0;
        }
        else if (seconds >= 2 && seconds <15){
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.linear.y = 0;
            cmd_vel.twist.angular.z = 0;
            pubSpeed->publish(cmd_vel);

            sport_req.StopMove(req);
            pubGo2Request->publish(req);

            if (seconds >= 5){
                state = 1;
            }
        }
        else if (seconds >= 15 && seconds < 35){
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.linear.y = 0;
            cmd_vel.twist.angular.z = ang_vel;
            pubSpeed->publish(cmd_vel);

            sport_req.Move(req, cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z);
            pubGo2Request->publish(req);

            state = 2;
        }
        else if (seconds >= 35 && seconds < 37){
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.linear.y = 0;
            cmd_vel.twist.angular.z = 0;
            pubSpeed->publish(cmd_vel);

            sport_req.StopMove(req);
            pubGo2Request->publish(req);

            state = 3;
        }

        if (state == 0){
            std::cout << "Adjusting the robot to the initial position..." << std::endl;
        }
        else if (state == 1){
            std::cout << "Collecting static data..." << std::endl;
        }
        else if (state == 2){
            std::cout << "Collecting positive z-axis rotation data..." << std::endl;
        }
        else if (state == 3){
            std::cout << "Writing to file..." << std::endl;
        }
        if (state == 3 && !file_written){
            estimate_bias();
            serialize_to_file();
            file_written = true;
            std::cout << "Calibration finished!" << std::endl;
            break;
        }
        status = rclcpp::ok();
        rate.sleep();
    }
}
