#include <cstdio>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"             // odom_drone
#include "geometry_msgs/msg/twist.hpp"           // gt_vel, cmd_vel
#include "geometry_msgs/msg/pose.hpp"            // gt_pose
#include "geometry_msgs/msg/point_stamped.hpp"   // altitude
#include "geometry_msgs/msg/vector3_stamped.hpp" // magnetic
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp" // gps
#include "sensor_msgs/msg/range.hpp"

#include "Eigen/Dense"

class Statistician : public rclcpp::Node
{
public:
    Statistician();
    ~Statistician();
private:
    void odom_drone_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void gt_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void gt_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void altitude_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void magnetic_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void range_callback(const sensor_msgs::msg::Range::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_drone_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr gt_vel_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr gt_pose_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr altitude_sub;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr magnetic_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub;

    std::string 

};