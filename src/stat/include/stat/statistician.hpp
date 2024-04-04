#include <cstdio>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <vector>
#include <cassert>
#include <fstream>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"             // odom_drone
#include "geometry_msgs/msg/twist_stamped.hpp"           // gt_vel, cmd_vel
#include "geometry_msgs/msg/pose.hpp"            // gt_pose
#include "geometry_msgs/msg/pose_stamped.hpp"            // gt_pose
#include "geometry_msgs/msg/point_stamped.hpp"   // altitude
#include "geometry_msgs/msg/vector3_stamped.hpp" // magnetic
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp" // gps
#include "sensor_msgs/msg/range.hpp"
#include "ee4308_lib/core.hpp"

#include "Eigen/Dense"

struct StatisticianParameters
{
    struct topics{
        std::string gt_pose = "gt_pose";
        std::string gt_vel = "gt_vel";
        std::string baro = "altitude"; 
        std::string magnetic = "magnetic";
        std::string imu = "imu";
        std::string gps = "gps";
        std::string sonar = "sonar";
    } topics;
    const double sample_frequency = 10.0;
    const uint64_t num_samples = 1e2;
    const double rad_polar = 6356752.3;
    const double rad_equator = 6378137;
    const double G = 9.81;
    std::string output_file = "statistics.csv";
};

class Statistician : public rclcpp::Node
{
public:
    Statistician(
        std::string node_name);
private:

    StatisticianParameters params_;
    void gt_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void gt_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void baro_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void magnetic_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void sonar_callback(const sensor_msgs::msg::Range::SharedPtr msg);

    void check_data_ready();
    void calculate_statistics();
    Eigen::Vector3d getECEF(
        const double &sin_lat, const double &cos_lat,
        const double &sin_lon, const double &cos_lon,
        const double &alt);

    void initTopics();
    // void initParams();
    // TODO: Create launch file and parameter file for this node

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sonar_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr baro_sub;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr magnetic_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gt_pose_sub;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr gt_vel_sub;

    Eigen::Matrix<double, 3, Eigen::Dynamic> gt_pose_data;
    Eigen::Matrix<double, 3, Eigen::Dynamic> gt_vel_data;
    Eigen::Matrix<double, 1, Eigen::Dynamic> baro_data;
    Eigen::Matrix<double, 2, Eigen::Dynamic> magnetic_data;
    Eigen::Matrix<double, 4, Eigen::Dynamic> imu_data;
    Eigen::Matrix<double, 3, Eigen::Dynamic> gps_data;
    Eigen::Matrix<double, 1, Eigen::Dynamic> sonar_data;

    bool gt_pose_data_ready; 
    bool gt_vel_data_ready; 
    bool baro_data_ready; 
    bool magnetic_data_ready;
    bool imu_data_ready; 
    bool gps_data_ready;
    bool sonar_data_ready;

    bool initialized_ECEF;
    Eigen::Vector3d initial_ECEF;
};