#include "stat/statistician.hpp"

double interpolate(double x0, double x1, double t0, double t1, double t){
    /*
    Linear interpolation between two points (x0, t0) and (x1, t1) at time t
    Assumes t0 <= t1 and t0 <= t <= t1
    */
    assert(t0 <= t1);
    assert(t0 <= t && t <= t1);
    if(t1 == t){
        return x1;
    } 
    return x0 + (x1 - x0)*(t - t0)/(t1 - t0);
}

void Statistician::initTopics(){
    gt_vel_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        params_.topics.gt_vel, 10, std::bind(&Statistician::gt_vel_callback, this, std::placeholders::_1));
    gt_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        params_.topics.gt_pose, 10, std::bind(&Statistician::gt_pose_callback, this, std::placeholders::_1));
    baro_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
        params_.topics.baro, 10, std::bind(&Statistician::baro_callback, this, std::placeholders::_1));
    magnetic_sub = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        params_.topics.magnetic, 10, std::bind(&Statistician::magnetic_callback, this, std::placeholders::_1));
    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        params_.topics.imu, 10, std::bind(&Statistician::imu_callback, this, std::placeholders::_1));
    gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        params_.topics.gps, 10, std::bind(&Statistician::gps_callback, this, std::placeholders::_1));
    sonar_sub = this->create_subscription<sensor_msgs::msg::Range>(
        params_.topics.sonar, 10, std::bind(&Statistician::sonar_callback, this, std::placeholders::_1));
}

// void Statistician::initParams(){
//     initParam(this, "topics.gt_pose", params_.topics.gt_pose);
//     initParam(this, "topics.gt_vel", params_.topics.gt_vel);
//     initParam(this, "topics.baro", params_.topics.baro);
//     initParam(this, "topics.magnetic", params_.topics.magnetic);
//     initParam(this, "topics.imu", params_.topics.imu);
//     initParam(this, "topics.gps", params_.topics.gps);
//     initParam(this, "topics.sonar", params_.topics.sonar);
//     initParam(this, "sample_frequency", params_.sample_frequency);
//     initParam(this, "num_samples", params_.num_samples);
//     initParam(this, "rad_polar", params_.rad_polar);
//     initParam(this, "rad_equator", params_.rad_equator);
//     initParam(this, "G", params_.G);
// 
// }


Eigen::Vector3d Statistician::getECEF(
    const double &sin_lat, const double &cos_lat,
    const double &sin_lon, const double &cos_lon,
    const double &alt)
{
    Eigen::Vector3d ECEF;
    // --- FIXME ---
    // params_.rad_polar, params_.rad_equator
    const static double e2 = 1 - pow(params_.rad_polar,2) / pow(params_.rad_equator,2); // eccentricity squared
    const double N = params_.rad_equator / sqrt(1-e2*pow(sin_lat,2));
    ECEF(0) = (N+alt)*cos_lat*cos_lon;
    ECEF(1) = (N+alt)*cos_lat*sin_lon;
    ECEF(2) = (N*(1-e2)+alt)*sin_lat;
    // --- EOFIXME ---
    return ECEF;
}
Statistician::Statistician(std::string node_name)
    : Node(node_name)

{
    initTopics();
    // Initialize the data matrices
    const int num_samples = static_cast<int>(params_.num_samples);
    gt_pose_data = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, num_samples);
    gt_vel_data = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, num_samples);
    baro_data = Eigen::Matrix<double, 1, Eigen::Dynamic>::Zero(1, num_samples);
    magnetic_data = Eigen::Matrix<double, 2, Eigen::Dynamic>::Zero(2, num_samples);
    imu_data = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, num_samples);
    gps_data = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, num_samples);
    sonar_data = Eigen::Matrix<double, 1, Eigen::Dynamic>::Zero(1, num_samples);

    // Initialize the data ready flags
    gt_pose_data_ready = false;
    gt_vel_data_ready = false;
    baro_data_ready = false;
    magnetic_data_ready = false;
    imu_data_ready = false;
    gps_data_ready = false;
    sonar_data_ready = false;
}

void Statistician::calculate_statistics(){
    Eigen::Vector3d imu_mean = imu_data.rowwise().mean();
    Eigen::Vector3d gps_mean = gps_data.rowwise().mean();
    Eigen::Vector2d magnetic_mean = magnetic_data.rowwise().mean();
    double baro_mean = baro_data.mean(); 
    double sonar_mean = sonar_data.mean();

    Eigen::Matrix3d imu_cov = 1/(params_.num_samples-1)*
        (imu_data.colwise() - imu_mean)*(imu_data.colwise() - imu_mean).transpose();
    Eigen::Matrix3d gps_cov = 1/(params_.num_samples-1)*
        (gps_data.colwise() - gps_mean)*(gps_data.colwise() - gps_mean).transpose();
    Eigen::Matrix2d magnetic_cov = 1/(params_.num_samples-1)*
        (magnetic_data.colwise() - magnetic_mean)*(magnetic_data.colwise() - magnetic_mean).transpose();
    double baro_var = 1/(params_.num_samples-1)*(baro_data.array() - baro_mean).square().sum();
    double sonar_var = 1/(params_.num_samples-1)*(sonar_data.array() - sonar_mean).square().sum();

    Eigen::MatrixXd gt_pose_z = gt_pose_data.row(2);

    double baro_bias = (baro_data- gt_pose_z).mean();
    double baro_bias_var = 1/(params_.num_samples-1)*(baro_data.array() - gt_pose_z.array()).square().sum();

    // TODO: Estimate optimal FIR Wiener filter for sonar data
    
    std::ofstream output_file;
    output_file.open(params_.output_file);
    output_file << "imu_mean_x,imu_mean_y,imu_mean_z,";
    output_file << "gps_mean_x,gps_mean_y,gps_mean_z,";
    output_file << "magnetic_mean_x,magnetic_mean_y,";
    output_file << "baro_mean,sonar_mean,";
    output_file << "imu_cov_xx,imu_cov_xy,imu_cov_xz,";
    output_file << "imu_cov_yx,imu_cov_yy,imu_cov_yz,";
    output_file << "imu_cov_zx,imu_cov_zy,imu_cov_zz,";
    output_file << "gps_cov_xx,gps_cov_xy,gps_cov_xz,";
    output_file << "gps_cov_yx,gps_cov_yy,gps_cov_yz,";
    output_file << "gps_cov_zx,gps_cov_zy,gps_cov_zz,";
    output_file << "magnetic_cov_xx,magnetic_cov_xy,";
    output_file << "magnetic_cov_yx,magnetic_cov_yy,";
    output_file << "baro_var,sonar_var,";
    output_file << "baro_bias,baro_bias_var\n";
    output_file << imu_mean(0) << "," << imu_mean(1) << "," << imu_mean(2) << ",";
    output_file << gps_mean(0) << "," << gps_mean(1) << "," << gps_mean(2) << ",";
    output_file << magnetic_mean(0) << "," << magnetic_mean(1) << ",";
    output_file << baro_mean << "," << sonar_mean << ",";
    output_file << imu_cov(0,0) << "," << imu_cov(0,1) << "," << imu_cov(0,2) << ",";
    output_file << imu_cov(1,0) << "," << imu_cov(1,1) << "," << imu_cov(1,2) << ",";
    output_file << imu_cov(2,0) << "," << imu_cov(2,1) << "," << imu_cov(2,2) << ",";
    output_file << gps_cov(0,0) << "," << gps_cov(0,1) << "," << gps_cov(0,2) << ",";
    output_file << gps_cov(1,0) << "," << gps_cov(1,1) << "," << gps_cov(1,2) << ",";
    output_file << gps_cov(2,0) << "," << gps_cov(2,1) << "," << gps_cov(2,2) << ",";
    output_file << magnetic_cov(0,0) << "," << magnetic_cov(0,1) << ",";
    output_file << magnetic_cov(1,0) << "," << magnetic_cov(1,1) << ",";
    output_file << baro_var << "," << sonar_var << ",";
    output_file << baro_bias << "," << baro_bias_var << "\n";
    output_file.close();
    return;
}

void Statistician::check_data_ready(){
    if(gt_pose_data_ready && 
        gt_vel_data_ready && 
        baro_data_ready && 
        magnetic_data_ready && 
        imu_data_ready && 
        gps_data_ready && 
        sonar_data_ready){
        calculate_statistics();
        // ROS info that the statistics have been calculated
        RCLCPP_INFO_STREAM(this->get_logger(), "Statistics calculated, shutting down.");
        rclcpp::shutdown();
    }
    return;
}

void Statistician::gt_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg){
    static double prev_time = 0.0;
    static double delta_T = 0.0;
    static uint64_t sample_count = 0;
    static Eigen::Matrix<double, 3, 1> prev_data {msg->twist.linear.x,
                                                    msg->twist.linear.y,
                                                    msg->twist.linear.z};

    if(gt_vel_data_ready){
        return;
    }
    double current_time = msg->header.stamp.sec + 1e-9*msg->header.stamp.nanosec;
    auto data = msg->twist;
    delta_T += current_time - prev_time;
    if(delta_T < 1/params_.sample_frequency){
        prev_time = current_time;
        prev_data(0) = data.linear.x;
        prev_data(1) = data.linear.y;
        prev_data(2) = data.linear.z;
        return;
    }
    if(sample_count >= params_.num_samples){
        // Save the data to a file
        gt_vel_data_ready = true;
       // std::ofstream output_file;
       // output_file.open("gt_vel_data.csv");
       // output_file << "x,y,z\n";
       // for(uint64_t i = 0; i < num_samples; i++){
       //     output_file << gt_vel_data(0,i) << "," << gt_vel_data(1,i) << "," << gt_vel_data(2,i) << "\n";
       // }
       // output_file.close();
        RCLCPP_INFO_STREAM(this->get_logger(), "Ground truth velocity data ready.");
        check_data_ready();
        return;
    }
    delta_T -= 1/params_.sample_frequency;
    sample_count++;
    gt_vel_data(0,sample_count) = interpolate(prev_data(0),
        data.linear.x, prev_time, current_time, current_time-delta_T);
    gt_vel_data(1,sample_count) = interpolate(prev_data(1),
        data.linear.y, prev_time, current_time, current_time-delta_T);
    gt_vel_data(2,sample_count) = interpolate(prev_data(2),
        data.linear.z, prev_time, current_time, current_time-delta_T);
    return;
}
    
void Statistician::gt_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    static double prev_time = 0.0;
    static double delta_T = 0.0;
    static uint64_t sample_count = 0;
    static Eigen::Matrix<double, 3, 1> prev_data{msg->pose.position.x,
                                                 msg->pose.position.y,
                                                 msg->pose.position.z};
    if(gt_pose_data_ready){
        return;
    }
    double current_time = msg->header.stamp.sec + 1e-9*msg->header.stamp.nanosec;
    auto data = msg->pose.position;
    delta_T += current_time - prev_time;
    if(delta_T < 1/params_.sample_frequency){
        prev_time = current_time;
        prev_data(0) = data.x;
        prev_data(1) = data.y;
        prev_data(2) = data.z;
        return;
    }
    if(sample_count >= params_.num_samples){
        // Save the data to a file
        gt_pose_data_ready = true;
        //std::ofstream output_file;
        //output_file.open("gt_pose_data.csv");
        //output_file << "x,y,z\n";
        //for(uint64_t i = 0; i < num_samples; i++){
        //    output_file << gt_pose_data(0,i) << "," << gt_pose_data(1,i) << "," << gt_pose_data(2,i) << "\n";
        //}
        //output_file.close();
        RCLCPP_INFO_STREAM(this->get_logger(), "Ground thruth pose data ready.");
        check_data_ready();
        return;
    }
    delta_T -= 1/params_.sample_frequency;
    sample_count++;
    gt_pose_data(0,sample_count) = interpolate(prev_data(0),
        data.x, prev_time, current_time, current_time-delta_T);
    gt_pose_data(1,sample_count) = interpolate(prev_data(1),
        data.y, prev_time, current_time, current_time-delta_T);
    gt_pose_data(2,sample_count) = interpolate(prev_data(2),
        data.z, prev_time, current_time, current_time-delta_T);
    return;
}
void Statistician::baro_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg){
    static double prev_time = 0.0;
    static double delta_T = 0.0;
    static uint64_t sample_count = 0;
    static double prev_data = msg->point.z;
    if(baro_data_ready){
        return;
    }
    double current_time = msg->header.stamp.sec + 1e-9*msg->header.stamp.nanosec;
    auto data = msg->point.z;
    delta_T += current_time - prev_time;
    if(delta_T < 1/params_.sample_frequency){
        prev_time = current_time;
        prev_data = data;
        return;
    }
    if(sample_count >= params_.num_samples){
        // Save the data to a file
        baro_data_ready = true;
        //std::ofstream output_file;
        //output_file.open("baro_data.csv");
        //output_file << "z\n";
        //for(uint64_t i = 0; i < num_samples; i++){
        //    output_file << baro_data(i) << "\n";
        //}
        //output_file.close();
        RCLCPP_INFO_STREAM(this->get_logger(), "Barometer data ready.");
        check_data_ready();
        return;
    }
    delta_T -= 1/params_.sample_frequency;
    sample_count++;
    baro_data(sample_count) = interpolate(prev_data,
        data, prev_time, current_time, current_time-delta_T);
    return;
}
void Statistician::magnetic_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg){
    static double prev_time = 0.0;
    static double delta_T = 0.0;
    static uint64_t sample_count = 0;
    static Eigen::Matrix<double, 2, 1> prev_data{msg->vector.x,
                                                 msg->vector.y};
    if(magnetic_data_ready){
        return;
    }
    double current_time = msg->header.stamp.sec + 1e-9*msg->header.stamp.nanosec;
    auto data = msg->vector;
    delta_T += current_time - prev_time;
    if(delta_T < 1/params_.sample_frequency){
        prev_time = current_time;
        prev_data(0) = data.x;
        prev_data(1) = data.y;
        return;
    }
    if(sample_count >= params_.num_samples){
        // Save the data to a file
        magnetic_data_ready = true;
        //std::ofstream output_file;
        //output_file.open("magnetic_data.csv");
        //output_file << "x,y\n";
        //for(uint64_t i = 0; i < num_samples; i++){
        //    output_file << magnetic_data(0,i) << "," << magnetic_data(1,i) << "\n";
        //}
        //output_file.close();
        RCLCPP_INFO_STREAM(this->get_logger(), "Magnetometer data ready.");
        check_data_ready();
        return;
    }
    delta_T -= 1/params_.sample_frequency;
    sample_count++;
    magnetic_data(0,sample_count) = interpolate(prev_data(0),
        data.x, prev_time, current_time, current_time-delta_T);
    magnetic_data(1,sample_count) = interpolate(prev_data(1),
        data.y, prev_time, current_time, current_time-delta_T);
    return;
}
void Statistician::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
    static double prev_time = 0.0;
    static double delta_T = 0.0;
    static uint64_t sample_count = 0;
    static Eigen::Matrix<double, 3, 1> prev_data{msg->linear_acceleration.x,
                                                 msg->linear_acceleration.y,
                                                 msg->linear_acceleration.z};
    if(imu_data_ready){
        return;
    }
    double current_time = msg->header.stamp.sec + 1e-9*msg->header.stamp.nanosec;
    auto data = msg->linear_acceleration;
    delta_T += current_time - prev_time;
    if(delta_T < 1/params_.sample_frequency){
        prev_time = current_time;
        prev_data(0) = data.x;
        prev_data(1) = data.y;
        prev_data(2) = data.z;
        return;
    }
    if(sample_count >= params_.num_samples){
        // Save the data to a file
        imu_data_ready = true;
        //std::ofstream output_file;
        //output_file.open("imu_data.csv");
        //output_file << "x,y,z\n";
        //for(uint64_t i = 0; i < num_samples; i++){
        //    output_file << imu_data(0,i) << "," << imu_data(1,i) << "," << imu_data(2,i) << "\n";
        //}
        //output_file.close();
        RCLCPP_INFO_STREAM(this->get_logger(), "IMU data ready.");
        check_data_ready();
        return;
    }
    delta_T -= 1/params_.sample_frequency;
    sample_count++;
    imu_data(0,sample_count) = interpolate(prev_data(0),
        data.x, prev_time, current_time, current_time-delta_T);
    imu_data(1,sample_count) = interpolate(prev_data(1),
        data.y, prev_time, current_time, current_time-delta_T);
    imu_data(2,sample_count) = interpolate(prev_data(2),
        data.z, prev_time, current_time, current_time-delta_T);
    return;
}
void Statistician::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg){
    static double prev_time = 0.0;
    static double delta_T = 0.0;
    static uint64_t sample_count = 0;
    static const Eigen::Matrix3d R_mn {
        {0, 1, 0,},
        {1, 0, 0},
        {0, 0, -1}
    };
    if(gps_data_ready){
        return;
    }
    const double DEG2RAD = M_PI / 180;
    double lat = -msg->latitude * DEG2RAD;  // !!! Gazebo spherical coordinates have a bug. Need to negate.
    double lon = -msg->longitude * DEG2RAD; // !!! Gazebo spherical coordinates have a bug. Need to negate.
    double alt = msg->altitude;

    double sin_lat = sin(lat);
    double cos_lat = cos(lat);
    double sin_lon = sin(lon);
    double cos_lon = cos(lon);

    if (initialized_ECEF == false)
    {
        initial_ECEF = getECEF(sin_lat, cos_lat, sin_lon, cos_lon, alt);
        initialized_ECEF = true;
        return;
    }

    Eigen::Vector3d ECEF = getECEF(sin_lat, cos_lat, sin_lon, cos_lon, alt);
    // Rotation matrix from ECEF to NED
    const Eigen::Matrix3d R_ne{
        {-sin_lat*cos_lon, -sin_lat*sin_lon, cos_lat},
        {-sin_lon, cos_lon, 0},
        {-cos_lat*cos_lon, -cos_lat*sin_lon, -sin_lat}
    };
    const Eigen::Vector3d ned = R_ne*(ECEF - initial_ECEF);

    // Save data
    static Eigen::Matrix<double, 3, 1> prev_data = ned;
    double current_time = msg->header.stamp.sec + 1e-9*msg->header.stamp.nanosec;
    delta_T += current_time - prev_time;
    if(delta_T < 1/params_.sample_frequency){
        prev_time = current_time;
        prev_data = ned;
        return;
    }
    if(sample_count >= params_.num_samples){
        // Save the data to a file
        gps_data_ready = true;
        //std::ofstream output_file;
        //output_file.open("gps_data.csv");
        //output_file << "latitude,longitude,altitude\n";
        //for(uint64_t i = 0; i < num_samples; i++){
        //    output_file << gps_data(0,i) << "," << gps_data(1,i) << "," << gps_data(2,i) << "\n";
        //}
        //output_file.close();
        RCLCPP_INFO_STREAM(this->get_logger(), "GPS data ready.");
        check_data_ready();
        return;
    }
    delta_T -= 1/params_.sample_frequency;
    sample_count++;
    gps_data(0,sample_count) = interpolate(prev_data(0),
        ned(0), prev_time, current_time, current_time-delta_T);
    gps_data(1,sample_count) = interpolate(prev_data(1),
        ned(1), prev_time, current_time, current_time-delta_T);
    gps_data(2,sample_count) = interpolate(prev_data(2),
        ned(2), prev_time, current_time, current_time-delta_T);
    return;
}
void Statistician::sonar_callback(const sensor_msgs::msg::Range::SharedPtr msg){
    static double prev_time = 0.0;
    static double delta_T = 0.0;
    static uint64_t sample_count = 0;
    static double prev_data = msg->range;
    if(sonar_data_ready){
        return;
    }
    double current_time = msg->header.stamp.sec + 1e-9*msg->header.stamp.nanosec;
    auto data = msg->range;
    delta_T += current_time - prev_time;
    if(delta_T < 1/params_.sample_frequency){
        prev_time = current_time;
        prev_data = data;
        return;
    }
    if(sample_count >= params_.num_samples){
        // Save the data to a file
        sonar_data_ready = true;
        //std::ofstream output_file;
        //output_file.open("sonar_data.csv");
        //output_file << "range\n";
        //for(uint64_t i = 0; i < num_samples; i++){
        //    output_file << sonar_data(i) << "\n";
        //}
        //output_file.close();
        RCLCPP_INFO_STREAM(this->get_logger(), "Sonar data ready.");
        check_data_ready();
        return;
    }
    delta_T -= 1/params_.sample_frequency;
    sample_count++;
    sonar_data(sample_count) = interpolate(prev_data,
        data, prev_time, current_time, current_time-delta_T);
    return;
}

int main(int argc, char ** argv)
{
    (void) argc;
    (void) argv;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Statistician>("statistician");
    rclcpp::spin(node);
    return 0;
}
