#include "mini_lidar_imu_odometry/mini_LIO.h" 


void DELTA::Reset()
{
    t = 0;
    q.setIdentity();
    p.setZero();
    v.setZero;
}

mini_LIO::mini_LIO(ros::NodeHandle &nh)
{
    bg_.setZero();
    ba_.setZero(); //默认所有bias都为0

    nh_priv.param<double>("cali_x_", cali_x_, 0.0);
    nh_priv.param<double>("cali_y_", cali_y_, 0.0);
    nh_priv.param<double>("cali_z_", cali_z_, 0.0);

    sub_imu_ = nh.subscribe("/livox/imu", 1000, &mini_LIO::imu_callback, this);
    sub_pl2_ = nh.subscribe("/livox/lidar", 1000, &mini_LIO::pl2_callback, this);

    pub_merged_pointcloud_ = nh.advertise<sensor_msgs::PointCloud2>("pcl_output", 1);

}

void mini_LIO::imu_callback(const sensor_msgs::ImuConstPtr &input)
{
    sensor_msgs::Imu imu_msg = *input;
    ImuData imu_data;
    double time_stamp = imu_msg.header.stamp.toSec();

    if(!have_init_delta_){
        cur_delta_.t = time_stamp;
        have_init_delta_ = true;
        return
    }

    imu_data.t = time_stamp;

    imu_data.a << imu_msg.linear_acceleration.x, 
                  imu_msg.linear_acceleration.y, 
                  imu_msg.linear_acceleration.z; 

    imu_data.w << imu_msg.angular_velocity.x, 
                 imu_msg.angular_velocity.y, 
                 imu_msg.angular_velocity.z;
    
    integrate(imu_data, bg_, ba_);

}

void mini_LIO::pl2_callback(const sensor_msgs::PointCloud2ConstPtr &input)
{

}


Eigen::Quaterniond mini_LIO::expmap(const Eigen::Vector3d &w)
{
    Eigen::AngleAxisd aa(w.norm(), w.stableNormalized());
    Eigen::Quaterniond q;
    q = aa;
    return q;
}

void mini_LIO::integrate(const IMUData &data, const Eigen::Vector3d &bg, const Eigen::Vector3d &ba)
{
    Eigen::Vector3d w = data.w - bg;
    Eigen::Vector3d a = data.a - ba;

    double dt = data.t - cur_delta_.t;
    //根据预积分公式对位姿进行积分计算
    cur_delta_.t = data.t;
    cur_delta_.p = cur_delta_.p + dt * cur_delta_.v + 0.5 * dt * dt * (cur_delta_.q * a);
    cur_delta_.v = cur_delta_.v + dt * (cur_delta_.q * a);
    cur_delta_.q = (cur_delta_.q * expmap(w * dt)).normalized();
}

void mini_LIO::merge()
{

}
