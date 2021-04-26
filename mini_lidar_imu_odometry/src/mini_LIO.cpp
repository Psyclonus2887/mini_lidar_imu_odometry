/**
  *mini_LIO.cpp
  *brief:to merge several frames of point cloud
  *author:Yang Chenglin
  *date:2021
  **/
//#include <mini_lidar_imu_odometry/mini_LIO.h> 
#include "mini_LIO.h" 

void DELTA::Reset()
{
    t = 0;
    q.setIdentity();
    p.setZero();
    v.setZero();
}

mini_LIO::mini_LIO(ros::NodeHandle &nh)
{
    bg_.setZero();
    ba_ << -0.05, -0.15, 0; //默认bias
    std::cout<<"bias is "<<bg_<<" "<<ba_<<std::endl;

    nh.param<double>("cali_x_", cali_x_, 0.0);
    nh.param<double>("cali_y_", cali_y_, 0.0);
    nh.param<double>("cali_z_", cali_z_, 0.0);

    sub_imu_ = nh.subscribe("/Inertial/imu/data", 1000, &mini_LIO::imu_callback, this);
    sub_pl2_ = nh.subscribe("/driver/pandar/point_cloud", 1000, &mini_LIO::pl2_callback, this);
    //sub_vel_ = nh.subscribe("/e100/speed_feedback", 1000, &mini_LIO::vel_callback, this);
    sub_vel_ = nh.subscribe("/Inertial/gps/vel", 1000, &mini_LIO::vel_callback_2, this);

    pub_merged_pointcloud_ = nh.advertise<sensor_msgs::PointCloud2>("merged_point_cloud", 1);

    std::cout<<"Inited!"<<std::endl;
}

mini_LIO::~mini_LIO()
{

}

void mini_LIO::imu_callback(const sensor_msgs::ImuConstPtr &input)
{
    sensor_msgs::Imu imu_msg = *input;
    IMUData imu_data;
    double time_stamp = imu_msg.header.stamp.toSec();

    if(!have_init_delta_){
        cur_delta_.t = time_stamp;
        cur_delta_.v(0) = cur_vel_; //每过一个点云帧就重新开始积分计算以减少加速度计漂移造成的影响
        have_init_delta_ = true;
        return;
    }

    imu_data.t = time_stamp;

    imu_data.a << imu_msg.linear_acceleration.x, 
                  imu_msg.linear_acceleration.y, 
                  imu_msg.linear_acceleration.z; 

    imu_data.w << imu_msg.angular_velocity.x, 
                 imu_msg.angular_velocity.y, 
                 imu_msg.angular_velocity.z;
    
    integrate_2d(imu_data, bg_, ba_);
}

void mini_LIO::pl2_callback(const sensor_msgs::PointCloud2ConstPtr &input)
{
    frame new_frame;
    static CLOUD cur_cloud;
    pcl::fromROSMsg(*input, cur_cloud);

    new_frame.t = cur_delta_.t;
    //new_frame.cloud_ptr_ = cur_cloud.makeShared();
    new_frame.cloud_ptr_.reset(new CLOUD(cur_cloud));

    new_frame.state.block<3,3>(0,0) = cur_delta_.q.matrix();
    new_frame.state.block<3,1>(0,3) = cur_delta_.p;
    new_frame.state.row(3) << 0, 0, 0, 1; //将DELTA中的四元数和平移向量转化为齐次变换矩阵

    cur_delta_.Reset();
    have_init_delta_ = false;

    if(frames_.size() == frame_num_)
        frames_.pop_front();        

    for (auto i = frames_.begin(); i < frames_.end(); i++)
        (*i).state = (*i).state * new_frame.state; //将当前列表内所有变换矩阵变为关于最新帧的变换

    frames_.push_back(new_frame);

    merge();

    sensor_msgs::PointCloud2 pl2_msg;
    pcl::toROSMsg(*final_cloud_, pl2_msg);
    pl2_msg.header.frame_id = "mini_LIO";
    pub_merged_pointcloud_.publish(pl2_msg);
}

/*
void mini_LIO::vel_callback(const cyber_msgs::VehicleSpeedFeedbackConstPtr &vel_in)
{

}
*/

void mini_LIO::vel_callback_2(const geometry_msgs::TwistWithCovarianceStampedConstPtr &vel_in)
{
    geometry_msgs::TwistWithCovarianceStamped vel = *vel_in;
    double vel_x = vel.twist.twist.linear.x;
    double vel_y = vel.twist.twist.linear.y;
    cur_vel_ = sqrt(vel_x*vel_x + vel_y*vel_y);

    std::cout<<"Now speed is: "<<vel_x<<" "<<vel_y<<" "<<cur_vel_<<std::endl;
}

Eigen::Quaterniond mini_LIO::expmap(const Eigen::Vector3d &w)
{
    Eigen::AngleAxisd aa(w.norm(), w.normalized());
    Eigen::Quaterniond q;
    q = aa;
    return q;
}

Eigen::Matrix4d mini_LIO::inv_homo(Eigen::Matrix4d &in) //给齐次变换矩阵求逆
{
    Eigen::Matrix4d inv;
    Eigen::Matrix3d temp = in.topLeftCorner(3,3).transpose();
    inv.block<3,1>(0,3) = -temp*in.block<3,1>(0,3);
    inv.topLeftCorner(3,3) = temp;
    inv.row(3) << 0,0,0,1;
    return inv;
}

void mini_LIO::integrate_2d(const IMUData &data, const Eigen::Vector3d &bg, const Eigen::Vector3d &ba)
{
    Eigen::Vector3d w = data.w - bg;
    Eigen::Vector3d a = data.a - ba;

    //考虑到车载情况，对Z轴加速度和XY轴角速度进行忽略，这样也免去了重力补偿的必要
    w.head(2) << 0,0;
    a.tail(1) << 0;

    a(2) = -a(2);

    std::cout<<"Angular vel and Linear acc are "<<w<<" "<<a<<std::endl;

    double dt = data.t - cur_delta_.t;
    //根据预积分公式对位姿进行积分计算
    cur_delta_.t = data.t;
    cur_delta_.p = cur_delta_.p + dt * cur_delta_.v + 0.5 * dt * dt * (cur_delta_.q * a); //计算当前在帧起始坐标系下的
    cur_delta_.v = cur_delta_.v + dt * (cur_delta_.q * a); //计算当前在帧起始坐标系下的速度投影
    cur_delta_.q = (cur_delta_.q * expmap(w * dt)).normalized(); //计算旋转矩阵
}

void mini_LIO::merge()
{
    //CLOUD temp_cloud;
    CLOUD temp_final_cloud;
    
    for(auto i = frames_.begin(); i < frames_.end(); i++)
    {
        CLOUD temp_cloud;
        Eigen::Matrix4d inv;
        inv = inv_homo((*i).state);

        //对应pandar雷达的安装方式，坐标系要作出调整
        double x_fix = inv(1,3);
        double y_fix = -inv(0,3);
        inv(0,3) = x_fix;
        inv(1,3) = y_fix;

        std::cout<<"TF Matrix is: "<<inv<<std::endl;
        pcl::transformPointCloud(*((*i).cloud_ptr_), temp_cloud, inv);
        //temp_cloud = *((*i).cloud_ptr_);
        temp_final_cloud += temp_cloud;
    }

    final_cloud_.reset(new CLOUD(temp_final_cloud));
}
