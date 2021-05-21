/**
  *mini_LIO.cpp
  *brief:to merge several frames of point cloud
  *author:Yang Chenglin
  *date:2021
  **/
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sys/types.h>
#include <vector>
#include <deque>
#include <cmath>
#include <string>
#include <sstream>
#include <termios.h>
#include <ctime>
#include <Eigen/Eigen>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include "cyber_msgs/VehicleSpeedFeedback.h" //注意这个还没有添加
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include "std_msgs/String.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#define ACC_G 9.8

using POINT = pcl::PointXYZ;
using CLOUD = pcl::PointCloud<POINT>;
using CLOUD_PTR = CLOUD::Ptr;

struct DELTA //保存帧间相对位姿结构体
{
    double t;
    Eigen::Quaterniond q;
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    void Reset();
    DELTA(){
        t = 0;
        q.setIdentity();
        p.setZero();
        v.setZero();
    }
};

struct IMUData
{
    double t; 
    Eigen::Vector3d w;
    Eigen::Vector3d a;
};

class frame //将点云和其相对上一帧的位姿存储在一个类内
{
public:
    frame():cloud_ptr_(new CLOUD())
    {}
    //~frame();

    double t;
    Eigen::Matrix4d state;
    CLOUD_PTR cloud_ptr_;
};

class mini_LIO
{
public: 
    mini_LIO(ros::NodeHandle &nh);
    ~mini_LIO();

    int frame_num_; //执行多帧融合的帧数, default=4
    int interval_; //融合的两帧之间相差多少帧，default=0
    int frame_count_; //计算当前的帧数是否达到interval_所规定的限制

private:
    bool have_init_delta_ = false; //判断是否有初始化Δ

    pcl::VoxelGrid<POINT> sor_; //用于点云降采样
    float leafsize_; //体素网格降采样叶子大小

    CLOUD_PTR final_cloud_; //用于发布融合后的最终点云

    Eigen::Vector3d bg_; //角速度bias
    Eigen::Vector3d ba_; //加速度bias

    DELTA cur_delta_;

    Eigen::Quaterniond q_all; //此四元数用来积分记录相对于开始时刻的全局姿态，即一直进行积分计算

    double cur_vel_ = 0; //假设反馈的车辆速度方向与IMU的x轴方向相同

    std::deque<frame> frames_;

    ros::Publisher pub_merged_pointcloud_;

    ros::Subscriber sub_imu_;
    ros::Subscriber sub_pl2_;
    ros::Subscriber sub_vel_;

    void imu_callback(const sensor_msgs::ImuConstPtr &input);
    void pl2_callback(const sensor_msgs::PointCloud2ConstPtr &input);
    void vel_callback(const cyber_msgs::VehicleSpeedFeedbackConstPtr &vel_in);
    void vel_callback_2(const geometry_msgs::TwistWithCovarianceStampedConstPtr &vel_in);

    Eigen::Quaterniond expmap(const Eigen::Vector3d &w);
    Eigen::Matrix4d inv_homo(Eigen::Matrix4d &in);
    void integrate_2d(const IMUData &data, const Eigen::Vector3d &bg, const Eigen::Vector3d &ba);
    void integrate_3d(const IMUData &data, const Eigen::Vector3d &bg, const Eigen::Vector3d &ba);
    void merge(); //将当前类内全部的点云根据相对位姿进行融合叠加,最终结果是基于最后一帧坐标系的
};