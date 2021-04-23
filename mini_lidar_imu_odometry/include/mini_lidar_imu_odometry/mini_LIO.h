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
#include "std_msgs/String.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

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
};

struct IMUData
{
    double t; 
    Eigen::Vector3d w;
    Eigen::Vector3d a;
}

class frame //将点云和其相对上一帧的位姿存储在一个类内
{
public:
    frame():cloud_ptr_(new CLOUD)
    {}
    ~frame();
private:
    double t;
    Eigen::Matrix4d state;
    CLOUD_PTR cloud_ptr_;
};

class mini_LIO
{
public: 
    mini_LIO(ros::NodeHandle &nh);
    ~mini_LIO();

    int frame_num_ = 10; //执行多帧融合的帧数, default=10

private:
    bool have_init_delta_ false; //判断是否有初始化Δ

    double cali_x_;
    double cali_y_;
    double cali_z_; //激光雷达与IMU之间的标定关系

    Eigen::Vector3d bg_; //角速度bias
    Eigen::Vector3d ba_; //加速度bias

    DELTA cur_delta_;

    std::deque<frame> frames_;

    ros::Publisher pub_merged_pointcloud_;

    ros::Subscriber sub_imu_;
    ros::Subscriber sub_pl2_;

    void imu_callback(const sensor_msgs::ImuConstPtr &input);
    void pl2_callback(const sensor_msgs::PointCloud2ConstPtr &input);

    Eigen::Quaterniond expmap(const Eigen::Vector3d &w);
    void integrate(const IMUData &data, const Eigen::Vector3d &bg, const Eigen::Vector3d &ba);
    void merge(); //将当前类内全部的点云根据相对位姿进行融合叠加,最终结果是基于最后一帧坐标系的
};