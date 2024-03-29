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
    //为了从上一帧坐标系下的速度推算出当前帧坐标系下的速度，q暂时也不能置单位四元数
    //q.setIdentity();
    p.setZero();
    //速度属于帧间连续性质变量，在Reset时不清零，如果当前存在速度反馈则可以赋值，否则维持当前状态
    //v.setZero();
}

mini_LIO::mini_LIO(ros::NodeHandle &nh)
{
    frame_count_ = 0;
    bg_.setZero();
    ba_.setZero();
    //ba_ << -0.025, -0.11, -0.1; //pandar颠簸数据默认bias
    std::cout<<"bias is "<<bg_<<" "<<ba_<<std::endl;

    last_w.setZero();

    nh.param<float>("leafsize_", leafsize_, 0.3);
    nh.param<int>("frame_num_", frame_num_, 5);
    nh.param<int>("interval_", interval_, 0);

    sub_imu_ = nh.subscribe("/driver/imu", 1000, &mini_LIO::imu_callback, this);
    sub_pl2_ = nh.subscribe("/driver/livox/point_cloud", 1000, &mini_LIO::pl2_callback, this);
    sub_vel_ = nh.subscribe("/e100/speed_feedback", 1000, &mini_LIO::vel_callback, this);
    //sub_vel_ = nh.subscribe("/Inertial/gps/vel", 1000, &mini_LIO::vel_callback_2, this);

    pub_merged_pointcloud_ = nh.advertise<sensor_msgs::PointCloud2>("merged_point_cloud", 1);

    q_all.setIdentity(); //将全局姿态置为单位四元数
    q_last.setIdentity(); //上一帧姿态置单位四元数

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
        if(cur_vel_ != 0){
        //使用简化的汽车模型，认为每一帧开始积分时速度都是沿着x轴方向的
            cur_delta_.v(0) = cur_vel_; //每过一个点云帧就重新开始积分计算以减少加速度计漂移造成的影响，特别是通过当前的速度反馈来矫正速度，如果没有速度反馈就在当前速度基础上开始积分
            cur_delta_.v(1) = 0;
            cur_delta_.v(2) = 0;
        //不使用简化汽车模型，把速度变换投影到新的坐标系下
            //Eigen::Matrix3d vel_mat = cur_delta_.q.matrix().transpose();
            //cur_delta_.v = vel_mat*cur_delta_.v;
        //别忘了把q置单位四元数   
            cur_delta_.q.setIdentity();
        }
        else{
        //如果没有速度反馈，在简化的汽车模型下就把上一帧最后的速度矢量求范数叠加到x轴上
            //double vel = sqrt(cur_delta_.v(0)*cur_delta_.v(0) + cur_delta_.v(1)*cur_delta_.v(1) + cur_delta_.v(2)*cur_delta_.v(2));
            //cur_delta_.v(0) = vel;
            //cur_delta_.v(1) = 0;
            //cur_delta_.v(2) = 0;
        //如果不使用简化汽车模型，就需要考虑沿y轴的速度，将当前积分得到的，在前一帧点云坐标系下的速度投影到新坐标系下
            Eigen::Matrix3d vel_mat = cur_delta_.q.matrix().transpose();
            cur_delta_.v = vel_mat*cur_delta_.v;
        //别忘了把q置单位四元数
            cur_delta_.q.setIdentity();
        }
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

    if(last_w(0) == 0)
        last_w = imu_data.w;
    else{
        imu_data.w = (imu_data.w + last_w)/2;
        last_w << imu_msg.angular_velocity.x, 
                  imu_msg.angular_velocity.y, 
                  imu_msg.angular_velocity.z;
    }
    
    //直接采用惯导输出的全局姿态
    Eigen::Quaterniond q(imu_msg.orientation.w, 
                         imu_msg.orientation.x,
                         imu_msg.orientation.y,
                         imu_msg.orientation.z);

    //q_all = q;   
    
    integrate_2d(imu_data, bg_, ba_);
    //integrate_3d(imu_data, bg_, ba_);
}

void mini_LIO::pl2_callback(const sensor_msgs::PointCloud2ConstPtr &input)
{
    std::cout<<"frame_count_ is: "<<frame_count_<<std::endl;
    std::cout<<"interval_ is: "<<interval_<<std::endl;

    if((frame_count_ != 0) && (frame_count_ != (interval_+1)))
    {
        frame_count_++;
        return;
    }

    if(frame_count_ == (interval_+1))
    {
        frame_count_ = 0;
    }

    frame new_frame;
    static CLOUD_PTR temp_cloud(new CLOUD()); 
    static CLOUD cur_cloud;
    pcl::fromROSMsg(*input, *temp_cloud);

    sor_.setInputCloud(temp_cloud);
    sor_.setLeafSize(leafsize_, leafsize_, leafsize_);
    sor_.filter(cur_cloud);

    new_frame.t = cur_delta_.t;
    new_frame.cloud_ptr_.reset(new CLOUD(cur_cloud));

    new_frame.state.block<3,3>(0,0) = cur_delta_.q.matrix();
    new_frame.state.block<3,1>(0,3) = cur_delta_.p;
    new_frame.state.row(3) << 0, 0, 0, 1; //将DELTA中的四元数和平移向量转化为齐次变换矩阵

    cur_delta_.Reset();
    have_init_delta_ = false;

    if(frames_.size() == (frame_num_+1))
        frames_.pop_front();        

    for (auto i = frames_.begin(); i < frames_.end(); i++)
        (*i).state = (*i).state * new_frame.state; //将当前列表内所有变换矩阵变为关于最新帧的变换

    frames_.push_back(new_frame);

    merge();

    sensor_msgs::PointCloud2 pl2_msg;
    pcl::toROSMsg(*final_cloud_, pl2_msg);
    pl2_msg.header.frame_id = "mini_LIO";
    pub_merged_pointcloud_.publish(pl2_msg);

    frame_count_++;
}


void mini_LIO::vel_callback(const cyber_msgs::VehicleSpeedFeedbackConstPtr &vel_in)
{
    double vel = (double)(vel_in->speed_cmps) / 100.0;
    cur_vel_ = vel;

    std::cout<<"Now speed is: "<<cur_vel_<<std::endl;
}


void mini_LIO::vel_callback_2(const geometry_msgs::TwistWithCovarianceStampedConstPtr &vel_in)
{
    geometry_msgs::TwistWithCovarianceStamped vel = *vel_in;
    double vel_x = vel.twist.twist.linear.x;
    double vel_y = vel.twist.twist.linear.y;
    double vel_z = vel.twist.twist.linear.z;
    cur_vel_ = sqrt(vel_x*vel_x + vel_y*vel_y + vel_z*vel_z);

    std::cout<<"Now speed is: "<<vel_x<<" "<<vel_y<<" "<<vel_z<<" "<<cur_vel_<<std::endl;
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
    //因为e300的IMU是朝下安装的
    w = -w;

    a(1) = -a(1); //因为IMU系是左手系

    std::cout<<"Angular vel and Linear acc are "<<w<<" "<<a<<std::endl;

    double dt = data.t - cur_delta_.t;
    //根据预积分公式对位姿进行积分计算
    cur_delta_.t = data.t;
    cur_delta_.p = cur_delta_.p + dt * cur_delta_.v + 0.5 * dt * dt * (cur_delta_.q * a); //计算当前在帧起始坐标系下的
    cur_delta_.v = cur_delta_.v + dt * (cur_delta_.q * a); //计算当前在帧起始坐标系下的速度投影
    cur_delta_.q = (cur_delta_.q * expmap(w * dt)).normalized(); //计算旋转矩阵
}

void mini_LIO::integrate_3d(const IMUData &data, const Eigen::Vector3d &bg, const Eigen::Vector3d &ba)
{
    Eigen::Vector3d w = data.w - bg;
    Eigen::Vector3d a = data.a - ba; //减去零漂

    w(1) = -w(1);
    a(1) = -a(1); //因为IMU系是左手系

    //根据全局姿态投影出重力加速素的分量并在测量加速度中减去
    Eigen::Matrix3d r_all = q_all.matrix().transpose();  //从四元数得到当前的全局旋转矩阵，取完转置可以直接通过与原始坐标系的重力加速度向量相乘来得到重力加速度在当前坐标轴的投影
    Eigen::Vector3d g;
    g << 0, 0, ACC_G;
    Eigen::Vector3d g_now = r_all*g;
    a = a - g_now;

    std::cout<<"Now Acc is: "<<a<<std::endl;

    //在这里按照AVIA雷达的坐标系来考虑
    double dt = data.t - cur_delta_.t;
    cur_delta_.t = data.t;
    cur_delta_.p = cur_delta_.p + dt * cur_delta_.v + 0.5 * dt * dt * (cur_delta_.q * a); //计算当前在帧起始坐标系下的
    cur_delta_.v = cur_delta_.v + dt * (cur_delta_.q * a); //计算当前在帧起始坐标系下的速度投影
    cur_delta_.q = (cur_delta_.q * expmap(w * dt)).normalized(); //计算旋转矩阵

    std::cout<<"Now Speed is: "<<cur_delta_.v<<std::endl;
    std::cout<<"Now Translation is: "<<cur_delta_.p<<std::endl;

    //不断积分记录当前的姿态
    q_all = (q_all*expmap(w * dt)).normalized();
}

void mini_LIO::merge()
{
    //CLOUD temp_cloud;
    CLOUD temp_final_cloud;
    
    for(auto i = frames_.begin(); i < (frames_.end()-1); i++)
    {
        CLOUD temp_cloud;
        Eigen::Matrix4d inv;
        inv = inv_homo((*(i+1)).state);
        /*
        //对应pandar雷达的安装方式，坐标系要作出调整
        double x_fix = inv(1,3);
        double y_fix = -inv(0,3);
        inv(0,3) = x_fix;
        inv(1,3) = y_fix;
        */
        std::cout<<"TF Matrix is: "<<inv<<std::endl;
        pcl::transformPointCloud(*((*i).cloud_ptr_), temp_cloud, inv);
        //pltransform(*((*i).cloud_ptr_), temp_cloud, inv);
        //temp_cloud = *((*i).cloud_ptr_);
        temp_final_cloud += temp_cloud;
    }

    final_cloud_.reset(new CLOUD(temp_final_cloud));
    std::cout<<"PL size: "<<final_cloud_->points.size()<<std::endl;
}
