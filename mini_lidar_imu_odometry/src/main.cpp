/**
  *main.cpp
  *brief:main of node
  *author:Yang Chenglin
  *date:2021
  **/
//#include <mini_lidar_imu_odometry/mini_LIO.h> 
#include "mini_LIO.h" 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mini_LIO");
    ros::NodeHandle nh;
    mini_LIO ins(nh);
    ros::spin();
    return 0;
}