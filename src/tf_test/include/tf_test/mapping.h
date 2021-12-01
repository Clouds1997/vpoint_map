#pragma once
#include <ros/ros.h>
#include <fstream>
#include <iomanip>
#include<iostream>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_broadcaster.h>
#include "../../ThirdParty/GeographicLib/include/LocalCartesian.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_field_conversion.h>


class mapping
{
   public:
    mapping(ros::NodeHandle& nh);
    ~mapping();
    void GpsPositionCallback(const sensor_msgs::NavSatFix& gps_msg) ;
    void odom_sub(const nav_msgs::Odometry& odom_path);
    void point_cloud_sub(const sensor_msgs::PointCloud &pc1);
    void createmap();
    void updatemap();
    
    double X,Y,Z;

    int count =1;

    ros::Time current_time, last_time;
    ros::Subscriber gps_position_sub_;
    ros::Subscriber path_position_sub_;
    ros::Subscriber point_cloud_sub_;
    ros::Publisher map_pub_;
    ros::Publisher odom_publish;
    sensor_msgs::PointCloud point_cloud1;

    std::ofstream file_frist_gps_;
    nav_msgs::OccupancyGrid map;

    float resolution;
    int width,height,originx,originy;
    bool initGPS=false;
    bool initxyz = false;
    GeographicLib::LocalCartesian geoConverter;

    tf::TransformBroadcaster map_odom_baselink_broadcaster;

    // Eigen::Vector4d V4d_1;
    // Eigen::Vector4d V4d_2;
    // double map_x,map_y;
    // double res_inv = 10;
    // int origin=map.info.width*(map.info.height/2)+map.info.width/2;

    // Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

};
