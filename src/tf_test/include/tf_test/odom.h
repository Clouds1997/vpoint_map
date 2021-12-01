#pragma once
#include <ros/ros.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sensor_msgs/NavSatFix.h>
#include <cstdlib>
#include <nav_msgs/Path.h>
#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "../../ThirdParty/GeographicLib/include/LocalCartesian.hpp"

using namespace std;

class mapload
{
public:
    mapload(ros::NodeHandle& nh);
    ~mapload();
    void initxyz();
    void local(const sensor_msgs::NavSatFix& gps_msg);
    void odom_pub(const nav_msgs::Odometry& pose);

    bool init=false;
    double X,Y,Z;

    std::ifstream file_frist_gps_;
    ros::Time current_time, last_time;

    nav_msgs::Path ros_path_;
    nav_msgs::OccupancyGrid map;
    GeographicLib::LocalCartesian geoConverter;


    ros::Subscriber map_sub_;
    ros::Subscriber gps_position_sub_ ;
    ros::Subscriber path_position_sub_ ;

    ros::Publisher map_pub_;
    ros::Publisher state_pub_;
    ros::Publisher odom_publish;

    tf::TransformBroadcaster odom_world_broadcaster;
    tf::TransformBroadcaster odom_baselink_broadcaster;

};