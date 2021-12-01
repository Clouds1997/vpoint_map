#pragma once
#include <ros/ros.h>
#include <fstream>
#include <iomanip>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <Eigen/Core>
#include "../../ThirdParty/GeographicLib/include/LocalCartesian.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include "base_type.h"


class gpstest
{
public:
    gpstest(ros::NodeHandle& nh);
    ~gpstest();
    void GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr);
    void LogGps(const GpsLocalization::GpsPositionDataPtr gps_data);
    void LogXyz(const GpsLocalization::xyzPositionDataPtr xyz_data);
    void gps2xyz(const GpsLocalization::GpsPositionDataPtr gps_data);
    void ConvertStateToRosTopic(const GpsLocalization::xyzPositionDataPtr xyz_data);
    void createmap();
    void updatemap(const GpsLocalization::xyzPositionDataPtr xyz_data);

    float resolution;
    int width,height,originx,originy;
    int count=0;

    std::ofstream file_gps_;
    std::ofstream file_xyz_;
    std::ofstream file_frist_gps_;
    ros::Subscriber gps_position_sub_;
    ros::Publisher state_pub_;
    ros::Publisher map_pub_;
    ros::Publisher grid_pub_;

    bool initGPS;
    nav_msgs::Path ros_path_;
    nav_msgs::OccupancyGrid map;
    GeographicLib::LocalCartesian geoConverter;
};

