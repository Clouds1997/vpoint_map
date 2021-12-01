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
#include "../ThirdParty/GeographicLib/include/LocalCartesian.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_field_conversion.h>

static int count =1;
static sensor_msgs::PointCloud point_cloud1;
static nav_msgs::OccupancyGrid map;
static ros::Publisher map_pub_;

void createmap()
{
    float resolution = 0.1;
    int width = 2000;
    int height =2000;

    map.header.frame_id="/map_odom";
    map.header.stamp = ros::Time::now();

    map.info.resolution=resolution;
    map.info.width= width;
    map.info.height = height;
    map.info.origin.position.x=-resolution*width/2;
    map.info.origin.position.y=-resolution*height/2;

    //int p[map.info.width*map.info.height] = {-1};   // [0,100]
    std::vector<signed char> p(map.info.width*map.info.height,-1);
    map.data = p;
}

void point_cloud_sub(const sensor_msgs::PointCloud &pc1) 
{ 
    point_cloud1 = pc1;
}

void updatemap(const nav_msgs::Odometry& pose){
    Eigen::Quaterniond Q(pose.pose.pose.orientation.w,pose.pose.pose.orientation.x,pose.pose.pose.orientation.y,pose.pose.pose.orientation.z);
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    // cout <<T<<endl;
    T.rotate(Q);
    // T.pretranslate(Eigen::Vector3d(pose.pose.pose.position.x,pose.pose.pose.position.y,0));
    T.pretranslate(Eigen::Vector3d(pose.pose.pose.position.x,pose.pose.pose.position.y,0));
    // // std::cout<<T<<std::endl;
    // // Sophus::SE3 T_pose(Q,);
    Eigen::Vector4d V4d_1;
    Eigen::Vector4d V4d_2;
    double map_x,map_y;
    double res_inv = 10;
    int origin=map.info.width*(map.info.height/2)+map.info.width/2;
    if(count % 10 ==0){
        for (size_t cp = 460800; cp < point_cloud1.points.size (); ++cp){
            if(point_cloud1.points[cp].x > 0){ 
                // cout<<point_cloud1.points[cp].x<<" "<<point_cloud1.points[cp].y<<endl;
                V4d_1=Eigen::Vector4d(point_cloud1.points[cp].x,point_cloud1.points[cp].y,0,1);
                V4d_2 = T* V4d_1;
                map_x=V4d_2[0]*res_inv;
                map_y=V4d_2[1]*res_inv;
                int xyzposition=(int)map_x+(int)map_y*map.info.width;
                int id;
                id=origin+xyzposition;
                map.data[id]=0;
                // flag++;
            }
       }
        std::cout << "point_time::"<<point_cloud1.header.stamp<<std::endl;
         std::cout <<"pose_time::"<< pose.header.stamp<<std::endl;
        std::cout<<"________________________________"<<std::endl;
    }
    count ++;
    std::cout<<count<<std::endl;
    map_pub_.publish(map);
}

int main (int argc, char** argv) {

    // Initialize ros.
    //
    ros::init(argc, argv, "point_update_mapping");
    ros::NodeHandle nh;
    // 
    // point_cloud_sub_ = nh.subscribe("/point_cloud1", 10,  pointCloud2ToZ);
    createmap();

    map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/gridMap", 1);
    ros::Subscriber point_cloud_sub_ = nh.subscribe("/point_cloud1",100,point_cloud_sub);
    ros::Subscriber path_position_sub_ = nh.subscribe("/map_odom", 100,  updatemap);

    ros::spin();
    return 0;
}