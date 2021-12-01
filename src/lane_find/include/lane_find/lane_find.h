#pragma once
#include <ros/ros.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>  
#include <vector>

using namespace std;

class lane_find
{
public:
    lane_find(ros::NodeHandle& nh);
    ~lane_find();
    void img0_callback(const sensor_msgs::ImageConstPtr &img_msg);
    // int vector_ave(vector<int> vec);
    cv::Vec3b vector_ave(vector<cv::Vec3b> vec);
    int max(int a,int b,int c);
    int min(int a,int b,int c);
    // void find_range();

    ros::Subscriber sub_img0;
    int h_p,s_p,v_p;
};