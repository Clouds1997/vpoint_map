#include <ros/ros.h>
#include <iostream>
#include "../include/lane_find/lane_find.h"

using namespace std;

int main (int argc, char** argv) {

    // Initialize ros.
    //
    ros::init(argc, argv, "map");
    ros::NodeHandle nh;
    // cv::Mat img=cv::imread("school.png");
    
    lane_find test(nh);
    // Initialize localizer.
    // mapload test(nh);

    ros::spin();
    return 0;
}