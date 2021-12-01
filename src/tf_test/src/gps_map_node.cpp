#include <memory>
#include <ros/ros.h>
#include "../include/tf_test/gps_wrapper.h"

int main (int argc, char** argv) {

    // Initialize ros.
    //
    ros::init(argc, argv, "gps");
    ros::NodeHandle nh;
    
    // Initialize localizer.
    gpstest test(nh);

    ros::spin();
    return 0;
}