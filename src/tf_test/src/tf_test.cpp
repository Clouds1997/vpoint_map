#include <ros/ros.h>
#include "../include/tf_test/odom.h"

int main (int argc, char** argv) {

    // Initialize ros.
    //
    ros::init(argc, argv, "map");
    ros::NodeHandle nh;
    
    // Initialize localizer.
    mapload test(nh);

    ros::spin();
    return 0;
}