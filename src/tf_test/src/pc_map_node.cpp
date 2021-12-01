#include "../include/tf_test/mapping.h"

using namespace std;


int main (int argc, char** argv) {

    // Initialize ros.
    //
    ros::init(argc, argv, "point_cloud_mapping");
    ros::NodeHandle nh;
    mapping test(nh);
    // 
    // point_cloud_sub_ = nh.subscribe("/point_cloud1", 10,  pointCloud2ToZ);

    ros::spin();
    return 0;
}