#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <iostream>
#include <ros/ros.h>
#include "../include/point_cloud/point_cloud_conversion.h"

using namespace std;
 
ros::Publisher point_cloud_pub;

void pointCloud2ToZ(const sensor_msgs::PointCloud2 &msg)
{
	int rows=720;
	int cols=1280;
	sensor_msgs::PointCloud out_pointcloud;
	sensor_msgs::PointCloud2 new_pointcloud;
	point_cloud_conversion::convertPointCloud2ToPointCloud(msg, out_pointcloud);
	out_pointcloud.header.frame_id = "map_base_link";
	out_pointcloud.header.stamp =  ros::Time::now();
	// for (int i=0; i<out_pointcloud.points.size(); i++) {
	// 	cout << out_pointcloud.points[i].x << ", " << out_pointcloud.points[i].y << ", " << out_pointcloud.points[i].z << endl;
	// }
	// cout << "------" << endl;
	// cout << out_pointcloud.points[920555].x << ", " << out_pointcloud.points[920555].y << ", " << out_pointcloud.points[920555].z << endl;
	point_cloud_pub.publish(out_pointcloud);
}

int main (int argc, char** argv) {

    // Initialize ros.
    //
    ros::init(argc, argv, "point_cloud");
    ros::NodeHandle nh;
    
    // Initialize localizer.
	 ros::Subscriber  point_cloud2_sub_;
	point_cloud_pub=nh.advertise<sensor_msgs::PointCloud>("/point_cloud1",10);
    point_cloud2_sub_ = nh.subscribe("/zed2/zed_node/point_cloud/cloud_registered", 10,  pointCloud2ToZ);


    ros::spin();
    return 0;
}