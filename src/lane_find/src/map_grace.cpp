#include <ros/ros.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include "tf2/LinearMath/Matrix3x3.h"
#include <cv_bridge/cv_bridge.h>
#include "nav_msgs/OccupancyGrid.h"
#include <nav_msgs/Odometry.h>

using namespace std;

bool saved_map_ = false;

 void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
    {
      int height = 2000;
      int width = 2000;
      int matrix_size =2000;
      cv::Mat map_cv(height,width,CV_8UC1);
      map_cv.setTo(0);
      for(unsigned int y = 0; y < map->info.height; y++) {
        for(unsigned int x = 0; x < map->info.width; x++) {
          unsigned int i = x + (map->info.height - y - 1) * map->info.width;
          if (map->data[i] == 0 ) { // [0,free)
            map_cv.data[i] =254;
          } 
          else { //occ [0.25,0.65]
              map_cv.data[i] =0;
          }
        }
      }
      cv::Mat img = map_cv.clone();
      cv::Mat img_delate,img_erode,img_blur;
      cv::Mat img_min,img_delate_min,img_erode_min,img_blur_min;
      cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));  
      vector<vector<cv::Point>> contours;

      cv::erode(img,img_erode,element);
      cv::dilate(img_erode,img_delate,element);
      // cv::dilate(img,img_delate,element);
      // cv::erode(img_delate,img_erode,element);
      // // cv::medianBlur(img_erode,img_blur,15);

      cv::Mat img_binary;
      cv::threshold(img_delate, img_binary, 100, 255, cv::THRESH_BINARY);


      // img_erode.convertTo(img_erode,CV_8UC1);
      // cout <<img_erode.type() <<endl;
      // // cv::cvtColor(img_erode,img_erode_8UC1,CV_8UC1);

      cv::findContours(
          img_binary,
          contours,
          cv::noArray(),
          cv::RETR_LIST,
          cv::CHAIN_APPROX_SIMPLE
          );

      img_binary = cv::Scalar::all(0);
      cv::drawContours(img_delate, contours, -1, cv::Scalar::all(125));


      cv::resize(img,img_min,cv::Size(matrix_size/2, matrix_size/2));
      cv::resize(img_delate,img_delate_min,cv::Size(matrix_size/2, matrix_size/2));
      cv::resize(img_erode,img_erode_min,cv::Size(matrix_size/2, matrix_size/2));
      // cv::resize(img_blur,img_blur_min,cv::Size(matrix_size/2, matrix_size/2));
      // cv::resize(image_binary,image_binary,cv::Size(matrix_size/2, matrix_size/2));

      // cv::imshow("img",img_min);
      // cv::imshow("img_erodeo",img_erode_min);
      // cv::imshow("img_delate",img_delate_min);
      // // cv::imshow("img_binary",image_binary);
      // // cv::imshow("img_blur",img_blur_min);
      // cv::waitKey();

        // cout<<"hello2333"<<endl;
      // cv::Mat img=cv::imread("back_img.png");
      // cv::imshow("img",img);
      // cv::waitKey();
      string mapdatafile = "mymap.pgm";
      ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
      FILE* out = fopen(mapdatafile.c_str(), "w");
      if (!out)
      {
        ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
        return;
      }
      fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
              map->info.resolution, map->info.width, map->info.height);
      for(unsigned int y = 0; y < map->info.height; y++) {
        for(unsigned int x = 0; x < map->info.width; x++) {
          unsigned int i = x + (map->info.height - y - 1) * map->info.width;
          if (img_delate.data[i]  ==  254) { // [0,free)
          //路面
            fputc(254, out);
          } else if (img_delate.data[i] == 125) { // (occ,255]
          //占据
            fputc(000, out);
          } else { //occ [0.25,0.65]
          //未知
            fputc(205, out);
          }
        }
      }
      fclose(out);

      std::string mapmetadatafile =  "mymap.yaml";
      ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
      FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

      geometry_msgs::Quaternion orientation = map->info.origin.orientation;
      tf2::Matrix3x3 mat(tf2::Quaternion(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
      ));
      double yaw, pitch, roll;
      mat.getEulerYPR(yaw, pitch, roll);

      //yaw角后面可开发
      fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
              mapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, 0);

      fclose(yaml);

      ROS_INFO("Done\n");
      saved_map_ = true;
    }

int main (int argc, char** argv) {

    // Initialize ros.
    //
    ros::init(argc, argv, "map");
    ros::NodeHandle nh;
    // cv::Mat img=cv::imread("school.png");
    ros::Subscriber map_sub_ = nh.subscribe("/gridMap", 1, mapCallback);
    // cout<<"helllo "<<endl;

      // cv::Mat img=cv::imread("back_img.png",cv::IMREAD_GRAYSCALE);

    // ros::spin();
    while(!saved_map_ && ros::ok())
        ros::spinOnce();
    return 0;
}