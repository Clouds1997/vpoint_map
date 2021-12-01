#include "../include/tf_test/mapping.h"

using namespace std;

mapping::mapping(ros::NodeHandle& nh)
{
    // ros::param::get("map_resolution" , resolution);
    // ros::param::get("map_width",  width);
    // ros::param::get("map_height",  height );
    // resolution = 0.1;
    // width = 2000;
    // height =2000;
    // createmap();
    gps_position_sub_ = nh.subscribe("/gps", 10,  &mapping::GpsPositionCallback,this);
    odom_publish = nh.advertise<nav_msgs::Odometry>("map_odom", 100);
    // map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/gridMap", 1);
    // point_cloud_sub_ = nh.subscribe("/point_cloud1",100,&mapping::point_cloud_sub, this);
    path_position_sub_ = nh.subscribe("/globalEstimator/global_odometry", 100,  &mapping::odom_sub, this);

}

mapping::~mapping(){

}

void mapping::GpsPositionCallback(const sensor_msgs::NavSatFix& gps_msg) {
    // Check the gps_status.
    if(!initGPS)
    {
        string log_folder = "/home";
         ros::param::get("log_folder", log_folder);
        // ros::param::get("log_folder", log_folder);
        // Log.
        // cout<<log_folder<<endl;
        file_frist_gps_.open(log_folder+"/map/first_gps.csv", ios::out);
        if(!file_frist_gps_)
        { 
            cout<<"error !"<<endl;
        }

        geoConverter.Reset(gps_msg.latitude, gps_msg.longitude, gps_msg.altitude);

        cout<< std::fixed << std::setprecision(15)
         << gps_msg.latitude<< "," << gps_msg.longitude << "," << gps_msg.altitude<<"23333"<<endl;

        file_frist_gps_ << std::fixed << std::setprecision(15)
         << gps_msg.latitude<< "," << gps_msg.longitude << "," << gps_msg.altitude<< "\n";
        
        initGPS = true;
        file_frist_gps_.close();
        cout<<"gps init complite!!!"<<endl;
    }
}

void mapping::odom_sub(const nav_msgs::Odometry& pose)
{
    // geometry_msgs::PoseStamped pose;
    // pose=odom_path.pose;
    if(!initxyz){
        X=pose.pose.pose.position.x;
        Y=pose.pose.pose.position.y;
        initxyz=true;
    }
      // first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "map_odom";
    odom_trans.child_frame_id = "map_base_link";

    odom_trans.transform.translation.x = pose.pose.pose.position.x-X;
    odom_trans.transform.translation.y = pose.pose.pose.position.y-Y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = pose.pose.pose.orientation;

    //send the transform
    map_odom_baselink_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map_odom";

    //set the position
    odom.pose.pose.position.x = pose.pose.pose.position.x-X;
    odom.pose.pose.position.y = pose.pose.pose.position.y-Y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = pose.pose.pose.orientation;

    //set the velocity
    odom.child_frame_id = "map_base_link";
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = 0;

    //publish the message
    odom_publish.publish(odom);


    // Eigen::Quaterniond Q(pose.pose.pose.orientation.w,pose.pose.pose.orientation.x,pose.pose.pose.orientation.y,pose.pose.pose.orientation.z);
    // Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    // // cout <<T<<endl;
    // T.rotate(Q);
    // // T.pretranslate(Eigen::Vector3d(pose.pose.pose.position.x,pose.pose.pose.position.y,0));
    // T.pretranslate(Eigen::Vector3d(odom.pose.pose.position.x,odom.pose.pose.position.y,0));
    // // // std::cout<<T<<std::endl;
    // // // Sophus::SE3 T_pose(Q,);
    // Eigen::Vector4d V4d_1;
    // Eigen::Vector4d V4d_2;
    // double map_x,map_y;
    // double res_inv = 10;
    // int origin=map.info.width*(map.info.height/2)+map.info.width/2;
    // if(count % 5 ==0){
    //     for (size_t cp = 460800; cp < point_cloud1.points.size (); ++cp){
    //         if(point_cloud1.points[cp].x > 0){ 
    //             // cout<<point_cloud1.points[cp].x<<" "<<point_cloud1.points[cp].y<<endl;
    //             V4d_1=Eigen::Vector4d(point_cloud1.points[cp].x,point_cloud1.points[cp].y,0,1);
    //             V4d_2 = T* V4d_1;
    //             map_x=V4d_2[0]*res_inv;
    //             map_y=V4d_2[1]*res_inv;
    //             int xyzposition=(int)map_x+(int)map_y*map.info.width;
    //             int id;
    //             id=origin+xyzposition;
    //             map.data[id]=0;
    //             // flag++;
    //         }
    //    }
    // }
    // count ++;
    // cout<<count<<endl;
    // map_pub_.publish(map);
}

void mapping::point_cloud_sub(const sensor_msgs::PointCloud &pc1)
{ 
    point_cloud1 = pc1;
}

void mapping::createmap()
{
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

void mapping::updatemap()
{

}