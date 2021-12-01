#include "../include/tf_test/gps_wrapper.h"


gpstest::gpstest(ros::NodeHandle& nh)
{
//    LOG(INFO) << "hello ros!!";
//    rostest test;
//    int n=test.backone();
//    if(n==1)
//    {
//        LOG(INFO) << "hello ros!!!";
//    }

    // nh.param("map_resolution" , resolution);
    // nh.param("map_width",  width);
    // nh.param("map_height",  height );
    // nh.param("originx",  originx);
    // nh.param("originy",  originy );

    ros::param::get("map_resolution" , resolution);
    ros::param::get("map_width",  width);
    ros::param::get("map_height",  height );
    // ros::param::get("originx",  originx);
    // ros::param::get("originy",  originy );

    // LOG(INFO) << "resolution="<<resolution;

    initGPS=false;
    createmap();
    std::string log_folder = "/home";
    ros::param::get("log_folder", log_folder);


    // Log.
    file_gps_.open(log_folder +"/dataset/gps.csv");
    file_xyz_.open(log_folder +"/dataset/xyz.csv");
    file_frist_gps_.open(log_folder +"/map/frist_gps.csv");

    gps_position_sub_ = nh.subscribe("/fix", 10,  &gpstest::GpsPositionCallback, this);

    state_pub_ = nh.advertise<nav_msgs::Path>("gps_path", 10);
    map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/gridMap", 1);
}

gpstest::~gpstest() {
    file_gps_.close();
    file_xyz_.close();
    file_frist_gps_.close();
}

void gpstest::LogXyz(const GpsLocalization::xyzPositionDataPtr xyz_data) {
    file_xyz_ << std::fixed << std::setprecision(15)
              << xyz_data->x << "," << xyz_data->y << "," << xyz_data->z << "\n";
}

void gpstest::LogGps(const GpsLocalization::GpsPositionDataPtr gps_data) {
    count++;
    file_gps_ << std::fixed << std::setprecision(15)
              << gps_data->timestamp << ","
              << gps_data->lla[0] << "," << gps_data->lla[1] << "," << gps_data->lla[2] << "\n";
    if(count==1)
    {
            file_frist_gps_ << std::fixed << std::setprecision(15)
              << gps_data->timestamp << ","
              << gps_data->lla[0] << "," << gps_data->lla[1] << "," << gps_data->lla[2] << "\n";
    }
}

void gpstest::GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr) {
    // Check the gps_status.
    GpsLocalization::GpsPositionDataPtr gps_data_ptr = std::make_shared<GpsLocalization::GpsPositionData>();
    gps_data_ptr->timestamp = gps_msg_ptr->header.stamp.toSec();
    gps_data_ptr->lla << gps_msg_ptr->latitude,
            gps_msg_ptr->longitude,
            gps_msg_ptr->altitude;
    gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gps_msg_ptr->position_covariance.data());
    if (gps_msg_ptr->status.status != 2) {
        // LOG(WARNING) << "[GpsCallBack]: Bad gps message!";
        // printf("wtf: %f\n",gps_msg_ptr->position_covariance.data()[0]);
    }

    LogGps(gps_data_ptr);
    gps2xyz(gps_data_ptr);

}

void gpstest::gps2xyz(const GpsLocalization::GpsPositionDataPtr gps_data)
{
    GpsLocalization::xyzPositionDataPtr xyz_data_ptr = std::make_shared<GpsLocalization::XYZposition>();
    if(!initGPS)
    {
        geoConverter.Reset(gps_data->lla[0], gps_data->lla[1], gps_data->lla[2]);
        initGPS = true;
    }
    geoConverter.Forward(gps_data->lla[0],  gps_data->lla[1], gps_data->lla[2], xyz_data_ptr->x, xyz_data_ptr->y, xyz_data_ptr->z);
    LogXyz(xyz_data_ptr);
    ConvertStateToRosTopic(xyz_data_ptr);
    updatemap(xyz_data_ptr);
}

void gpstest::ConvertStateToRosTopic(const GpsLocalization::xyzPositionDataPtr xyz_data) {
    ros_path_.header.frame_id = "/world";
    ros_path_.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped pose;
    pose.header = ros_path_.header;

    pose.pose.position.x =xyz_data->x ;
    pose.pose.position.y =xyz_data->y;
    pose.pose.position.z =xyz_data->z;

    //const Eigen::Quaterniond G_q_I(state.G_R_I);
//    pose.pose.orientation.x = 0;
//    pose.pose.orientation.y = 0;
//    pose.pose.orientation.z = 0;
//    pose.pose.orientation.w = 1;

    ros_path_.poses.push_back(pose);
    state_pub_.publish(ros_path_);
}

void gpstest::createmap()
{

    map.header.frame_id="/map";
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

void gpstest::updatemap(const GpsLocalization::xyzPositionDataPtr xyz_data)
{
    int res_inv = 1/resolution;
    int lenth=(1.5/resolution)+3;
    int origin=map.info.width*(map.info.height/2)+map.info.width/2;
    int xyzposition=(int)(xyz_data->x*res_inv)+(int)(xyz_data->y*res_inv*map.info.width);
    int x,y,id;
    for(int i=-lenth;i<=lenth;i++)
        for(int j=-lenth;j<=lenth;j++)
        {
            x=j;
            y=i;
            id=origin+x+y*map.info.width+xyzposition;
            if(x*x+y*y<=((lenth-4)*(lenth-4)))
            {
                map.data[id]=0;
            }
            else if(x*x+y*y <=lenth*lenth  &&  map.data[id]==-1)
            {
                map.data[id]=100;
            }
        }
    map_pub_.publish(map);
}