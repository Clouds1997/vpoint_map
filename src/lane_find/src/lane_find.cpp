#include "../include/lane_find/lane_find.h"

using namespace std;

lane_find::lane_find(ros::NodeHandle& nh)
{
    sub_img0 = nh.subscribe("/zed2/zed_node/left/image_rect_color", 100, &lane_find::img0_callback,this);
}

lane_find::~lane_find()
{

}

void lane_find::img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    // cv::Point2f p1=cv::Point2f(40, 40); 
    cv::Mat img = ptr->image.clone(); 
    cv::Mat img_blur,hsv_img , hsv_img_mask;
    
    cv::blur(img,img_blur,cv::Size(45,45),cv::Point(20,30),4);

    cv::cvtColor(img_blur,hsv_img,cv::COLOR_BGR2HSV);
    // hsv_img.at<cv::Vec3b>(66,66)[1]=50;
    // int a = hsv_img.at<cv::Vec3b>(66,66)[1];
    int rows=hsv_img.rows;
    int cols= hsv_img.cols;
    // cout<< hsv_img.rows<<endl;
    // cout<<a<<endl;

    vector<cv::Vec3b> hsv1, hsv2 ,hsv3;
    cv::Vec3b temp;
   for(int i = rows/8*6;i<=rows/8*7;i++)
   {
       for(int j = cols/12*5;j<=cols/12*7;j++)
       {
        //    h1.push_back(hsv_img.at<cv::Vec3b>(i,j)[0]);
        //    s1.push_back(hsv_img.at<cv::Vec3b>(i,j)[1]);
        //    v1.push_back(hsv_img.at<cv::Vec3b>(i,j)[2]);
           temp[0]=hsv_img.at<cv::Vec3b>(i,j)[0];
           temp[1]=hsv_img.at<cv::Vec3b>(i,j)[1];
           temp[2]=hsv_img.at<cv::Vec3b>(i,j)[2];
           hsv1.push_back(temp);
       }
        for(int j = cols/12*3;j<cols/12*4;j++)
       {
           if((j-cols/12*3)>=(-(i-rows/8*7)))
           {
                // h2.push_back(hsv_img.at<cv::Vec3b>(i,j)[0]);
                // s2.push_back(hsv_img.at<cv::Vec3b>(i,j)[1]);
                // v2.push_back(hsv_img.at<cv::Vec3b>(i,j)[2]);
                temp[0]=hsv_img.at<cv::Vec3b>(i,j)[0];
                temp[1]=hsv_img.at<cv::Vec3b>(i,j)[1];
                temp[2]=hsv_img.at<cv::Vec3b>(i,j)[2];
                hsv2.push_back(temp);
           }
       }
        for(int j = cols/12*8;j<=cols/12*9;j++)
       {
           if((i-rows/8*6)>=(j-cols/12*8))
           {
                // h3.push_back(hsv_img.at<cv::Vec3b>(i,j)[0]);
                // s3.push_back(hsv_img.at<cv::Vec3b>(i,j)[1]);
                // v3.push_back(hsv_img.at<cv::Vec3b>(i,j)[2]);
                temp[0]=hsv_img.at<cv::Vec3b>(i,j)[0];
                temp[1]=hsv_img.at<cv::Vec3b>(i,j)[1];
                temp[2]=hsv_img.at<cv::Vec3b>(i,j)[2];
                hsv3.push_back(temp);
           }
       }
   } 
    cv::Vec3b ave1=vector_ave(hsv1);
    cv::Vec3b ave2=vector_ave(hsv2);
    cv::Vec3b ave3=vector_ave(hsv3);
    // cv::Vec3b hsv_max , hsv_min;
    vector<int> h_vec,s_vec,v_vec;

    h_vec.push_back(ave1[0]);
    h_vec.push_back(ave2[0]);
    h_vec.push_back(ave3[0]);

    s_vec.push_back(ave1[1]);
    s_vec.push_back(ave2[1]);
    s_vec.push_back(ave3[1]);

    v_vec.push_back(ave1[2]);
    v_vec.push_back(ave2[2]);
    v_vec.push_back(ave3[2]);

    sort(h_vec.begin(),h_vec.end());
    sort(s_vec.begin(),s_vec.end());
    sort(v_vec.begin(),v_vec.end());

    cv::inRange(hsv_img,cv::Scalar(h_vec[0]-10,s_vec[0]-10,v_vec[0]-10),cv::Scalar(h_vec[2]+10,s_vec[2]+10,v_vec[2]+10),hsv_img_mask);
    //  cv::inRange(hsv_img,cv::Scalar(0,s_vec[0],v_vec[0]),cv::Scalar(150,s_vec[2],v_vec[2]),hsv_img_mask);

    cv::imshow("blur_img",hsv_img);
    cv::imshow("hsv_img",hsv_img_mask);
    cv::imshow("img",img);
    cv::waitKey(50);
}

cv::Vec3b lane_find::vector_ave(vector<cv::Vec3b> vec)
{
    cv::Vec3d temp;
    cv::Vec3b ave;
    temp[0]=0;
    temp[1]=0;
    temp[2]=0;  
    for(vector<cv::Vec3b>::iterator it=vec.begin();it!=vec.end();it++)
    {
        temp[0] +=(*it)[0];
        temp[1] +=(*it)[1];
        temp[2] +=(*it)[2];
    }
    temp[0]=temp[0]/vec.size();
    temp[1]=temp[1]/vec.size();
    temp[2]=temp[2]/vec.size();
    ave=temp;
    return ave;
}

int lane_find::max(int a,int b,int c)
{
    if(a>=b && a>=c)
        return a;
    else if(b>=a && b>=c)
        return b;
    else 
        return c;
}

int lane_find::min(int a,int b,int c)
{
    if(a<=b && a<=c)
        return a;
    else if(b<=a && b<=c)
        return b;
    else 
        return c;
}

// void lane_find::find_range()
// {
//     if(h_vec[0]==h_vec[1] && h_vec[1]==h_vec[2])
//     {
//         h_p=0;
//     }
//     else if(h_vec[0]<=h_vec[1] && h_vec[1]<=h_vec[2])
//     {
//         int detal1,detal2;
//         detal1=h_vec[1]-h_vec[0];
//         detal2=h_vec[2]-h_vec[1];
//         if(detal1 > detal2)
//             h_p=detal1;
//         else
//         {
//             h_p=detal2;
//         }
//     }
//     else if(h_vec[0]<h_vec[1] && h_vec[1]>=h_vec[2])
// }