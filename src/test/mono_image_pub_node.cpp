//I/O stream
//std::cout
#include <iostream>

//ROS
#include "ros/ros.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


//global variables
ros::Publisher  image_pubs_;

using namespace std;

void CamImageCallback(const sensor_msgs::Image& msg);

int main(int argc,char **argv)
{

    ros::init(argc, argv, "mono_cam_image");
    ros::NodeHandle nh;


    ros::Subscriber cam_img_sub  = nh.subscribe("/camera/color/image_raw", 1, &CamImageCallback);

    image_pubs_                  = nh.advertise<sensor_msgs::Image>("/camera/image_mono", 1);

    //while(ros::ok())
    {
        ros::spin();
    }



    return 1;
}

void CamImageCallback(const sensor_msgs::Image &msg)
{

     cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat image_grey;
    image_grey = cv_ptr->image;
    cv::cvtColor(image_grey, image_grey, cv::COLOR_RGB2GRAY);


    //publishing the converted grey scale image with the same timestamp as the color image
    cv_bridge::CvImage img_bridge;
    std_msgs::Header image_header;
    image_header.stamp = msg.header.stamp;
    sensor_msgs::Image ros_image;

    img_bridge = cv_bridge::CvImage(image_header, sensor_msgs::image_encodings::MONO8, image_grey);
    img_bridge.toImageMsg(ros_image);
    image_pubs_.publish(ros_image);
}
