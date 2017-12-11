#include <iostream>
#include <string>
#include <math.h>

#include "ros/ros.h"
#include "eigen3/Eigen/Eigen"
#include <binders.h>

//ROS messages
#include "geometry_msgs/PoseStamped.h"

const float camera_pitch_angle_ = 0;

class semantic_SLAM
{

public:
     semantic_SLAM();
     ~semantic_SLAM();

public:
     void open(ros::NodeHandle n);

private:
    void init();

     //Publishers and subscribers
protected:
     ros::Subscriber stereo_odometry_sub_;
     void stereoOdometryCallback(const geometry_msgs::PoseStamped& msg);


protected:
     float pose_x_, pose_y_, pose_z_;
     void transformCameraToWorld(float x, float y, float z, float roll, float pitch, float yaw, Eigen::Matrix4f &transformation_mat);
};
