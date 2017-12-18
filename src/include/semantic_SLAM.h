#include <iostream>
#include <string>
#include <math.h>
#include <mutex>

#include "ros/ros.h"
#include "eigen3/Eigen/Eigen"
#include <binders.h>

//particle filter lib
#include "particle_filter.h"

#include "tf/transform_datatypes.h"

//ROS messages
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"

//aruco_eye msgs
#include "aruco_eye_msgs/MarkerList.h"


const float camera_pitch_angle_ = 0;
const int state_size_ = 6;
const int num_particles_ = 10;

class semantic_SLAM
{

public:
     semantic_SLAM();
     ~semantic_SLAM();

public:
     void open(ros::NodeHandle n);
     void run();

private:
    void init();
    double prev_time_, current_time_;

    particle_filter particle_filter_obj_;

     //Publishers and subscribers
protected:
     ros::Subscriber stereo_odometry_sub_;
     void stereoOdometryCallback(const geometry_msgs::PoseStamped& msg);

     ros::Subscriber imu_sub_;
     void imuCallback(const sensor_msgs::Imu& msg);

     ros::Subscriber aruco_observation_sub_;
     void arucoObservationCallback(const aruco_eye_msgs::MarkerList& msg);


protected:
     //variables regarding VO
     float pose_x_, pose_y_, pose_z_;
     void transformCameraToRobot(Eigen::Matrix4f &transformation_mat);
     void transformIMUtoWorld(float ax, float ay, float az, Eigen::Matrix4f &transformation_mat);
     void setVOPose(Eigen::VectorXf VO_pose);
     void getVOPose(Eigen::VectorXf& VO_pose);

     //variables regarding imu
     bool imu_data_available_;
     Eigen::Matrix4f transformation_mat_acc_, transformation_mat_ang_vel_;
     Eigen::Vector4f imu_local_acc_mat_, imu_world_acc_mat_;
     Eigen::Vector4f imu_local_ang_vel_, imu_world_ang_vel_;

     std::mutex vo_pose_lock_;
     Eigen::VectorXf VO_pose_;
     bool vo_data_available_;

     std::vector<Eigen::VectorXf> filtered_pose_;

     //variables regarding aruco detection
     std::vector<Eigen::Vector4f> aruco_pose_cam_;

};





