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
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/PoseArray.h"

//aruco_eye msgs
#include "aruco_eye_msgs/MarkerList.h"

//bebop imu message
#include "semantic_SLAM/Ardrone3PilotingStateAttitudeChanged.h"

//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

const float camera_pitch_angle_ = 0;
const int state_size_ = 6;
const int num_particles_ = 500;

class semantic_slam_ros
{

public:
     semantic_slam_ros();
     ~semantic_slam_ros();

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

    ros::Subscriber magnetic_field_sub_;
    void magneticFieldCallback(const sensor_msgs::MagneticField& msg);

    ros::Subscriber bebop_imu_sub_;
    void bebopIMUCallback(const semantic_SLAM::Ardrone3PilotingStateAttitudeChanged& msg);
    bool bebop_imu_data_ready_;
    float yaw_first_;

    ros::Subscriber aruco_observation_sub_;
    void arucoObservationCallback(const aruco_eye_msgs::MarkerList& msg);

    ros::Subscriber slam_dunk_pose_sub_;
    void slamdunkPoseCallback(const geometry_msgs::PoseStamped& msg);

     ros::Publisher particle_poses_pub_;
     void publishParticlePoses();

     ros::Publisher final_pose_pub_;
     void publishFinalPose();

     ros::Publisher corres_vo_pose_pub_;
     void publishCorresVOPose();

protected:
     //variables regarding VO
     float pose_x_, pose_y_, pose_z_;
     void transformCameraToRobot(Eigen::Matrix4f &transformation_mat);
     void transformIMUtoWorld(float ax, float ay, float az, Eigen::Matrix4f &transformation_mat);
     void setVOPose(Eigen::VectorXf VO_pose);
     void getVOPose(Eigen::VectorXf& VO_pose);
     void setArucoPose(std::vector<Eigen::Vector4f> aruco_pose);
     void getArucoPose(std::vector<Eigen::Vector4f>& aruco_pose);

     void setSlamdunkPose(Eigen::Vector3f pose);
     void getSlamdunkPose(Eigen::Vector3f& pose);

     std::mutex bebop_imu_lock_;
     bool bebop_imu_data_available_;
     float bebop_imu_roll_, bebop_imu_pitch_, bebop_imu_yaw_;
     void setBebopIMUData(float roll,float pitch, float yaw);
     void getBebopIMUData(float& roll, float& pitch, float& yaw);

     std::mutex imu_lock_;
     float acc_x_, acc_y_, acc_z_;
     void setIMUdata(float acc_x,float acc_y,float acc_z);
     void getIMUdata(float& acc_x, float& acc_y, float& acc_z);

     std::mutex magnetometer_data_lock_;
     float mag_x_, mag_y_, mag_z_;
     void setMagData(float mag_x, float mag_y, float mag_z);
     void getMagData(float& mag_x, float& mag_y,float& mag_z);

     bool magnetic_field_available_;
     void calculateRPYAngles();


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
     std::vector<Eigen::Vector3f> aruco_poses_;
     std::mutex aruco_pose_lock_;
     bool aruco_data_available_;
     std::vector<Eigen::Vector4f> aruco_pose_;
     Eigen::VectorXf final_pose_;

     //variables for slamdunk pose
     std::mutex slamdunk_pose_lock_;
     bool slamdunk_data_available_;
     Eigen::Vector3f slamdunk_pose_;

};





