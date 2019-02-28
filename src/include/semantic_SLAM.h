#include <iostream>
#include <string>
#include <math.h>
#include <mutex>

#include "ros/ros.h"
#include "eigen3/Eigen/Eigen"
#include <binders.h>

//particle filter lib
#include "particle_filter.h"

//point cloud segmentation
#include "plane_segmentation.h"

//tools lib
#include "tools.h"

#include "tf/transform_datatypes.h"

//ROS messages
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

//PCL ROS
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

//PCL
#include "pcl/point_types.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>

//aruco_eye msgs
#include "aruco_eye_msgs/MarkerList.h"

//bebop imu message
#include "semantic_SLAM/Ardrone3PilotingStateAttitudeChanged.h"

//darknet object detector
#include "semantic_SLAM/DetectedObjects.h"
#include "semantic_SLAM/ObjectInfo.h"
#include "semantic_SLAM/MappedPointCloud.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"

#include "sensor_msgs/image_encodings.h"
//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

const float real_sense_pitch_angle =0*(M_PI/180);
const int state_size_ = 6;
const int num_particles_ = 800;

const float optitrack_x_transform =  2.9;
const float optitrack_y_transform = -0.1;
const float optitrack_z_transform = -0.05;

class semantic_slam_ros
{

public:
    semantic_slam_ros();
    ~semantic_slam_ros();

public:
    void open(ros::NodeHandle n);
    void run();

    //pcl variables
    //pcl viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer;

private:
    void init();
    void readGroundTruthPoint();


    double prev_time_, current_time_;

    //particle filter class object
    particle_filter particle_filter_obj_;

    //point cloud segmentation class object
    plane_segmentation plane_segmentation_obj_;

    //tools for pose conversions
    semantic_tools semantic_tools_obj_;

    //Publishers and subscribers
protected:
    ros::Subscriber rovio_odometry_sub_;
    void rovioOdometryCallback(const nav_msgs::Odometry& msg);

    ros::Subscriber imu_sub_;
    void imuCallback(const sensor_msgs::Imu& msg);

    ros::Subscriber detected_object_sub_;
    void detectedObjectCallback(const semantic_SLAM::DetectedObjects& msg);
    void detectedObjectDarknetCallback(const darknet_ros_msgs::BoundingBoxes& msg);

    ros::Subscriber point_cloud_sub_;
    void pointCloudCallback(const sensor_msgs::PointCloud2& msg);

    ros::Subscriber optitrack_pose_sub_;
    void optitrackPoseCallback(const nav_msgs::Odometry& msg);

    ros::Subscriber optitrack_pose_sub_for_plottin_path_;
    void optitrackPoseForPlottingPathCallback(const geometry_msgs::PoseStamped& msg);

    ros::Publisher particle_poses_pub_;
    void publishParticlePoses();

    ros::Publisher final_pose_pub_;
    ros::Publisher final_path_pub_;
    void publishFinalPose();

    ros::Publisher corres_vo_pose_pub_;
    ros::Publisher corres_vo_path_;
    void publishCorresVOPose();

    ros::Publisher segmented_point_cloud_pub_;
    void publishSegmentedPointCloud(sensor_msgs::PointCloud2 point_cloud_seg);

    ros::Publisher detected_object_point_pub_;
    void publishFinalDetectedObjectPoint(geometry_msgs::Point final_point);

    ros::Publisher mapped_objects_visualizer_pub_;
    void publishMappedObjects(std::vector<particle_filter::object_info_struct_pf> mapped_object_vec);
    void publishAllMappedObjects(std::vector<particle_filter::all_object_info_struct_pf> mapped_object_vec);

    ros::Publisher mapped_points_pub_;
    void publishNewMappedObjects(std::vector<particle_filter::object_info_struct_all_points_pf> mapped_object_vec);



    ros::Publisher ground_truth_points_pub_;
    void publishGroundTruthPoints(std::vector<geometry_msgs::Point>  points);

    ros::Publisher optitrack_pose_pub_;
    ros::Publisher optitrack_path_pub_;

    //ros node params
    std::string text_file_;

protected:

    std::vector<geometry_msgs::Point> points_vec_;

    //variables regarding RVIO
    std::mutex rvio_pose_lock_;
    bool rvio_pose_available_;
    Eigen::VectorXf RVIO_pose_;
    void setRVIOPose(Eigen::VectorXf RVIO_pose);
    void getRVIOPose(Eigen::VectorXf& RVIO_pose);


    //variables regarding IMU
    std::mutex imu_lock_;
    float imu_roll_, imu_pitch_, imu_yaw_;
    void setIMUdata(float roll, float pitch, float yaw);
    void getIMUdata(float& roll, float& pitch, float& yaw);

    void setDetectedObjectInfo(std::vector<semantic_SLAM::ObjectInfo> object_info);
    void getDetectedObjectInfo(std::vector<semantic_SLAM::ObjectInfo>& object_info);

    void setPointCloudData(sensor_msgs::PointCloud2 point_cloud);
    void getPointCloudData(sensor_msgs::PointCloud2& point_cloud);

    std::vector<particle_filter::object_info_struct_pf> segmentPointCloudData();
    std::vector<particle_filter::all_object_info_struct_pf> segmentallPointCloudData();
    std::vector<particle_filter::object_info_struct_all_points_pf> segmentPointsfromDetections();

    std::vector<particle_filter::all_object_info_struct_pf> segmentChairPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_point_cloud,
                                                                               pcl::PointCloud<pcl::Normal>::Ptr segmented_point_cloud_normal,
                                                                               Eigen::Matrix4f transformation_mat,
                                                                               Eigen::VectorXf final_pose_);


    std::vector<particle_filter::all_object_info_struct_pf> segmenMonitorPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_point_cloud,
                                                                                pcl::PointCloud<pcl::Normal>::Ptr segmented_point_cloud_normal,
                                                                                Eigen::Matrix4f transformation_mat,
                                                                                Eigen::VectorXf final_pose_);

    //variables regarding imu
    bool imu_data_available_;
    bool imu_first_yaw_;
    float first_yaw_;
    Eigen::Matrix4f transformation_mat_acc_, transformation_mat_ang_vel_;
    Eigen::Vector4f imu_local_acc_mat_, imu_world_acc_mat_;
    Eigen::Vector4f imu_local_ang_vel_, imu_world_ang_vel_;

    std::vector<Eigen::VectorXf> filtered_pose_;
    Eigen::VectorXf final_pose_;

    //variables regarding detected object
    std::mutex detected_object_lock_;
    bool object_detection_available_;
    std::vector<semantic_SLAM::ObjectInfo> object_info_;
    std::vector<particle_filter::object_info_struct_pf> mapped_object_vec_;
    std::vector<particle_filter::all_object_info_struct_pf> all_mapped_object_vec_;
    std::vector<particle_filter::object_info_struct_all_points_pf> new_mapped_object_vec_;

    //variables regarding the point cloud
    std::mutex point_cloud_lock_;
    bool point_cloud_available_;
    sensor_msgs::PointCloud2 point_cloud_msg_;

    //for publishing the path
    std::vector<geometry_msgs::PoseStamped> final_particle_pose_vec_;
    std::vector<geometry_msgs::PoseStamped> optitrack_pose_vec_;
    std::vector<geometry_msgs::PoseStamped> vo_pose_vec_;
};
