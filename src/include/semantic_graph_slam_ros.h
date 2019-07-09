#include <ctime>
#include <mutex>
#include <atomic>
#include <memory>
#include <iomanip>
#include <iostream>
#include <unordered_map>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>

//ros dependencies
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
//ros time synchronizer
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//ps graph slam headers
#include <ps_graph_slam/ros_utils.hpp>
#include <ps_graph_slam/ros_time_hash.hpp>

#include <ps_graph_slam/graph_slam.hpp>
#include <ps_graph_slam/keyframe.hpp>
#include <ps_graph_slam/keyframe_updater.hpp>
#include <ps_graph_slam/information_matrix_calculator.hpp>

//landmarks
#include "ps_graph_slam/landmark.h"

//data association
#include "ps_graph_slam/data_association.h"

//plane segmentation
//#include "plane_segmentation.h"

//darknet bounding boxes
#include "semantic_SLAM//BoundingBox.h"
#include "semantic_SLAM/BoundingBoxes.h"

//segmented pointcloud acc to detection
#include "planar_segmentation/point_cloud_segmentation.h"

//acl messages
#include "semantic_SLAM/ViconState.h"

#include "ps_graph_slam/semantic_graph_slam.h"

class semantic_graph_slam_ros
{

public:
    semantic_graph_slam_ros();
    ~semantic_graph_slam_ros();

public:
    std::unique_ptr<point_cloud_segmentation> pc_seg_obj_;
    std::unique_ptr<data_association> data_ass_obj_;
    std::unique_ptr<semantic_graph_slam> semantic_gslam_obj_;

public:
    void run();
    void open(ros::NodeHandle n);
    void init(ros::NodeHandle n);
    void addFirstPoseAndLandmark();
    void saveGraph();

private:
    bool use_rovio_odom_;
    bool use_rtab_map_odom_;
    bool use_snap_pose_;
    bool use_orb_slam_odom_;
    bool save_graph_;
    std::string save_graph_path_;

private:
    bool verbose_;

    //test stuff
private:
    bool counter_;
    landmark test_landmark_;
    ps_graph_slam::KeyFrame::Ptr test_keyframe_;
    int odom_increments_;

    void add_odom_pose_increments();
    void add_odom_position_increments();

protected:
    //messages sync
    //    message_filters::Subscriber<nav_msgs::Odometry> odom_msg_sub_;
    //    message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_msg_sub_;
    //    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bb_sub_;

    //    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,
    //    sensor_msgs::PointCloud2,
    //    darknet_ros_msgs::BoundingBoxes> SyncPolicy;

    //    message_filters::Synchronizer<SyncPolicy> sync;

    //    void synMsgsCallback(const nav_msgs::OdometryConstPtr &odom_msg,
    //                         const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
    //                         const darknet_ros_msgs::BoundingBoxesConstPtr &bbs_msg);

    //subscribers
protected:
    ros::Subscriber rvio_odom_pose_sub_;
    ros::Subscriber snap_odom_pose_sub_;
    ros::Subscriber jackal_odom_pose_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber detected_object_sub_;
    ros::Subscriber simple_detected_object_sub_;
    ros::Subscriber optitrack_pose_sub_;
    ros::Subscriber vicon_pose_sub_;

protected:
    void rovioVIOCallback(const nav_msgs::OdometryConstPtr &odom_msg);
    void snapVIOCallback(const geometry_msgs::PoseStamped &pose_msg);
    void jackalOdomCallback(const nav_msgs::OdometryConstPtr &odom_msg);
    void PointCloudCallback(const sensor_msgs::PointCloud2 &msg);
    void detectedObjectDarknetCallback(const semantic_SLAM::BoundingBoxes& msg);
    void detectedObjectSimpleCallback(const semantic_SLAM::DetectedObjects& msg);
    void optitrackPoseCallback(const nav_msgs::Odometry& msg);
    void viconPoseSubCallback(const semantic_SLAM::ViconState& msg);

    //publishers
protected:
    ros::Publisher landmarks_pub_;
    ros::Publisher detected_lans_pub_;
    ros::Publisher keyframe_pose_pub_;
    ros::Publisher robot_pose_pub_;
    ros::Publisher robot_transform_pub_;
    ros::Publisher keyframe_path_pub_;
    ros::Publisher optitrack_pose_pub_;
    ros::Publisher optitrack_path_pub_;
    ros::Publisher corres_vio_pose_pub_;
    ros::Publisher corres_vio_path_;

protected:
    void publishRobotPose();
    void publishLandmarks();
    void publishDetectedLandmarks();
    void publishKeyframePoses();
    void publishCorresVIOPose();

private:
    //odom related
    std::vector<geometry_msgs::PoseStamped> vio_pose_vec_;

private:
    //point cloud related
    ros::Time pc_stamp_;

private:
    //optitrack related
    std::vector<geometry_msgs::PoseStamped> optitrack_pose_vec_;

private:
    //jackal pose related
    float jack_x_transform_;
    float jack_y_transform_;
    float jack_z_transform_;
    float jack_yaw_transform_;
    bool first_jack_pose_;

private:
    //vicon pose related
    float gt_x_transform_;
    float gt_y_transform_;
    float gt_z_transform_;

    bool first_gt_pose_;

};
