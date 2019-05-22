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

//hdl graph slam headers
#include <ps_graph_slam/ros_utils.hpp>
#include <ps_graph_slam/ros_time_hash.hpp>

#include <ps_graph_slam/graph_slam.hpp>
#include <ps_graph_slam/keyframe.hpp>
#include <ps_graph_slam/keyframe_updater.hpp>
#include <ps_graph_slam/loop_detector.hpp>
#include <ps_graph_slam/information_matrix_calculator.hpp>
#include <ps_graph_slam/map_cloud_generator.hpp>
#include <ps_graph_slam/nmea_sentence_parser.hpp>

//landmarks
#include "landmark.h"

//data association
#include "data_association.h"

//plane segmentation
//#include "plane_segmentation.h"

//darknet bounding boxes
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"

//segmented pointcloud acc to detection
#include "point_cloud_segmentation.h"

//acl messages
#include "acl_msgs/ViconState.h"

class semantic_graph_slam
{

public:
    semantic_graph_slam();
    ~semantic_graph_slam();

public:
    std::unique_ptr<point_cloud_segmentation> pc_seg_obj_;
    std::unique_ptr<data_association> data_ass_obj_;

public:
    void run();
    void open(ros::NodeHandle n);
    void init(ros::NodeHandle n);
    void addFirstPoseAndLandmark();
    void saveGraph();

    //test stuff
private:
    bool counter_;
    landmark test_landmark_;
    hdl_graph_slam::KeyFrame::Ptr test_keyframe_;
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
    ros::Subscriber cloud_sub_;
    ros::Subscriber detected_object_sub_;
    ros::Subscriber simple_detected_object_sub_;
    ros::Subscriber optitrack_pose_sub_;
    ros::Subscriber vicon_pose_sub_;

protected:
    void rovioVIOCallback(const nav_msgs::OdometryConstPtr &odom_msg);
    void snapVIOCallback(const geometry_msgs::PoseStamped &pose_msg);
    void PointCloudCallback(const sensor_msgs::PointCloud2 &msg);
    void detectedObjectDarknetCallback(const darknet_ros_msgs::BoundingBoxes& msg);
    void detectedObjectSimpleCallback(const semantic_SLAM::DetectedObjects& msg);
    void optitrackPoseCallback(const nav_msgs::Odometry& msg);
    void viconPoseSubCallback(const acl_msgs::ViconState& msg);

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
    void publishDetectedLandmarks(Eigen::VectorXf robot_pose, std::vector<detected_object> det_obj_info);
    void publishKeyframePoses();
    void publishCorresVIOPose();

private:
    //odom related
    void VIOCallback(const ros::Time& stamp,
                     Eigen::Isometry3d odom,
                     Eigen::MatrixXf odom_cov);
    bool first_key_added_;
    Eigen::Isometry3d prev_odom_, vio_pose_;
    std::vector<geometry_msgs::PoseStamped> vio_pose_vec_;

    void setVIOPose(Eigen::Isometry3d vio_pose);

private:
    //point cloud related
    void setPointCloudData(sensor_msgs::PointCloud2 point_cloud);
    void getPointCloudData(sensor_msgs::PointCloud2& point_cloud);
    sensor_msgs::PointCloud2 point_cloud_msg_;
    bool point_cloud_available_;
    ros::Time pc_stamp_;

private:
    //detection related
    bool update_keyframes_using_detections_;
    bool object_detection_available_;
    std::vector<semantic_SLAM::ObjectInfo> object_info_;
    void setDetectedObjectInfo(std::vector<semantic_SLAM::ObjectInfo> object_info);
    void getDetectedObjectInfo(std::vector<semantic_SLAM::ObjectInfo>& object_info);

private:
    //optitrack related
    std::vector<geometry_msgs::PoseStamped> optitrack_pose_vec_;

private:
    bool flush_keyframe_queue();
    int max_keyframes_per_update_;

    std::mutex trans_odom2map_mutex;
    Eigen::Matrix4f trans_odom2map_;

private:
    //landmark related functions
    std::vector<landmark> semantic_data_ass(const hdl_graph_slam::KeyFrame::Ptr curr_keyframe);

    void flush_landmark_queue(std::vector<landmark> current_lan_queue,
                              const auto current_keyframe);

    void getAndSetLandmarkCov();

private:
    //vicon pose related
    float gt_x_transform_;
    float gt_y_transform_;
    float gt_z_transform_;

    bool first_gt_pose_;

protected:
    //robot pose related
    Eigen::Isometry3d robot_pose_;
    double cam_angled_;
    float cam_angle_;
    bool add_first_lan_;
    double first_lan_x_, first_lan_y_,first_lan_z_;

private:

    // keyframe related params
    std::deque<hdl_graph_slam::KeyFrame::Ptr> new_keyframes_;
    std::vector<hdl_graph_slam::KeyFrame::Ptr> keyframes_;
    std::unordered_map<ros::Time, hdl_graph_slam::KeyFrame::Ptr, RosTimeHash> keyframe_hash_;
    std::deque<hdl_graph_slam::KeyFrame::Ptr> keyframe_queue_;

    //landmark relared params
    std::vector<landmark> landmarks_vec_;

    //loop detector
    std::unique_ptr<hdl_graph_slam::LoopDetector> loop_detector_;

    std::mutex keyframe_queue_mutex;

    std::unique_ptr<hdl_graph_slam::GraphSLAM> graph_slam_;
    std::unique_ptr<hdl_graph_slam::KeyframeUpdater> keyframe_updater_;
    std::unique_ptr<hdl_graph_slam::InformationMatrixCalculator> inf_calclator_;

};
