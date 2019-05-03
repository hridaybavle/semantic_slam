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
#include <hdl_graph_slam/ros_utils.hpp>
#include <hdl_graph_slam/ros_time_hash.hpp>

#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/keyframe.hpp>
#include <hdl_graph_slam/keyframe_updater.hpp>
#include <hdl_graph_slam/loop_detector.hpp>
#include <hdl_graph_slam/information_matrix_calculator.hpp>
#include <hdl_graph_slam/map_cloud_generator.hpp>
#include <hdl_graph_slam/nmea_sentence_parser.hpp>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>

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


class semantic_graph_slam
{

public:
    semantic_graph_slam();
    ~semantic_graph_slam();

public:
  data_association data_ass_obj_;

public:
    void run();
    void open(ros::NodeHandle n);
    void init(ros::NodeHandle n);
    void saveGraph();

protected:
    //messages sync
    ros::Subscriber odom_pose_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber detected_object_sub_;


protected:
    void VIOCallback(const nav_msgs::OdometryConstPtr &odom_msg);
    void PointCloudCallback(const sensor_msgs::PointCloud2 &msg);
    void detectedObjectDarknetCallback(const darknet_ros_msgs::BoundingBoxes& msg);

private:
    //point cloud related
    void setPointCloudData(sensor_msgs::PointCloud2 point_cloud);
    void getPointCloudData(sensor_msgs::PointCloud2& point_cloud);
    sensor_msgs::PointCloud2 point_cloud_msg_;
    bool point_cloud_available_;

private:
    //detection related
    bool object_detection_available_;
    std::vector<semantic_SLAM::ObjectInfo> object_info_;
    void setDetectedObjectInfo(std::vector<semantic_SLAM::ObjectInfo> object_info);
    void getDetectedObjectInfo(std::vector<semantic_SLAM::ObjectInfo>& object_info);

private:
    bool flush_keyframe_queue();
    int max_keyframes_per_update_;

    std::mutex trans_odom2map_mutex;
    Eigen::Matrix4f trans_odom2map_;

private:
    void flush_landmark_queue(std::vector<landmark> current_lan_queue,
                              const auto current_keyframe);

protected:
    //robot pose related
    Eigen::VectorXf robot_pose_;
    float cam_angle_;

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
