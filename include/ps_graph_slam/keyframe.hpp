#ifndef KEYFRAME_HPP
#define KEYFRAME_HPP

#include "semantic_SLAM/DetectedObjects.h"
#include "sensor_msgs/PointCloud2.h"
#include <boost/optional.hpp>
#include <g2o/types/slam3d/vertex_se3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

namespace g2o {
class VertexSE3;
}

namespace ps_graph_slam {

/**
 * @brief KeyFrame (pose node)
 */
struct KeyFrame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PointT = pcl::PointXYZRGB;
  using Ptr = std::shared_ptr<KeyFrame>;

  KeyFrame(const ros::Time &stamp, const Eigen::Isometry3d &odom,
           const Eigen::Isometry3d &robot_pose, const Eigen::MatrixXf &odom_cov,
           double accum_distance, const sensor_msgs::PointCloud2 &cloud_msg,
           const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
           std::vector<semantic_SLAM::ObjectInfo> &obj_info);
  ~KeyFrame();

  void dump(const std::string &directory);

public:
  ros::Time stamp;        // timestamp
  Eigen::Isometry3d odom; // odometry (estimated by scan_matching_odometry)
  Eigen::Isometry3d robot_pose; // corrected pose of the robot
  Eigen::MatrixXf odom_cov;     // odometry covariance
  double accum_distance;        // accumulated distance from the first node (by
                                // scan_matching_odometry)
  const sensor_msgs::PointCloud2 cloud_msg;           // point cloud ros msg
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud; // pcl point cloud
  std::vector<semantic_SLAM::ObjectInfo> obj_info;    // the detected b

  g2o::VertexSE3 *node; // node instance
};

/**
 * @brief KeyFramesnapshot for map cloud generation
 */
struct KeyFrameSnapshot {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using PointT = KeyFrame::PointT;
  using Ptr = std::shared_ptr<KeyFrameSnapshot>;

  KeyFrameSnapshot(const KeyFrame::Ptr &key);
  KeyFrameSnapshot(const Eigen::Isometry3d &pose,
                   const pcl::PointCloud<PointT>::ConstPtr &cloud);

  ~KeyFrameSnapshot();

public:
  Eigen::Isometry3d pose; // pose estimated by graph optimization
  pcl::PointCloud<PointT>::ConstPtr cloud; // point cloud
};

} // namespace ps_graph_slam

#endif // KEYFRAME_HPP
