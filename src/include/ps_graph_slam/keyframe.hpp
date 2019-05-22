#ifndef KEYFRAME_HPP
#define KEYFRAME_HPP

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/optional.hpp>
#include "semantic_SLAM/DetectedObjects.h"
#include "sensor_msgs/PointCloud2.h"
#include <g2o/types/slam3d/vertex_se3.h>

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
    using PointT = pcl::PointXYZI;
    using Ptr = std::shared_ptr<KeyFrame>;

    KeyFrame(const ros::Time& stamp,
             const Eigen::Isometry3d& odom,
             const Eigen::Isometry3d& robot_pose,
             const Eigen::MatrixXf& odom_cov,
             double accum_distance,
             const sensor_msgs::PointCloud2& cloud_msg,
             std::vector<semantic_SLAM::ObjectInfo>& obj_info);
    ~KeyFrame();

    void dump(const std::string& directory);

public:
    ros::Time stamp;                                // timestamp
    Eigen::Isometry3d odom;                         // odometry (estimated by scan_matching_odometry)
    Eigen::Isometry3d robot_pose;                    // corrected pose of the robot
    Eigen::MatrixXf odom_cov;                       // odometry covariance
    double accum_distance;                          // accumulated distance from the first node (by scan_matching_odometry)
    const sensor_msgs::PointCloud2 cloud_msg;       // point cloud ros msg
    boost::optional<Eigen::Vector4d> floor_coeffs;  // detected floor's coefficients
    boost::optional<Eigen::Vector3d> utm_coord;     // UTM coord obtained by GPS
    std::vector<semantic_SLAM::ObjectInfo> obj_info;// the detected bb

    g2o::VertexSE3* node;                           // node instance
};

/**
 * @brief KeyFramesnapshot for map cloud generation
 */
struct KeyFrameSnapshot {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using PointT = KeyFrame::PointT;
    using Ptr = std::shared_ptr<KeyFrameSnapshot>;

    KeyFrameSnapshot(const KeyFrame::Ptr& key);
    KeyFrameSnapshot(const Eigen::Isometry3d& pose, const pcl::PointCloud<PointT>::ConstPtr& cloud);

    ~KeyFrameSnapshot();

public:
    Eigen::Isometry3d pose;                   // pose estimated by graph optimization
    pcl::PointCloud<PointT>::ConstPtr cloud;  // point cloud
};

}

#endif // KEYFRAME_HPP
