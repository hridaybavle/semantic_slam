#ifndef ROS_UTILS_HPP
#define ROS_UTILS_HPP

#include <Eigen/Dense>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf_conversions/tf_eigen.h>

namespace hdl_graph_slam {

/**
 * @brief convert Eigen::Matrix to geometry_msgs::TransformStamped
 * @param stamp            timestamp
 * @param pose             Eigen::Matrix to be converted
 * @param frame_id         tf frame_id
 * @param child_frame_id   tf child frame_id
 * @return converted TransformStamped
 */
static geometry_msgs::TransformStamped matrix2transform(const ros::Time& stamp, const Eigen::Matrix4f& pose, const std::string& frame_id, const std::string& child_frame_id) {
    Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
    quat.normalize();
    geometry_msgs::Quaternion odom_quat;
    odom_quat.w = quat.w();
    odom_quat.x = quat.x();
    odom_quat.y = quat.y();
    odom_quat.z = quat.z();

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = stamp;
    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id = child_frame_id;

    odom_trans.transform.translation.x = pose(0, 3);
    odom_trans.transform.translation.y = pose(1, 3);
    odom_trans.transform.translation.z = pose(2, 3);
    odom_trans.transform.rotation = odom_quat;

    return odom_trans;
}

static geometry_msgs::Pose matrix2pose(const ros::Time& stamp, const Eigen::Matrix4f& pose, std::string frame_id) {
    Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
    quat.normalize();
    geometry_msgs::Quaternion odom_quat;
    odom_quat.w = quat.w();
    odom_quat.x = quat.x();
    odom_quat.y = quat.y();
    odom_quat.z = quat.z();

    geometry_msgs::Pose odom_pose;

    odom_pose.position.x  = pose(0, 3);
    odom_pose.position.y  = pose(1, 3);
    odom_pose.position.z  = pose(2, 3);
    odom_pose.orientation = odom_quat;

    return odom_pose;
}


static Eigen::VectorXf matrix2vector(const Eigen::Matrix4f& pose) {
    Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
    quat.normalize();

    //converting quaternions to euler angles
    tf::Quaternion q(quat.x(), quat.y(), quat.z(), quat.w());
    tf::Matrix3x3 m(q);

    double yaw, pitch, roll;
    m.getEulerYPR(yaw, pitch, roll);

    Eigen::VectorXf robot_pose;
    robot_pose.resize(6,6);
    robot_pose << pose(0, 3), pose(1, 3), pose(2, 3), roll, pitch, yaw;

    return robot_pose;
}

static Eigen::Isometry3d odom2isometry(const nav_msgs::OdometryConstPtr& odom_msg) {
    const auto& orientation = odom_msg->pose.pose.orientation;
    const auto& position = odom_msg->pose.pose.position;

    Eigen::Quaterniond quat;
    quat.w() = orientation.w;
    quat.x() = orientation.x;
    quat.y() = orientation.y;
    quat.z() = orientation.z;

    Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
    isometry.linear() = quat.toRotationMatrix();
    isometry.translation() = Eigen::Vector3d(position.x, position.y, position.z);
    return isometry;
}

static Eigen::MatrixXf arrayToMatrix(const nav_msgs::OdometryConstPtr& odom_msg) {

    Eigen::MatrixXf odom_cov; odom_cov.resize(6,6);
    float id =0;
    for(int i = 0; i < 6; ++i)
    {
        for(int j= 0; j < 6; ++j)
        {
            odom_cov(i,j) = odom_msg->pose.covariance[id];
            id++;
        }
    }

    return odom_cov;

}


}

#endif // ROS_UTILS_HPP
