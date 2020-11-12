#ifndef ROS_UTILS_HPP
#define ROS_UTILS_HPP

#include <Eigen/Dense>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>

namespace ps_graph_slam {

/**
 * @brief convert Eigen::Matrix to geometry_msgs::TransformStamped
 * @param stamp            timestamp
 * @param pose             Eigen::Matrix to be converted
 * @param frame_id         tf frame_id
 * @param child_frame_id   tf child frame_id
 * @return converted TransformStamped
 */
static geometry_msgs::TransformStamped
matrix2transform(const ros::Time &stamp, const Eigen::Matrix4f &pose,
                 const std::string &frame_id,
                 const std::string &child_frame_id) {
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

static geometry_msgs::Pose matrix2pose(const ros::Time &stamp,
                                       const Eigen::Matrix4f &pose,
                                       std::string frame_id) {
  Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
  quat.normalize();
  geometry_msgs::Quaternion odom_quat;
  odom_quat.w = quat.w();
  odom_quat.x = quat.x();
  odom_quat.y = quat.y();
  odom_quat.z = quat.z();

  geometry_msgs::Pose odom_pose;

  odom_pose.position.x = pose(0, 3);
  odom_pose.position.y = pose(1, 3);
  odom_pose.position.z = pose(2, 3);
  odom_pose.orientation = odom_quat;

  return odom_pose;
}

static geometry_msgs::PoseStamped
matrix2posestamped(const ros::Time &stamp, const Eigen::Matrix4f &pose,
                   std::string frame_id) {
  Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
  quat.normalize();
  geometry_msgs::Quaternion odom_quat;
  odom_quat.w = quat.w();
  odom_quat.x = quat.x();
  odom_quat.y = quat.y();
  odom_quat.z = quat.z();

  geometry_msgs::PoseStamped odom_pose;
  odom_pose.header.stamp = stamp;
  odom_pose.header.frame_id = frame_id;

  odom_pose.pose.position.x = pose(0, 3);
  odom_pose.pose.position.y = pose(1, 3);
  odom_pose.pose.position.z = pose(2, 3);
  odom_pose.pose.orientation = odom_quat;

  return odom_pose;
}

static Eigen::VectorXf matrix2vector(const Eigen::Matrix4f &pose) {
  Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
  quat.normalize();

  // converting quaternions to euler angles
  tf::Quaternion q(quat.x(), quat.y(), quat.z(), quat.w());
  tf::Matrix3x3 m(q);

  double yaw, pitch, roll;
  m.getEulerYPR(yaw, pitch, roll);

  Eigen::VectorXf robot_pose;
  robot_pose.resize(6, 6);
  robot_pose << pose(0, 3), pose(1, 3), pose(2, 3), roll, pitch, yaw;

  return robot_pose;
}

static Eigen::Isometry3d
odom2isometry(const nav_msgs::OdometryConstPtr &odom_msg) {
  const auto &orientation = odom_msg->pose.pose.orientation;
  const auto &position = odom_msg->pose.pose.position;

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

static Eigen::Isometry3d
pose2isometry(const geometry_msgs::PoseStamped &pose_msg) {
  const auto &orientation = pose_msg.pose.orientation;
  const auto &position = pose_msg.pose.position;

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

static nav_msgs::OdometryPtr
PoseCam2Robot(const nav_msgs::Odometry::ConstPtr odom_msg) {

  // rotation of -90
  Eigen::Matrix3f rot_x_robot, rot_z_robot, transformation_mat;
  rot_x_robot << 1, 0, 0, 0, cos(-1.5708), -sin(-1.5708), 0, sin(-1.5708),
      cos(-1.5708);

  rot_z_robot << cos(-1.5708), -sin(-1.5708), 0, sin(-1.5708), cos(-1.5708), 0,
      0, 0, 1;

  transformation_mat = rot_z_robot * rot_x_robot;

  // converting quaternions to euler angles
  tf::Quaternion quat_cam(
      odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
      odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(quat_cam);

  double yaw, pitch, roll;
  m.getEulerYPR(yaw, pitch, roll);

  Eigen::Vector3f angle_cam, angle_robot;
  angle_cam << roll, pitch, yaw;

  angle_robot = transformation_mat * angle_cam;

  tf::Quaternion quat_robot = tf::createQuaternionFromRPY(
      angle_robot(0), angle_robot(1), angle_robot(2));

  // converting the translations
  Eigen::Vector3f trans_cam, trans_robot;
  trans_cam << odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
      odom_msg->pose.pose.position.z;

  trans_robot = transformation_mat * trans_cam;

  nav_msgs::OdometryPtr odom_converted(new nav_msgs::Odometry());
  odom_converted->pose.pose.position.x = trans_robot(0);
  odom_converted->pose.pose.position.y = trans_robot(1);
  odom_converted->pose.pose.position.z = trans_robot(2);
  odom_converted->pose.pose.orientation.x = quat_robot.x();
  odom_converted->pose.pose.orientation.y = quat_robot.y();
  odom_converted->pose.pose.orientation.z = quat_robot.z();
  odom_converted->pose.pose.orientation.w = quat_robot.w();

  return odom_converted;
}

static geometry_msgs::PoseStamped
poseNED2ENU(const geometry_msgs::PoseStamped &pose_msg) {

  // converting quaternions to euler angles
  tf::Quaternion quat_ned(
      pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
      pose_msg.pose.orientation.z, pose_msg.pose.orientation.w);
  tf::Matrix3x3 m(quat_ned);

  double yaw, pitch, roll;
  m.getEulerYPR(yaw, pitch, roll);

  Eigen::Matrix3f trans_mat;
  trans_mat << 1, 0, 0, 0, cos(-3.14), sin(-3.14), 0, sin(-3.14), cos(-3.14);

  Eigen::Vector3f angle_ned;
  angle_ned << roll, pitch, yaw;

  Eigen::Vector3f angle_enu = trans_mat * angle_ned;
  tf::Quaternion quat_enu =
      tf::createQuaternionFromRPY(angle_enu(0), angle_enu(1), angle_enu(2));

  // converting the translations
  Eigen::Vector3f trans_ned;
  trans_ned << pose_msg.pose.position.x, pose_msg.pose.position.y,
      pose_msg.pose.position.z;

  Eigen::Vector3f trans_enu = trans_mat * trans_ned;

  geometry_msgs::PoseStamped pose_converted;
  pose_converted.pose.position.x = trans_enu(0);
  pose_converted.pose.position.y = trans_enu(1);
  pose_converted.pose.position.z = trans_enu(2);
  pose_converted.pose.orientation.x = quat_enu.x();
  pose_converted.pose.orientation.y = quat_enu.y();
  pose_converted.pose.orientation.z = quat_enu.z();
  pose_converted.pose.orientation.w = quat_enu.w();

  return pose_converted;
}

static nav_msgs::OdometryPtr
RotPoseZ(const nav_msgs::OdometryConstPtr &odom_msg, float first_yaw) {
  // converting quaternions to euler angles
  tf::Quaternion quat(
      odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
      odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(quat);

  double yaw, pitch, roll;
  m.getEulerYPR(yaw, pitch, roll);

  Eigen::Vector3f angle;
  angle << roll, pitch, (yaw - first_yaw);

  Eigen::Matrix3f trans_mat;
  trans_mat << cos(1.57), -sin(1.57), 0, sin(1.57), cos(1.57), 0, 0, 0, 1;

  Eigen::Vector3f angle_enu = trans_mat * angle;
  tf::Quaternion quat_enu =
      tf::createQuaternionFromRPY(angle_enu(0), angle_enu(1), angle_enu(2));

  // converting the translations
  Eigen::Vector3f trans;
  trans << odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
      odom_msg->pose.pose.position.z;

  Eigen::Vector3f trans_enu = trans_mat * trans;

  nav_msgs::OdometryPtr odom_conv_msg(new nav_msgs::Odometry());
  odom_conv_msg->pose.pose.position.x = trans_enu(0);
  odom_conv_msg->pose.pose.position.y = trans_enu(1);
  odom_conv_msg->pose.pose.position.z = trans_enu(2);
  odom_conv_msg->pose.pose.orientation.x = quat_enu.x();
  odom_conv_msg->pose.pose.orientation.y = quat_enu.y();
  odom_conv_msg->pose.pose.orientation.z = quat_enu.z();
  odom_conv_msg->pose.pose.orientation.w = quat_enu.w();

  return odom_conv_msg;
}

static nav_msgs::OdometryPtr
navMsgsToOrigin(const nav_msgs::OdometryConstPtr &odom_msg, float transform_x,
                float transform_y, float transform_z) {
  nav_msgs::OdometryPtr odom_to_origin_msg(new nav_msgs::Odometry());
  odom_to_origin_msg->pose.pose.position.x =
      odom_msg->pose.pose.position.x - transform_x;
  odom_to_origin_msg->pose.pose.position.y =
      odom_msg->pose.pose.position.y - transform_y;
  odom_to_origin_msg->pose.pose.position.z =
      odom_msg->pose.pose.position.z - transform_z;
  odom_to_origin_msg->pose.pose.orientation = odom_msg->pose.pose.orientation;

  return odom_to_origin_msg;
}

static Eigen::MatrixXf
arrayToMatrix(const nav_msgs::OdometryConstPtr &odom_msg) {

  Eigen::MatrixXf odom_cov;
  odom_cov.resize(6, 6);
  float id = 0;
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      odom_cov(i, j) = odom_msg->pose.covariance[id];
      id++;
    }
  }

  return odom_cov;
}

} // namespace ps_graph_slam

#endif // ROS_UTILS_HPP
