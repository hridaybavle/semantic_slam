#ifndef KEYFRAME_UPDATER_HPP
#define KEYFRAME_UPDATER_HPP

#include <Eigen/Dense>
#include <ros/ros.h>

namespace ps_graph_slam {

/**
 * @brief this class decides if a new frame should be registered to the pose
 * graph as a keyframe
 */
class KeyframeUpdater {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief constructor
   * @param pnh
   */
  KeyframeUpdater()
      : is_first(true), prev_keypose(Eigen::Isometry3d::Identity()) {
    ros::param::param<double>("~keyframe_delta_trans", keyframe_delta_trans,
                              0.5);
    ros::param::param<double>("~keyframe_delta_angle", keyframe_delta_angle,
                              0.5);
    ros::param::param<double>("~keyframe_delta_time", keyframe_delta_time, 1);

    std::cout << "keyframe_delta_trans " << keyframe_delta_trans << std::endl;
    std::cout << "keyframe_delta_angle " << keyframe_delta_angle << std::endl;
    std::cout << "keyframe_delta_time " << keyframe_delta_time << std::endl;

    accum_distance = 0.0;
  }

  /**
   * @brief decide if a new frame should be registered to the graph
   * @param pose  pose of the frame
   * @return  if true, the frame should be registered
   */
  bool update(const Eigen::Isometry3d &pose, ros::Time current_time) {
    // first frame is always registered to the graph
    if (is_first) {
      is_first = false;
      prev_time = current_time;
      prev_keypose = pose;
      return true;
    }

    // calculate the delta transformation from the previous keyframe
    Eigen::Isometry3d delta = prev_keypose.inverse() * pose;
    double dx = delta.translation().norm();
    double da = std::acos(Eigen::Quaterniond(delta.linear()).w());

    // updating based on time
    if ((current_time - prev_time).sec < keyframe_delta_time &&
        dx < keyframe_delta_trans && da < keyframe_delta_angle) {
      return false;
    }

    accum_distance += dx;
    prev_keypose = pose;
    prev_time = current_time;
    return true;
  }

  /**
   * @brief the last keyframe's accumulated distance from the first keyframe
   * @return accumulated distance
   */
  double get_accum_distance() const { return accum_distance; }

private:
  // parameters
  double keyframe_delta_trans; //
  double keyframe_delta_angle; //
  double keyframe_delta_time;

  bool is_first;
  double accum_distance;
  Eigen::Isometry3d prev_keypose;
  ros::Time prev_time;
};

} // namespace ps_graph_slam

#endif // KEYFRAME_UPDATOR_HPP
