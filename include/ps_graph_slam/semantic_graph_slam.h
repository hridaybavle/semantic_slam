#include "ros/ros.h"

// ps graph slam headers
#include <ps_graph_slam/ros_time_hash.hpp>
#include <ps_graph_slam/ros_utils.hpp>

#include <ps_graph_slam/graph_slam.hpp>
#include <ps_graph_slam/information_matrix_calculator.hpp>
#include <ps_graph_slam/keyframe.hpp>
#include <ps_graph_slam/keyframe_updater.hpp>

// landmarks
#include "ps_graph_slam/landmark.h"

// data association
#include "ps_graph_slam/data_association.h"

// segmented pointcloud acc to detection
#include "planar_segmentation/point_cloud_segmentation.h"

class semantic_graph_slam {
public:
  semantic_graph_slam();
  ~semantic_graph_slam();

public:
  bool run();
  void init(bool verbose);

private:
  bool verbose_;

private:
  std::unique_ptr<point_cloud_segmentation> pc_seg_obj_;
  std::unique_ptr<data_association> data_ass_obj_;

private:
  bool empty_keyframe_queue();
  int max_keyframes_per_update_;

  std::mutex trans_odom2map_mutex;
  Eigen::Matrix4f trans_odom2map_;

  // odom related
protected:
  Eigen::Isometry3d prev_odom_, vio_pose_;
  bool first_key_added_;

public:
  void VIOCallback(const ros::Time &stamp, Eigen::Isometry3d odom,
                   Eigen::MatrixXf odom_cov);
  void setVIOPose(Eigen::Isometry3d vio_pose);
  void getVIOPose(Eigen::Isometry3d &vio_pose);

protected:
  // robot pose related
  Eigen::Isometry3d robot_pose_;
  Eigen::Isometry3d map2odom_;
  double cam_angled_;
  float cam_angle_;
  bool add_first_lan_;
  double first_lan_x_, first_lan_y_, first_lan_z_;

public:
  void getRobotPose(Eigen::Isometry3d &robot_pose) ;
  void getMap2OdomTrans(Eigen::Isometry3d &map2odom);

private:
  // landmark related functions
  std::vector<landmark>
  semantic_data_ass(const ps_graph_slam::KeyFrame::Ptr curr_keyframe);

  void empty_landmark_queue(std::vector<landmark> current_lan_queue,
                            const auto current_keyframe);

  void getAndSetLandmarkCov();
  void addFirstPoseAndLandmark();

  std::vector<detected_object> seg_obj_vec_;
  void setDetectedObjectsPose(std::vector<detected_object> seg_obj_vec);

public:
  void getMappedLandmarks(std::vector<landmark> &l_vec);
  void getDetectedObjectsPose(std::vector<detected_object> &seg_obj_vec);

public:
  void getKeyframes(std::vector<ps_graph_slam::KeyFrame::Ptr> &keyframes);

private:
  // point cloud related
  sensor_msgs::PointCloud2 point_cloud_msg_;
  bool point_cloud_available_;
  ros::Time pc_stamp_;

public:
  void setPointCloudData(sensor_msgs::PointCloud2 point_cloud);
  void getPointCloudData(sensor_msgs::PointCloud2 &point_cloud);

private:
  // detection related
  bool update_keyframes_using_detections_;
  bool object_detection_available_;
  std::vector<semantic_SLAM::ObjectInfo> object_info_;

public:
  void
  setDetectedObjectInfo(std::vector<semantic_SLAM::ObjectInfo> object_info);
  void
  getDetectedObjectInfo(std::vector<semantic_SLAM::ObjectInfo> &object_info);

  // graph related
public:
  void saveGraph(std::string save_graph_path);

private:
  // keyframe related params
  std::deque<ps_graph_slam::KeyFrame::Ptr> new_keyframes_;
  std::vector<ps_graph_slam::KeyFrame::Ptr> keyframes_;
  std::unordered_map<ros::Time, ps_graph_slam::KeyFrame::Ptr, RosTimeHash>
      keyframe_hash_;
  std::deque<ps_graph_slam::KeyFrame::Ptr> keyframe_queue_;

  // landmark relared params
  std::vector<landmark> landmarks_vec_;

  std::mutex keyframe_queue_mutex;

  std::unique_ptr<ps_graph_slam::GraphSLAM> graph_slam_;
  std::unique_ptr<ps_graph_slam::KeyframeUpdater> keyframe_updater_;
  std::unique_ptr<ps_graph_slam::InformationMatrixCalculator> inf_calclator_;
};
