#include "ps_graph_slam/semantic_graph_slam.h"

semantic_graph_slam::semantic_graph_slam() {
  std::cout << "semantic graph slam constructor " << std::endl;
}

semantic_graph_slam::~semantic_graph_slam() {
  std::cout << "semantic graph slam destructor " << std::endl;
}

void semantic_graph_slam::init(bool verbose) {
  verbose_ = verbose;
  // default values
  object_detection_available_ = false;
  point_cloud_available_ = false;
  first_key_added_ = false;
  update_keyframes_using_detections_ = false;
  max_keyframes_per_update_ = 10;
  new_keyframes_.clear();
  seg_obj_vec_.clear();

  ros::param::param<bool>("~update_key_using_det",
                          update_keyframes_using_detections_, false);
  ros::param::param<double>("~camera_angle", cam_angled_, 0);
  ros::param::param<bool>("~add_first_lan", add_first_lan_, false);
  ros::param::param<double>("~first_lan_x", first_lan_x_, 1.8);
  ros::param::param<double>("~first_lan_y", first_lan_y_, 0);
  ros::param::param<double>("~first_lan_z", first_lan_z_, 0.3);
  cam_angle_ = static_cast<double>(cam_angled_) * (M_PI / 180);

  pc_seg_obj_.reset(new point_cloud_segmentation(verbose_));
  data_ass_obj_.reset(new data_association(verbose_));
  graph_slam_.reset(new ps_graph_slam::GraphSLAM(verbose_));
  keyframe_updater_.reset(new ps_graph_slam::KeyframeUpdater());
  inf_calclator_.reset(new ps_graph_slam::InformationMatrixCalculator());
  // semantic_mapping_obj_ = new mapping(cam_angle_);
  // semantic_mapping_th_ = new
  // std::thread(&mapping::generateMap,semantic_mapping_obj_);
  // semantic_mapping_opt_th_ = new
  // std::thread(&mapping::opitmizeMap,semantic_mapping_obj_);

  trans_odom2map_.setIdentity();
  landmarks_vec_.clear();
  robot_pose_.setIdentity();
  vio_pose_.setIdentity();
  prev_odom_.setIdentity();
  map2odom_.setIdentity();
  
  std::cout << "camera angle in radians: " << cam_angle_ << std::endl;
  std::cout << "update keyframe every detection: "
            << update_keyframes_using_detections_ << std::endl;
  std::cout << "add first landmark: " << add_first_lan_ << std::endl;

  if (add_first_lan_)
    this->addFirstPoseAndLandmark();
}

bool semantic_graph_slam::run() {
  // add keyframe nodes to graph if keyframes available
  if (empty_keyframe_queue()) {
    // add semantic data nodes to graph if available
    for (int i = 0; i < new_keyframes_.size(); ++i) {
      if (!new_keyframes_[i]->obj_info.empty()) {
        // segmenting and matching keyframes
        std::vector<landmark> current_landmarks_vec =
            this->semantic_data_ass(new_keyframes_[i]);
        // add the segmented landmarks to the graph for the current keyframe
        this->empty_landmark_queue(current_landmarks_vec, new_keyframes_[i]);
      }
    }

    // pass the keyframe to the mapping module
    // semantic_mapping_obj_->setKeyframes(new_keyframes_);

    // graph slam opitimization
    std::copy(new_keyframes_.begin(), new_keyframes_.end(),
              std::back_inserter(keyframes_));
    new_keyframes_.clear();

    // optimizing the graph
    if (graph_slam_->optimize()) {
      if (verbose_)
        std::cout << "optimizing the graph " << std::endl;

      // give the optimized keypoints to the mapping
      // semantic_mapping_obj_->setoptimizedKeyframes(keyframes_);

      // get and set the landmark covariances
      this->getAndSetLandmarkCov();

      // getting the optimized pose
      const auto &keyframe = keyframes_.back();

      robot_pose_ = keyframe->node->estimate();
      map2odom_   = keyframe->node->estimate() * keyframe->odom.inverse();
    }

    first_key_added_ = true;
    return true;
  } else
    return false;
}

bool semantic_graph_slam::empty_keyframe_queue() {
  std::lock_guard<std::mutex> lock(keyframe_queue_mutex);

  if (keyframe_queue_.empty()) {
    return false;
  }

  int num_processed = 0;
  for (int i = 0;
       i < std::min<int>(keyframe_queue_.size(), max_keyframes_per_update_);
       i++) {
    num_processed = i;

    const auto &keyframe = keyframe_queue_[i];
    new_keyframes_.push_back(keyframe);

    Eigen::Isometry3d odom = keyframe->odom;
    keyframe->node = graph_slam_->add_se3_node(odom);
    // keyframe_hash_[keyframe->stamp] = keyframe;
    if (verbose_)
      std::cout << "added new keyframe to the graph" << std::endl;

    if (i == 0 && keyframes_.empty()) {
      continue;
    }

    // add edge between keyframes
    const auto &prev_keyframe =
        i == 0 ? keyframes_.back() : keyframe_queue_[i - 1];

    Eigen::Isometry3d relative_pose =
        prev_keyframe->odom.inverse() * keyframe->odom;
    Eigen::MatrixXd information =
        inf_calclator_
            ->calc_information_matrix(); /*keyframe->odom_cov.inverse().cast<double>();
                                          */
    graph_slam_->add_se3_edge(prev_keyframe->node, keyframe->node,
                              relative_pose, information);
    if (verbose_)
      std::cout << "added new odom measurement to the graph" << std::endl;
  }

  keyframe_queue_.erase(keyframe_queue_.begin(),
                        keyframe_queue_.begin() + num_processed + 1);

  return true;
}

void semantic_graph_slam::empty_landmark_queue(
    std::vector<landmark> current_lan_queue, const auto current_keyframe) {
  if (current_lan_queue.empty())
    return;

  for (int i = 0; i < current_lan_queue.size(); ++i) {
    // adding new landmark vertex if only the landmark is seen once
    if (current_lan_queue[i].is_new_landmark) {
      current_lan_queue[i].node = graph_slam_->add_point_xyz_node(
          current_lan_queue[i].pose.cast<double>());
      current_lan_queue[i].is_new_landmark = false;
      data_ass_obj_->assignLandmarkNode(current_lan_queue[i].id,
                                        current_lan_queue[i].node);
      if (verbose_)
        std::cout << "added the landmark position node " << std::endl;
    }

    // add an edge between landmark and the current keyframe
    Eigen::Matrix3f information = current_lan_queue[i].covariance.inverse();
    graph_slam_->add_se3_point_xyz_edge(
        current_keyframe->node, current_lan_queue[i].node,
        current_lan_queue[i].local_pose.cast<double>(),
        information.cast<double>());
    if (verbose_)
      std::cout << "added an edge between the landmark and it keyframe "
                << std::endl;
  }
}

void semantic_graph_slam::getAndSetLandmarkCov() {
  std::vector<landmark> l;
  data_ass_obj_->getMappedLandmarks(l);
  g2o::SparseBlockMatrix<Eigen::MatrixXd> lan_spinv_vec;

  std::vector<std::pair<int, int>> vert_pairs_vec;
  for (int i = 0; i < l.size(); ++i) {
    l[i].node->unlockQuadraticForm();
    vert_pairs_vec.push_back(
        std::make_pair(l[i].node->hessianIndex(), l[i].node->hessianIndex()));
  }

  if (graph_slam_->computeLandmarkMarginals((lan_spinv_vec), vert_pairs_vec)) {
    for (int i = 0; i < l.size(); ++i) {
      // std::cout << "landmark spinv block " <<
      // lan_spinv_vec.block(l[i].node->hessianIndex(),l[i].node->hessianIndex())->eval()
      // << std::endl;
      data_ass_obj_->setLandmarkCovs(
          i, lan_spinv_vec
                 .block(l[i].node->hessianIndex(), l[i].node->hessianIndex())
                 ->eval()
                 .cast<float>());
    }
  }
}

std::vector<landmark> semantic_graph_slam::semantic_data_ass(
    const ps_graph_slam::KeyFrame::Ptr curr_keyframe) {
  std::vector<semantic_SLAM::ObjectInfo> object_info = curr_keyframe->obj_info;
  sensor_msgs::PointCloud2 point_cloud_msg = curr_keyframe->cloud_msg;
  Eigen::VectorXf current_robot_pose = ps_graph_slam::matrix2vector(
      curr_keyframe->robot_pose.matrix().cast<float>());
  if (verbose_)
    std::cout << "current robot pose " << current_robot_pose << std::endl;

  std::vector<detected_object> seg_obj_vec =
      pc_seg_obj_->segmentallPointCloudData(current_robot_pose, cam_angle_,
                                            object_info, point_cloud_msg);
  if (verbose_)
    std::cout << "seg_obj_vec size " << seg_obj_vec.size() << std::endl;

  std::vector<landmark> current_landmarks_vec =
      data_ass_obj_->find_matches(seg_obj_vec, current_robot_pose, cam_angle_);

  if (verbose_)
    std::cout << "current_landmarks_vec size " << current_landmarks_vec.size()
              << std::endl;
  // this->publishDetectedLandmarks(current_robot_pose, seg_obj_vec);
  this->setDetectedObjectsPose(seg_obj_vec);

  return current_landmarks_vec;
}

void semantic_graph_slam::VIOCallback(const ros::Time &stamp,
                                      Eigen::Isometry3d odom,
                                      Eigen::MatrixXf odom_cov) {
  // dont update keyframes if the keyframe time and distance is less or no
  // detection is available
  if (update_keyframes_using_detections_) {
    if (!keyframe_updater_->update(odom, stamp) &&
        !object_detection_available_) {
      if (first_key_added_) {
        Eigen::Isometry3d pose_inc = prev_odom_.inverse() * odom;
        robot_pose_ = robot_pose_ * pose_inc;
      }

      this->setVIOPose(odom);
      prev_odom_ = odom;
      return;
    }
  } else {
    if (!keyframe_updater_->update(odom, stamp)) {
      if (first_key_added_) {
        Eigen::Isometry3d pose_inc = prev_odom_.inverse() * odom;
        robot_pose_ = robot_pose_ * pose_inc;
      }

      this->setVIOPose(odom);
      prev_odom_ = odom;
      return;
    }
  }

  sensor_msgs::PointCloud2 cloud_msg;
  this->getPointCloudData(cloud_msg);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());

  std::vector<semantic_SLAM::ObjectInfo> obj_info;
  obj_info.clear();
  if (object_detection_available_)
    this->getDetectedObjectInfo(obj_info);

  double accum_d = keyframe_updater_->get_accum_distance();
  ps_graph_slam::KeyFrame::Ptr keyframe(new ps_graph_slam::KeyFrame(
      stamp, odom, robot_pose_, odom_cov, accum_d, cloud_msg, cloud, obj_info));

  std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
  keyframe_queue_.push_back(keyframe);
  if (verbose_)
    std::cout << "added keyframe in queue" << std::endl;

  this->setVIOPose(odom);
  prev_odom_ = odom;

  return;
}

void semantic_graph_slam::addFirstPoseAndLandmark() {
  std::vector<landmark> first_lan_vec;
  landmark first_landmark;
  first_landmark.is_new_landmark = true;
  first_landmark.id = 0;
  first_landmark.type = "bucket";
  first_landmark.plane_type = "vertical";
  first_landmark.pose << first_lan_x_, first_lan_y_, first_lan_z_;
  first_landmark.local_pose = first_landmark.pose;
  first_landmark.normal_orientation << -0.4, 0.86, 0, 0;
  first_landmark.covariance << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;

  first_lan_vec.push_back(first_landmark);
  data_ass_obj_->addFirstLandmark(first_landmark);

  Eigen::Isometry3d odom;
  odom.setIdentity();
  Eigen::MatrixXf odom_cov;
  odom_cov.setIdentity(6, 6);
  double accum_d = 0;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());
  std::vector<semantic_SLAM::ObjectInfo> obj_info;

  ps_graph_slam::KeyFrame::Ptr keyframe(
      new ps_graph_slam::KeyFrame(ros::Time::now(), odom, robot_pose_, odom_cov,
                                  accum_d, cloud_msg, cloud, obj_info));
  keyframe_queue_.push_back(keyframe);

  if (empty_keyframe_queue()) {
    for (int i = 0; i < new_keyframes_.size(); ++i)
      empty_landmark_queue(first_lan_vec, new_keyframes_[i]);

    std::copy(new_keyframes_.begin(), new_keyframes_.end(),
              std::back_inserter(keyframes_));
    new_keyframes_.clear();
  }

  if (verbose_)
    std::cout << "add the first landmark and keyframe pose " << std::endl;

  return;
}

void semantic_graph_slam::setVIOPose(Eigen::Isometry3d vio_pose) {
  vio_pose_ = vio_pose;
}

void semantic_graph_slam::getVIOPose(Eigen::Isometry3d &vio_pose) {
  vio_pose = vio_pose_;
}

void semantic_graph_slam::setPointCloudData(
    sensor_msgs::PointCloud2 point_cloud) {
  point_cloud_available_ = true;
  this->point_cloud_msg_ = point_cloud;
}

void semantic_graph_slam::getPointCloudData(
    sensor_msgs::PointCloud2 &point_cloud) {
  point_cloud_available_ = false;
  point_cloud = this->point_cloud_msg_;
}

void semantic_graph_slam::setDetectedObjectInfo(
    std::vector<semantic_SLAM::ObjectInfo> object_info) {
  object_detection_available_ = true;
  this->object_info_ = object_info;
}

void semantic_graph_slam::getDetectedObjectInfo(
    std::vector<semantic_SLAM::ObjectInfo> &object_info) {
  object_detection_available_ = false;
  object_info = this->object_info_;
}

void semantic_graph_slam::getMappedLandmarks(std::vector<landmark> &l_vec) {
  data_ass_obj_->getMappedLandmarks(l_vec);
}

void semantic_graph_slam::getKeyframes(
    std::vector<ps_graph_slam::KeyFrame::Ptr> &keyframes) {
  keyframes = keyframes_;
}

void semantic_graph_slam::getRobotPose(Eigen::Isometry3d &robot_pose) {
  robot_pose = robot_pose_;
}

void semantic_graph_slam::getMap2OdomTrans(Eigen::Isometry3d &map2odom){
  map2odom = map2odom_;
}

void semantic_graph_slam::setDetectedObjectsPose(
    std::vector<detected_object> seg_obj_vec) {
  seg_obj_vec_ = seg_obj_vec;
}

void semantic_graph_slam::getDetectedObjectsPose(
    std::vector<detected_object> &seg_obj_vec) {
  seg_obj_vec = seg_obj_vec_;
}

void semantic_graph_slam::saveGraph(std::string save_graph_path) {
  graph_slam_->save(save_graph_path);
  std::cout << "saved the graph at " << save_graph_path << std::endl;
}
