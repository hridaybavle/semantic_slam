#include "semantic_graph_slam_ros.h"

semantic_graph_slam_ros::semantic_graph_slam_ros()
//    : sync(SyncPolicy(10))
{
  std::cout << "Intialializing semantic graph slam ros node " << std::endl;
}

semantic_graph_slam_ros::~semantic_graph_slam_ros() {
  std::cout << "Destructing semantic graph slam ros node " << std::endl;
}

void semantic_graph_slam_ros::init(ros::NodeHandle n) {
  counter_ = false;
  first_gt_pose_ = false;
  first_jack_pose_ = false;
  verbose_ = false;

  robot_pose_array_.poses.clear();
  vio_pose_array_.poses.clear();

  robot_pose_vec_.clear();
  vio_key_pose_vec_.clear();
  vio_pose_vec_.clear();
  orb_slam_pose_vec_.clear();

  jack_yaw_transform_ = -1.57;

  ros::param::param<bool>("~verbose", verbose_, false);
  ros::param::param<bool>("~save_graph", save_graph_, false);
  ros::param::param<std::string>("~save_graph_path", save_graph_path_,
                                 "semantic_graph.g2o");
  ros::param::param<bool>("~use_snap_pose", use_snap_pose_, true);
  ros::param::param<bool>("~use_orb_slam_odom", use_orb_slam_odom_, false);
  ros::param::param<bool>("~use_rovio_odom", use_rovio_odom_, true);
  ros::param::param<bool>("~use_rtab_map_odom", use_rtab_map_odom_, false);
  ros::param::param<bool>("~compute_txt_for_ate", compute_txt_for_ate_, false);

  std::cout << "should save graph: " << save_graph_ << std::endl;
  std::cout << "saving graph path: " << save_graph_path_ << std::endl;
  std::cout << "using snap pose: " << use_snap_pose_ << std::endl;
  std::cout << "using orb slam odom " << use_orb_slam_odom_ << std::endl;
  std::cout << "using rovio odom " << use_rovio_odom_ << std::endl;
  std::cout << "using rtab map odom " << use_rtab_map_odom_ << std::endl;
  std::cout << "computing txt for ate " << compute_txt_for_ate_ << std::endl;

  semantic_gslam_obj_.reset(new semantic_graph_slam());
  semantic_gslam_obj_->init(verbose_);
}

void semantic_graph_slam_ros::run() {
  if (semantic_gslam_obj_->run()) {
    if (use_rtab_map_odom_) this->transformListener();
    this->publishLandmarks();
    this->publishKeyframePoses();
    this->publishDetectedLandmarks();
  }

  this->publishCorresVIOPose();
  this->publishRobotPose();
  this->publishMap2Odom();
  return;
}

void semantic_graph_slam_ros::open(ros::NodeHandle n) {
  init(n);

  // subscribers
  rvio_odom_pose_sub_ = n.subscribe(
      "/rovio/odometry", 1, &semantic_graph_slam_ros::rovioVIOCallback, this);
  snap_odom_pose_sub_ =
      n.subscribe("/SQ04/snap_vislam/vislam/pose", 1,
                  &semantic_graph_slam_ros::snapVIOCallback, this);
  orb_slam_pose_sub_ = n.subscribe(
      "orb_slam/pose", 1, &semantic_graph_slam_ros::orbslamPoseCallback, this);
  jackal_odom_pose_sub_ =
      n.subscribe("/JA01/odometry/filtered", 1,
                  &semantic_graph_slam_ros::jackalOdomCallback, this);
  cloud_sub_ = n.subscribe("/depth_registered/points", 1,
                           &semantic_graph_slam_ros::PointCloudCallback, this);
  detected_object_sub_ = n.subscribe(
      "/darknet_ros/bounding_boxes", 1,
      &semantic_graph_slam_ros::detectedObjectDarknetCallback, this);
  simple_detected_object_sub_ =
      n.subscribe("/image_processed/bounding_boxes", 1,
                  &semantic_graph_slam_ros::detectedObjectSimpleCallback, this);
  optitrack_pose_sub_ =
      n.subscribe("/vrpn_client_node/realsense/pose", 1,
                  &semantic_graph_slam_ros::optitrackPoseCallback, this);
  vicon_pose_sub_ = n.subscribe(
      "/SQ04/vicon", 1, &semantic_graph_slam_ros::viconPoseSubCallback, this);

  // publishers
  keyframe_pose_pub_ =
      n.advertise<geometry_msgs::PoseArray>("keyframe_poses", 1);
  landmarks_pub_ =
      n.advertise<visualization_msgs::MarkerArray>("mapped_landmarks", 1);
  detected_lans_pub_ =
      n.advertise<visualization_msgs::MarkerArray>("detected_landmarks", 1);
  robot_pose_pub_ = n.advertise<geometry_msgs::PoseStamped>("robot_pose", 1);
  robot_transform_pub_ =
      n.advertise<geometry_msgs::TransformStamped>("robot_transform", 1);
  keyframe_path_pub_ = n.advertise<nav_msgs::Path>("robot_path", 1);
  optitrack_pose_pub_ =
      n.advertise<geometry_msgs::PoseStamped>("optitrack_pose", 1);
  optitrack_path_pub_ = n.advertise<nav_msgs::Path>("optitrack_path", 1);
  corres_vio_pose_pub_ =
      n.advertise<geometry_msgs::PoseStamped>("corres_vo_pose", 1);
  corres_vio_path_ = n.advertise<nav_msgs::Path>("corres_vo_path", 1);
  map_points_pub_ = n.advertise<sensor_msgs::PointCloud2>("map_points", 1);
}

void semantic_graph_slam_ros::rovioVIOCallback(
    const nav_msgs::Odometry::ConstPtr& odom_msg) {
  const ros::Time& stamp = odom_msg->header.stamp;
  Eigen::Isometry3d odom;
  Eigen::MatrixXf odom_cov;

  if (use_rovio_odom_) {
    odom = ps_graph_slam::odom2isometry(odom_msg);
    odom_cov = ps_graph_slam::arrayToMatrix(odom_msg);
  } else if (use_rtab_map_odom_) {
    nav_msgs::OdometryPtr odom_msg_conv;
    odom_msg_conv = ps_graph_slam::PoseCam2Robot(odom_msg);
    odom = ps_graph_slam::odom2isometry(odom_msg);
    odom_cov = ps_graph_slam::arrayToMatrix(odom_msg);
  }

  semantic_gslam_obj_->VIOCallback(stamp, odom, odom_cov);

  return;
}

void semantic_graph_slam_ros::snapVIOCallback(
    const geometry_msgs::PoseStamped& pose_msg) {
  const ros::Time& stamp = ros::Time::now();
  Eigen::Isometry3d odom;
  Eigen::MatrixXf odom_cov;
  odom_cov.setIdentity(6, 6);  // manually adding pose cov

  if (use_snap_pose_) {
    geometry_msgs::PoseStamped pose_enu = ps_graph_slam::poseNED2ENU(pose_msg);
    odom = ps_graph_slam::pose2isometry(pose_enu);
  } else if (use_orb_slam_odom_) {
    odom = ps_graph_slam::pose2isometry(pose_msg);
  }

  semantic_gslam_obj_->VIOCallback(stamp, odom, odom_cov);

  return;
}

void semantic_graph_slam_ros::orbslamPoseCallback(
    const geometry_msgs::PoseStamped& pose_msg) {
  orb_slam_pose_vec_.push_back(pose_msg);

  return;
}

void semantic_graph_slam_ros::jackalOdomCallback(
    const nav_msgs::OdometryConstPtr& odom_msg) {
  const ros::Time& stamp = odom_msg->header.stamp;
  nav_msgs::OdometryPtr odom_msg_convert =
      ps_graph_slam::RotPoseZ(odom_msg, jack_yaw_transform_);
  if (!first_jack_pose_) {
    jack_x_transform_ = odom_msg_convert->pose.pose.position.x;
    jack_y_transform_ = odom_msg_convert->pose.pose.position.y;
    jack_z_transform_ = odom_msg_convert->pose.pose.position.z;

    first_jack_pose_ = true;
  }

  nav_msgs::OdometryPtr odom_msg_orig =
      ps_graph_slam::navMsgsToOrigin(odom_msg_convert, jack_x_transform_,
                                     jack_y_transform_, jack_z_transform_);
  Eigen::Isometry3d odom = ps_graph_slam::odom2isometry(odom_msg_orig);
  Eigen::MatrixXf odom_cov;
  odom_cov.setIdentity(6, 6);  // jackal pose covariance is bad

  semantic_gslam_obj_->VIOCallback(stamp, odom, odom_cov);

  return;
}

void semantic_graph_slam_ros::PointCloudCallback(
    const sensor_msgs::PointCloud2& msg) {
  semantic_gslam_obj_->setPointCloudData(msg);
  pc_stamp_ = msg.header.stamp;
}

void semantic_graph_slam_ros::detectedObjectDarknetCallback(
    const semantic_SLAM::BoundingBoxes& msg) {
  std::vector<semantic_SLAM::ObjectInfo> object_info;
  object_info.resize(msg.bounding_boxes.size());

  for (int i = 0; i < msg.bounding_boxes.size(); ++i) {
    object_info[i].type = msg.bounding_boxes[i].Class;
    object_info[i].tl_x = msg.bounding_boxes[i].xmin;
    object_info[i].tl_y = msg.bounding_boxes[i].ymin;
    object_info[i].height =
        abs(msg.bounding_boxes[i].ymax - msg.bounding_boxes[i].ymin);
    object_info[i].width =
        abs(msg.bounding_boxes[i].xmax - msg.bounding_boxes[i].xmin);
    object_info[i].prob = msg.bounding_boxes[i].probability;
  }

  semantic_gslam_obj_->setDetectedObjectInfo(object_info);
}

void semantic_graph_slam_ros::detectedObjectSimpleCallback(
    const semantic_SLAM::DetectedObjects& msg) {
  std::vector<semantic_SLAM::ObjectInfo> object_info;
  object_info.resize(msg.objects.size());

  for (int i = 0; i < msg.objects.size(); ++i) {
    object_info[i].type = msg.objects[i].type;
    object_info[i].tl_x = msg.objects[i].tl_x;
    object_info[i].tl_y = msg.objects[i].tl_y;
    object_info[i].height = msg.objects[i].height;
    object_info[i].width = msg.objects[i].width;
    object_info[i].prob = msg.objects[i].prob;
  }

  semantic_gslam_obj_->setDetectedObjectInfo(object_info);
}

void semantic_graph_slam_ros::publishLandmarks() {
  visualization_msgs::MarkerArray marker_arrays;
  int marker_id = 0;
  std::vector<landmark> l_vec;
  semantic_gslam_obj_->getMappedLandmarks(l_vec);

  for (int i = 0; i < l_vec.size(); ++i) {
    Eigen::Vector3d lan_node_pose = l_vec[i].node->estimate();

    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker.ns = "my_namespace";
    marker.id = marker_id;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.pose.position.x = lan_node_pose(0);
    marker.pose.position.y = lan_node_pose(1);
    marker.pose.position.z = lan_node_pose(2);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    if (l_vec[i].plane_type == "horizontal") {
      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.scale.z = 0.01;
    } else if (l_vec[i].plane_type == "vertical") {
      marker.scale.x = 0.01;
      marker.scale.y = 0.3;
      marker.scale.z = 0.3;
    }

    if (l_vec[i].type == "chair") {
      marker.color.a = 1.0;  // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    } else if (l_vec[i].type == "tvmonitor") {
      marker.color.a = 1.0;  // Don't forget to set the alpha!
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    } else if (l_vec[i].type == "laptop") {
      marker.color.a = 1.0;  // Don't forget to set the alpha!
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    } else if (l_vec[i].type == "keyboard") {
      marker.color.a = 1.0;  // Don't forget to set the alpha!
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
    } else if (l_vec[i].type == "book") {
      marker.color.a = 1.0;  // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
    } else if (l_vec[i].type == "bucket") {
      marker.color.a = 1.0;  // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
    } else if (l_vec[i].type == "car") {
      marker.color.a = 1.0;  // Don't forget to set the alpha!
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }

    marker_arrays.markers.push_back(marker);
    marker_id++;
  }

  landmarks_pub_.publish(marker_arrays);
}

void semantic_graph_slam_ros::publishDetectedLandmarks() {
  std::vector<detected_object> det_obj_info;
  semantic_gslam_obj_->getDetectedObjectsPose(det_obj_info);

  visualization_msgs::MarkerArray marker_arrays;
  int marker_id = 0;

  for (int i = 0; i < det_obj_info.size(); ++i) {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker.ns = "my_namespace";
    marker.id = marker_id;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.pose.position.x = det_obj_info[i].world_pose(0);
    marker.pose.position.y = det_obj_info[i].world_pose(1);
    marker.pose.position.z = det_obj_info[i].world_pose(2);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    if (det_obj_info[i].plane_type == "horizontal") {
      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.scale.z = 0.01;
    } else if (det_obj_info[i].plane_type == "vertical") {
      marker.scale.x = 0.01;
      marker.scale.y = 0.3;
      marker.scale.z = 0.3;
    }

    marker_arrays.markers.push_back(marker);
    marker_id++;
  }

  detected_lans_pub_.publish(marker_arrays);
}

void semantic_graph_slam_ros::publishKeyframePoses() {
  ros::Time current_time = ros::Time::now();

  geometry_msgs::PoseArray pose_array;
  pose_array.header.stamp = current_time;
  pose_array.header.frame_id = "map";

  nav_msgs::Path final_path;
  final_path.header.stamp = current_time;
  final_path.header.frame_id = "map";

  // copying the new keyframes for publishing
  std::vector<ps_graph_slam::KeyFrame::Ptr> keyframes;
  semantic_gslam_obj_->getKeyframes(keyframes);
  geometry_msgs::Pose key_pose;
  geometry_msgs::PoseStamped key_pose_stamped;
  geometry_msgs::PoseStamped vio_key_pose_stamped;

  vio_key_pose_vec_.clear();
  robot_pose_vec_.clear();
  for (int i = 0; i < keyframes.size(); ++i) {
    key_pose = ps_graph_slam::matrix2pose(
        ros::Time::now(), keyframes[i]->node->estimate().matrix().cast<float>(),
        "map");

    key_pose_stamped.header.stamp = keyframes[i]->stamp;
    key_pose_stamped.pose = key_pose;

    pose_array.poses.push_back(key_pose);
    final_path.poses.push_back(key_pose_stamped);
    robot_pose_vec_.push_back(key_pose_stamped);

    vio_key_pose_stamped = ps_graph_slam::matrix2posestamped(
        keyframes[i]->stamp, keyframes[i]->odom.matrix().cast<float>(), "map");
    vio_key_pose_vec_.push_back(vio_key_pose_stamped);
  }

  keyframe_pose_pub_.publish(pose_array);
  keyframe_path_pub_.publish(final_path);
}

void semantic_graph_slam_ros::publishRobotPose() {
  Eigen::Isometry3d robot_pose;
  semantic_gslam_obj_->getRobotPose(robot_pose);

  geometry_msgs::PoseStamped robot_pose_msg = ps_graph_slam::matrix2posestamped(
      ros::Time::now(), robot_pose.matrix().cast<float>(), "map");
  // this->publishRobotPoseTF(robot_pose_msg);

  geometry_msgs::TransformStamped robot_transform;
  robot_transform.header.stamp = pc_stamp_;
  robot_transform.header.frame_id = "map";
  robot_transform.child_frame_id = "base_link";
  robot_transform.transform.translation.x = robot_pose_msg.pose.position.x;
  robot_transform.transform.translation.y = robot_pose_msg.pose.position.y;
  robot_transform.transform.translation.z = robot_pose_msg.pose.position.z;
  robot_transform.transform.rotation = robot_pose_msg.pose.orientation;

  robot_transform_pub_.publish(robot_transform);
  robot_pose_pub_.publish(robot_pose_msg);
}

void semantic_graph_slam_ros::publishRobotPoseTF(
    geometry_msgs::PoseStamped robot_pose) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(robot_pose.pose.position.x,
                                  robot_pose.pose.position.y,
                                  robot_pose.pose.position.z));
  tf::Quaternion tf_quat;
  tf_quat[0] = robot_pose.pose.orientation.x;
  tf_quat[1] = robot_pose.pose.orientation.y;
  tf_quat[2] = robot_pose.pose.orientation.z;
  tf_quat[3] = robot_pose.pose.orientation.w;
  transform.setRotation(tf_quat);
  br.sendTransform(tf::StampedTransform(transform, robot_pose.header.stamp,
                                        "map", "base_link"));
}

void semantic_graph_slam_ros::publishMap2Odom() {
  Eigen::Isometry3d map2odom;
  semantic_gslam_obj_->getMap2OdomTrans(map2odom);

  geometry_msgs::PoseStamped map2odom_msg = ps_graph_slam::matrix2posestamped(
      ros::Time::now(), map2odom.matrix().cast<float>(), "map");
  this->publishMap2OdomTF(map2odom_msg);
}

void semantic_graph_slam_ros::publishMap2OdomTF(
    geometry_msgs::PoseStamped map2odom_pose) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(map2odom_pose.pose.position.x,
                                  map2odom_pose.pose.position.y,
                                  map2odom_pose.pose.position.z));
  tf::Quaternion tf_quat;
  tf_quat[0] = map2odom_pose.pose.orientation.x;
  tf_quat[1] = map2odom_pose.pose.orientation.y;
  tf_quat[2] = map2odom_pose.pose.orientation.z;
  tf_quat[3] = map2odom_pose.pose.orientation.w;
  transform.setRotation(tf_quat);
  br.sendTransform(tf::StampedTransform(transform, map2odom_pose.header.stamp,
                                        "map", "odom"));
}

void semantic_graph_slam_ros::optitrackPoseCallback(
    const nav_msgs::Odometry& msg) {
  geometry_msgs::PoseStamped optitrack_pose;
  optitrack_pose.header.stamp = ros::Time::now();
  optitrack_pose.header.frame_id = "map";

  optitrack_pose.pose = msg.pose.pose;
  optitrack_pose_pub_.publish(optitrack_pose);

  optitrack_pose_vec_.push_back(optitrack_pose);
  nav_msgs::Path optitrack_path;
  optitrack_path.header.stamp = msg.header.stamp;
  optitrack_path.header.frame_id = "map";
  optitrack_path.poses = optitrack_pose_vec_;
  optitrack_path_pub_.publish(optitrack_path);
}

void semantic_graph_slam_ros::viconPoseSubCallback(
    const semantic_SLAM::ViconState& msg) {
  if (!first_gt_pose_) {
    gt_x_transform_ = -msg.pose.position.x;
    gt_y_transform_ = -msg.pose.position.y;
    gt_z_transform_ = -msg.pose.position.z;
    first_gt_pose_ = true;
  }

  geometry_msgs::PoseStamped vicon_pose;
  vicon_pose.header.stamp = ros::Time::now();
  vicon_pose.header.frame_id = "map";

  vicon_pose.pose.position.x = msg.pose.position.x + gt_x_transform_;
  vicon_pose.pose.position.y = msg.pose.position.y + gt_y_transform_;
  vicon_pose.pose.position.z = msg.pose.position.z + gt_z_transform_;
  vicon_pose.pose.orientation = msg.pose.orientation;

  optitrack_pose_pub_.publish(vicon_pose);
  optitrack_pose_vec_.push_back(vicon_pose);

  nav_msgs::Path vicon_path;
  vicon_path.header.stamp = msg.header.stamp;
  vicon_path.header.frame_id = "map";
  vicon_path.poses = optitrack_pose_vec_;
  optitrack_path_pub_.publish(vicon_path);
}

void semantic_graph_slam_ros::transformListener() {
  tf::StampedTransform transform;
  try {
    gt_pose_listener_.lookupTransform("/world", "/openni_camera", ros::Time(0),
                                      transform);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    // ros::Duration(1.0).sleep();
  }

  geometry_msgs::PoseStamped vicon_pose;
  vicon_pose.header.stamp = transform.stamp_;
  vicon_pose.header.frame_id = "map";

  vicon_pose.pose.position.x = transform.getOrigin().x();
  vicon_pose.pose.position.y = transform.getOrigin().y();
  vicon_pose.pose.position.z = transform.getOrigin().z();
  vicon_pose.pose.orientation.x = transform.getRotation().x();
  vicon_pose.pose.orientation.y = transform.getRotation().y();
  vicon_pose.pose.orientation.z = transform.getRotation().z();
  vicon_pose.pose.orientation.w = transform.getRotation().w();

  optitrack_pose_pub_.publish(vicon_pose);
  optitrack_pose_vec_.push_back(vicon_pose);

  nav_msgs::Path vicon_path;
  vicon_path.header.stamp = ros::Time::now();
  vicon_path.header.frame_id = "map";
  vicon_path.poses = optitrack_pose_vec_;
  optitrack_path_pub_.publish(vicon_path);
}

void semantic_graph_slam_ros::publishCorresVIOPose() {
  Eigen::Isometry3d vio_pose;
  semantic_gslam_obj_->getVIOPose(vio_pose);

  ros::Time current_time = ros::Time::now();
  geometry_msgs::PoseStamped corres_vio_pose_msg =
      ps_graph_slam::matrix2posestamped(
          current_time, vio_pose.matrix().cast<float>(), "odom");

  corres_vio_pose_pub_.publish(corres_vio_pose_msg);
  this->publishVIOTF(corres_vio_pose_msg);

  nav_msgs::Path vio_path;
  vio_path.header.stamp = current_time;
  vio_path.header.frame_id = "odom";

  vio_pose_vec_.push_back(corres_vio_pose_msg);
  vio_path.poses = vio_pose_vec_;
  corres_vio_path_.publish(vio_path);
}

void semantic_graph_slam_ros::publishVIOTF(
    geometry_msgs::PoseStamped vio_pose) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(vio_pose.pose.position.x,
                                  vio_pose.pose.position.y,
                                  vio_pose.pose.position.z));
  tf::Quaternion tf_quat;
  tf_quat[0] = vio_pose.pose.orientation.x;
  tf_quat[1] = vio_pose.pose.orientation.y;
  tf_quat[2] = vio_pose.pose.orientation.z;
  tf_quat[3] = vio_pose.pose.orientation.w;
  transform.setRotation(tf_quat);
  br.sendTransform(tf::StampedTransform(transform, vio_pose.header.stamp,
                                        "odom", "base_link"));
}

void semantic_graph_slam_ros::saveGraph() {
  if (save_graph_) semantic_gslam_obj_->saveGraph(save_graph_path_);
}

void semantic_graph_slam_ros::computeATE() {
  if (compute_txt_for_ate_) {
    // writing the txt for robot
    std::ofstream robot_data;
    robot_data.open("/home/hriday/Desktop/robot_pose.txt",
                    std::ios::out | std::ios::ate | std::ios::app);
    robot_data << "#timestamp ,tx,ty,tz,qx,qy,qz,qw" << std::endl;
    robot_data.close();

    for (int i = 0; i < robot_pose_vec_.size(); ++i) {
      robot_data.open("/home/hriday/Desktop/robot_pose.txt",
                      std::ios::out | std::ios::ate | std::ios::app);
      robot_data << robot_pose_vec_[i].header.stamp << " "
                 << robot_pose_vec_[i].pose.position.x << " "
                 << robot_pose_vec_[i].pose.position.y << " "
                 << robot_pose_vec_[i].pose.position.z << " "
                 << robot_pose_vec_[i].pose.orientation.x << " "
                 << robot_pose_vec_[i].pose.orientation.y << " "
                 << robot_pose_vec_[i].pose.orientation.z << " "
                 << robot_pose_vec_[i].pose.orientation.w << std::endl;
      robot_data.close();
    }

    // writing the text for vio data
    std::ofstream vio_data;
    vio_data.open("/home/hriday/Desktop/vio_pose.txt",
                  std::ios::out | std::ios::ate | std::ios::app);
    vio_data << "#timestamp ,tx,ty,tz,qx,qy,qz,qw" << std::endl;
    vio_data.close();

    for (int i = 0; i < vio_key_pose_vec_.size(); ++i) {
      vio_data.open("/home/hriday/Desktop/vio_pose.txt",
                    std::ios::out | std::ios::ate | std::ios::app);
      vio_data << vio_key_pose_vec_[i].header.stamp << " "
               << vio_key_pose_vec_[i].pose.position.x << " "
               << vio_key_pose_vec_[i].pose.position.y << " "
               << vio_key_pose_vec_[i].pose.position.z << " "
               << vio_key_pose_vec_[i].pose.orientation.x << " "
               << vio_key_pose_vec_[i].pose.orientation.y << " "
               << vio_key_pose_vec_[i].pose.orientation.z << " "
               << vio_key_pose_vec_[i].pose.orientation.w << std::endl;
      vio_data.close();
    }

    // writing the text for gt data
    std::ofstream gt_data;
    gt_data.open("/home/hriday/Desktop/gt_pose.txt",
                 std::ios::out | std::ios::ate | std::ios::app);
    gt_data << "#timestamp ,tx,ty,tz,qx,qy,qz,qw" << std::endl;
    gt_data.close();

    for (int i = 0; i < optitrack_pose_vec_.size(); ++i) {
      gt_data.open("/home/hriday/Desktop/gt_pose.txt",
                   std::ios::out | std::ios::ate | std::ios::app);
      gt_data << optitrack_pose_vec_[i].header.stamp << " "
              << optitrack_pose_vec_[i].pose.position.x << " "
              << optitrack_pose_vec_[i].pose.position.y << " "
              << optitrack_pose_vec_[i].pose.position.z << " "
              << optitrack_pose_vec_[i].pose.orientation.x << " "
              << optitrack_pose_vec_[i].pose.orientation.y << " "
              << optitrack_pose_vec_[i].pose.orientation.z << " "
              << optitrack_pose_vec_[i].pose.orientation.w << std::endl;
      gt_data.close();
    }

    // writing orb slam data for comparision
    std::ofstream orb_slam_data;
    orb_slam_data.open("/home/hriday/Desktop/orb_slam_pose.txt",
                       std::ios::out | std::ios::ate | std::ios::app);
    orb_slam_data << "#timestamp ,tx,ty,tz,qx,qy,qz,qw" << std::endl;
    orb_slam_data.close();

    for (int i = 0; i < orb_slam_pose_vec_.size(); ++i) {
      orb_slam_data.open("/home/hriday/Desktop/orb_slam_pose.txt",
                         std::ios::out | std::ios::ate | std::ios::app);
      orb_slam_data << orb_slam_pose_vec_[i].header.stamp << " "
                    << orb_slam_pose_vec_[i].pose.position.x << " "
                    << orb_slam_pose_vec_[i].pose.position.y << " "
                    << orb_slam_pose_vec_[i].pose.position.z << " "
                    << orb_slam_pose_vec_[i].pose.orientation.x << " "
                    << orb_slam_pose_vec_[i].pose.orientation.y << " "
                    << orb_slam_pose_vec_[i].pose.orientation.z << " "
                    << orb_slam_pose_vec_[i].pose.orientation.w << std::endl;
      orb_slam_data.close();
    }
  }
}