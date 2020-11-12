#ifndef DATA_ASSOCIATION
#define DATA_ASSOCIATION

#include "ros/ros.h"

#include "eigen3/Eigen/Eigen"
#include <fstream>
#include <functional>
#include <iostream>
#include <math.h>
#include <random>
#include <string>
#include <thread>

#include "planar_segmentation/detected_object.h"
#include "ps_graph_slam/landmark.h"
#include "tools.h"

class data_association {

public:
  data_association(bool verbose) {
    std::cout << "data association constructor " << std::endl;
    verbose_ = verbose;
    init();
  }
  ~data_association() {
    std::cout << "data association destructor " << std::endl;
  }

private:
  bool verbose_;
  bool first_object_;
  std::vector<landmark> landmarks_;
  Eigen::Matrix3f Q_;
  double maha_dist_thres_, eq_dist_thres_;
  semantic_tools sem_tool_obj_;
  double land_noise_low_, land_noise_high_;
  bool use_maha_dist_, use_eq_dist_;
  bool eq_dist_;
  bool use_rtab_map_odom_;

  void init() {
    first_object_ = true;
    use_maha_dist_ = true;
    landmarks_.clear();
    Q_.setZero();

    ros::param::param<double>("~maha_dist_thres", maha_dist_thres_, 0.5);
    ros::param::param<double>("~eq_dist_thres", eq_dist_thres_, 1.21);
    ros::param::param<double>("~land_noise_low", land_noise_low_, 0.5);
    ros::param::param<double>("~land_noise_high", land_noise_high_, 0.9);
    ros::param::param<bool>("~use_maha_dist", use_maha_dist_, true);
    ros::param::param<bool>("~use_eq_dist", use_eq_dist_, false);
    ros::param::param<bool>("~use_rtab_map_odom", use_rtab_map_odom_, false);

    std::cout << "land_noise_low: " << land_noise_low_ << std::endl;
    std::cout << "land_noise_high: " << land_noise_high_ << std::endl;
    std::cout << "use_maha_dist: " << use_maha_dist_ << std::endl;
    std::cout << "use_eq_dist: " << use_eq_dist_ << std::endl;
    std::cout << "maha dist thres: " << maha_dist_thres_ << std::endl;
    std::cout << "eq dist thres: " << eq_dist_thres_ << std::endl;

    Q_(0, 0) = land_noise_low_;
    Q_(1, 1) = land_noise_low_;
    Q_(2, 2) = land_noise_low_;
  }

public:
  void addFirstLandmark(landmark l) {
    first_object_ = false;
    landmarks_.push_back(l);
  }

  std::vector<landmark> find_matches(std::vector<detected_object> seg_obj_info,
                                     Eigen::VectorXf robot_pose,
                                     float cam_angle) {
    std::vector<landmark> landmark_vec;
    if (first_object_) {
      for (int i = 0; i < seg_obj_info.size(); ++i)
        landmark_vec.push_back(
            this->map_a_new_lan(seg_obj_info[i], robot_pose, cam_angle));
      ;
      if (landmark_vec.size() > 0)
        first_object_ = false;
    } else {
      landmark_vec =
          this->associate_lanmarks(seg_obj_info, robot_pose, cam_angle);
    }

    if (verbose_)
      std::cout << "landmarks size " << landmarks_.size() << std::endl;

    return landmark_vec;
  }

  std::vector<landmark>
  associate_lanmarks(std::vector<detected_object> seg_obj_info,
                     Eigen::VectorXf robot_pose, float cam_angle) {
    bool found_nearest_neighbour = false;
    float distance = 0;
    float distance_min = std::numeric_limits<float>::max();

    std::vector<landmark> landmark_vec;

    for (int j = 0; j < seg_obj_info.size(); ++j) {
      int neareast_landmarks_id;

      //            if(seg_obj_info[j].num_points < 300)
      //            {
      //                Q_(0,0) = land_noise_high_;
      //                Q_(1,1) = land_noise_high_;
      //                Q_(2,2) = land_noise_high_;
      //            }
      //            else
      //            {
      //                Q_(0,0) = land_noise_low_;
      //                Q_(1,1) = land_noise_low_;
      //                Q_(2,2) = land_noise_low_;
      //            }

      for (int i = 0; i < landmarks_.size(); ++i) {
        if (seg_obj_info[j].type == landmarks_[i].type) {
          if (seg_obj_info[j].plane_type == landmarks_[i].plane_type) {

            // transform the detected object pose to world frame
            Eigen::Vector4f obj_pose_cam;
            obj_pose_cam.setOnes();

            obj_pose_cam << seg_obj_info[j].pose(0), seg_obj_info[j].pose(1),
                seg_obj_info[j].pose(2);

            Eigen::Vector4f obj_pose_world =
                this->convertPoseToWorld(robot_pose, cam_angle, obj_pose_cam);

            // transform detected normals in world
            Eigen::Vector4f obj_normals_world = this->convertNormalsToWorld(
                robot_pose, cam_angle, seg_obj_info[j].normal_orientation);

            //                        if(fabs(obj_normals_world(0) -
            //                        landmarks_[i].normal_orientation(0)) < 0.1
            //                        &&
            //                                fabs(obj_normals_world(1) -
            //                                landmarks_[i].normal_orientation(1))
            //                                < 0.1 && fabs(obj_normals_world(2)
            //                                -
            //                                landmarks_[i].normal_orientation(2))
            //                                < 0.1)
            {

              found_nearest_neighbour = true;

              Eigen::VectorXf expected_meas;
              Eigen::MatrixXf H =
                  this->landmarkMeasurementModel(landmarks_[i], expected_meas);

              Eigen::VectorXf actual_meas;
              actual_meas.resize(3);
              actual_meas = obj_pose_world;

              if (verbose_) {
                std::cout << "actual meas: " << actual_meas << std::endl;
                std::cout << "expected meas: " << expected_meas << std::endl;
              }

              // sigma recovered from graph slam optimization
              if (use_maha_dist_) {
                Eigen::MatrixXf sigma;
                sigma.resize(3, 3);
                sigma = landmarks_[i].covariance;
                if (verbose_)
                  std::cout << "landmark: " << i << "sigma: " << sigma
                            << std::endl;

                Eigen::MatrixXf Q = H * sigma * H.transpose() + Q_;

                Eigen::VectorXf z_diff;
                z_diff.resize(3);
                z_diff = actual_meas - expected_meas;

                if (verbose_)
                  std::cout << "z_diff: " << z_diff << std::endl;

                distance = z_diff.transpose() * Q.inverse() * z_diff;
                if (verbose_)
                  std::cout << "cov maha distance: " << distance << std::endl;
              } else if (use_eq_dist_) {
                distance = sem_tool_obj_.dist(actual_meas(0), expected_meas(0),
                                              actual_meas(1), expected_meas(1),
                                              actual_meas(2), expected_meas(2));
                if (verbose_)
                  std::cout << "euclidean distance: " << distance << std::endl;
              }

              if (distance < distance_min) {
                distance_min = distance;
                neareast_landmarks_id = i;
              }
            }
          }
        }
      }

      // if the type of the object did not get matched
      if (!found_nearest_neighbour) {
        landmark_vec.push_back(
            this->map_a_new_lan(seg_obj_info[j], robot_pose, cam_angle));
      } else if (found_nearest_neighbour) {
        found_nearest_neighbour = false;

        if (verbose_)
          std::cout << "distance_min: " << distance_min << std::endl;

        if (use_maha_dist_) {
          if (distance_min > maha_dist_thres_)
            landmark_vec.push_back(
                this->map_a_new_lan(seg_obj_info[j], robot_pose, cam_angle));
          else
            landmark_vec.push_back(this->inserst_a_mapped_lan(
                seg_obj_info[j], robot_pose, cam_angle,
                landmarks_[neareast_landmarks_id]));
        } else if (use_eq_dist_) {
          if (distance_min > eq_dist_thres_)
            landmark_vec.push_back(
                this->map_a_new_lan(seg_obj_info[j], robot_pose, cam_angle));
          else
            landmark_vec.push_back(this->inserst_a_mapped_lan(
                seg_obj_info[j], robot_pose, cam_angle,
                landmarks_[neareast_landmarks_id]));
        }
      }
    }

    return landmark_vec;
  }

  landmark map_a_new_lan(detected_object seg_obj_info,
                         Eigen::VectorXf robot_pose, float cam_angle) {

    // transform the detected object pose to world frame
    Eigen::Vector4f obj_pose_cam;
    obj_pose_cam.setOnes();

    obj_pose_cam << seg_obj_info.pose(0), seg_obj_info.pose(1),
        seg_obj_info.pose(2);

    Eigen::Vector4f obj_pose_world =
        this->convertPoseToWorld(robot_pose, cam_angle, obj_pose_cam);

    // transform detected normals in world
    Eigen::Vector4f obj_normals_world = this->convertNormalsToWorld(
        robot_pose, cam_angle, seg_obj_info.normal_orientation);

    // obj pose from cam to robot
    Eigen::Vector4f obj_pose_rob =
        this->convertCamToRobot(robot_pose, cam_angle, obj_pose_cam);

    landmark new_landmark;
    new_landmark.is_new_landmark = true;
    new_landmark.id = landmarks_.size();
    new_landmark.local_pose << obj_pose_rob(0), obj_pose_rob(1),
        obj_pose_rob(2);
    new_landmark.pose << obj_pose_world(0), obj_pose_world(1),
        obj_pose_world(2);
    new_landmark.covariance = Q_;
    new_landmark.normal_orientation = obj_normals_world;
    new_landmark.plane_type = seg_obj_info.plane_type;
    new_landmark.type = seg_obj_info.type;
    landmarks_.push_back(new_landmark);

    if (verbose_)
      std::cout << "\033[1;31m new landmark pose \033[0m\n"
                << new_landmark.pose << std::endl;

    return new_landmark;
  }

  landmark inserst_a_mapped_lan(detected_object seg_obj_info,
                                Eigen::VectorXf robot_pose, float cam_angle,
                                landmark l) {

    // transform the detected object pose to world frame
    Eigen::Vector4f obj_pose_cam;
    obj_pose_cam.setOnes();

    obj_pose_cam << seg_obj_info.pose(0), seg_obj_info.pose(1),
        seg_obj_info.pose(2);

    Eigen::Vector4f obj_pose_world =
        this->convertPoseToWorld(robot_pose, cam_angle, obj_pose_cam);

    // transform detected normals in world
    Eigen::Vector4f obj_normals_world = this->convertNormalsToWorld(
        robot_pose, cam_angle, seg_obj_info.normal_orientation);

    // obj pose from cam to robot
    Eigen::Vector4f obj_pose_rob =
        this->convertCamToRobot(robot_pose, cam_angle, obj_pose_cam);

    landmark landmark;
    landmark.is_new_landmark = false;
    landmark.id = l.id;
    landmark.local_pose << obj_pose_rob(0), obj_pose_rob(1), obj_pose_rob(2);
    landmark.pose << obj_pose_world(0), obj_pose_world(1), obj_pose_world(2);
    landmark.covariance = Q_;
    landmark.normal_orientation = obj_normals_world;
    landmark.plane_type = seg_obj_info.plane_type;
    landmark.type = seg_obj_info.type;
    landmark.node = l.node;

    if (verbose_)
      std::cout << "\033[1;34m matched landmark \033[0m " << landmark.pose
                << " "
                << "\033[1;34m with mapped landmark \033[0m " << l.pose
                << std::endl;

    return landmark;
  }

  Eigen::VectorXf convertPoseToWorld(Eigen::VectorXf robot_pose,
                                     float cam_angle,
                                     Eigen::VectorXf pose_to_transform) {

    Eigen::Matrix4f transformation_mat;
    sem_tool_obj_.transformNormalsToWorld(robot_pose, transformation_mat,
                                          cam_angle);

    Eigen::VectorXf transformed_pose;
    transformed_pose.resize(4);

    transformed_pose = transformation_mat * pose_to_transform;

    // adding the robots pose
    transformed_pose(0) += robot_pose(0);
    if (!use_rtab_map_odom_)
      transformed_pose(1) += robot_pose(1);
    else
      transformed_pose(1) += robot_pose(1) - 0.04;

    transformed_pose(2) += robot_pose(2);

    return transformed_pose;
  }

  Eigen::VectorXf convertNormalsToWorld(Eigen::VectorXf robot_pose,
                                        float cam_angle,
                                        Eigen::VectorXf pose_to_transform) {

    Eigen::Matrix4f transformation_mat;
    sem_tool_obj_.transformNormalsToWorld(robot_pose, transformation_mat,
                                          cam_angle);

    Eigen::VectorXf transformed_pose;
    transformed_pose.resize(4);

    transformed_pose = transformation_mat * pose_to_transform;

    return transformed_pose;
  }

  Eigen::VectorXf convertCamToRobot(Eigen::VectorXf robot_pose, float cam_angle,
                                    Eigen::VectorXf pose_to_transform) {

    Eigen::Matrix4f transformation_mat;
    sem_tool_obj_.transformPoseFromCameraToRobot(transformation_mat, cam_angle);

    Eigen::VectorXf transformed_pose;
    transformed_pose.resize(4);

    transformed_pose = transformation_mat * pose_to_transform;

    return transformed_pose;
  }

  Eigen::MatrixXf landmarkMeasurementModel(landmark l, Eigen::VectorXf &h) {
    h.resize(3, 3);
    h.setZero();
    h = l.node->estimate().cast<float>();

    Eigen::MatrixXf H;
    H.resize(3, 3);
    H.setZero();

    H(0, 0) = 1;
    H(1, 1) = 1;
    H(2, 2) = 1;

    return H;
  }

  void assignLandmarkNode(int id, g2o::VertexPointXYZ *node) {
    landmarks_[id].node = node;
  }

  inline void setLandmarkCovs(int id, Eigen::Matrix3f cov) {
    landmarks_[id].covariance = cov;
  }

  void getMappedLandmarks(std::vector<landmark> &l) { l = landmarks_; }
};

#endif
