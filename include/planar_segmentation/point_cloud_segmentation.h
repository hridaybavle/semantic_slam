#ifndef POINT_CLOUD_SEGMENTATION
#define POINT_CLOUD_SEGMENTATION

#include "planar_segmentation/detected_object.h"
#include "planar_segmentation/plane_segmentation.h"
#include "tools.h"

class point_cloud_segmentation {

public:
  std::unique_ptr<plane_segmentation> plane_seg_obj;
  bool verbose_;

public:
  point_cloud_segmentation(bool verbose) {
    verbose_ = verbose;
    plane_seg_obj.reset(new plane_segmentation(verbose_));
    std::cout << "pc segmentation constructor " << std::endl;
  }

  ~point_cloud_segmentation() {
    std::cout << "pc segmentation destructor " << std::endl;
  }

public:
  std::vector<detected_object> segmentPlanarSurfaces(
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_point_cloud,
      pcl::PointCloud<pcl::Normal>::Ptr segmented_point_cloud_normal,
      pcl::PointIndices::Ptr inliers, Eigen::Matrix4f transformation_mat,
      Eigen::VectorXf robot_pose, std::string object_type, float prob) {
    std::vector<detected_object> complete_obj_info_vec;

    std::vector<plane_segmentation::segmented_planes> planar_surf_vec;
    planar_surf_vec.clear();

    // ros::Time t1 = ros::Time::now();
    planar_surf_vec = plane_seg_obj->multiPlaneSegmentation(
        segmented_point_cloud, segmented_point_cloud_normal, inliers,
        transformation_mat);

    if (!planar_surf_vec.empty()) {
      for (int j = 0; j < planar_surf_vec.size(); ++j) {
        Eigen::Vector4f final_detected_point_cam_frame,
            final_detected_point_world_frame;
        final_detected_point_cam_frame.setOnes(),
            final_detected_point_world_frame.setOnes();

        final_detected_point_cam_frame(0) =
            planar_surf_vec[j].final_pose_mat.at<float>(0, 0);
        final_detected_point_cam_frame(1) =
            planar_surf_vec[j].final_pose_mat.at<float>(0, 1);
        final_detected_point_cam_frame(2) =
            planar_surf_vec[j].final_pose_mat.at<float>(0, 2);

        final_detected_point_world_frame =
            transformation_mat * final_detected_point_cam_frame;

        float final_object_height;
        final_object_height =
            final_detected_point_world_frame(2) + robot_pose(2);

        // pose_vec.push_back(final_pose_of_object_in_robot);
        // do the same above procedure for the normal orientation
        Eigen::Vector4f normal_orientation_cam_frame;
        normal_orientation_cam_frame.setOnes();

        normal_orientation_cam_frame(0) =
            planar_surf_vec[j].final_pose_mat.at<float>(0, 3);
        normal_orientation_cam_frame(1) =
            planar_surf_vec[j].final_pose_mat.at<float>(0, 4);
        normal_orientation_cam_frame(2) =
            planar_surf_vec[j].final_pose_mat.at<float>(0, 5);
        normal_orientation_cam_frame(3) =
            planar_surf_vec[j].final_pose_mat.at<float>(0, 6);

        detected_object complete_obj_info;

        // if its zeros its horizontal else its vertical
        if (planar_surf_vec[j].final_pose_mat.at<float>(0, 7) == 0)
          complete_obj_info.plane_type = "horizontal";
        else if (planar_surf_vec[j].final_pose_mat.at<float>(0, 7) == 1)
          complete_obj_info.plane_type = "vertical";

        complete_obj_info.type = object_type;
        complete_obj_info.prob = prob;
        complete_obj_info.num_points = planar_surf_vec[j].planar_points.size();
        complete_obj_info.pose(0) = final_detected_point_cam_frame(0);
        complete_obj_info.pose(1) = final_detected_point_cam_frame(1);
        complete_obj_info.pose(2) = final_detected_point_cam_frame(2);
        complete_obj_info.normal_orientation = normal_orientation_cam_frame;
        complete_obj_info.world_pose
            << final_detected_point_world_frame(0) + robot_pose(0),
            final_detected_point_world_frame(1) + robot_pose(1),
            final_detected_point_world_frame(2) + robot_pose(2);
        // complete_obj_info.planar_points                 =
        // planar_surf_vec[j].planar_points;

        complete_obj_info_vec.push_back(complete_obj_info);
      }
    }

    return complete_obj_info_vec;
  }

  std::vector<detected_object>
  segmentallPointCloudData(Eigen::VectorXf robot_pose, float cam_angle,
                           std::vector<semantic_SLAM::ObjectInfo> object_info,
                           sensor_msgs::PointCloud2 point_cloud) {

    sensor_msgs::PointCloud2 segmented_point_cloud;
    std::vector<plane_segmentation::segmented_objects>
        segmented_objects_from_point_cloud;

    Eigen::Matrix4f transformation_mat;
    semantic_tools sem_tool_obj;
    sem_tool_obj.transformNormalsToWorld(robot_pose, transformation_mat,
                                         cam_angle);

    std::vector<detected_object> complete_obj_info_vec;
    complete_obj_info_vec.clear();

    // This segments the PC according to the received bounding box data in the
    // 2D image
    segmented_objects_from_point_cloud.clear();
    for (int i = 0; i < object_info.size(); ++i) {
      if (object_info[i].type == "chair" ||
          object_info[i].type == "tvmonitor" || object_info[i].type == "book" ||
          object_info[i].type == "keyboard" ||
          object_info[i].type == "laptop" || object_info[i].type == "bucket" ||
          object_info[i].type == "car") {
        plane_segmentation::segmented_objects
            single_segmented_object_from_point_cloud;
        single_segmented_object_from_point_cloud =
            plane_seg_obj->segmentPointCloudData(object_info[i], point_cloud,
                                                 segmented_point_cloud);

        if (single_segmented_object_from_point_cloud.type != "spurious")
          segmented_objects_from_point_cloud.push_back(
              single_segmented_object_from_point_cloud);
      }
    }

    for (int i = 0; i < segmented_objects_from_point_cloud.size(); ++i) {

      if (!segmented_objects_from_point_cloud[i]
               .segmented_point_cloud->empty()) {
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(
            new pcl::PointCloud<pcl::PointXYZRGB>);
        //            cloud_filtered =
        //            plane_segmentation_obj_.preprocessPointCloud(segmented_objects_from_point_cloud[i].segmented_point_cloud,
        //                                                                          inliers);

        // This calculates the normals of the segmented pointcloud
        pcl::PointCloud<pcl::Normal>::Ptr segmented_point_cloud_normal(
            new pcl::PointCloud<pcl::Normal>);
        segmented_point_cloud_normal =
            plane_seg_obj->computeNormalsFromPointCloud(
                segmented_objects_from_point_cloud[i].segmented_point_cloud,
                inliers);

        if (segmented_point_cloud_normal->empty())
          continue;

        std::vector<detected_object> current_obj_info_vec;
        current_obj_info_vec.clear();
        // this computes all the planar surfaces from the detected segmented
        // data
        current_obj_info_vec = segmentPlanarSurfaces(
            segmented_objects_from_point_cloud[i].segmented_point_cloud,
            segmented_point_cloud_normal, inliers, transformation_mat,
            robot_pose, segmented_objects_from_point_cloud[i].type,
            segmented_objects_from_point_cloud[i].prob);

        for (int j = 0; j < current_obj_info_vec.size(); ++j)
          complete_obj_info_vec.push_back(current_obj_info_vec[j]);
      }
    }

    return complete_obj_info_vec;
  }
};

#endif
