#include <iostream>
#include <math.h>
#include <mutex>
#include <random>
#include <string>

#include "semantic_SLAM/ObjectInfo.h"

#include "sensor_msgs/PointCloud2.h"

// ros
#include "ros/ros.h"

// PCL ROS
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

// PCL
#include "pcl/point_types.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

// opencv
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui_c.h>


const int num_centroids_normals = 4;
const int num_centroids_distance = 2;
const int num_centroids_vert_dist = 1;
const int num_centroids_pose = 2;

class plane_segmentation {
public:
  plane_segmentation(bool verbose);
  ~plane_segmentation();

private:
  bool verbose_;
  double num_points_seg_;
  double normal_point_thres_;
  double planar_area_;

public:
  struct segmented_objects {
    std::string type;
    float prob;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_point_cloud;
  };

  struct segmented_planes {
    cv::Mat final_pose_mat;
    pcl::PointCloud<pcl::PointXYZRGB> planar_points;
  };

  plane_segmentation::segmented_objects
  segmentPointCloudData(semantic_SLAM::ObjectInfo object_info,
                        sensor_msgs::PointCloud2 point_cloud,
                        sensor_msgs::PointCloud2 &segmented_point_cloud);

  pcl::PointCloud<pcl::Normal>::Ptr computeNormalsFromPointCloud(
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
      pcl::PointIndices::Ptr inliers);

  cv::Mat
  computeHorizontalPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
                         pcl::PointCloud<pcl::Normal>::Ptr point_normal,
                         Eigen::Matrix4f transformation_mat,
                         Eigen::MatrixXf final_pose, float &point_size,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr
                             &segemented_plane_from_point_cloud);

  std::vector<cv::Mat>
  computeAllVerticalPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
                           pcl::PointCloud<pcl::Normal>::Ptr point_normal,
                           Eigen::Matrix4f transformation_mat,
                           Eigen::MatrixXf final_pose, float &point_size,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr
                               &segemented_plane_from_point_cloud);

  std::vector<segmented_planes>
  multiPlaneSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
                         pcl::PointCloud<pcl::Normal>::Ptr point_normal,
                         pcl::PointIndices::Ptr inliers,
                         Eigen::Matrix4f transformation_mat);

  std::vector<cv::Mat>
  clusterAndSegmentAllPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
                             pcl::PointCloud<pcl::Normal>::Ptr point_normal,
                             Eigen::Matrix4f transformation_mat);

  cv::Mat NormalBasedClusteringAndSegmentation(
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
      pcl::PointCloud<pcl::Normal>::Ptr point_normal,
      Eigen::Matrix4f transformation_mat,
      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
          &segmented_points_using_first_kmeans);

  std::vector<cv::Mat>
  distanceBasedSegmentation(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
                                segmented_points_using_first_kmeans,
                            cv::Mat filtered_centroids,
                            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
                                &final_segmented_vec_of_points_from_distances);

  std::vector<cv::Mat>
  getFinalPoseWithNormals(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
                              segmented_points_vec_using_second_kmeans,
                          std::vector<cv::Mat> final_normals_with_distances);

  void removeNans(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
                  pcl::PointCloud<pcl::Normal>::Ptr point_normal,
                  cv::Mat &normal_filtered_points_mat,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points_without_nans);

  cv::Mat filterCentroids(cv::Mat centroids,
                          Eigen::Vector4f normals_horizontal_plane_in_cam,
                          std::vector<int> &filtered_centroids_id);

  double computeKmeans(cv::Mat points, const int num_centroids, cv::Mat &labels,
                       cv::Mat &centroids);

  cv::Mat findNearestNeighbours(cv::Mat centroids);

  float computeDotProduct(Eigen::Vector4f vector_a, Eigen::Vector4f vector_b);

  std::vector<cv::Mat>
  computeAllPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
                   Eigen::Matrix4f transformation_mat);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr compute2DConvexHull(
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_point_cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  preprocessPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
                       pcl::PointIndices::Ptr &inliers);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  downsamplePointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
                       pcl::PointIndices::Ptr &inliers);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  removeOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
                 pcl::PointIndices::Ptr &inliers);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  distance_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud);
};
