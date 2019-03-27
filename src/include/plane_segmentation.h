#include <iostream>
#include <string>
#include <math.h>
#include <mutex>
#include <random>

#include "semantic_SLAM/ObjectInfo.h"

#include "sensor_msgs/PointCloud2.h"

//PCL ROS
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

//PCL
#include "pcl/point_types.h"
#include <pcl/features/integral_image_normal.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

const int num_centroids_normals = 2;
const int num_centroids_height = 2;
const int num_centroids_vert_dist = 1;
const int num_centroids_pose = 2;

class plane_segmentation
{
public:
    plane_segmentation();
    ~plane_segmentation();

    struct segmented_objects {
        std::string type;
        float prob;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_point_cloud;
    };

    plane_segmentation::segmented_objects segmentPointCloudData(semantic_SLAM::ObjectInfo object_info,
                                                                sensor_msgs::PointCloud2 point_cloud,
                                                                sensor_msgs::PointCloud2& segmented_point_cloud);

    pcl::PointCloud<pcl::Normal>::Ptr computeNormalsFromPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud);

    cv::Mat computeHorizontalPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
                                   pcl::PointCloud<pcl::Normal>::Ptr point_normal,
                                   Eigen::Matrix4f transformation_mat,
                                   Eigen::MatrixXf final_pose,
                                   float& point_size,
                                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr &segemented_plane_from_point_cloud);


    std::vector<cv::Mat> computeAllHorizontalPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
                                                    pcl::PointCloud<pcl::Normal>::Ptr point_normal,
                                                    Eigen::Matrix4f transformation_mat,
                                                    Eigen::MatrixXf final_pose,
                                                    float& point_size, int number_of_height_centroids);

    std::vector<cv::Mat> computeAllVerticalPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
                                                  pcl::PointCloud<pcl::Normal>::Ptr point_normal,
                                                  Eigen::Matrix4f transformation_mat,
                                                  Eigen::MatrixXf final_pose,
                                                  float& point_size,
                                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& segemented_plane_from_point_cloud);

    std::vector<cv::Mat> computeAllPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud, Eigen::Matrix4f transformation_mat);

    double computeKmeans(cv::Mat points,
                         const int num_centroids,
                         cv::Mat& labels,
                         cv::Mat& centroids);

    float computeDotProduct(Eigen::Vector4f vector_a, Eigen::Vector4f vector_b);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr compute2DConvexHull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_point_cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr computeNew2DConvexHull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_point_cloud,
                                                                  pcl::PointIndices::Ptr inliers,
                                                                  pcl::ModelCoefficients::Ptr coefficients);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr preprocessPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsamplePointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr distance_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud);
};
