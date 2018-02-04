#include <iostream>
#include <string>
#include <math.h>
#include <mutex>

#include "semantic_SLAM/ObjectInfo.h"

#include "sensor_msgs/PointCloud2.h"

//PCL ROS
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

//PCL
#include "pcl/point_types.h"
#include <pcl/features/integral_image_normal.h>

//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

const int num_centroids_normals = 3;
const int num_centroids_height = 3;

class plane_segmentation
{
public:
    plane_segmentation();
    ~plane_segmentation();

    sensor_msgs::PointCloud2 segmentPointCloudData(std::vector<semantic_SLAM::ObjectInfo> object_info, sensor_msgs::PointCloud2 point_cloud,
                                                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr& segmented_point_cloud_pcl);
     pcl::PointCloud<pcl::Normal>::Ptr computeNormalsFromPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud);
     cv::Mat computeHorizontalPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud, pcl::PointCloud<pcl::Normal>::Ptr point_normal, Eigen::Matrix4f transformation_mat);

};
