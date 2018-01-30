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

class plane_segmentation
{
public:
    plane_segmentation();
    ~plane_segmentation();

    sensor_msgs::PointCloud2 segmentPointCloudData(std::vector<semantic_SLAM::ObjectInfo> object_info, sensor_msgs::PointCloud2 point_cloud);


};
