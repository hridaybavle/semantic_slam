#ifndef MAP_CLOUD
#define MAP_CLOUD

#include "Eigen/Dense"

#include "pcl/point_types.h"
#include "ps_graph_slam/keyframe.hpp"

struct map_cloud {

	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud;
      Eigen::Matrix4f keyframe_pose;

};


#endif

