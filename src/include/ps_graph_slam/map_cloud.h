#ifndef MAP_CLOUD
#define MAP_CLOUD

#include "Eigen/Dense"

#include "pcl/point_types.h"
#include "ps_graph_slam/keyframe.hpp"

struct map_cloud {

	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud;
	  Eigen::MatrixXf keyframe_pose;

};


#endif

