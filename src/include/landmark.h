#ifndef LANDMARK
#define LANDMARK


#include <iostream>
#include <string>
#include <math.h>
#include <fstream>
#include <random>
#include <thread>
#include <functional>

#include "Eigen/Dense"
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include "ps_graph_slam/keyframe.hpp"

struct landmark {

public:

    int id;
    int vert_id;
    Eigen::Vector3f pose;
    Eigen::Vector3f local_pose;
    Eigen::Matrix3f covariance;
    bool is_new_landmark;
    std::string type;
    std::string plane_type;
    Eigen::Vector4f normal_orientation;
    //pcl::PointCloud<pcl::PointXYZRGB> mapped_planar_points;
    float prob;

    g2o::VertexPointXYZ* node;
};

#endif
