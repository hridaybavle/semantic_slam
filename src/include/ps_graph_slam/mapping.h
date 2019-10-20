#ifndef MAPPING_H
#define MAPPING_H

#include <Eigen/Dense>

#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include "ps_graph_slam/keyframe.hpp"

//PCL ROS
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include "pcl/point_types.h"
#include <pcl/filters/project_inliers.h>


class mapping
{

public:
    mapping()
    {

    }
    ~mapping()
    {

    }


public:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr generateMap(std::vector<ps_graph_slam::KeyFrame::Ptr> keyframes)
    {
        if(keyframes.empty())
            return nullptr;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

        for(int i=0; i< keyframes.size(); ++i)
        {
            Eigen::Matrix4f pose = keyframes[i]->node->estimate().matrix().cast<float>();

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            std::vector<int> indices;
            pcl::fromROSMsg(keyframes[i]->cloud_msg, *cloud);
            pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

            for(const auto& src_pt : cloud->points)
            {
                pcl::PointXYZRGB dst_pt;
                dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
                out_cloud->push_back(dst_pt);
            }

       }

        out_cloud->width = out_cloud->size();
        out_cloud->height = 1;
        out_cloud->is_dense = false;

        return out_cloud;
    }



};

#endif
