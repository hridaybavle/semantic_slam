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
#include <pcl/octree/octree_search.h>

#include "ps_graph_slam/ros_utils.hpp"

#include "tools.h"

class mapping
{

public:
    mapping(float cam_angle)
    {

        std::cout << "initialized mapping thread " << std::endl;
        this->init();
    }
    ~mapping()
    {

    }

private:
    float cam_angle_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> global_cloud_;
    std::deque<ps_graph_slam::KeyFrame::Ptr> new_keyframes_;
    std::vector<ps_graph_slam::KeyFrame::Ptr> keyframes_optimized_;
    std::mutex keyframe_lock_, keyframe_lock_2_;


private:
    bool add_new_keyframe_;

private:
    void init()
    {
        global_cloud_.clear();
        out_cloud_ .reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        out_cloud_->points.clear();
        new_keyframes_.clear();
        add_new_keyframe_ = false;
    }


public:
    void setKeyframes(std::deque<ps_graph_slam::KeyFrame::Ptr> keyframes)
    {
        keyframe_lock_2_.lock();
        new_keyframes_ = keyframes;
        keyframe_lock_2_.unlock();
        add_new_keyframe_ = true;
    }

    void optimizedKeyframes(std::vector<ps_graph_slam::KeyFrame::Ptr> keyframes_optimized)
    {
        keyframes_optimized_ = keyframes_optimized;
    }

    void generateMap()
    {
        while(1)
        {
            for(int i=0; i< new_keyframes_.size(); ++i)
            {

                //pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
                Eigen::MatrixXf pose =  ps_graph_slam::matrix2vector(new_keyframes_[i]->robot_pose.matrix().cast<float>());

                Eigen::Matrix4f transformation_mat;
                semantic_tools sem_tool_obj;
                sem_tool_obj.transformNormalsToWorld(pose,
                                                     transformation_mat,
                                                     cam_angle_);

                for(int j = 0; j< new_keyframes_[i]->cloud->points.size(); ++j)
                {
                    Eigen::Matrix4f cam_points, map_points;
                    cam_points.setOnes(), map_points.setOnes();

                    cam_points(0) = new_keyframes_[i]->cloud->points[j].x;
                    cam_points(1) = new_keyframes_[i]->cloud->points[j].y;
                    cam_points(2) = new_keyframes_[i]->cloud->points[j].z;

                    map_points = transformation_mat * cam_points;

                    pcl::PointXYZRGB dst_pt;
                    dst_pt.x = map_points(0) + pose(0);
                    dst_pt.y = map_points(1) + pose(1);
                    dst_pt.z = map_points(2) + pose(2);
                    dst_pt.rgb = new_keyframes_[i]->cloud->points[j].rgb;

                    out_cloud_->push_back(dst_pt);
                }

                out_cloud_->width = out_cloud_->size();
                out_cloud_->height = 1;
                out_cloud_->is_dense = false;
                //global_cloud_.push_back(out_cloud);
            }

            keyframe_lock_.lock();
            new_keyframes_.clear();
            keyframe_lock_.unlock();

        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getOutputMap()
    {
        return out_cloud_;
    }



};

#endif
