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
#include "ps_graph_slam/map_cloud.h"
#include "tools.h"

class mapping
{

public:
    mapping(float cam_angle)
    {

        std::cout << "Initialized mapping thread " << std::endl;
        this->init(cam_angle);
    }
    ~mapping()
    {

    }

private:
    float cam_angle_;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud_;
    std::vector<map_cloud> map_cloud_vec_;

    std::deque<ps_graph_slam::KeyFrame::Ptr> new_keyframes_;
    std::vector<ps_graph_slam::KeyFrame::Ptr> keyframes_optimized_;
    std::mutex keyframe_lock_, cloud_lock_, opt_keyframe_lock_;


private:

    semantic_tools sem_tool_obj_;

private:
    void init(float cam_angle)
    {
        cam_angle_ = cam_angle;
        out_cloud_.clear();
        map_cloud_vec_.clear();
        new_keyframes_.clear();
    }


public:
    void setKeyframes(std::deque<ps_graph_slam::KeyFrame::Ptr> keyframes)
    {
        keyframe_lock_.lock();
        for(int i = 0; i< keyframes.size(); ++i)
        {
            new_keyframes_.push_back(keyframes[i]);
            keyframe_lock_.unlock();
        }
        keyframe_lock_.unlock();
    }

    void setoptimizedKeyframes(std::vector<ps_graph_slam::KeyFrame::Ptr> keyframes_optimized)
    {
        opt_keyframe_lock_.lock();
        keyframes_optimized_ = keyframes_optimized;
        opt_keyframe_lock_.unlock();
    }

    void generateMap()
    {
        while(1)
        {
            keyframe_lock_.lock();
            std::deque<ps_graph_slam::KeyFrame::Ptr> keyframes;
            keyframes = new_keyframes_;
            new_keyframes_.clear();
            keyframe_lock_.unlock();

            for(int i=0; i< keyframes.size(); ++i)
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
                cloud = this->processPointCloud(keyframes[i]->cloud_msg);

                //pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
                Eigen::MatrixXf pose =  ps_graph_slam::matrix2vector(keyframes[i]->robot_pose.matrix().cast<float>());

                Eigen::Matrix4f transformation_mat;
                sem_tool_obj_.transformMapPointsToWorld(pose,
                                                        transformation_mat,
                                                        cam_angle_);

                for(int j = 0; j< cloud->points.size(); ++j)
                {
                    Eigen::Matrix4f cam_points, map_points;
                    cam_points.setOnes(), map_points.setOnes();

                    cam_points(0) = keyframes[i]->cloud->points[j].x;
                    cam_points(1) = keyframes[i]->cloud->points[j].y;
                    cam_points(2) = keyframes[i]->cloud->points[j].z;

                    map_points = transformation_mat * cam_points;

                    pcl::PointXYZRGB dst_pt;
                    dst_pt.x = map_points(0);
                    dst_pt.y = map_points(1);
                    dst_pt.z = map_points(2);
                    dst_pt.rgb = keyframes[i]->cloud->points[j].rgb;

                    cloud->push_back(dst_pt);
                }

                cloud->width = cloud->size();
                cloud->height = 1;
                cloud->is_dense = false;

                map_cloud current_map_cloud;
                current_map_cloud.keyframe_pose = pose;
                current_map_cloud.out_cloud     = cloud;

                cloud_lock_.lock();
                map_cloud_vec_.push_back(current_map_cloud);
                cloud_lock_.unlock();
                //global_cloud_.push_back(out_cloud);
            }

        }
    }

    void opitmizeMap()
    {
        while(1)
        {
            std::vector<ps_graph_slam::KeyFrame::Ptr> opt_keyframe;
            opt_keyframe_lock_.lock();
            opt_keyframe = keyframes_optimized_;
            keyframes_optimized_.clear();
            opt_keyframe_lock_.unlock();

            cloud_lock_.lock();
            for(int i=0; i < map_cloud_vec_.size(); ++i)
            {
                if(opt_keyframe.size() > 0)
                {
                    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
                    Eigen::MatrixXf pose_opt  = ps_graph_slam::matrix2vector(opt_keyframe[i]->node->estimate().matrix().cast<float>());
                    Eigen::MatrixXf pose_old  = map_cloud_vec_[i].keyframe_pose;
                    Eigen::MatrixXf pose_diff = pose_opt - pose_old;

                    Eigen::Matrix4f transformation_mat;
                    semantic_tools sem_tool_obj;
                    sem_tool_obj.transformRobotToWorld(pose_diff,
                                                       transformation_mat);

                    for(int j = 0; j< map_cloud_vec_[i].out_cloud->points.size(); ++j)
                    {
                        Eigen::Matrix4f old_points, map_points;
                        old_points.setOnes(), map_points.setOnes();

                        old_points(0) = map_cloud_vec_[i].out_cloud->points[j].x;
                        old_points(1) = map_cloud_vec_[i].out_cloud->points[j].y;
                        old_points(2) = map_cloud_vec_[i].out_cloud->points[j].z;

                        //map_points = transformation_mat * old_points;

                        map_cloud_vec_[i].out_cloud->points[j].x = old_points(0) + pose_diff(0);
                        map_cloud_vec_[i].out_cloud->points[j].y = old_points(1) + pose_diff(1);
                        map_cloud_vec_[i].out_cloud->points[j].z = old_points(2) + pose_diff(2);
                        map_cloud_vec_[i].keyframe_pose =  pose_opt;
                    }
                }
            }
            cloud_lock_.unlock();
        }

    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr processPointCloud(sensor_msgs::PointCloud2 point_cloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());

        std::vector<int> indices;
        pcl::fromROSMsg(point_cloud_msg, *cloud);
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
        //donwsampling the point cloud
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (0.2f, 0.2f, 0.2f);
        sor.filter (*cloud);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_out;
        sor_out.setInputCloud (cloud);
        sor_out.setMeanK (50);
        sor_out.setStddevMulThresh (1.0);
        sor_out.filter (*cloud);


        return cloud;
    }

    std::vector<map_cloud> getOutputMap()
    {
        return map_cloud_vec_;
    }



};

#endif
