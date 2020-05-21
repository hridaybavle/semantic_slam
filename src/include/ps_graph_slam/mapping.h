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
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>());
                cloud_in = this->processPointCloud(keyframes[i]->cloud_msg);

                //pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
                Eigen::MatrixXf pose =  ps_graph_slam::matrix2vector(keyframes[i]->robot_pose.matrix().cast<float>());

                Eigen::Matrix4f transformation_mat;
                sem_tool_obj_.transformMapPointsToWorld(pose,
                                                        transformation_mat,
                                                        cam_angle_);

                for(int j = 0; j< cloud_in->points.size(); ++j)
                {
                    Eigen::Matrix4f cam_points, map_points;
                    cam_points.setOnes(), map_points.setOnes();

                    cam_points(0) = cloud_in->points[j].x;
                    cam_points(1) = cloud_in->points[j].y;
                    cam_points(2) = cloud_in->points[j].z;

                    map_points = transformation_mat * cam_points;

                    pcl::PointXYZRGB dst_pt;
                    dst_pt.x = map_points(0);
                    dst_pt.y = map_points(1);
                    dst_pt.z = map_points(2);
                    dst_pt.rgb = cloud_in->points[j].rgb;

                    cloud_out->push_back(dst_pt);
                }

                cloud_out->width = cloud_in->size();
                cloud_out->height = 1;
                cloud_out->is_dense = false;

                map_cloud current_map_cloud;
                current_map_cloud.keyframe_pose = keyframes[i]->robot_pose.matrix().cast<float>();
                current_map_cloud.out_cloud     = cloud_out;

                cloud_lock_.lock();
                map_cloud_vec_.push_back(current_map_cloud);
                cloud_lock_.unlock();
            }
            std::chrono::milliseconds dur(5);
            std::this_thread::sleep_for(dur);
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
                    Eigen::VectorXf pose_opt  = ps_graph_slam::matrix2vector(opt_keyframe[i]->node->estimate().matrix().cast<float>());
                    Eigen::VectorXf pose_old  = ps_graph_slam::matrix2vector(map_cloud_vec_[i].keyframe_pose);

                    //Eigen::Matrix3f r_old = map_cloud_vec_[i].keyframe_pose.block<3,3>(0,0);
                    //Eigen::Matrix3f r_opt = opt_keyframe[i]->node->estimate().matrix().cast<float>().block<3,3>(0,0);
                    //double yaw_drift = semantic_tools::R2ypr(r_opt).x() - semantic_tools::R2ypr(r_old).x();

                    Eigen::VectorXf pose_diff = pose_opt - pose_old;

                    //Eigen::Matrix4f pose_diff = opt_keyframe[i]->node->estimate().matrix().inverse().cast<float>() * map_cloud_vec_[i].keyframe_pose;

                    for(int j = 0; j< map_cloud_vec_[i].out_cloud->points.size(); ++j)
                    {

                        Eigen::Vector4f old_points, map_points;
                        old_points.setOnes(), map_points.setOnes();

                        old_points(0) = map_cloud_vec_[i].out_cloud->points[j].x;
                        old_points(1) = map_cloud_vec_[i].out_cloud->points[j].y;
                        old_points(2) = map_cloud_vec_[i].out_cloud->points[j].z;

                        //map_points = pose_diff * old_points;

                        map_cloud_vec_[i].out_cloud->points[j].x = old_points(0) + pose_diff(0);
                        map_cloud_vec_[i].out_cloud->points[j].y = old_points(1) + pose_diff(1);
                        map_cloud_vec_[i].out_cloud->points[j].z = old_points(2) + pose_diff(2);
                        map_cloud_vec_[i].keyframe_pose =  opt_keyframe[i]->node->estimate().matrix().cast<float>();
                    }
                }
            }
            cloud_lock_.unlock();
            std::chrono::milliseconds dur(5);
            std::this_thread::sleep_for(dur);
        }

    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr processPointCloud(sensor_msgs::PointCloud2 point_cloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());

        if(point_cloud_msg.data.empty())
            return cloud;

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
