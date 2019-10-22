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
        new_keyframes_ = keyframes;
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

            //if(add_new_keyframe_)
            {
                //out_cloud_->reserve(keyframes_.front()->cloud->size() * keyframes_.size());
                Eigen::Matrix4f rot_x_cam, rot_x_robot, rot_z_robot;
                rot_x_cam.setZero(4,4), rot_x_robot.setZero(4,4), rot_z_robot.setZero(4,4);

                rot_x_cam(0,0) = 1;
                rot_x_cam(1,1) =  cos(-cam_angle_);
                rot_x_cam(1,2) = -sin(-cam_angle_);
                rot_x_cam(2,1) =  sin(-cam_angle_);
                rot_x_cam(2,2) =  cos(-cam_angle_);
                rot_x_cam(3,3) = 1;

                //rotation of -90
                rot_x_robot(0,0) = 1;
                rot_x_robot(1,1) =  cos(-1.5708);
                rot_x_robot(1,2) = -sin(-1.5708);
                rot_x_robot(2,1) =  sin(-1.5708);
                rot_x_robot(2,2) =  cos(-1.5708);
                rot_x_robot(3,3) = 1;

                //rotation of -90
                rot_z_robot(0,0) = cos(-1.5708);
                rot_z_robot(0,1) = -sin(-1.5708);
                rot_z_robot(1,0) = sin(-1.5708);
                rot_z_robot(1,1) = cos(-1.5708);
                rot_z_robot(2,2) = 1;
                rot_z_robot(3,3) = 1;

                Eigen::Matrix4f robot_pose = rot_z_robot * rot_x_robot * rot_x_cam;

                for(int i=0; i< new_keyframes_.size(); ++i)
                {
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
                    Eigen::Matrix4f pose = new_keyframes_[i]->node->estimate().matrix().cast<float>();

                    for(int j = 0; j< new_keyframes_[i]->cloud->points.size(); ++j)
                    {

                        double roll, pitch, yaw;
                        Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
                        quat.normalize();

                        //converting quaternions to euler angles
                        tf::Quaternion q(quat.x(), quat.y(), quat.z(), quat.w());
                        tf::Matrix3x3 m(q);
                        m.getRPY(roll, pitch, yaw);

                        //transformation from robot to world
                        Eigen::Matrix4f T_robot_world; T_robot_world.setZero(4,4);
                        T_robot_world(0,0) = cos(yaw)*cos(pitch);
                        T_robot_world(0,1) = cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll);
                        T_robot_world(0,2) = cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(pitch);

                        T_robot_world(1,0) = sin(yaw)*cos(pitch);
                        T_robot_world(1,1) = sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll);
                        T_robot_world(1,2) = sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll);

                        T_robot_world(2,0) = -sin(pitch);
                        T_robot_world(2,1) = cos(pitch)*sin(roll);
                        T_robot_world(2,2) = cos(pitch)*cos(roll);
                        T_robot_world(3,3) = 1;


                        pcl::PointXYZRGB dst_pt;
                        dst_pt.getVector4fMap() = T_robot_world * robot_pose * new_keyframes_[i]->cloud->points[j].getVector4fMap();
                        dst_pt.x += pose(0,3);
                        dst_pt.y += pose(1,3);
                        dst_pt.z += pose(2,3);
                        dst_pt.rgb = new_keyframes_[i]->cloud->points[j].rgb;

                        out_cloud_->push_back(dst_pt);
                    }

                    out_cloud_->width = out_cloud_->size();
                    out_cloud_->height = 1;
                    out_cloud_->is_dense = false;
                    //global_cloud_.push_back(out_cloud);
                }

                add_new_keyframe_ = false;
                new_keyframes_.clear();
            }
            //            else
            //            {

            //                for(int i = 0; i < 10; ++i)
            //                {
            //                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
            //                    cloud = global_cloud_.back() - i;
            //                    ps_graph_slam::KeyFrame::Ptr keyframe (new ps_graph_slam::KeyFrame::Ptr);
            //                    keyframe = keyframes_optimized_.back() - i;

            //                    Eigen::Matrix4f pose = keyframe->node->estimate().matrix().cast<float>();

            //                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_opt (new pcl::PointCloud<pcl::PointXYZRGB>);
            //                    for(int j = 0; j< cloud->points.size(); ++j)
            //                    {
            //                        double roll, pitch, yaw;
            //                        Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
            //                        quat.normalize();

            //                        //converting quaternions to euler angles
            //                        tf::Quaternion q(quat.x(), quat.y(), quat.z(), quat.w());
            //                        tf::Matrix3x3 m(q);
            //                        m.getRPY(roll, pitch, yaw);

            //                        //transformation from robot to world
            //                        Eigen::Matrix4f T_robot_world; T_robot_world.setZero(4,4);
            //                        T_robot_world(0,0) = cos(yaw)*cos(pitch);
            //                        T_robot_world(0,1) = cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll);
            //                        T_robot_world(0,2) = cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(pitch);

            //                        T_robot_world(1,0) = sin(yaw)*cos(pitch);
            //                        T_robot_world(1,1) = sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll);
            //                        T_robot_world(1,2) = sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll);

            //                        T_robot_world(2,0) = -sin(pitch);
            //                        T_robot_world(2,1) = cos(pitch)*sin(roll);
            //                        T_robot_world(2,2) = cos(pitch)*cos(roll);
            //                        T_robot_world(3,3) = 1;


            //                        pcl::PointXYZRGB dst_pt;
            //                        dst_pt.getVector4fMap() = T_robot_world * robot_pose * new_keyframes_[i]->cloud->points[j].getVector4fMap();
            //                        dst_pt.x += pose(0,3);
            //                        dst_pt.y += pose(1,3);
            //                        dst_pt.z += pose(2,3);
            //                        cloud_opt->push_back(dst_pt);
            //                    }

            //                    cloud_opt->width = out_cloud->size();
            //                    cloud_opt->height = 1;
            //                    cloud_opt->is_dense = false;

            //                    global_cloud_[].push_back(out_cloud);

            //                }
            //            }


        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getOutputMap()
    {
        return out_cloud_;
    }



};

#endif
