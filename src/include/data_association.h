#ifndef DATA_ASSOCIATION
#define DATA_ASSOCIATION

#include <iostream>
#include <string>
#include <math.h>
#include <fstream>
#include <random>
#include <thread>
#include <functional>
#include "eigen3/Eigen/Eigen"

#include "tools.h"
#include "detected_object.h"
#include "landmark.h"

class data_association {

public:
    data_association()
    {
        std::cout << "data association constructort " << std::endl;

        init();

    }
    ~data_association()
    {
        std::cout << "data association destructor " << std::endl;
    }


private:
    bool first_object_;
    std::vector<landmark> landmarks_;
    Eigen::Matrix3f Q_;

    void init()
    {
        first_object_ = true;
        landmarks_.clear();
        Q_.setZero();
        Q_(0,0) = 0.9;
        Q_(1,1) = 0.9;
        Q_(2,2) = 0.9;
    }


public:

    std::vector<landmark> find_matches(std::vector<detected_object> seg_obj_info,
                                       Eigen::VectorXf robot_pose,
                                       float cam_angle)
    {
        std::vector<landmark> landmark_vec;
        if(first_object_)
        {
            landmark_vec  =  map_a_new_lan(seg_obj_info, robot_pose, cam_angle);
            if(landmark_vec.size()>0)
                first_object_ = false;
        }
        else
        {
            this->associate_lanmarks();
        }

        return landmark_vec;

    }

    std::vector<landmark> map_a_new_lan(std::vector<detected_object> seg_obj_info,
                                        Eigen::VectorXf robot_pose,
                                        float cam_angle)
    {

        std::vector<landmark> landmark_vec;
        for(int i = 0; i < seg_obj_info.size(); ++i)
        {
            //transform the detected object pose to world frame
            Eigen::Vector4f lan_pose_cam; lan_pose_cam.setOnes();

            lan_pose_cam << seg_obj_info[i].pose(0), seg_obj_info[i].pose(1), seg_obj_info[i].pose(2);

            Eigen::Vector4f lan_pose_world = this->convertToWorld(robot_pose,
                                                                  cam_angle,
                                                                  lan_pose_cam);

            //transform detected normals in world
            Eigen::Vector4f lan_normals_world = this->convertToWorld(robot_pose,
                                                                     cam_angle,
                                                                     seg_obj_info[i].normal_orientation);

            landmark new_landmark;
            new_landmark.is_new_landmark = true;
            new_landmark.pose << lan_pose_world(0), lan_pose_world(1), lan_pose_world(2);
            new_landmark.covariance         = Q_;
            new_landmark.normal_orientation = lan_normals_world;
            new_landmark.plane_type         = seg_obj_info[i].plane_type;
            new_landmark.type               = seg_obj_info[i].type;
            landmark_vec.push_back(new_landmark);
            landmarks_.push_back(new_landmark);
        }

        return landmark_vec;
    }

    void associate_lanmarks()
    {

    }


    Eigen::MatrixXf convertToWorld(Eigen::VectorXf robot_pose,
                                   float cam_angle,
                                   Eigen::MatrixXf pose_to_transform)
    {

        Eigen::Matrix4f transformation_mat;
        semantic_tools sem_tool_obj;
        sem_tool_obj.transformNormalsToWorld(robot_pose,
                                             transformation_mat,
                                             cam_angle);

        Eigen::Matrix4f transformed_pose;
        pose_to_transform.resize(4,4);

        transformed_pose = transformation_mat * pose_to_transform;

        return transformed_pose;

    }


};

#endif
