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
    float MAHA_DIST_THRESHOLD;

    void init()
    {
        first_object_ = true;
        landmarks_.clear();
        Q_.setZero();
        Q_(0,0) = 0.9;
        Q_(1,1) = 0.9;
        Q_(2,2) = 0.9;

        MAHA_DIST_THRESHOLD = 1.5;
    }


public:

    std::vector<landmark> find_matches(std::vector<detected_object> seg_obj_info,
                                       Eigen::VectorXf robot_pose,
                                       float cam_angle)
    {
        std::vector<landmark> landmark_vec;
        if(first_object_)
        {
            for(int i = 0; i < seg_obj_info.size(); ++i)
                landmark_vec.push_back(this->map_a_new_lan(seg_obj_info[i], robot_pose, cam_angle));   ;
            if(landmark_vec.size()>0)
                first_object_ = false;
        }
        else
        {
            landmark_vec = this->associate_lanmarks(seg_obj_info, robot_pose, cam_angle);
        }

        std::cout << "landmarks size " << landmarks_.size() << std::endl;

        return landmark_vec;

    }

    std::vector<landmark> associate_lanmarks(std::vector<detected_object> seg_obj_info,
                                             Eigen::VectorXf robot_pose,
                                             float cam_angle)
    {
        bool found_nearest_neighbour = false;
        float maha_distance=0;
        float maha_distance_min = std::numeric_limits<float>::max();

        std::vector<landmark> landmark_vec;


        for(int j=0; j < seg_obj_info.size(); ++j)
        {
            int neareast_landmarks_id;
            for(int i = 0; i < landmarks_.size(); ++i)
            {
                if(seg_obj_info[j].type == landmarks_[i].type)
                {
                    if(seg_obj_info[j].plane_type == landmarks_[i].plane_type)
                    {
                        found_nearest_neighbour = true;
                        //transform the detected object pose to world frame
                        Eigen::Vector4f obj_pose_cam; obj_pose_cam.setOnes();

                        obj_pose_cam << seg_obj_info[j].pose(0), seg_obj_info[j].pose(1), seg_obj_info[j].pose(2);

                        Eigen::Vector4f obj_pose_world = this->convertPoseToWorld(robot_pose,
                                                                                  cam_angle,
                                                                                  obj_pose_cam);

                        //transform detected normals in world
                        Eigen::Vector4f obj_normals_world = this->convertNormalsToWorld(robot_pose,
                                                                                        cam_angle,
                                                                                        seg_obj_info[j].normal_orientation);

                        Eigen::VectorXf expected_meas;
                        Eigen::MatrixXf H   = this->landmarkMeasurementModel(landmarks_[i],
                                                                             expected_meas);
                        //sigma needs to be recovered from the graph someway
                        Eigen::MatrixXf sigma; sigma.resize(3,3);
                        sigma << 0.9,0,  0,
                                0,0.9,0,
                                0,  0,0.9;

                        Eigen::MatrixXf Q = H * sigma * H.transpose() + Q_;

                        Eigen::VectorXf actual_meas; actual_meas.resize(3);
                        actual_meas = obj_pose_world;

                        Eigen::VectorXf z_diff; z_diff.resize(3);
                        z_diff = actual_meas - expected_meas;

                        std::cout << "z_diff " << z_diff << std::endl;

                        maha_distance = z_diff.transpose() * Q.inverse() * z_diff;

                        if(maha_distance < maha_distance_min)
                        {
                            maha_distance_min     = maha_distance;
                            neareast_landmarks_id = i;
                        }

                    }
                }
            }

            //if the type of the object did not get matched
            if(!found_nearest_neighbour)
            {
                landmark_vec.push_back(this->map_a_new_lan(seg_obj_info[j],
                                                           robot_pose,
                                                           cam_angle));
                found_nearest_neighbour = false;
            }
            else if(found_nearest_neighbour)
            {
                std::cout << "maha_distance_min " << maha_distance_min << std::endl;
                if(maha_distance_min > MAHA_DIST_THRESHOLD)
                    landmark_vec.push_back(this->map_a_new_lan(seg_obj_info[j],
                                                               robot_pose,
                                                               cam_angle));
                else
                {
                    landmark_vec.push_back(landmarks_[neareast_landmarks_id]);
                }
            }
        }


        return landmark_vec;

    }

    landmark map_a_new_lan(detected_object seg_obj_info,
                           Eigen::VectorXf robot_pose,
                           float cam_angle)
    {

        //transform the detected object pose to world frame
        Eigen::Vector4f obj_pose_cam; obj_pose_cam.setOnes();

        obj_pose_cam << seg_obj_info.pose(0), seg_obj_info.pose(1), seg_obj_info.pose(2);


        Eigen::Vector4f obj_pose_world = this->convertPoseToWorld(robot_pose,
                                                                  cam_angle,
                                                                  obj_pose_cam);

        //transform detected normals in world
        Eigen::Vector4f obj_normals_world = this->convertNormalsToWorld(robot_pose,
                                                                        cam_angle,
                                                                        seg_obj_info.normal_orientation);

        landmark new_landmark;
        new_landmark.is_new_landmark = true;
        new_landmark.id              = landmarks_.size();
        new_landmark.pose << obj_pose_world(0), obj_pose_world(1), obj_pose_world(2);
        new_landmark.covariance         = Q_;
        new_landmark.normal_orientation = obj_normals_world;
        new_landmark.plane_type         = seg_obj_info.plane_type;
        new_landmark.type               = seg_obj_info.type;
        landmarks_.push_back(new_landmark);

        std::cout << "\033[1;31m new landmark pose \033[0m\n" << new_landmark.pose << std::endl;

        return new_landmark;
    }


    Eigen::MatrixXf convertPoseToWorld(Eigen::VectorXf robot_pose,
                                       float cam_angle,
                                       Eigen::MatrixXf pose_to_transform)
    {

        Eigen::Matrix4f transformation_mat;
        semantic_tools sem_tool_obj;
        sem_tool_obj.transformNormalsToWorld(robot_pose,
                                             transformation_mat,
                                             cam_angle);

        Eigen::Matrix4f transformed_pose;
        transformed_pose.resize(4,4);

        transformed_pose = transformation_mat * pose_to_transform;


        //adding the robots pose
        transformed_pose(0) += robot_pose(0);
        transformed_pose(1) += robot_pose(1);
        transformed_pose(2) += robot_pose(2);

        return transformed_pose;

    }

    Eigen::MatrixXf convertNormalsToWorld(Eigen::VectorXf robot_pose,
                                          float cam_angle,
                                          Eigen::MatrixXf pose_to_transform)
    {

        Eigen::Matrix4f transformation_mat;
        semantic_tools sem_tool_obj;
        sem_tool_obj.transformNormalsToWorld(robot_pose,
                                             transformation_mat,
                                             cam_angle);

        Eigen::Matrix4f transformed_pose;
        transformed_pose.resize(4,4);

        transformed_pose = transformation_mat * pose_to_transform;

        return transformed_pose;
    }

    Eigen::MatrixXf landmarkMeasurementModel(landmark l,
                                             Eigen::VectorXf& h)
    {
        h.resize(3,3); h.setZero();
        h = l.pose;//l.node->estimate().cast<float>();
        std::cout << "h " << h << std::endl;

        Eigen::MatrixXf H; H.resize(3,3); H.setZero();

        H(0,0) =1;
        H(1,1) =1;
        H(2,2) =1;

        return H;

    }

    void assignLandmarkNode(int id, g2o::VertexPointXYZ* node)
    {
        landmarks_[id].node = node;
    }


    void getMappedLandmarks(std::vector<landmark>& l)
    {
        l = landmarks_;
    }
};

#endif
