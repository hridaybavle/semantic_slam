#include <iostream>
#include <string>
#include <math.h>
#include <fstream>
#include <random>

#include "ros/ros.h"
#include "eigen3/Eigen/Eigen"
#include <binders.h>

//PCL ros
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

//PCL
#include "pcl/point_types.h"
#include <pcl/features/integral_image_normal.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/ndt.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>

//tools lib
#include "particle_tools.h"

const float MAHA_DIST_THRESHOLD = 1.239;
const float MATCHING_THRESHOLD  = 0.3;

class particle_filter
{

public:
    particle_filter();
    ~particle_filter();

    //particle filter tools object
    particle_filter_tools particle_filter_tools_obj_;

    struct particles {
        int id;
        float x;
        float y;
        float z;
        float roll;
        float pitch;
        float yaw;
        float weight;
    };

    struct landmark_particles {
        int id;
        float x;
        float y;
        float z;
        float weight;
    };

    struct obs_vector
    {
        int obs_id;
        Eigen::Vector3f map_pose;
        float maha_dist;
    };

    struct object_info_struct_pf {
        int id;
        std::string type;
        float prob;
        float num_points;
        Eigen::Vector3f pose;

    };


    struct new_object_info_struct_pf {
        int id;
        std::string type;
        float prob;
        float num_points;
        std::vector<Eigen::Vector3f> pose;

    };

    struct object_info_struct_all_points_pf {
        int id;
        std::string type;
        float prob;
        float num_points;
        pcl::PointCloud<pcl::PointXYZRGB> detected_points;
    };

    struct all_object_info_struct_pf {
        int id;
        std::string type;
        float prob;
        float num_points;
        std::string plane_type;
        Eigen::Vector3f pose;
        Eigen::Vector4f normal_orientation;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_point_cloud_plane;
    };

    struct landmark {
        int id;
        Eigen::Vector3f mu;
        Eigen::MatrixXf sigma;
        std::string type;
        std::string plane_type;
        Eigen::Vector4f normal_orientation;
        float prob;
    };

    struct particle {
        Eigen::VectorXf pose;
        std::vector<landmark> landmarks;
        float weight;
    };

    struct new_landmarks {
        int particle_id;
        int object_id;
    };

private:
    Eigen::MatrixXf Q_;


    bool first_object_;
    bool first_horizontal_plane_,first_vertical_plane;

    bool first_chair_, first_monitor_, first_book_, first_keyboard_, first_laptop_;

    int state_size_, num_particles_;
    std::vector<particles> particles_vec_;
    //std::vector<particles> new_particles_vec_;
    std::vector<particle>  all_particles_;

    Eigen::VectorXf vo_prev_pose_, vo_current_pose_;
    Eigen::MatrixXf jacobian_f_x_, VO_prediction_cov_, VO_prediction_noise_cov_;
    Eigen::Matrix4f pose_increament_;

    std::vector<float> weights_, new_weights_;

    std::vector<object_info_struct_pf> object_map_;
    std::vector<all_object_info_struct_pf> all_object_map_;
    std::vector<object_info_struct_all_points_pf> new_object_map_;

public:
    std::vector<Eigen::VectorXf> init(int state_size,
                                      int num_particles,
                                      std::vector<object_info_struct_pf> mapped_objects);

    void predictionVO(float deltaT,
                      Eigen::VectorXf vo_pose_world,
                      Eigen::VectorXf &final_pose);

    //    void computeJacobian();
    //    void computePredictionNoiseCov();
    void IMUUpdate(float roll,
                   float pitch,
                   float yaw,
                   Eigen::VectorXf& final_pose);

    std::vector<Eigen::VectorXf> ObjectMapAndUpdate(std::vector<object_info_struct_pf> complete_object_pose,
                                                    std::vector<Eigen::VectorXf> filtered_pose,
                                                    Eigen::VectorXf &final_pose,
                                                    Eigen::VectorXf VO_pose,
                                                    std::vector<particle_filter::object_info_struct_pf>& mapped_objects);

    void AllObjectMapAndUpdate(std::vector<all_object_info_struct_pf> complete_object_info, Eigen::VectorXf &final_pose,
                               Eigen::Matrix4f transformation_mat,
                               std::vector<particle_filter::all_object_info_struct_pf>& mapped_objects);

    std::vector<Eigen::VectorXf> newObjectMapAndUpdate(std::vector<object_info_struct_all_points_pf> complete_object_pose,
                                                       std::vector<Eigen::VectorXf> filtered_pose,
                                                       Eigen::VectorXf &final_pose, Eigen::VectorXf VO_pose,
                                                       std::vector<object_info_struct_all_points_pf> &mapped_objects);


    void DataAssociation(std::vector<Eigen::VectorXf>& filtered_pose,
                         std::vector<object_info_struct_pf> complete_objec_info,
                         std::vector<int> &new_landmark);

    void AllDataAssociation(std::vector<all_object_info_struct_pf> complete_object_info,
                            std::vector<new_landmarks>& new_landmark_for_mapping);


    void DataResample(std::vector<Eigen::VectorXf>& filtered_pose, std::vector<object_info_struct_pf> complete_objec_info,
                      Eigen::VectorXf &final_pose,
                      std::vector<particle_filter::object_info_struct_pf>& mapped_objects,
                      std::vector<int> new_landmark);

    void AllDataResample(std::vector<all_object_info_struct_pf> complete_objec_info,
                         Eigen::VectorXf &final_pose,
                         std::vector<particle_filter::all_object_info_struct_pf>& mapped_objects,
                         std::vector<new_landmarks> new_landmarks_for_mapping);

    void LandmarkMeasurementModel(particle p,
                                  landmark new_landmark,
                                  Eigen::VectorXf &h,
                                  Eigen::MatrixXf &H);

    void LandmarkNormalsMeasurementModel(particle p,
                                         landmark new_landmark,
                                         Eigen::Vector4f &h,
                                         Eigen::MatrixXf &H);



    void MapNewLandmarksForEachParticle(std::vector<all_object_info_struct_pf> complete_object_info,
                                        std::vector<new_landmarks>& new_landmarks_for_mapping);


    int MaxIndex();

    std::vector<particle> getAllParticles()
    {
        return all_particles_;
    }



};
