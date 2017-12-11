#include "semantic_SLAM.h"

semantic_SLAM::semantic_SLAM()
{
    std::cout << "semantic SLAM constructor " << std::endl;
}

semantic_SLAM::~semantic_SLAM()
{
    std::cout << "semantic SLAM destructor " << std::endl;
}

void semantic_SLAM::init()
{

    return;

}

void semantic_SLAM::open(ros::NodeHandle n)
{
    //ros subsriber
    stereo_odometry_sub_ = n.subscribe("/stereo_odometer/pose", 1, &semantic_SLAM::stereoOdometryCallback, this);

}

void semantic_SLAM::stereoOdometryCallback(const geometry_msgs::PoseStamped &msg)
{


    Eigen::Vector4f camera_pose_mat, world_pose_mat;
    Eigen::Matrix4f transformation_mat;

    camera_pose_mat.setOnes(), world_pose_mat.setOnes();
    //assume roll, pitch and yaw zero for now//
    this->transformCameraToWorld(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,0,0,0, transformation_mat);

    camera_pose_mat(0) = msg.pose.position.x;
    camera_pose_mat(1) = msg.pose.position.y;
    camera_pose_mat(2) = msg.pose.position.z;

    world_pose_mat = transformation_mat * camera_pose_mat;

    std::cout << "world pose mat " << std::endl
              << "x: " << world_pose_mat(0) << std::endl
              << "y: " << world_pose_mat(1) << std::endl
              << "z: " << world_pose_mat(2) << std::endl;

}

void semantic_SLAM::transformCameraToWorld(float x, float y, float z,
                                           float roll, float pitch, float yaw,
                                           Eigen::Matrix4f &transformation_mat)
{

    Eigen::Matrix4f rot_x_cam, rot_x_robot, rot_z_robot, T_robot_world, translation_cam;
    rot_x_cam.setZero(4,4), rot_x_robot.setZero(4,4), rot_z_robot.setZero(4,4), T_robot_world.setZero(4,4), translation_cam.setZero(4,4);

    //    rot_x_cam(0,0) = 1;
    //    rot_x_cam(1,1) =  cos(-camera_pitch_angle_);
    //    rot_x_cam(1,2) = -sin(-camera_pitch_angle_);
    //    rot_x_cam(2,1) =  sin(-camera_pitch_angle_);
    //    rot_x_cam(2,2) =  cos(-camera_pitch_angle_);
    //    rot_x_cam(3,3) = 1;

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

    //    //tranlation of 0cm in
    //    translation_cam(0,0) = 1;
    //    translation_cam(0,3) = 0.0;

    //    translation_cam(1,1) = 1;
    //    translation_cam(2,2) = 1;
    //    translation_cam(3,3) = 1;


    //transformation from robot to world
    T_robot_world(0,0) = cos(yaw)*cos(pitch);
    T_robot_world(0,1) = cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll);
    T_robot_world(0,2) = cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(pitch);

    T_robot_world(1,0) = sin(yaw)*cos(pitch);
    T_robot_world(1,1) = sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll);
    T_robot_world(1,2) = sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll);

    T_robot_world(2,0) = -sin(pitch);
    T_robot_world(2,1) = cos(pitch)*sin(roll);
    T_robot_world(2,2) = cos(pitch)*cos(roll);

    //fill the x, y and z variables over here if there is a fixed transform between the camera and the world frame
    //currently its they are all zero as the pose is obtained of the camera with respect to the world.
    //    T_robot_world(0,3) = prev_pose_x_;
    //    T_robot_world(1,3) = prev_pose_y_;
    //    T_robot_world(2,3) = prev_pose_z_;
    T_robot_world(3,3) = 1;

    transformation_mat = T_robot_world * rot_z_robot * rot_x_robot ;


    //std::cout << "transformation matrix " << transformation_mat << std::endl;

}
