#ifndef SEMANTIC_TOOLS_H
#define SEMANTIC_TOOLS_H

#include "eigen3/Eigen/Eigen"
#include <binders.h>
#include <iostream>
#include <math.h>
#include <mutex>
#include <string>

class semantic_tools {
public:
  semantic_tools() {}

  ~semantic_tools() {}

public:
  void transformNormalsToWorld(Eigen::VectorXf final_pose,
                               Eigen::Matrix4f &transformation_mat,
                               const float real_sense_pitch_angle) {
    Eigen::Matrix4f rot_x_cam, rot_x_robot, rot_z_robot, translation_cam,
        T_robot_world;
    rot_x_cam.setZero(4, 4), rot_x_robot.setZero(4, 4),
        rot_z_robot.setZero(4, 4), translation_cam.setZero(4, 4),
        T_robot_world.setZero(4, 4);

    float x, y, z, roll, pitch, yaw;

    x = final_pose(0);
    y = final_pose(1);
    z = final_pose(2);
    roll = final_pose(3);
    pitch = final_pose(4);
    yaw = final_pose(5);

    // pitch angle correction:
    //        std::cout << "X: " << x << std::endl
    //                  << "Y: " << y << std::endl
    //                  << "Z: " << z << std::endl
    //                  << "roll: " << roll << std::endl
    //                  << "pitch: " << pitch << std::endl
    //                  << "yaw: "   << yaw << std::endl;

    rot_x_cam(0, 0) = 1;
    rot_x_cam(1, 1) = cos(-real_sense_pitch_angle);
    rot_x_cam(1, 2) = -sin(-real_sense_pitch_angle);
    rot_x_cam(2, 1) = sin(-real_sense_pitch_angle);
    rot_x_cam(2, 2) = cos(-real_sense_pitch_angle);
    rot_x_cam(3, 3) = 1;

    // rotation of -90
    rot_x_robot(0, 0) = 1;
    rot_x_robot(1, 1) = cos(-1.5708);
    rot_x_robot(1, 2) = -sin(-1.5708);
    rot_x_robot(2, 1) = sin(-1.5708);
    rot_x_robot(2, 2) = cos(-1.5708);
    rot_x_robot(3, 3) = 1;

    // rotation of -90
    rot_z_robot(0, 0) = cos(-1.5708);
    rot_z_robot(0, 1) = -sin(-1.5708);
    rot_z_robot(1, 0) = sin(-1.5708);
    rot_z_robot(1, 1) = cos(-1.5708);
    rot_z_robot(2, 2) = 1;
    rot_z_robot(3, 3) = 1;

    // translation of -10cm in x, 10cm in y and -5cm in z
    //        translation_cam(0,0) = 1;
    //        translation_cam(0,3) = 0;
    //        translation_cam(1,1) = 1;
    //        translaton_cam(1,3) = -0.1;
    //        translation_cam(2,2) = 1;
    //        translation_cam(2,3) = 0.0;
    //        translation_cam(3,3) = 1;

    // transformation from robot to world
    T_robot_world(0, 0) = cos(yaw) * cos(pitch);
    T_robot_world(0, 1) =
        cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
    T_robot_world(0, 2) =
        cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(pitch);

    T_robot_world(1, 0) = sin(yaw) * cos(pitch);
    T_robot_world(1, 1) =
        sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
    T_robot_world(1, 2) =
        sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll);

    T_robot_world(2, 0) = -sin(pitch);
    T_robot_world(2, 1) = cos(pitch) * sin(roll);
    T_robot_world(2, 2) = cos(pitch) * cos(roll);
    T_robot_world(3, 3) = 1;

    // fill the x, y and z variables over here if there is a fixed transform
    // between the camera and the world frame
    //    T_robot_world(0,3) = x;
    //    T_robot_world(1,3) = y;
    //    T_robot_world(2,3) = z;
    //    T_robot_world(3,3) = 1;

    transformation_mat = T_robot_world * rot_z_robot * rot_x_robot * rot_x_cam;
  }

  void transformPoseFromCameraToRobot(Eigen::Matrix4f &transformation_mat,
                                      const float real_sense_pitch_angle) {

    Eigen::Matrix4f rot_x_cam, rot_x_robot, rot_z_robot, translation_cam;
    rot_x_cam.setZero(4, 4), rot_x_robot.setZero(4, 4),
        rot_z_robot.setZero(4, 4), translation_cam.setZero(4, 4);

    rot_x_cam(0, 0) = 1;
    rot_x_cam(1, 1) = cos(-real_sense_pitch_angle);
    rot_x_cam(1, 2) = -sin(-real_sense_pitch_angle);
    rot_x_cam(2, 1) = sin(-real_sense_pitch_angle);
    rot_x_cam(2, 2) = cos(-real_sense_pitch_angle);
    rot_x_cam(3, 3) = 1;

    // rotation of -90
    rot_x_robot(0, 0) = 1;
    rot_x_robot(1, 1) = cos(-1.5708);
    rot_x_robot(1, 2) = -sin(-1.5708);
    rot_x_robot(2, 1) = sin(-1.5708);
    rot_x_robot(2, 2) = cos(-1.5708);
    rot_x_robot(3, 3) = 1;

    // rotation of -90
    rot_z_robot(0, 0) = cos(-1.5708);
    rot_z_robot(0, 1) = -sin(-1.5708);
    rot_z_robot(1, 0) = sin(-1.5708);
    rot_z_robot(1, 1) = cos(-1.5708);
    rot_z_robot(2, 2) = 1;
    rot_z_robot(3, 3) = 1;

    transformation_mat = rot_z_robot * rot_x_robot * rot_x_cam;
  }

  void transformIMUtoWorld(float ax, float ay, float az,
                           Eigen::Matrix4f &transformation_mat) {
    Eigen::Matrix4f rot_x_imu, T_robot_world;
    double roll, pitch, yaw;

    // for now considering roll, pitch and yaw zero
    roll = pitch = yaw = 0;

    rot_x_imu.setZero(), T_robot_world.setZero();

    // rotation of -180 to convert to robot frame
    rot_x_imu(0, 0) = 1;
    rot_x_imu(1, 1) = cos(-3.14);
    rot_x_imu(1, 2) = -sin(-3.14);
    rot_x_imu(2, 1) = sin(-3.14);
    rot_x_imu(2, 2) = cos(-3.14);
    rot_x_imu(3, 3) = 1;

    // rotation of the robot with respect to the world
    T_robot_world(0, 0) = cos(yaw) * cos(pitch);
    T_robot_world(0, 1) =
        cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
    T_robot_world(0, 2) =
        cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(pitch);

    T_robot_world(1, 0) = sin(yaw) * cos(pitch);
    T_robot_world(1, 1) =
        sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
    T_robot_world(1, 2) =
        sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll);

    T_robot_world(2, 0) = -sin(pitch);
    T_robot_world(2, 1) = cos(pitch) * sin(roll);
    T_robot_world(2, 2) = cos(pitch) * cos(roll);
    T_robot_world(3, 3) = 1;

    transformation_mat = T_robot_world * rot_x_imu;
  }

  void transformRobotToWorld(Eigen::VectorXf pose,
                             Eigen::Matrix4f &transformation_mat) {
    Eigen::Matrix4f T_robot_world;
    float roll = pose(3);
    float pitch = pose(4);
    float yaw = pose(5);

    // rotation of the robot with respect to the world
    T_robot_world(0, 0) = cos(yaw) * cos(pitch);
    T_robot_world(0, 1) =
        cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
    T_robot_world(0, 2) =
        cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(pitch);

    T_robot_world(1, 0) = sin(yaw) * cos(pitch);
    T_robot_world(1, 1) =
        sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
    T_robot_world(1, 2) =
        sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll);

    T_robot_world(2, 0) = -sin(pitch);
    T_robot_world(2, 1) = cos(pitch) * sin(roll);
    T_robot_world(2, 2) = cos(pitch) * cos(roll);
    T_robot_world(3, 3) = 1;

    transformation_mat = T_robot_world;
  }

  void transformMapPointsToWorld(Eigen::VectorXf final_pose,
                                 Eigen::Matrix4f &transformation_mat,
                                 const float real_sense_pitch_angle) {
    Eigen::Matrix4f rot_x_cam, rot_x_robot, rot_z_robot, translation_cam,
        T_robot_world;
    rot_x_cam.setZero(4, 4), rot_x_robot.setZero(4, 4),
        rot_z_robot.setZero(4, 4), translation_cam.setZero(4, 4),
        T_robot_world.setZero(4, 4);

    float x, y, z, roll, pitch, yaw;

    x = final_pose(0);
    y = final_pose(1);
    z = final_pose(2);
    roll = final_pose(3);
    pitch = final_pose(4);
    yaw = final_pose(5);

    // pitch angle correction:
    //        std::cout << "X: " << x << std::endl
    //                  << "Y: " << y << std::endl
    //                  << "Z: " << z << std::endl
    //                  << "roll: " << roll << std::endl
    //                  << "pitch: " << pitch << std::endl
    //                  << "yaw: "   << yaw << std::endl;

    rot_x_cam(0, 0) = 1;
    rot_x_cam(1, 1) = cos(-real_sense_pitch_angle);
    rot_x_cam(1, 2) = -sin(-real_sense_pitch_angle);
    rot_x_cam(2, 1) = sin(-real_sense_pitch_angle);
    rot_x_cam(2, 2) = cos(-real_sense_pitch_angle);
    rot_x_cam(3, 3) = 1;

    // rotation of -90
    rot_x_robot(0, 0) = 1;
    rot_x_robot(1, 1) = cos(-1.5708);
    rot_x_robot(1, 2) = -sin(-1.5708);
    rot_x_robot(2, 1) = sin(-1.5708);
    rot_x_robot(2, 2) = cos(-1.5708);
    rot_x_robot(3, 3) = 1;

    // rotation of -90
    rot_z_robot(0, 0) = cos(-1.5708);
    rot_z_robot(0, 1) = -sin(-1.5708);
    rot_z_robot(1, 0) = sin(-1.5708);
    rot_z_robot(1, 1) = cos(-1.5708);
    rot_z_robot(2, 2) = 1;
    rot_z_robot(3, 3) = 1;

    // translation of -10cm in x, 10cm in y and -5cm in z
    //        translation_cam(0,0) = 1;
    //        translation_cam(0,3) = 0;
    //        translation_cam(1,1) = 1;
    //        translaton_cam(1,3) = -0.1;
    //        translation_cam(2,2) = 1;
    //        translation_cam(2,3) = 0.0;
    //        translation_cam(3,3) = 1;

    // transformation from robot to world
    T_robot_world(0, 0) = cos(yaw) * cos(pitch);
    T_robot_world(0, 1) =
        cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
    T_robot_world(0, 2) =
        cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(pitch);
    T_robot_world(0, 3) = x;

    T_robot_world(1, 0) = sin(yaw) * cos(pitch);
    T_robot_world(1, 1) =
        sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
    T_robot_world(1, 2) =
        sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll);
    T_robot_world(1, 3) = y;

    T_robot_world(2, 0) = -sin(pitch);
    T_robot_world(2, 1) = cos(pitch) * sin(roll);
    T_robot_world(2, 2) = cos(pitch) * cos(roll);
    T_robot_world(2, 3) = z;
    T_robot_world(3, 3) = 1;

    // fill the x, y and z variables over here if there is a fixed transform
    // between the camera and the world frame
    //    T_robot_world(0,3) = x;
    //    T_robot_world(1,3) = y;
    //    T_robot_world(2,3) = z;
    //    T_robot_world(3,3) = 1;

    transformation_mat = T_robot_world * rot_z_robot * rot_x_robot * rot_x_cam;
  }

  inline float dist(float x1, float x2, float y1, float y2, float z1,
                    float z2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) +
                (z2 - z1) * (z2 - z1));
  }

  static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R) {
    Eigen::Vector3d n = R.col(0);
    Eigen::Vector3d o = R.col(1);
    Eigen::Vector3d a = R.col(2);

    Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r =
        atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr / M_PI * 180.0;
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 3, 3>
  ypr2R(const Eigen::MatrixBase<Derived> &ypr) {
    typedef typename Derived::Scalar Scalar_t;

    Scalar_t y = ypr(0) / 180.0 * M_PI;
    Scalar_t p = ypr(1) / 180.0 * M_PI;
    Scalar_t r = ypr(2) / 180.0 * M_PI;

    Eigen::Matrix<Scalar_t, 3, 3> Rz;
    Rz << cos(y), -sin(y), 0, sin(y), cos(y), 0, 0, 0, 1;

    Eigen::Matrix<Scalar_t, 3, 3> Ry;
    Ry << cos(p), 0., sin(p), 0., 1., 0., -sin(p), 0., cos(p);

    Eigen::Matrix<Scalar_t, 3, 3> Rx;
    Rx << 1., 0., 0., 0., cos(r), -sin(r), 0., sin(r), cos(r);

    return Rz * Ry * Rx;
  }
};

#endif
