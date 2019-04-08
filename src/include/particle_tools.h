#include <iostream>
#include <string>
#include <math.h>
#include <mutex>
#include "eigen3/Eigen/Eigen"
#include <binders.h>


class particle_filter_tools
{
public:
    particle_filter_tools(){}
    ~particle_filter_tools(){}


public:
    inline float dist(float x1, float x2, float y1, float y2, float z1, float z2)
    {
        return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) *  (z2 - z1));
    }

    inline float squared_dist(double x1, double x2, double y1, double y2, double z1, double z2)
    {
        return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) *  (z2 - z1));
    }


public:

    Eigen::VectorXf computeVectorInv(Eigen::VectorXf pose)
    {
        Eigen::MatrixXf inv_pose_mat;
        inv_pose_mat = this->invTrans(pose(0), pose(1), pose(2), pose(3), pose(4), pose(5));

        Eigen::VectorXf inv_pose_vec;
        inv_pose_vec = this->compVecFromMat(inv_pose_mat);

        return inv_pose_vec;

    }

    float crossProduct(Eigen::Vector3f vector_a, Eigen::Vector3f vector_b)
    {

    }

    Eigen::Matrix3f transformNormalsToWorld(Eigen::VectorXf particle_pose)
    {
        Eigen::Matrix3f rot_x_cam, rot_x_robot, rot_z_robot, translation_cam, T_robot_world, transformation_mat;
        rot_x_cam.setZero(3,3), rot_x_robot.setZero(3,3), rot_z_robot.setZero(3,3), translation_cam.setZero(3,3), T_robot_world.setZero(3,3);

        float x, y, z, roll, pitch, yaw;

        x = particle_pose(0);
        y = particle_pose(1);
        z = particle_pose(2);
        roll = particle_pose(3);
        pitch = particle_pose(4);
        yaw = particle_pose(5);


        //rotation of -90
        rot_x_robot(0,0) = 1;
        rot_x_robot(1,1) =  cos(-1.5708);
        rot_x_robot(1,2) = -sin(-1.5708);
        rot_x_robot(2,1) =  sin(-1.5708);
        rot_x_robot(2,2) =  cos(-1.5708);

        //rotation of -90
        rot_z_robot(0,0) = cos(-1.5708);
        rot_z_robot(0,1) = -sin(-1.5708);
        rot_z_robot(1,0) = sin(-1.5708);
        rot_z_robot(1,1) = cos(-1.5708);
        rot_z_robot(2,2) = 1;


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

        transformation_mat = T_robot_world * rot_z_robot * rot_x_robot;

        return transformation_mat;
    }


    Eigen::VectorXf computeDiffUsingHomoMethod(Eigen::VectorXf prev_pose, Eigen::VectorXf current_pose)
    {
        Eigen::MatrixXf prev_pose_mat, current_pose_mat, pose_multiplied_mat;
        pose_multiplied_mat.resize(4,4);

        prev_pose_mat = this->compHomoTrans(prev_pose(0), prev_pose(1), prev_pose(2), prev_pose(3), prev_pose(4), prev_pose(5));
        current_pose_mat = this->compHomoTrans(current_pose(0),current_pose(1),current_pose(2),current_pose(3),current_pose(4),current_pose(5));

        pose_multiplied_mat = prev_pose_mat * current_pose_mat;

        Eigen::VectorXf pose_multiplied_vec;
        pose_multiplied_vec = compVecFromMat(pose_multiplied_mat);

        return pose_multiplied_vec;

    }


    Eigen::MatrixXf compHomoTrans(float ox, float oy, float oz, float r, float p, float y)
    {
        Eigen::MatrixXf mat;
        mat.resize(4,4);

        mat(0,0)=cos(r)*cos(p); mat(0,1)=cos(r)*sin(p)*sin(y)-sin(r)*cos(y); mat(0,2)=cos(r)*sin(p)*cos(y)+sin(r)*sin(y); mat(0,3)=ox;
        mat(1,0)=sin(r)*cos(p); mat(1,1)=sin(r)*sin(p)*sin(y)+cos(r)*cos(y); mat(1,2)=sin(r)*sin(p)*cos(y)-cos(r)*sin(y); mat(1,3)=oy;
        mat(2,0)=-sin(p);		mat(2,1)=cos(p)*sin(y);					     mat(2,2)=cos(p)*cos(y);                      mat(2,3)=oz;
        mat(3,0)=0;             mat(3,1)=0;								     mat(3,2)=0;								  mat(3,3)=1;

        return mat;
    }

    Eigen::MatrixXf invTrans(float ox, float oy, float oz, float r, float p, float y)
    {
        Eigen::MatrixXf mat;
        mat.resize(4,4);

        mat(0,0)=cos(r)*cos(p);					     mat(0,1)=sin(r)*cos(p);                      mat(0,2)=-sin(p);	    	mat(0,3)=-ox*cos(r)*cos(p)-oy*sin(r)*cos(p)+oz*sin(p);
        mat(1,0)=cos(r)*sin(p)*sin(y)-sin(r)*cos(y); mat(1,1)=sin(r)*sin(p)*sin(y)+cos(r)*cos(y); mat(1,2)=cos(p)*sin(y); 	mat(1,3)=ox*(sin(r)*cos(y)-cos(r)*sin(p)*sin(y))-oy*(cos(r)*cos(y)+sin(p)*sin(r)*sin(y))-oz*cos(p)*sin(y);
        mat(2,0)=cos(r)*sin(p)*cos(y)+sin(r)*sin(y); mat(2,1)=sin(r)*sin(p)*cos(y)-cos(r)*sin(y); mat(2,2)=cos(p)*cos(y);	mat(2,3)=-ox*(sin(r)*sin(y)+cos(r)*sin(p)*cos(y))+oy*(cos(r)*sin(y)-sin(p)*sin(r)*cos(y))-oz*cos(p)*cos(y);
        mat(3,0)=0;                                  mat(3,1)=0;								  mat(3,2)=0;				mat(3,3)=1;

        return mat;

    }

    Eigen::VectorXf compVecFromMat(Eigen::MatrixXf pose_mat)
    {
        float x=pose_mat(0,3);
        float y=pose_mat(1,3);
        float z=pose_mat(2,3);

        float nx=pose_mat(0,0);      float ny=pose_mat(1,0); float nz=pose_mat(2,0);
        float ox=pose_mat(0,1);      float oy=pose_mat(1,1); float oz=pose_mat(2,1);
        float ax=pose_mat(0,2);      float ay=pose_mat(1,2); float az=pose_mat(2,2);

        float roll=atan2(ny,nx);
        float pitch=atan2(-nz,nx*cos(roll)+ny*sin(roll));
        float yaw=atan2(ax*sin(roll)-ay*cos(roll), -ox*sin(roll)+oy*cos(roll));


        Eigen::VectorXf pose_vec;
        pose_vec.resize(6), pose_vec.setZero();

        pose_vec(0) = x;
        pose_vec(1) = y;
        pose_vec(2) = z;

        pose_vec(3) = roll;
        pose_vec(4) = pitch;
        pose_vec(5) = yaw;

        return pose_vec;

    }


};
