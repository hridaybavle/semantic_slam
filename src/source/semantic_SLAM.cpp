#include "semantic_SLAM.h"

semantic_SLAM::semantic_SLAM()
{
    init();
    std::cout << "semantic SLAM constructor " << std::endl;
}

semantic_SLAM::~semantic_SLAM()
{
    std::cout << "semantic SLAM destructor " << std::endl;
}

void semantic_SLAM::init()
{
    imu_data_available_, vo_data_available_ = false, aruco_data_available_ = false;
    prev_time_ = 0;
    VO_pose_.resize(6);
    first_aruco_pose_(0) = 2.19, first_aruco_pose_(1) = -0.017, first_aruco_pose_(2) = 0.61;

    filtered_pose_ = particle_filter_obj_.init(state_size_, num_particles_, first_aruco_pose_);

    final_pose_.resize(6), final_pose_.setZero();

//    filtered_pose_.resize(num_particles_);
//    for(int i= 0; i < num_particles_; ++i)
//    {
//        filtered_pose_[i].resize(state_size_);
//        filtered_pose_[i].setZero();
//    }

    return;

}

void semantic_SLAM::run()
{
    current_time_ = (double) ros::Time::now().sec + ((double) ros::Time::now().nsec / (double) 1E9);

    //std::cout << "current time  " << current_time_ << std::endl;
    float time_diff;

    time_diff = current_time_ - prev_time_;
    //std::cout << "time diff " << time_diff << std::endl;

    Eigen::VectorXf avg_pose;
    if(vo_data_available_)
    {
        Eigen::VectorXf VO_pose;
        VO_pose.resize(6), VO_pose.setZero();

        getVOPose(VO_pose);

        //std::cout << "VO_pose data " << VO_pose << std::endl
        filtered_pose_ = particle_filter_obj_.predictionVO(time_diff, VO_pose, filtered_pose_, final_pose_);

        if(aruco_data_available_)
        {
            final_pose_.setZero();
            std::vector<Eigen::Vector4f> aruco_pose;
            getArucoPose(aruco_pose);

            filtered_pose_ = this->particle_filter_obj_.arucoMapAndUpdate(aruco_pose, filtered_pose_, final_pose_, VO_pose_);
        }
    }


    publishParticlePoses();

//    cv::Mat particles_image(200, 200, CV_8UC3, cv::Scalar(255,255,255));
//    //particles_image = cv::Mat::zeros(480,640, CV_8UC3);
//    int circle_radius = 5;

//    std::vector<cv::Point> particle_point;
//    particle_point.clear();
//    for(int i =0; i < num_particles_; ++i)
//    {
//        particle_point.push_back(cv::Point(filtered_pose_[i](0)+100, filtered_pose_[i](1)+100));
//        cv::circle(particles_image, particle_point[i], circle_radius, cv::Scalar(255,0,0),-1);
//    }

//    cv::Point avg_pose_point = cv::Point(final_pose_(0)+100, final_pose_(1)+100);
//    cv::circle(particles_image, avg_pose_point , circle_radius, cv::Scalar(0,255,0),-1);

//    //std::cout << "VO pose " << this->VO_pose_ << std::endl;
//    //std::cout << "Pose " << final_pose_ << std::endl;

//    //cv::Point particle_point2 = cv::Point(5+240, 5+320);
//    //cv::circle(particles_image, particle_point2, circle_radius, cv::Scalar(255,0,0),-1);

//    //particles_image = cv::Mat::zeros((480, 640,3), CV_8UC3);
//    cv::imshow("particle distribution", particles_image);
//    cv::waitKey(1);

    prev_time_ = current_time_;
}

void semantic_SLAM::open(ros::NodeHandle n)
{
    //ros subsriber
    stereo_odometry_sub_   = n.subscribe("/stereo_odometer/pose", 1, &semantic_SLAM::stereoOdometryCallback, this);
    imu_sub_               = n.subscribe("/imu", 1, &semantic_SLAM::imuCallback, this);
    aruco_observation_sub_ = n.subscribe("/aruco_eye/aruco_observation",1, &semantic_SLAM::arucoObservationCallback, this);

    particle_poses_pub_ = n.advertise<geometry_msgs::PoseArray>("particle_poses",1);

}

void semantic_SLAM::stereoOdometryCallback(const geometry_msgs::PoseStamped &msg)
{
    Eigen::Vector4f camera_pose_local_mat, camera_pose_world_mat;
    Eigen::Vector4f camera_ang_local_mat, camera_ang_world_mat;

    /* Calculating Roll, Pitch, Yaw */
    tf::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
    tf::Matrix3x3 m(q);

    //convert quaternion to euler angels
    double yaw, pitch, roll;
    m.getEulerYPR(yaw, pitch, roll);

    camera_ang_local_mat.setOnes(), camera_ang_world_mat.setOnes();

    camera_ang_local_mat(0) = roll;
    camera_ang_local_mat(1) = pitch;
    camera_ang_local_mat(2) = yaw;

    Eigen::Matrix4f euler_transformation_mat;
    this->transformCameraToRobot(euler_transformation_mat);
    camera_ang_world_mat = euler_transformation_mat * camera_ang_local_mat;

    std::cout << "camera angles in world " << std::endl
              << "roll " << camera_ang_world_mat(0) << std::endl
              << "pitch " << camera_ang_world_mat(1) << std::endl
              << "yaw " << camera_ang_world_mat(2) << std::endl;


    Eigen::Matrix4f transformation_mat;
    camera_pose_local_mat.setOnes(), camera_pose_world_mat.setOnes();
    //assume roll, pitch and yaw zero for now//
    this->transformCameraToRobot(transformation_mat);

    camera_pose_local_mat(0) = msg.pose.position.x;
    camera_pose_local_mat(1) = msg.pose.position.y;
    camera_pose_local_mat(2) = msg.pose.position.z;

    camera_pose_world_mat = transformation_mat * camera_pose_local_mat;

    std::cout << "world pose mat " << std::endl
              << "x: " << camera_pose_world_mat(0) << std::endl
              << "y: " << camera_pose_world_mat(1) << std::endl
              << "z: " << camera_pose_world_mat(2) << std::endl;

    Eigen::VectorXf VO_pose;
    VO_pose.resize(6), VO_pose.setZero();

    VO_pose(0) = camera_pose_world_mat(0);
    VO_pose(1) = camera_pose_world_mat(1);
    VO_pose(2) = camera_pose_world_mat(2);
    VO_pose(3) = camera_ang_world_mat(0);
    VO_pose(4) = camera_ang_world_mat(1);
    VO_pose(5) = camera_ang_world_mat(2);

    this->setVOPose(VO_pose);


}

void semantic_SLAM::setVOPose(Eigen::VectorXf VO_pose)
{
    vo_pose_lock_.lock();          // locking the data when filling the received pose
    vo_data_available_ = true;
    this->VO_pose_ = VO_pose;
    vo_pose_lock_.unlock();        //unlocking the data
}

void semantic_SLAM::getVOPose(Eigen::VectorXf& VO_pose)
{
    vo_pose_lock_.lock();           // locking the data when giving it to the filter
    vo_data_available_ = false;
    VO_pose = this->VO_pose_;
    vo_pose_lock_.unlock();         // unlocking the data once the data is received
}

void semantic_SLAM::imuCallback(const sensor_msgs::Imu &msg)
{
    //for converting the IMU from NED to world frame (ENU)
    transformation_mat_acc_.setZero(), transformation_mat_ang_vel_.setZero();
    imu_local_acc_mat_.setOnes(), imu_world_acc_mat_.setOnes();
    imu_local_ang_vel_.setOnes(), imu_world_ang_vel_.setOnes();

    imu_local_acc_mat_(0) = msg.linear_acceleration.x;
    imu_local_acc_mat_(1) = msg.linear_acceleration.y;
    imu_local_acc_mat_(2) = msg.linear_acceleration.z;

    this->transformIMUtoWorld(imu_local_acc_mat_(0), imu_local_acc_mat_(1), imu_local_acc_mat_(2), transformation_mat_acc_);

    //converting the imu acclerations in world frame
    imu_world_acc_mat_ = transformation_mat_acc_ * imu_local_acc_mat_;

    //    std::cout << "Imu acc in world " << std::endl
    //              << "ax: " << imu_world_acc_mat_(0) << std::endl
    //              << "ay: " << imu_world_acc_mat_(1) << std::endl
    //              << "az: " << imu_world_acc_mat_(2) << std::endl;


    imu_local_ang_vel_(0) = msg.angular_velocity.x;
    imu_local_ang_vel_(1) = msg.angular_velocity.y;
    imu_local_ang_vel_(2) = msg.angular_velocity.z;

    this->transformIMUtoWorld(imu_local_ang_vel_(0), imu_local_ang_vel_(1), imu_local_ang_vel_(2), transformation_mat_ang_vel_);

    imu_world_ang_vel_ = transformation_mat_acc_ * imu_local_ang_vel_;


    //    std::cout << "Imu angular vel in world " << std::endl
    //              << "wx: " << imu_world_ang_vel_(0) << std::endl
    //              << "wy: " << imu_world_ang_vel_(1) << std::endl
    //              << "wz: " << imu_world_ang_vel_(2) << std::endl;

    imu_data_available_ = true;

}

void semantic_SLAM::arucoObservationCallback(const aruco_eye_msgs::MarkerList &msg)
{
    std::vector<Eigen::Vector4f> aruco_pose_cam, aruco_pose_robot;
    aruco_pose_cam.resize(msg.markers.size()), aruco_pose_robot.resize(msg.markers.size());
    Eigen::Matrix4f transformation_mat;

    this->transformCameraToRobot(transformation_mat);

    for (int i = 0; i < msg.markers.size(); ++i)
    {
        aruco_pose_cam[i].setOnes();
        aruco_pose_robot[i].setOnes();

        aruco_pose_cam[i](0) = msg.markers[i].pose.pose.position.x;
        aruco_pose_cam[i](1) = msg.markers[i].pose.pose.position.y;
        aruco_pose_cam[i](2) = msg.markers[i].pose.pose.position.z;

        aruco_pose_robot[i] = transformation_mat * aruco_pose_cam[i];

        //filling the last element with the aruco id
        aruco_pose_robot[i](3) = msg.markers[i].id;
        std::cout << "aruco marker pose in cam " << i << " " << "pose " << aruco_pose_cam[i] << std::endl;
        std::cout << "aruco marker pose in robot " << i << " " << "pose " << aruco_pose_robot[i] << std::endl;
    }

    this->setArucoPose(aruco_pose_robot);

}

void semantic_SLAM::setArucoPose(std::vector<Eigen::Vector4f> aruco_pose)
{
    aruco_pose_lock_.lock();
    aruco_data_available_ = true;
    this->aruco_pose_ = aruco_pose;
    aruco_pose_lock_.unlock();
}

void semantic_SLAM::getArucoPose(std::vector<Eigen::Vector4f> &aruco_pose)
{
    aruco_pose_lock_.lock();
    aruco_data_available_ = false;
    aruco_pose = this->aruco_pose_;
    aruco_pose_lock_.unlock();
}

void semantic_SLAM::transformCameraToRobot(Eigen::Matrix4f &transformation_mat)
{

    Eigen::Matrix4f rot_x_cam, rot_x_robot, rot_z_robot, translation_cam;
    rot_x_cam.setZero(4,4), rot_x_robot.setZero(4,4), rot_z_robot.setZero(4,4), translation_cam.setZero(4,4);

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
    //    T_robot_world(0,0) = cos(yaw)*cos(pitch);
    //    T_robot_world(0,1) = cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll);
    //    T_robot_world(0,2) = cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(pitch);

    //    T_robot_world(1,0) = sin(yaw)*cos(pitch);
    //    T_robot_world(1,1) = sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll);
    //    T_robot_world(1,2) = sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll);

    //    T_robot_world(2,0) = -sin(pitch);
    //    T_robot_world(2,1) = cos(pitch)*sin(roll);
    //    T_robot_world(2,2) = cos(pitch)*cos(roll);

    //fill the x, y and z variables over here if there is a fixed transform between the camera and the world frame
    //currently its they are all zero as the pose is obtained of the camera with respect to the world.
    //    T_robot_world(0,3) = prev_pose_x_;
    //    T_robot_world(1,3) = prev_pose_y_;
    //    T_robot_world(2,3) = prev_pose_z_;
    //    T_robot_world(3,3) = 1;

    transformation_mat = rot_z_robot * rot_x_robot ;


    //std::cout << "transformation matrix " << transformation_mat << std::endl;

}

void semantic_SLAM::transformIMUtoWorld(float ax, float ay, float az, Eigen::Matrix4f &transformation_mat)
{
    Eigen::Matrix4f rot_x_imu, T_robot_world;
    double roll, pitch, yaw;

    //for now considering roll, pitch and yaw zero
    roll = pitch = yaw =0;

    rot_x_imu.setZero(), T_robot_world.setZero();

    //rotation of -180 to convert to robot frame
    rot_x_imu(0,0) = 1;
    rot_x_imu(1,1) =  cos(-3.14);
    rot_x_imu(1,2) = -sin(-3.14);
    rot_x_imu(2,1) =  sin(-3.14);
    rot_x_imu(2,2) =  cos(-3.14);
    rot_x_imu(3,3) = 1;

    //rotation of the robot with respect to the world
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

    transformation_mat = T_robot_world * rot_x_imu;


}

void semantic_SLAM::publishParticlePoses()
{
    geometry_msgs::PoseArray particle_poses_vec;
    particle_poses_vec.header.stamp = ros::Time::now();
    particle_poses_vec.header.frame_id = "map";

    for(int i =0; i < num_particles_; ++i)
    {
       geometry_msgs::Pose pose;
       pose.position.x = filtered_pose_[i](0);
       pose.position.y = filtered_pose_[i](1);
       pose.position.z = filtered_pose_[i](2);

       particle_poses_vec.poses.push_back(pose);
    }

    particle_poses_pub_.publish(particle_poses_vec);
}
