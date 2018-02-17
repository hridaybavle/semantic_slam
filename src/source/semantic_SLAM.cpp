#include "semantic_SLAM.h"

semantic_slam_ros::semantic_slam_ros()
// : pclViewer (new pcl::visualization::PCLVisualizer ("Point cloud segmented"))
{
    std::cout << "semantic SLAM constructor " << std::endl;
}

semantic_slam_ros::~semantic_slam_ros()
{
    std::cout << "semantic SLAM destructor " << std::endl;
}

void semantic_slam_ros::init()
{
    imu_data_available_, vo_data_available_ = false, aruco_data_available_ = false, slamdunk_data_available_ = false, bebop_imu_data_available_ = false,
            object_detection_available_ = false, point_cloud_available_ = false;
    magnetic_field_available_ = false, bebop_imu_data_ready_ = false;
    prev_time_ = 0;
    yaw_first_ = 0;
    VO_pose_.resize(6);
    aruco_poses_.resize(1);

    //    //first aruco pose
    //    aruco_poses_[0](0) = 2.0, aruco_poses_[0](1) = 0.0, aruco_poses_[0](2) = 0.56;
    //    //second aruco pose
    //    aruco_poses_[1](0) = 3.51, aruco_poses_[1](1) = -1.55, aruco_poses_[1](2) = 1.38;

    //first aruco pose
    //    aruco_poses_[0](0) = 3.0, aruco_poses_[0](1) = 0.0, aruco_poses_[0](2) = 1.39;
    //    //second aruco pose
    //    aruco_poses_[1](0) = 3.0, aruco_poses_[1](1) = -1.49, aruco_poses_[1](2) = 1.39;


    //this is temporary will be filled from an xml file
    //object pose
    std::vector<particle_filter::object_info_struct_pf> mapped_object_vec;

    std::vector<Eigen::Vector3f> mapped_object_pose;
    mapped_object_pose.resize(2);
    //rosbag of lab
    //    mapped_object_pose[0](0) = 2.0;
    //    mapped_object_pose[0](1) = 0.0;
    //    mapped_object_pose[0](2) = 0.53;

    //rosbag of nave
    mapped_object_pose[0](0) = 3.0;
    mapped_object_pose[0](1) = 0.0;
    mapped_object_pose[0](2) = 0.44;

    //second chair pose
    //rosbag of nave
    mapped_object_pose[1](0) = 5.0;
    mapped_object_pose[1](1) = 1.0;
    mapped_object_pose[1](2) = 0.44;


    mapped_object_vec.resize(mapped_object_pose.size());
    for(int i = 0; i < mapped_object_vec.size(); ++i)
    {
        mapped_object_vec[i].type = "chair";
        mapped_object_vec[i].pose =  mapped_object_pose[i];
    }

    this->mapped_object_vec_ = mapped_object_vec;
    filtered_pose_ = particle_filter_obj_.init(state_size_, num_particles_, aruco_poses_, mapped_object_vec);
    final_pose_.resize(6), final_pose_.setZero();

    //    filtered_pose_.resize(num_particles_);
    //    for(int i= 0; i < num_particles_; ++i)
    //    {
    //        filtered_pose_[i].resize(state_size_);
    //        filtered_pose_[i].setZero();
    //    }

    //Display Window
    //    pclViewer->setBackgroundColor (0, 0, 0);
    //    pclViewer->initCameraParameters ();
    //    pclViewer->setCameraPosition(0,0,0,0,0,1,0,-1,0);
    //    vtkSmartPointer<vtkRenderWindow> renderWindowNormals = pclViewer->getRenderWindow();
    //    renderWindowNormals->SetSize(800,450);
    //    renderWindowNormals->Render();


    return;

}

void semantic_slam_ros::run()
{

    current_time_ = (double) ros::Time::now().sec + ((double) ros::Time::now().nsec / (double) 1E9);

    //std::cout << "current time  " << current_time_ << std::endl;
    float time_diff;

    time_diff = current_time_ - prev_time_;
    //std::cout << "time diff " << time_diff << std::endl;

    Eigen::VectorXf avg_pose;
    geometry_msgs::Point detected_object_avg_position;


    //--------------------------Prediction stage using the visual odometry pose--------------------------------------//
    if(vo_data_available_)
    {
        Eigen::VectorXf VO_pose;
        VO_pose.resize(6), VO_pose.setZero();

        getVOPose(VO_pose);

        //std::cout << "VO_pose data " << VO_pose << std::endl;
        filtered_pose_ = particle_filter_obj_.predictionVO(time_diff, VO_pose, filtered_pose_, final_pose_);


    }
    //---------------------------------------------------------------------------------------------------------------//

    //--------------------------Update stage for updating the roll, pitch and yaw from bebop imu--------------------//
    if(bebop_imu_data_available_)
    {
        float roll, pitch, yaw;
        getBebopIMUData(roll, pitch, yaw);

        filtered_pose_ = particle_filter_obj_.bebopIMUUpdate(roll, pitch, yaw, filtered_pose_, final_pose_);

    }
    //--------------------------------------------------------------------------------------------------------------//


    //-------------------------------------segmentation of the point_cloud------------------------------------------//
    sensor_msgs::PointCloud2 segmented_point_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_point_cloud_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
    cv::Mat detected_object_postion_mat, centroids, labels;
    geometry_msgs::Point final_detected_point;
    Eigen::Vector4f final_detected_point_cam_frame, final_detected_point_robot_frame;
    std::vector<plane_segmentation::segmented_objects> segmented_objects_from_point_cloud;

    if(object_detection_available_)
    {
        if(point_cloud_available_)
        {
            std::vector<semantic_SLAM::ObjectInfo> object_info;
            this->getDetectedObjectInfo(object_info);

            sensor_msgs::PointCloud2 point_cloud;
            this->getPointCloudData(point_cloud);

            //This segments the PC according to the received bounding box data in the 2D image
            for (int i = 0; i < object_info.size(); ++i)
            {
                if(object_info[i].type == "chair")
                    segmented_objects_from_point_cloud.push_back(plane_segmentation_obj_.segmentPointCloudData(object_info[i], point_cloud, segmented_point_cloud));
            }

            for(int i = 0; i < segmented_objects_from_point_cloud.size(); ++i)
            {
                if(!segmented_objects_from_point_cloud[i].segmented_point_cloud->empty())
                {
                    //                    std::cout << "segmented point cloud height " << segmented_objects_from_point_cloud[i].segmented_point_cloud->height << std::endl;

                    //                    float height, width;
                    //                    height = segmented_objects_from_point_cloud[i].segmented_point_cloud->height/2;
                    //                    width  = segmented_objects_from_point_cloud[i].segmented_point_cloud->width/2;
                    //                    std::cout << "center point of the point cloud " << segmented_objects_from_point_cloud[i].segmented_point_cloud->at(width, height) << std::endl;

                    //                    std::vector<geometry_msgs::Point> detected_object_position_vec;
                    //                    geometry_msgs::Point point;
                    //                    for(size_t k =0; k < segmented_objects_from_point_cloud[i].segmented_point_cloud->size(); ++k)
                    //                    {
                    //                        if(!(std::isnan(segmented_objects_from_point_cloud[i].segmented_point_cloud->points[k].x)) &&
                    //                                !(std::isnan(segmented_objects_from_point_cloud[i].segmented_point_cloud->points[k].y)) &&
                    //                                !(std::isnan(segmented_objects_from_point_cloud[i].segmented_point_cloud->points[k].y)) &&
                    //                                segmented_objects_from_point_cloud[i].segmented_point_cloud->points[k].x < 3 &&
                    //                                segmented_objects_from_point_cloud[i].segmented_point_cloud->points[k].y < 3 &&
                    //                                segmented_objects_from_point_cloud[i].segmented_point_cloud->points[k].z < 3 &&
                    //                                segmented_objects_from_point_cloud[i].segmented_point_cloud->points[k].z > 0)
                    //                        {
                    //                            //                            std::cout << "segmented points have no nans" << std::endl;
                    //                            //                            std::cout << "points are: " << std::endl
                    //                            //                                      << "X: " << segmented_point_cloud_pcl->points[k].x << std::endl
                    //                            //                                      << "Y: " << segmented_point_cloud_pcl->points[k].y << std::endl
                    //                            //                                      << "Z: " << segmented_point_cloud_pcl->points[k].z << std::endl;

                    //                            point.x = segmented_objects_from_point_cloud[i].segmented_point_cloud->points[k].x;
                    //                            point.y = segmented_objects_from_point_cloud[i].segmented_point_cloud->points[k].y;
                    //                            point.z = segmented_objects_from_point_cloud[i].segmented_point_cloud->points[k].z;
                    //                            detected_object_position_vec.push_back(point);
                    //                        }

                    //                    }

                    //                    detected_object_postion_mat = cv::Mat::zeros(detected_object_position_vec.size(), 3, CV_32F);

                    //                    for(int k=0; k < detected_object_position_vec.size(); ++k)
                    //                    {
                    //                        detected_object_postion_mat.at<float>(i,0) = static_cast<float>(detected_object_position_vec[k].x);
                    //                        detected_object_postion_mat.at<float>(i,1) = static_cast<float>(detected_object_position_vec[k].y);
                    //                        detected_object_postion_mat.at<float>(i,2) = static_cast<float>(detected_object_position_vec[k].z);
                    //                    }
                    //                    double compactness = 0.0;

                    //                    if(detected_object_postion_mat.rows < 5)
                    //                    {
                    //                        continue;
                    //                        std::cout << "not enough points for finding the mean" << std::endl;
                    //                    }

                    //                    cv::TermCriteria criteria_kmeans(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 50, 0.01);
                    //                    compactness = cv::kmeans(detected_object_postion_mat, num_centroids, labels, criteria_kmeans, 50, cv::KMEANS_RANDOM_CENTERS, centroids);

                    //                    std::cout << "kmeans centroid  " << centroids << std::endl;

                    //                    if(centroids.at<float>(0,2) < centroids.at<float>(1,2) && centroids.at<float>(0,2) > 0)
                    //                    {
                    //                        final_detected_point.x = centroids.at<float>(0,0);
                    //                        final_detected_point.y = centroids.at<float>(0,1);
                    //                        final_detected_point.z = centroids.at<float>(0,2);
                    //                    }
                    //                    else
                    //                    {
                    //                        final_detected_point.x = centroids.at<float>(1,0);
                    //                        final_detected_point.y = centroids.at<float>(1,1);
                    //                        final_detected_point.z = centroids.at<float>(1,2);
                    //                    }


                    //                    std::cout << "final object point " << std::endl
                    //                              << "X: " << final_detected_point.x << std::endl
                    //                              << "Y: " << final_detected_point.y << std::endl
                    //                              << "Z: " << final_detected_point.z << std::endl;

                    //This calculates the normals of the segmented pointcloud
                    pcl::PointCloud<pcl::Normal>::Ptr segmented_point_cloud_normal(new pcl::PointCloud<pcl::Normal>);
                    segmented_point_cloud_normal = plane_segmentation_obj_.computeNormalsFromPointCloud(segmented_objects_from_point_cloud[i].segmented_point_cloud);

                    //inputting the current roll, pitch and yaw from the pf updated from imu
                    Eigen::Matrix4f transformation_mat;
                    this->transformNormalsToWorld(final_pose_, transformation_mat);

                    float horizontal_point_size =0;
                    cv::Mat final_pose_from_horizontal_plane;
                    final_pose_from_horizontal_plane = plane_segmentation_obj_.computeHorizontalPlane(segmented_objects_from_point_cloud[i].segmented_point_cloud, segmented_point_cloud_normal,
                                                                                                      transformation_mat, final_pose_, horizontal_point_size);

                    //                    Eigen::Vector4f final_detected_point_in_robot_frame_from_mean, final_detected_point_in_cam_frame_from_mean;
                    //                    final_detected_point_in_robot_frame_from_mean.setZero(), final_detected_point_in_cam_frame_from_mean.setZero();

                    //                    final_detected_point_in_cam_frame_from_mean(0) = final_detected_point.x;
                    //                    final_detected_point_in_cam_frame_from_mean(1) = final_detected_point.y;
                    //                    final_detected_point_in_cam_frame_from_mean(2) = final_detected_point.z;
                    //                    final_detected_point_in_cam_frame_from_mean(3) = 1;
                    //final_detected_point_in_robot_frame_from_mean = transformation_mat * final_detected_point_in_cam_frame_from_mean;
                    //std::cout << "final detected point in robot frame obtained from mean " << final_detected_point_in_robot_frame_from_mean << std::endl;

                    final_detected_point_cam_frame.setZero(), final_detected_point_robot_frame.setZero();
                    if(!final_pose_from_horizontal_plane.empty())
                    {

                        final_detected_point_cam_frame(0) = final_pose_from_horizontal_plane.at<float>(0,0);
                        final_detected_point_cam_frame(1) = final_pose_from_horizontal_plane.at<float>(0,1);
                        final_detected_point_cam_frame(2) = final_pose_from_horizontal_plane.at<float>(0,2);

                        final_detected_point_robot_frame = transformation_mat * final_detected_point_cam_frame;
                        std::cout << "final pose from horizontal plane in robot frame " << final_detected_point_robot_frame << std::endl;
                        std::cout << "segmented pc points " << horizontal_point_size << std::endl;

                        final_detected_point.x = final_detected_point_robot_frame(0);
                        final_detected_point.y = final_detected_point_robot_frame(1);
                        final_detected_point.z = final_detected_point_robot_frame(2);
                        publishFinalDetectedObjectPoint(final_detected_point);


                        Eigen::Vector3f final_pose_of_object_in_robot;

                        final_pose_of_object_in_robot(0) = final_detected_point.x;
                        final_pose_of_object_in_robot(1) = final_detected_point.y;
                        final_pose_of_object_in_robot(2) = final_detected_point.z;


                        //this will be changed to add more objects, for now its a start
                        particle_filter::object_info_struct_pf complete_obj_info;

                        complete_obj_info.type       = segmented_objects_from_point_cloud[i].type;
                        complete_obj_info.prob       = segmented_objects_from_point_cloud[i].prob;
                        complete_obj_info.pose       = final_pose_of_object_in_robot;
                        complete_obj_info.num_points = horizontal_point_size;

                        std::cout << "detection prob " << complete_obj_info.prob << std::endl;
                        if(segmented_objects_from_point_cloud[i].prob > 0.5)
                            filtered_pose_ = particle_filter_obj_.ObjectMapAndUpdate(complete_obj_info, filtered_pose_, final_pose_, VO_pose_);
                        else
                            std::cout << "\033[1;31mbold returning as the probability of the object is less\033[0m" << std::endl;

                    }
                    //                pclViewer->removeAllPointClouds();
                    //                //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> rgb(kmeans_segmented_points);
                    //                pclViewer->addPointCloud<pcl::PointXYZRGB>(segmented_point_cloud_pcl,"segmented cloud");
                    //                pclViewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(segmented_point_cloud_pcl, segmented_point_cloud_normal, 100, 0.02f, "segmented cloud normals");
                    //                pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "segmented cloud");
                    publishSegmentedPointCloud(segmented_point_cloud);
                }
            }
        }
    }
    //----------------------------------------------------------------------------------------------------------------------------//

    //------------------------------Slamdunk update stage(just used for validating the particle filter)---------------------------//
    //        if(slamdunk_data_available_)
    //        {
    //            Eigen::VectorXf slamdunk_pose_simulated;

    //            final_pose_.setZero();
    //            Eigen::Vector3f slamdunk_pose;
    //            slamdunk_pose.setZero();
    //            getSlamdunkPose(slamdunk_pose);

    //            slamdunk_pose_simulated.setZero(), slamdunk_pose_simulated.resize(6);
    //            slamdunk_pose_simulated(0) = slamdunk_pose(0);
    //            slamdunk_pose_simulated(1) = slamdunk_pose(1);
    //            slamdunk_pose_simulated(2) = slamdunk_pose(2);

    //            //filtered_pose_ = this->particle_filter_obj_.predictionVO(time_diff, slamdunk_pose_simulated, filtered_pose_, final_pose_);

    //            filtered_pose_ = this->particle_filter_obj_.slamdunkPoseUpdate(slamdunk_pose, filtered_pose_, VO_pose_);
    //        }
    //-------------------------------------------------------------------------------------------------------------------------------//

    //------------------------------------Aruco update stage(also used for validating the particle filter)---------------------------//
    //    if(aruco_data_available_)
    //    {
    //        final_pose_.setZero();
    //        std::vector<Eigen::Vector4f> aruco_pose;
    //        getArucoPose(aruco_pose);

    //        filtered_pose_ = this->particle_filter_obj_.arucoMapAndUpdate(aruco_pose, filtered_pose_, final_pose_, VO_pose_);
    //    }
    //-------------------------------------------------------------------------------------------------------------------------------//

    publishFinalPose();
    publishCorresVOPose();
    publishParticlePoses();
    publishMappedObjects(mapped_object_vec_);

    prev_time_ = current_time_;

}

void semantic_slam_ros::open(ros::NodeHandle n)
{
    init();

    //ros subsriber
    stereo_odometry_sub_   = n.subscribe("/inertial_stereo/absolute_extrinsics", 1, &semantic_slam_ros::stereoOdometryCallback, this);
    imu_sub_               = n.subscribe("/imu", 1, &semantic_slam_ros::imuCallback, this);
    aruco_observation_sub_ = n.subscribe("/aruco_eye/aruco_observation",1, &semantic_slam_ros::arucoObservationCallback, this);
    slam_dunk_pose_sub_    = n.subscribe("/pose", 1, &semantic_slam_ros::slamdunkPoseCallback, this);
    magnetic_field_sub_    = n.subscribe("magnetometer", 1, &semantic_slam_ros::magneticFieldCallback, this);
    bebop_imu_sub_         = n.subscribe("/drone4/states/ardrone3/PilotingState/AttitudeChanged", 1, &semantic_slam_ros::bebopIMUCallback, this);
    detected_object_sub_   = n.subscribe("/darknet_ros/detected_objects",1, &semantic_slam_ros::detectedObjectCallback, this);
    point_cloud_sub_       = n.subscribe("/points", 1, &semantic_slam_ros::pointCloudCallback, this);

    //Publishers
    particle_poses_pub_             = n.advertise<geometry_msgs::PoseArray>("particle_poses",1);
    final_pose_pub_                 = n.advertise<geometry_msgs::PoseStamped>("final_pose",1);
    corres_vo_pose_pub_             = n.advertise<geometry_msgs::PoseStamped>("corres_vo_pose",1);
    segmented_point_cloud_pub_      = n.advertise<sensor_msgs::PointCloud2>("segmented_point_cloud",1);
    detected_object_point_pub_      = n.advertise<geometry_msgs::PointStamped>("detected_object_pose_cam", 1);
    mapped_objects_visualizer_pub_  = n.advertise<visualization_msgs::MarkerArray>("mapped_objects",1);


}

void semantic_slam_ros::stereoOdometryCallback(const geometry_msgs::PoseStamped &msg)
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

    //converting to the correct camera coordinate frame as in case of stefans its inverted
    camera_ang_local_mat(0) = -roll;
    camera_ang_local_mat(1) = -pitch;
    camera_ang_local_mat(2) = -yaw;

    //    std::cout << "camera angles in cam frame " << std::endl
    //              << "roll " << camera_ang_local_mat(0) << std::endl
    //              << "pitch " << camera_ang_local_mat(1) << std::endl
    //              << "yaw " << camera_ang_local_mat(2) << std::endl;

    Eigen::Matrix4f euler_transformation_mat;
    this->transformCameraToRobot(euler_transformation_mat);
    camera_ang_world_mat = euler_transformation_mat * camera_ang_local_mat;

    //    std::cout << "camera angles in world " << std::endl
    //              << "roll " << camera_ang_world_mat(0) << std::endl
    //              << "pitch " << camera_ang_world_mat(1) << std::endl
    //              << "yaw " << camera_ang_world_mat(2) << std::endl;


    Eigen::Matrix4f transformation_mat;
    camera_pose_local_mat.setOnes(), camera_pose_world_mat.setOnes();
    //assume roll, pitch and yaw zero for now//
    this->transformCameraToRobot(transformation_mat);

    //converting to the correct camera coordinate frame as in case of stefans its inverted
    camera_pose_local_mat(0) = -msg.pose.position.x;
    camera_pose_local_mat(1) = -msg.pose.position.y;
    camera_pose_local_mat(2) = -msg.pose.position.z;

    camera_pose_world_mat = transformation_mat * camera_pose_local_mat;

    //    std::cout << "world pose mat " << std::endl
    //              << "x: " << camera_pose_world_mat(0) << std::endl
    //              << "y: " << camera_pose_world_mat(1) << std::endl
    //              << "z: " << camera_pose_world_mat(2) << std::endl;

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

void semantic_slam_ros::setVOPose(Eigen::VectorXf VO_pose)
{
    vo_pose_lock_.lock();          // locking the data when filling the received pose
    vo_data_available_ = true;
    this->VO_pose_ = VO_pose;
    vo_pose_lock_.unlock();        //unlocking the data
}

void semantic_slam_ros::getVOPose(Eigen::VectorXf& VO_pose)
{
    vo_pose_lock_.lock();           // locking the data when giving it to the filter
    vo_data_available_ = false;
    VO_pose = this->VO_pose_;
    vo_pose_lock_.unlock();         // unlocking the data once the data is received
}

void semantic_slam_ros::imuCallback(const sensor_msgs::Imu &msg)
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

    this->setIMUdata(msg.linear_acceleration.x, msg.linear_acceleration.y,msg.linear_acceleration.z);
    //this->calculateRPYAngles();

}

void semantic_slam_ros::setIMUdata(float acc_x, float acc_y, float acc_z)
{
    imu_lock_.lock();
    this->acc_x_ = acc_x;
    this->acc_y_ = acc_y;
    this->acc_z_ = acc_z;
    imu_lock_.unlock();

}

void semantic_slam_ros::getIMUdata(float &acc_x, float &acc_y, float &acc_z)
{
    imu_lock_.lock();
    acc_x = this->acc_x_;
    acc_y = this->acc_y_;
    acc_z = this->acc_z_;
    imu_lock_.unlock();

}

void semantic_slam_ros::magneticFieldCallback(const sensor_msgs::MagneticField &msg)
{

    double mag_x, mag_y, mag_z;
    mag_x = msg.magnetic_field.x;
    mag_y = msg.magnetic_field.y;
    mag_z = msg.magnetic_field.z;

    this->setMagData(mag_x, mag_y, mag_z);

}

void semantic_slam_ros::setMagData(float mag_x, float mag_y, float mag_z)
{
    magnetometer_data_lock_.lock();
    magnetic_field_available_ = true;
    this->mag_x_ = mag_x;
    this->mag_y_ = mag_y;
    this->mag_z_ = mag_z;
    magnetometer_data_lock_.unlock();
}

void semantic_slam_ros::getMagData(float &mag_x, float &mag_y, float &mag_z)
{
    magnetometer_data_lock_.lock();
    magnetic_field_available_ = false;
    mag_x = this->mag_x_;
    mag_y = this->mag_y_;
    mag_z = this->mag_z_;
    magnetometer_data_lock_.unlock();

}

void semantic_slam_ros::calculateRPYAngles()
{

    float pitch, roll, yaw;
    float ax, ay, az;
    float mx, my, mz;
    float xh, yh;

    if(magnetic_field_available_ = true)
    {
        this->getIMUdata(ax, ay, az);
        pitch =  atan2(ax, sqrt(pow(ay,2) + pow(az,2)));
        roll  = -atan2(ay, sqrt(pow(ax,2) + pow(az,2)));

        std::cout << "roll obtained from acc: " << roll*(180/M_PI) << std::endl;
        std::cout << "pitch obtained from acc: " << pitch*(180/M_PI) << std::endl;

        this->getMagData(mx, my, mz);

        float mag_norm=sqrt((mx*mx)+(my*my)+(mz*mz));
        mx = mx / mag_norm;
        my = my / mag_norm;
        mz = mz / mag_norm;

        xh  = mx * cos(pitch)+ my * sin(pitch)* sin(roll)+ mz*sin(pitch)*cos(roll);
        yh  =-my * cos(roll) + mz * sin(roll);
        yaw = atan2(yh,xh);

        //keeping the yaw between zero to 2*PI
        if (yaw < 0)
            yaw = yaw + 2*M_PI;

        std::cout << "yaw obtained from imu " << yaw*(180/M_PI) << std::endl;
    }

}

void semantic_slam_ros::bebopIMUCallback(const semantic_SLAM::Ardrone3PilotingStateAttitudeChanged &msg)
{
    float roll, pitch, yaw;

    //this makes sure that the yaw data is taken again after takeoff, as the yaw of the bebop changes during takeoff//
    if(final_pose_(2) > 0.8)
        bebop_imu_data_ready_ = false;


    if(!bebop_imu_data_ready_)
    {
        yaw_first_   = msg.yaw;
        bebop_imu_data_ready_ = true;
    }

    //bebop provides the angles in NED, hence converting them to ENU
    roll  =  msg.roll;
    pitch = -msg.pitch;
    yaw   = -(msg.yaw - yaw_first_);

    //    if (yaw < 0)
    //        yaw = yaw + 2*M_PI;
    //std::cout << "yaw from bebop " << yaw << std::endl;

    this->setBebopIMUData(roll, pitch, yaw);

}

void semantic_slam_ros::setBebopIMUData(float roll, float pitch, float yaw)
{
    bebop_imu_lock_.lock();
    bebop_imu_data_available_ = true;
    bebop_imu_roll_  = roll;
    bebop_imu_pitch_ = pitch;
    bebop_imu_yaw_   = yaw;
    bebop_imu_lock_.unlock();

}

void semantic_slam_ros::getBebopIMUData(float &roll, float &pitch, float &yaw)
{

    bebop_imu_lock_.lock();
    bebop_imu_data_available_ = false;
    roll  = bebop_imu_roll_;
    pitch = bebop_imu_pitch_;
    yaw   = bebop_imu_yaw_;
    bebop_imu_lock_.unlock();
}

void semantic_slam_ros::arucoObservationCallback(const aruco_eye_msgs::MarkerList &msg)
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
        //std::cout << "aruco marker pose in cam " << i << " " << "pose " << aruco_pose_cam[i] << std::endl;
        //std::cout << "aruco marker pose in robot " << i << " " << "pose " << aruco_pose_robot[i] << std::endl;
    }

    this->setArucoPose(aruco_pose_robot);

}

void semantic_slam_ros::setArucoPose(std::vector<Eigen::Vector4f> aruco_pose)
{
    aruco_pose_lock_.lock();
    aruco_data_available_ = true;
    this->aruco_pose_ = aruco_pose;
    aruco_pose_lock_.unlock();
}

void semantic_slam_ros::getArucoPose(std::vector<Eigen::Vector4f> &aruco_pose)
{
    aruco_pose_lock_.lock();
    aruco_data_available_ = false;
    aruco_pose = this->aruco_pose_;
    aruco_pose_lock_.unlock();
}


void semantic_slam_ros::slamdunkPoseCallback(const geometry_msgs::PoseStamped &msg)
{
    Eigen::Vector3f slamdunk_pose;
    slamdunk_pose.setZero();

    slamdunk_pose(0) = msg.pose.position.x;
    slamdunk_pose(1) = msg.pose.position.y;
    slamdunk_pose(2) = msg.pose.position.z;

    setSlamdunkPose(slamdunk_pose);
}

void semantic_slam_ros::setSlamdunkPose(Eigen::Vector3f pose)
{
    slamdunk_pose_lock_.lock();
    slamdunk_data_available_ = true;
    this->slamdunk_pose_ = pose;
    slamdunk_pose_lock_.unlock();
}

void semantic_slam_ros::getSlamdunkPose(Eigen::Vector3f &pose)
{
    slamdunk_pose_lock_.lock();
    slamdunk_data_available_ = false;
    pose = this->slamdunk_pose_;
    slamdunk_pose_lock_.unlock();
}

void semantic_slam_ros::detectedObjectCallback(const semantic_SLAM::DetectedObjects &msg)
{
    //std::cout << "objects size " << msg.objects.size() << std::endl;
    std::vector<semantic_SLAM::ObjectInfo>  object_info;
    object_info.resize(msg.objects.size());

    for(int i =0; i < msg.objects.size(); ++i)
    {
        object_info[i].type   = msg.objects[i].type;
        object_info[i].tl_x   = msg.objects[i].tl_x;
        object_info[i].tl_y   = msg.objects[i].tl_y;
        object_info[i].height = msg.objects[i].height;
        object_info[i].width  = msg.objects[i].width;
        object_info[i].prob   = msg.objects[i].prob;
    }

    this->setDetectedObjectInfo(object_info);

}

void semantic_slam_ros::setDetectedObjectInfo(std::vector<semantic_SLAM::ObjectInfo> object_info)
{
    detected_object_lock_.lock();
    object_detection_available_ = true;
    this->object_info_ = object_info;
    detected_object_lock_.unlock();
}

void semantic_slam_ros::getDetectedObjectInfo(std::vector<semantic_SLAM::ObjectInfo> &object_info)
{
    detected_object_lock_.lock();
    object_detection_available_ = false;
    object_info = this->object_info_;
    detected_object_lock_.unlock();
}

void semantic_slam_ros::pointCloudCallback(const sensor_msgs::PointCloud2 &msg)
{
    //    pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
    //    pcl::fromROSMsg(msg, point_cloud);

    this->setPointCloudData(msg);
}

void semantic_slam_ros::setPointCloudData(sensor_msgs::PointCloud2 point_cloud)
{
    point_cloud_lock_.lock();
    point_cloud_available_ = true;
    this->point_cloud_msg_ = point_cloud;
    point_cloud_lock_.unlock();

}

void semantic_slam_ros::getPointCloudData(sensor_msgs::PointCloud2 &point_cloud)
{
    point_cloud_lock_.lock();
    point_cloud_available_ = false;
    point_cloud = this->point_cloud_msg_;
    point_cloud_lock_.unlock();

}


void semantic_slam_ros::transformCameraToRobot(Eigen::Matrix4f &transformation_mat)
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

    //tranlation of 5cm in x, 10cm in y and 1.5cm in z
    translation_cam(0,0) = 1;
    translation_cam(0,3) = -0.05;
    translation_cam(1,1) = 1;
    translation_cam(1,3) = -0.10;
    translation_cam(2,2) = 1;
    translation_cam(2,2) = 0.015;
    translation_cam(3,3) = 1;


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

    transformation_mat = translation_cam * rot_z_robot * rot_x_robot ;


    //std::cout << "transformation matrix " << transformation_mat << std::endl;

}

void semantic_slam_ros::transformIMUtoWorld(float ax, float ay, float az, Eigen::Matrix4f &transformation_mat)
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

void semantic_slam_ros::transformNormalsToWorld(Eigen::VectorXf final_pose, Eigen::Matrix4f &transformation_mat)
{

    Eigen::Matrix4f rot_x_cam, rot_x_robot, rot_z_robot, translation_cam, T_robot_world;
    rot_x_cam.setZero(4,4), rot_x_robot.setZero(4,4), rot_z_robot.setZero(4,4), translation_cam.setZero(4,4), T_robot_world.setZero(4,4);

    float x, y, z, roll, pitch, yaw;

    x = final_pose(0);
    y = final_pose(1);
    z = final_pose(2);
    roll = final_pose(3);
    pitch = final_pose(4);
    yaw = final_pose(5);

    //    std::cout << "X: " << x << std::endl
    //              << "Y: " << y << std::endl
    //              << "Z: " << z << std::endl
    //              << "roll: " << roll << std::endl
    //              << "pitch: " << pitch << std::endl
    //              << "yaw: "   << yaw << std::endl;

    rot_x_cam(0,0) = 1;
    rot_x_cam(1,1) =  cos(-real_sense_pitch_angle);
    rot_x_cam(1,2) = -sin(-real_sense_pitch_angle);
    rot_x_cam(2,1) =  sin(-real_sense_pitch_angle);
    rot_x_cam(2,2) =  cos(-real_sense_pitch_angle);
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

    //translation of -10cm in x, -0.5cm in y and -5cm in z
    translation_cam(0,0) = 1;
    translation_cam(0,3) = -0.10;
    translation_cam(1,1) = 1;
    translation_cam(1,3) = -0.005;
    translation_cam(2,2) = 1;
    translation_cam(2,3) = -0.5;
    translation_cam(3,3) = 1;


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
    T_robot_world(3,3) = 1;

    //fill the x, y and z variables over here if there is a fixed transform between the camera and the world frame
    //    T_robot_world(0,3) = x;
    //    T_robot_world(1,3) = y;
    //    T_robot_world(2,3) = z;
    //    T_robot_world(3,3) = 1;

    transformation_mat = T_robot_world * translation_cam * rot_z_robot * rot_x_robot * rot_x_cam;
}

void semantic_slam_ros::publishParticlePoses()
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

        //converting euler to quaternion
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(filtered_pose_[i](3),filtered_pose_[i](4),filtered_pose_[i](5));
                pose.orientation.x = quaternion.getX();
        pose.orientation.y = quaternion.getY();
        pose.orientation.z = quaternion.getZ();
        pose.orientation.w = quaternion.getW();

        particle_poses_vec.poses.push_back(pose);
    }

    particle_poses_pub_.publish(particle_poses_vec);
}

void semantic_slam_ros::publishFinalPose()
{
    geometry_msgs::PoseStamped final_particle_pose;
    final_particle_pose.header.stamp = ros::Time::now();
    final_particle_pose.header.frame_id = "map";

    final_particle_pose.pose.position.x = final_pose_(0);
    final_particle_pose.pose.position.y = final_pose_(1);
    final_particle_pose.pose.position.z = final_pose_(2);

    tf::Quaternion quaternion = tf::createQuaternionFromRPY(final_pose_(3),final_pose_(4),final_pose_(5));
    final_particle_pose.pose.orientation.x = quaternion.getX();
    final_particle_pose.pose.orientation.y = quaternion.getY();
    final_particle_pose.pose.orientation.z = quaternion.getZ();
    final_particle_pose.pose.orientation.w = quaternion.getW();

    final_pose_pub_.publish(final_particle_pose);
}

void semantic_slam_ros::publishCorresVOPose()
{
    geometry_msgs::PoseStamped corres_vo_pose;
    corres_vo_pose.header.stamp = ros::Time::now();
    corres_vo_pose.header.frame_id = "map";

    corres_vo_pose.pose.position.x = VO_pose_(0);
    corres_vo_pose.pose.position.y = VO_pose_(1);
    corres_vo_pose.pose.position.z = VO_pose_(2);

    tf::Quaternion quaternion = tf::createQuaternionFromRPY(VO_pose_(3),VO_pose_(4),VO_pose_(5));
    corres_vo_pose.pose.orientation.x = quaternion.getX();
    corres_vo_pose.pose.orientation.y = quaternion.getY();
    corres_vo_pose.pose.orientation.z = quaternion.getZ();
    corres_vo_pose.pose.orientation.w = quaternion.getW();

    corres_vo_pose_pub_.publish(corres_vo_pose);

}

void semantic_slam_ros::publishSegmentedPointCloud(sensor_msgs::PointCloud2 point_cloud_seg)
{
    point_cloud_seg.header.stamp = ros::Time::now();
    point_cloud_seg.header.frame_id = "cam0_optical";

    segmented_point_cloud_pub_.publish(point_cloud_seg);

}

void semantic_slam_ros::publishFinalDetectedObjectPoint(geometry_msgs::Point final_point)
{
    geometry_msgs::PointStamped final_point_to_pub;
    final_point_to_pub.header.stamp = ros::Time::now();
    final_point_to_pub.header.frame_id = "cam0_optical";

    final_point_to_pub.point.x = final_point.x;
    final_point_to_pub.point.y = final_point.y;
    final_point_to_pub.point.z = final_point.z;

    detected_object_point_pub_.publish(final_point_to_pub);

}

void semantic_slam_ros::publishMappedObjects(std::vector<particle_filter::object_info_struct_pf> mapped_object_vec)
{
    visualization_msgs::MarkerArray marker_arrays;

    for(int i =0; i < mapped_object_vec.size(); ++i)
    {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time();
        marker.header.frame_id = "map";
        marker.ns = "my_namespace";
        marker.pose.position.x = mapped_object_vec[i].pose(0);
        marker.pose.position.y = mapped_object_vec[i].pose(1);
        marker.pose.position.z = mapped_object_vec[i].pose(2);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.id = i;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.5;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker_arrays.markers.push_back(marker);
    }

    mapped_objects_visualizer_pub_.publish(marker_arrays);
}
