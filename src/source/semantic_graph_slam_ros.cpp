#include "semantic_graph_slam_ros.h"

semantic_graph_slam_ros::semantic_graph_slam_ros()
//    : sync(SyncPolicy(10))
{
    std::cout << "semantic graph slam ros constructor " << std::endl;

}

semantic_graph_slam_ros::~semantic_graph_slam_ros()
{
    std::cout << "semantic graph slam ros destructor " << std::endl;

}

void semantic_graph_slam_ros::init(ros::NodeHandle n)
{
    semantic_gslam_obj_.reset(new semantic_graph_slam());
    semantic_gslam_obj_->init(n);

    counter_        = false;
    first_gt_pose_  = false;

    //this is test run
    //    if(!counter_)
    //    {
    //        //this->add_odom_increments();
    //        this->add_odom_pose_increments();
    //        counter_ = true;
    //    }

}

void semantic_graph_slam_ros::run()
{

    if(semantic_gslam_obj_->run())
    {
        this->publishLandmarks();
        this->publishKeyframePoses();
    }

    this->publishCorresVIOPose();
    this->publishRobotPose();
    return;

}


void semantic_graph_slam_ros::open(ros::NodeHandle n)
{

    init(n);

    //time synchronization of messages
    //    odom_msg_sub_.subscribe(n,"/rovio/odometry",10);
    //    point_cloud_msg_sub_.subscribe(n,"/depth_registered/points",10);
    //    bb_sub_.subscribe(n,"/darknet_ros/bounding_boxes",10);
    //    sync.connectInput(odom_msg_sub_,point_cloud_msg_sub_, bb_sub_);
    //    sync.registerCallback(boost::bind(&semantic_graph_slam::synMsgsCallback, this, _1, _2, _3));

    //subscribers
    rvio_odom_pose_sub_         = n.subscribe("/rovio/odometry", 1, &semantic_graph_slam_ros::rovioVIOCallback, this);
    snap_odom_pose_sub_         = n.subscribe("/SQ04/snap_vislam/vislam/pose", 1, &semantic_graph_slam_ros::snapVIOCallback, this);
    cloud_sub_                  = n.subscribe("/depth_registered/points",1,&semantic_graph_slam_ros::PointCloudCallback, this);
    detected_object_sub_        = n.subscribe("/darknet_ros/bounding_boxes",1, &semantic_graph_slam_ros::detectedObjectDarknetCallback, this);
    simple_detected_object_sub_ = n.subscribe("/image_processed/bounding_boxes",1, &semantic_graph_slam_ros::detectedObjectSimpleCallback, this);
    optitrack_pose_sub_         = n.subscribe("/vrpn_client_node/realsense/pose",1,&semantic_graph_slam_ros::optitrackPoseCallback, this);
    vicon_pose_sub_             = n.subscribe("/SQ04/vicon",1, &semantic_graph_slam_ros::viconPoseSubCallback, this);

    //publishers
    keyframe_pose_pub_      = n.advertise<geometry_msgs::PoseArray>("keyframe_poses",1);
    landmarks_pub_          = n.advertise<visualization_msgs::MarkerArray>("mapped_landmarks", 1);
    detected_lans_pub_      = n.advertise<visualization_msgs::MarkerArray>("detected_landmars",1);
    robot_pose_pub_         = n.advertise<geometry_msgs::PoseStamped>("robot_pose",1);
    robot_transform_pub_    = n.advertise<geometry_msgs::TransformStamped>("robot_transform",1);
    keyframe_path_pub_      = n.advertise<nav_msgs::Path>("robot_path",1);
    optitrack_pose_pub_     = n.advertise<geometry_msgs::PoseStamped>("optitrack_pose",1);
    optitrack_path_pub_     = n.advertise<nav_msgs::Path>("optitrack_path",1);
    corres_vio_pose_pub_    = n.advertise<geometry_msgs::PoseStamped>("corres_vo_pose",1);
    corres_vio_path_        = n.advertise<nav_msgs::Path>("corres_vo_path",1);

}

//void semantic_graph_slam::synMsgsCallback(const nav_msgs::OdometryConstPtr &odom_msg,
//                                          const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
//                                          const darknet_ros_msgs::BoundingBoxesConstPtr &bbs_msg)
//{
//    std::cout << "odom msg " << odom_msg << std::endl;
//}

void semantic_graph_slam_ros::rovioVIOCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{


    const ros::Time& stamp      = odom_msg->header.stamp;
    Eigen::Isometry3d odom      = ps_graph_slam::odom2isometry(odom_msg);
    Eigen::MatrixXf odom_cov    = ps_graph_slam::arrayToMatrix(odom_msg);

    //this->VIOCallback(stamp, odom, odom_cov);
    semantic_gslam_obj_->VIOCallback(stamp, odom, odom_cov);

    return;
}

void semantic_graph_slam_ros::snapVIOCallback(const geometry_msgs::PoseStamped &pose_msg)
{

    const ros::Time& stamp                  = pose_msg.header.stamp;
    geometry_msgs::PoseStamped pose_enu     = ps_graph_slam::poseNED2ENU(pose_msg);
    Eigen::Isometry3d odom                  = ps_graph_slam::pose2isometry(pose_enu);
    Eigen::MatrixXf odom_cov; odom_cov.setIdentity(6,6); //snap dragon pose doesnt have covariance

    semantic_gslam_obj_->VIOCallback(stamp, odom, odom_cov);

    return;
}

void semantic_graph_slam_ros::PointCloudCallback(const sensor_msgs::PointCloud2 &msg)
{
    semantic_gslam_obj_->setPointCloudData(msg);
    pc_stamp_ = msg.header.stamp;
}


void semantic_graph_slam_ros::detectedObjectDarknetCallback(const darknet_ros_msgs::BoundingBoxes &msg)
{
    std::vector<semantic_SLAM::ObjectInfo>  object_info;
    object_info.resize(msg.bounding_boxes.size());

    for(int i =0; i < msg.bounding_boxes.size(); ++i)
    {
        object_info[i].type   = msg.bounding_boxes[i].Class;
        object_info[i].tl_x   = msg.bounding_boxes[i].xmin;
        object_info[i].tl_y   = msg.bounding_boxes[i].ymin;
        object_info[i].height = abs(msg.bounding_boxes[i].ymax - msg.bounding_boxes[i].ymin);
        object_info[i].width  = abs(msg.bounding_boxes[i].xmax - msg.bounding_boxes[i].xmin);
        object_info[i].prob   = msg.bounding_boxes[i].probability;
    }

    semantic_gslam_obj_->setDetectedObjectInfo(object_info);
}

void semantic_graph_slam_ros::detectedObjectSimpleCallback(const semantic_SLAM::DetectedObjects &msg)
{
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

    semantic_gslam_obj_->setDetectedObjectInfo(object_info);
}

void semantic_graph_slam_ros::publishLandmarks()
{

    visualization_msgs::MarkerArray marker_arrays;
    int marker_id = 0;
    std::vector<landmark> l_vec;
    semantic_gslam_obj_->getMappedLandmarks(l_vec);

    for(int i=0; i < l_vec.size(); ++i)
    {

        Eigen::Vector3d lan_node_pose = l_vec[i].node->estimate();

        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "map";
        marker.ns = "my_namespace";
        marker.id = marker_id;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.pose.position.x = lan_node_pose(0);
        marker.pose.position.y = lan_node_pose(1);
        marker.pose.position.z = lan_node_pose(2);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        if(l_vec[i].plane_type == "horizontal")
        {
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.01;

        }
        else if(l_vec[i].plane_type == "vertical")
        {
            marker.scale.x = 0.01;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
        }


        if(l_vec[i].type == "chair")
        {
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        else if(l_vec[i].type == "tvmonitor")
        {

            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        else if (l_vec[i].type == "laptop")
        {

            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        else if(l_vec[i].type == "keyboard")
        {

            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }
        else if(l_vec[i].type == "book")
        {


            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }
        else if(l_vec[i].type == "Bucket")
        {

            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }

        marker_arrays.markers.push_back(marker);
        marker_id++;
    }

    landmarks_pub_.publish(marker_arrays);
}

void semantic_graph_slam_ros::publishDetectedLandmarks(Eigen::VectorXf robot_pose, std::vector<detected_object> det_obj_info)
{

    visualization_msgs::MarkerArray marker_arrays;
    int marker_id = 0;

    for(int i=0; i < det_obj_info.size(); ++i)
    {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "map";
        marker.ns = "my_namespace";
        marker.id = marker_id;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.pose.position.x = det_obj_info[i].world_pose(0) + robot_pose(0);
        marker.pose.position.y = det_obj_info[i].world_pose(1) + robot_pose(1);
        marker.pose.position.z = det_obj_info[i].world_pose(2) + robot_pose(2);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;

        if(det_obj_info[i].plane_type == "horizontal")
        {
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.01;

        }
        else if(det_obj_info[i].plane_type == "vertical")
        {
            marker.scale.x = 0.01;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
        }

        marker_arrays.markers.push_back(marker);
        marker_id++;
    }

    detected_lans_pub_.publish(marker_arrays);


}


void semantic_graph_slam_ros::publishKeyframePoses()
{
    ros::Time current_time = ros::Time::now();

    geometry_msgs::PoseArray pose_array;
    pose_array.header.stamp = current_time;
    pose_array.header.frame_id = "map";

    nav_msgs::Path final_path;
    final_path.header.stamp = current_time;
    final_path.header.frame_id = "map";

    //copying the new keyframes for publishing
    std::vector<ps_graph_slam::KeyFrame::Ptr> keyframes;
    semantic_gslam_obj_->getKeyframes(keyframes);
    geometry_msgs::Pose key_pose;
    geometry_msgs::PoseStamped key_pose_stamped;
    for(int i=0; i < keyframes.size(); ++i)
    {
        key_pose = ps_graph_slam::matrix2pose(ros::Time::now(),
                                              keyframes[i]->node->estimate().matrix().cast<float>(),
                                              "map");

        key_pose_stamped.header.stamp = current_time;
        key_pose_stamped.pose = key_pose;

        pose_array.poses.push_back(key_pose);
        final_path.poses.push_back(key_pose_stamped);
    }

    keyframe_pose_pub_.publish(pose_array);
    keyframe_path_pub_.publish(final_path);
}

void semantic_graph_slam_ros::publishRobotPose()
{
    Eigen::Isometry3d robot_pose;
    semantic_gslam_obj_->getRobotPose(robot_pose);

    geometry_msgs::PoseStamped robot_pose_msg = ps_graph_slam::matrix2posestamped(ros::Time::now(),
                                                                                  robot_pose.matrix().cast<float>(),
                                                                                  "map");
    geometry_msgs::TransformStamped robot_transform;
    robot_transform.header.stamp = pc_stamp_;
    robot_transform.header.frame_id = "map";
    robot_transform.child_frame_id  = "cam";
    robot_transform.transform.translation.x = robot_pose_msg.pose.position.x;
    robot_transform.transform.translation.y = robot_pose_msg.pose.position.y;
    robot_transform.transform.translation.z = robot_pose_msg.pose.position.z;
    robot_transform.transform.rotation      = robot_pose_msg.pose.orientation;

    robot_transform_pub_.publish(robot_transform);
    robot_pose_pub_.publish(robot_pose_msg);

}

void semantic_graph_slam_ros::optitrackPoseCallback(const nav_msgs::Odometry &msg)
{
    geometry_msgs::PoseStamped optitrack_pose;
    optitrack_pose.header.stamp = ros::Time::now();
    optitrack_pose.header.frame_id = "map";


    optitrack_pose.pose = msg.pose.pose;
    optitrack_pose_pub_.publish(optitrack_pose);

    optitrack_pose_vec_.push_back(optitrack_pose);
    nav_msgs::Path optitrack_path;
    optitrack_path.header.stamp = msg.header.stamp;
    optitrack_path.header.frame_id = "map";
    optitrack_path.poses = optitrack_pose_vec_;
    optitrack_path_pub_.publish(optitrack_path);

}

void semantic_graph_slam_ros::viconPoseSubCallback(const acl_msgs::ViconState &msg)
{
    if(!first_gt_pose_)
    {
        gt_x_transform_ = -msg.pose.position.x;
        gt_y_transform_ = -msg.pose.position.y;
        gt_z_transform_ = -msg.pose.position.z;
        first_gt_pose_ = true;
    }

    geometry_msgs::PoseStamped vicon_pose;
    vicon_pose.header.stamp = msg.header.stamp;
    vicon_pose.header.frame_id = "map";


    vicon_pose.pose.position.x = msg.pose.position.x + gt_x_transform_;
    vicon_pose.pose.position.y = msg.pose.position.y + gt_y_transform_;
    vicon_pose.pose.position.z = msg.pose.position.z + gt_z_transform_;
    vicon_pose.pose.orientation = msg.pose.orientation;

    //vicon_pose.pose.position.x = cos(-0.074) * vicon_pose.pose.position.x   - sin(-0.074) * vicon_pose.pose.position.y;
    //vicon_pose.pose.position.y = sin(-0.074) * vicon_pose.pose.position.x   + cos(-0.074) * vicon_pose.pose.position.y;

    optitrack_pose_pub_.publish(vicon_pose);
    optitrack_pose_vec_.push_back(vicon_pose);

    nav_msgs::Path vicon_path;
    vicon_path.header.stamp = msg.header.stamp;
    vicon_path.header.frame_id = "map";
    vicon_path.poses = optitrack_pose_vec_;
    optitrack_path_pub_.publish(vicon_path);

}

void semantic_graph_slam_ros::publishCorresVIOPose()
{
    Eigen::Isometry3d vio_pose;
    semantic_gslam_obj_->getVIOPose(vio_pose);

    ros::Time current_time = ros::Time::now();
    geometry_msgs::PoseStamped corres_vio_pose_msg = ps_graph_slam::matrix2posestamped(current_time,
                                                                                       vio_pose.matrix().cast<float>(),
                                                                                       "map");

    corres_vio_pose_pub_.publish(corres_vio_pose_msg);

    nav_msgs::Path vio_path;
    vio_path.header.stamp = current_time;
    vio_path.header.frame_id = "map";

    vio_pose_vec_.push_back(corres_vio_pose_msg);
    vio_path.poses = vio_pose_vec_;
    corres_vio_path_.publish(vio_path);
}

void semantic_graph_slam_ros::saveGraph()
{
    semantic_gslam_obj_->saveGraph();
}


//void semantic_graph_slam_ros::add_odom_pose_increments()
//{

//    Eigen::Isometry3d odom_iso;
//    Eigen::Quaterniond quat; quat.setIdentity();

//    odom_iso.linear() = quat.toRotationMatrix();
//    double x,y,z;
//    x=y=z=0;

//    g2o::VertexSE3* prev_node;
//    for(int i= 0; i < odom_increments_; ++i)
//    {

//        g2o::VertexSE3* node;
//        //adding odom measurements to the graph
//        odom_iso.translation() = Eigen::Vector3d(2*i, y, z);
//        node = graph_slam_->add_se3_node(odom_iso);


//        if(i >0)
//        {
//            Eigen::Matrix3d information; information.setIdentity(6,6);
//            Eigen::Isometry3d rel_pose;
//            rel_pose.linear() = quat.toRotationMatrix();
//            rel_pose.translation() = Eigen::Vector3d(2, 0, 0);
//            g2o::EdgeSE3* edge = graph_slam_->add_se3_edge(prev_node, node, rel_pose, information);
//        }

//        prev_node = node;

//    }

//    graph_slam_->optimize();
//    std::cout << "optimized graph " << std::endl;
//}

//void semantic_graph_slam::add_odom_position_increments()
//{

//    g2o::SparseOptimizer graph;

//    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 3> >  SlamBlockSolver;
//    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

//    auto linearSolver = g2o::make_unique<SlamLinearSolver>();
//    linearSolver->setBlockOrdering(false);
//    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(
//                g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));
//    graph.setAlgorithm(solver);

//    double x,y,z;
//    x=y=z=0;

//    g2o::VertexPointXYZ*  prev_node;
//    for(int i= 0; i < odom_increments_; ++i)
//    {

//        g2o::VertexPointXYZ* node (new g2o::VertexPointXYZ);
//        //adding odom measurements to the graph
//        Eigen::Vector3d position;
//        position = Eigen::Vector3d(x+2*i, y, z);
//        node->setId(graph.vertices().size());
//        node->setEstimate(position);
//        graph.addVertex(node);

//        if(i >0)
//        {
//            Eigen::Matrix3d information; information.setIdentity();
//            Eigen::Vector3d rel_pose;
//            rel_pose = Eigen::Vector3d(2, 0, 0);

//            g2o::EdgePointXYZ* edge(new g2o::EdgePointXYZ());
//            edge->setMeasurement(rel_pose);
//            edge->setInformation(information);
//            edge->vertices()[0] = prev_node;
//            edge->vertices()[1] = node;
//            edge->setParameterId(0, 0);
//            graph.addEdge(edge);
//        }

//        prev_node = node;

//    }

//    graph.initializeOptimization();
//    int iterations = graph.optimize(1024);
//    std::cout << "optimized graph " << std::endl;

//}

