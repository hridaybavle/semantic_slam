#include "semantic_graph_slam.h"

semantic_graph_slam::semantic_graph_slam()
//    : sync(SyncPolicy(10))
{
    std::cout << "semantic graph slam constructor " << std::endl;

}

semantic_graph_slam::~semantic_graph_slam()
{
    std::cout << "semantic graph slam destructor " << std::endl;

}

void semantic_graph_slam::init(ros::NodeHandle n)
{
    keyframe_updater_.reset(new hdl_graph_slam::KeyframeUpdater(n));
    graph_slam_.reset(new hdl_graph_slam::GraphSLAM());
    inf_calclator_.reset(new hdl_graph_slam::InformationMatrixCalculator(n));
    loop_detector_.reset(new hdl_graph_slam::LoopDetector(n));
    trans_odom2map_.setIdentity();
    landmarks_vec_.clear();

    object_detection_available_ = false;
    point_cloud_available_      = false;
    rvio_pose_available_        = false;
    max_keyframes_per_update_   = 10;

    robot_pose_.setZero(6);
    cam_angle_ = 0;
}

void semantic_graph_slam::run()
{

    if(flush_keyframe_queue())
    {
        //loop detection
        //        std::vector<hdl_graph_slam::Loop::Ptr> loops = loop_detector_->detect(keyframes_, new_keyframes_, *graph_slam_);
        //        for(const auto& loop : loops) {
        //            Eigen::Isometry3d relpose(loop->relative_pose.cast<double>());
        //            Eigen::MatrixXd information_matrix = inf_calclator_->calc_information_matrix(loop->key1->cloud, loop->key2->cloud, relpose);
        //            graph_slam_->add_se3_edge(loop->key1->node, loop->key2->node, relpose, information_matrix);
        //        }

        //semantic data association and mapping
        for(int i = 0; i < new_keyframes_.size(); ++i)
        {
            if(!new_keyframes_[i]->obj_info.empty())
            {
                std::vector<semantic_SLAM::ObjectInfo> object_info = new_keyframes_[i]->obj_info;

                sensor_msgs::PointCloud2 point_cloud_msg = new_keyframes_[i]->cloud_msg;

                Eigen::Isometry3d trans = new_keyframes_[i]->node->estimate();
                Eigen::VectorXf robot_pose = hdl_graph_slam::matrix2vector(trans.matrix().cast<float>());

                std::cout << "robot pose " << robot_pose << std::endl;
                std::vector<detected_object> seg_obj_vec = point_cloud_segmentation::segmentallPointCloudData(robot_pose,
                                                                                                              cam_angle_,
                                                                                                              object_info,
                                                                                                              point_cloud_msg);

                std::cout << "seg_obj_vec size " << seg_obj_vec.size() << std::endl;

                std::vector<landmark> current_landmarks_vec = data_ass_obj_.find_matches(seg_obj_vec,
                                                                                         robot_pose,
                                                                                         cam_angle_);

                std::cout << "current_landmarks_vec size " << current_landmarks_vec.size() << std::endl;

                this->publishDetectedLandmarks(robot_pose, seg_obj_vec);
                //add the segmented landmarks to the graph for the current keyframe
                this->flush_landmark_queue(current_landmarks_vec,
                                           new_keyframes_[i]);

            }
        }

        //graph slam opitimization
        std::copy(new_keyframes_.begin(), new_keyframes_.end(), std::back_inserter(keyframes_));
        new_keyframes_.clear();

        //optimizing the graph
        //std::cout << "optimizing the graph " << std::endl;
        graph_slam_->optimize();
        const auto& keyframe = keyframes_.back();
        Eigen::Isometry3d trans = keyframe->node->estimate();
        //std::cout << "curr_key_trans " << trans.translation().cast<float>() << std::endl;

        //getting the optimized pose
        //corrected_pose_ = trans;

        publishLandmarks();
        publishKeyframePoses();

        return;

    }
    else
    {

        //adding the odom to the prev opitimized pose if no new keyframe is available
        //        Eigen::Isometry3d current_pose;
        //        this->getRVIOPose(current_pose);

        //        corrected_pose_ =  corrected_pose_  + current_pose;
        //        //add prev_odom prove properly this part is not finished properly
        //        prev_odom_pose_ = current_pose;

        return;
    }

}


void semantic_graph_slam::open(ros::NodeHandle n)
{

    init(n);

    //time synchronization of messages
    //    odom_msg_sub_.subscribe(n,"/rovio/odometry",10);
    //    point_cloud_msg_sub_.subscribe(n,"/depth_registered/points",10);
    //    bb_sub_.subscribe(n,"/darknet_ros/bounding_boxes",10);
    //    sync.connectInput(odom_msg_sub_,point_cloud_msg_sub_, bb_sub_);
    //    sync.registerCallback(boost::bind(&semantic_graph_slam::synMsgsCallback, this, _1, _2, _3));

    //subscribers
    odom_pose_sub_          = n.subscribe("/rovio/odometry", 1, &semantic_graph_slam::VIOCallback, this);
    cloud_sub_              = n.subscribe("/depth_registered/points",1,&semantic_graph_slam::PointCloudCallback, this);
    detected_object_sub_    = n.subscribe("/darknet_ros/bounding_boxes",1, &semantic_graph_slam::detectedObjectDarknetCallback, this);

    //publishers
    keyframe_pose_pub_      = n.advertise<geometry_msgs::PoseArray>("keyframe_poses",1);
    landmarks_pub_          = n.advertise<visualization_msgs::MarkerArray>("mapped_landmarks", 1);
    detected_lans_pub_      = n.advertise<visualization_msgs::MarkerArray>("detected_landmars",1);

}

//void semantic_graph_slam::synMsgsCallback(const nav_msgs::OdometryConstPtr &odom_msg,
//                                          const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
//                                          const darknet_ros_msgs::BoundingBoxesConstPtr &bbs_msg)
//{
//    std::cout << "odom msg " << odom_msg << std::endl;

//}

void semantic_graph_slam::VIOCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{

    const ros::Time& stamp      = odom_msg->header.stamp;
    Eigen::Isometry3d odom      = hdl_graph_slam::odom2isometry(odom_msg);
    Eigen::MatrixXf odom_cov    = hdl_graph_slam::arrayToMatrix(odom_msg);

    //dont update keyframes only if the keyframe time is less or no detection is available
    if(!keyframe_updater_->update(odom, stamp) /*&& !object_detection_available_*/)
    {
        if(keyframe_queue_.empty())
        {
            //std::cout << "no keyframes in queue " << std::endl;
        }
        return;
    }


    sensor_msgs::PointCloud2 cloud_msg;
    this->getPointCloudData(cloud_msg);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    std::vector<int> indices;
    pcl::fromROSMsg(cloud_msg, *cloud);
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    std::vector<semantic_SLAM::ObjectInfo> obj_info; obj_info.clear();
    if(object_detection_available_)
        this->getDetectedObjectInfo(obj_info);

    double accum_d = keyframe_updater_->get_accum_distance();
    hdl_graph_slam::KeyFrame::Ptr keyframe(new hdl_graph_slam::KeyFrame(stamp, odom, odom_cov, accum_d, cloud, cloud_msg, obj_info));

    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
    keyframe_queue_.push_back(keyframe);
    std::cout << "added keyframe in queue" << std::endl;

    this->setRVIOPose(odom);
}

void semantic_graph_slam::setRVIOPose(Eigen::Isometry3d RVIO_pose)
{

    //RVIO_pose_ = RVIO_pose;
    rvio_pose_available_ = true;

}

void semantic_graph_slam::getRVIOPose(Eigen::Isometry3d &RVIO_pose)
{
    //RVIO_pose = RVIO_pose_;
    rvio_pose_available_ = false;

}

void semantic_graph_slam::PointCloudCallback(const sensor_msgs::PointCloud2 &msg)
{

    this->setPointCloudData(msg);
}

void semantic_graph_slam::detectedObjectDarknetCallback(const darknet_ros_msgs::BoundingBoxes &msg)
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

    this->setDetectedObjectInfo(object_info);
}

void semantic_graph_slam::setDetectedObjectInfo(std::vector<semantic_SLAM::ObjectInfo> object_info)
{
    object_detection_available_ = true;
    this->object_info_ = object_info;
}

void semantic_graph_slam::getDetectedObjectInfo(std::vector<semantic_SLAM::ObjectInfo> &object_info)
{
    object_detection_available_ = false;
    object_info = this->object_info_;
}


void semantic_graph_slam::setPointCloudData(sensor_msgs::PointCloud2 point_cloud)
{

    point_cloud_available_ = true;
    this->point_cloud_msg_ = point_cloud;

}

void semantic_graph_slam::getPointCloudData(sensor_msgs::PointCloud2 &point_cloud)
{

    point_cloud_available_ = false;
    point_cloud = this->point_cloud_msg_;

}

bool semantic_graph_slam::flush_keyframe_queue()
{
    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);

    if(keyframe_queue_.empty()) {
        return false;
    }


    int num_processed = 0;
    for(int i=0; i<std::min<int>(keyframe_queue_.size(), max_keyframes_per_update_); i++)
    {
        num_processed = i;

        const auto& keyframe = keyframe_queue_[i];
        new_keyframes_.push_back(keyframe);

        Eigen::Isometry3d odom = keyframe->odom;
        keyframe->node = graph_slam_->add_se3_node(odom);
        keyframe_hash_[keyframe->stamp] = keyframe;
        std::cout << "added new keyframe to the graph" << std::endl;

        if(i==0 && keyframes_.empty()) {
            continue;
        }


        // add edge between keyframes
        const auto& prev_keyframe = i == 0 ? keyframes_.back() : keyframe_queue_[i - 1];

        Eigen::Isometry3d relative_pose = keyframe->odom.inverse() * prev_keyframe->odom;
        Eigen::MatrixXd information = /*keyframe->odom_cov.inverse().cast<double>(); */ inf_calclator_->calc_information_matrix(prev_keyframe->cloud, keyframe->cloud, relative_pose);
        graph_slam_->add_se3_edge(keyframe->node, prev_keyframe->node, relative_pose, information);
        std::cout << "added new odom measurement to the graph" << std::endl;
    }

    keyframe_queue_.erase(keyframe_queue_.begin(), keyframe_queue_.begin() + num_processed + 1);

    return true;

}

void semantic_graph_slam::flush_landmark_queue(std::vector<landmark> current_lan_queue,
                                               const auto current_keyframe)
{

    if(current_lan_queue.empty())
        return;

    for(int i = 0; i < current_lan_queue.size(); ++i)
    {
        //adding new landmark vertex if only the landmark is seen once
        if(current_lan_queue[i].is_new_landmark)
        {
            current_lan_queue[i].node = graph_slam_->add_point_xyz_node(current_lan_queue[i].pose.cast<double>());
            current_lan_queue[i].is_new_landmark = false;
            data_ass_obj_.assignLandmarkNode(current_lan_queue[i].id, current_lan_queue[i].node);
            std::cout << "added the landmark position node " << std::endl;
        }


        //add an edge between landmark and the current keyframe
        Eigen::Matrix3f information = current_lan_queue[i].covariance.inverse();
        graph_slam_->add_se3_point_xyz_edge(current_keyframe->node, current_lan_queue[i].node, current_lan_queue[i].local_pose.cast<double>(), information.cast<double>());
        std::cout << "added an edge between the landmark and it keyframe " << std::endl;

    }

}

void semantic_graph_slam::publishLandmarks()
{

    visualization_msgs::MarkerArray marker_arrays;
    int marker_id = 0;
    std::vector<landmark> l_vec;
    data_ass_obj_.getMappedLandmarks(l_vec);

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

void semantic_graph_slam::publishDetectedLandmarks(Eigen::VectorXf robot_pose, std::vector<detected_object> det_obj_info)
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


void semantic_graph_slam::publishKeyframePoses()
{
    pose_array_.poses.clear();
    pose_array_.header.stamp = ros::Time::now();
    pose_array_.header.frame_id = "map";

    //copying the new keyframes for publishing
    geometry_msgs::Pose key_pose;
    for(int i=0; i < keyframes_.size(); ++i)
    {
        key_pose = hdl_graph_slam::matrix2pose(ros::Time::now(),
                                               keyframes_[i]->node->estimate().matrix().cast<float>(),
                                               "map");
        pose_array_.poses.push_back(key_pose);
    }


    keyframe_pose_pub_.publish(pose_array_);

}

void semantic_graph_slam::saveGraph()
{
    graph_slam_->save("/home/hriday/Desktop/test.g2o");
}
