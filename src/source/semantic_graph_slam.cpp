#include "semantic_graph_slam.h"

semantic_graph_slam::semantic_graph_slam()
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
    trans_odom2map_.setIdentity();

    object_detection_available_ = false;
    point_cloud_available_      = false;
    max_keyframes_per_update_   = 10;
    robot_pose_.setZero(6);
    cam_angle_ = 0;
}

void semantic_graph_slam::run()
{

    if(flush_keyframe_queue())
    {
        std::copy(new_keyframes_.begin(), new_keyframes_.end(), std::back_inserter(keyframes_));
        new_keyframes_.clear();
        //optimizing the graph
        std::cout << "optimizing the graph " << std::endl;
        graph_slam_->optimize();
        const auto& keyframe = keyframes_.back();
        Eigen::Isometry3d trans = keyframe->node->estimate() * keyframe->odom.inverse();
        trans_odom2map_mutex.lock();
        trans_odom2map_ = trans.matrix().cast<float>();
        trans_odom2map_mutex.unlock();

        return;
    }

    if(object_detection_available_ && point_cloud_available_)
    {
        std::vector<semantic_SLAM::ObjectInfo> object_info;
        this->getDetectedObjectInfo(object_info);

        sensor_msgs::PointCloud2 point_cloud_msg;
        this->getPointCloudData(point_cloud_msg);

        point_cloud_segmentation::segmentallPointCloudData(robot_pose_, cam_angle_, object_info, point_cloud_msg);
    }


    //std::cout << "translation " << keyframe->odom.translation() << std::endl;

}


void semantic_graph_slam::open(ros::NodeHandle n)
{

    init(n);

    odom_pose_sub_          = n.subscribe("/rovio/odometry", 1, &semantic_graph_slam::VIOCallback, this);
    cloud_sub_              = n.subscribe("/depth_registered/points",1,&semantic_graph_slam::PointCloudCallback, this);
    detected_object_sub_    = n.subscribe("/darknet_ros/bounding_boxes",1, &semantic_graph_slam::detectedObjectDarknetCallback, this);

}

void semantic_graph_slam::VIOCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{

    const ros::Time& stamp = odom_msg->header.stamp;
    Eigen::Isometry3d odom = hdl_graph_slam::odom2isometry(odom_msg);

    if(!keyframe_updater_->update(odom))
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
    pcl::fromROSMsg(cloud_msg, *cloud);

    double accum_d = keyframe_updater_->get_accum_distance();
    hdl_graph_slam::KeyFrame::Ptr keyframe(new hdl_graph_slam::KeyFrame(stamp, odom, accum_d, cloud));

    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
    keyframe_queue_.push_back(keyframe);
    std::cout << "added keyframe in queue" << std::endl;
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


    trans_odom2map_mutex.lock();
    Eigen::Isometry3d odom2map(trans_odom2map_.cast<double>());
    trans_odom2map_mutex.unlock();


    int num_processed = 0;
    for(int i=0; i<std::min<int>(keyframe_queue_.size(), max_keyframes_per_update_); i++)
    {
        num_processed = i;

        const auto& keyframe = keyframe_queue_[i];
        new_keyframes_.push_back(keyframe);

        Eigen::Isometry3d odom = odom2map * keyframe->odom;
        keyframe->node = graph_slam_->add_se3_node(odom);
        keyframe_hash_[keyframe->stamp] = keyframe;
        std::cout << "added new keyframe to the graph" << std::endl;

        if(i==0 && keyframes_.empty()) {
            continue;
        }


        // add edge between keyframes
        const auto& prev_keyframe = i == 0 ? keyframes_.back() : keyframe_queue_[i - 1];

        Eigen::Isometry3d relative_pose = keyframe->odom.inverse() * prev_keyframe->odom;
        Eigen::MatrixXd information = inf_calclator_->calc_information_matrix(prev_keyframe->cloud, keyframe->cloud, relative_pose);
        graph_slam_->add_se3_edge(keyframe->node, prev_keyframe->node, relative_pose, information);
        std::cout << "added new odom measurement to the graph" << std::endl;
    }

    keyframe_queue_.erase(keyframe_queue_.begin(), keyframe_queue_.begin() + num_processed + 1);

    return true;

}

