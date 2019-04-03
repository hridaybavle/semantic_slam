#include "plane_segmentation.h"


plane_segmentation::plane_segmentation()
{
    std::cout << "initialized plane segmentation " << std::endl;

}

plane_segmentation::~plane_segmentation()
{
    std::cout << "destructing plane segmentation " << std::endl;

}

inline bool comparator(const float lhs, const float rhs)
{
    return lhs > rhs;
}

plane_segmentation::segmented_objects plane_segmentation::segmentPointCloudData(semantic_SLAM::ObjectInfo object_info,
                                                                                sensor_msgs::PointCloud2 point_cloud,
                                                                                sensor_msgs::PointCloud2& segmented_point_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_point_cloud_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);
    plane_segmentation::segmented_objects segmented_objects_to_return;

    float start_u = object_info.tl_x, start_v = object_info.tl_y;
    float height = object_info.height, width = object_info.width;


    if(object_info.height < 0 || object_info.width < 0
            || (start_u + width) > 640 || (start_v + height) > 480)
    {
        segmented_objects_to_return.type = "spurious";
        return segmented_objects_to_return;
    }
    segmented_point_cloud_pcl->resize(object_info.height * object_info.width);
    segmented_point_cloud_pcl->height = object_info.height;
    segmented_point_cloud_pcl->width = object_info.width;
    segmented_point_cloud_pcl->is_dense = false;


    int p_u =0;
    for(size_t u = start_u; u < (start_u+width); ++u)
    {
        int p_v =0;
        for (size_t v = start_v; v < (start_v+height); ++v)
        {
            int arrayPosition = v*point_cloud.row_step + u*point_cloud.point_step;

            int arrayPosX, arrayPosY, arrayPosZ, arrayPosrgb;

            arrayPosX   = arrayPosition + point_cloud.fields[0].offset;
            arrayPosY   = arrayPosition + point_cloud.fields[1].offset;
            arrayPosZ   = arrayPosition + point_cloud.fields[2].offset;
            arrayPosrgb = arrayPosition + point_cloud.fields[3].offset;


            float X, Y, Z, RGB;
            memcpy(&X,   &point_cloud.data[arrayPosX], sizeof(float));
            memcpy(&Y,   &point_cloud.data[arrayPosY], sizeof(float));
            memcpy(&Z,   &point_cloud.data[arrayPosZ], sizeof(float));
            memcpy(&RGB, &point_cloud.data[arrayPosrgb], sizeof(float));


            pcl::PointXYZRGB point;
            point.x   = X;
            point.y   = Y;
            point.z   = Z;
            point.rgb = RGB;

            segmented_point_cloud_pcl->at(p_u,p_v) = point;
            p_v++;
        }
        p_u++;
    }

    segmented_objects_to_return.type = object_info.type;
    segmented_objects_to_return.prob = object_info.prob;
    segmented_objects_to_return.segmented_point_cloud = segmented_point_cloud_pcl;

    pcl::toROSMsg(*segmented_point_cloud_pcl, segmented_point_cloud);

    return segmented_objects_to_return;
}

pcl::PointCloud<pcl::Normal>::Ptr plane_segmentation::computeNormalsFromPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud)
{

    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
    normal_cloud->clear();


    if(point_cloud->points.size() < 5000)
    {
        return normal_cloud;
    }


    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor (0.03f);
    ne.setNormalSmoothingSize (20.0f);

    ne.setInputCloud(point_cloud);
    ne.compute (*normal_cloud);


    return normal_cloud;

}

std::vector<cv::Mat> plane_segmentation::multiPlaneSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
                                                                pcl::PointCloud<pcl::Normal>::Ptr point_normal,
                                                                Eigen::Matrix4f transformation_mat)
{
    std::vector<cv::Mat> final_pose_centroids_vec;
    final_pose_centroids_vec.clear();

    //first get the normals of horizontal planes using the transformation mat
    Eigen::Vector4f normals_of_the_horizontal_plane_in_world, normals_of_the_horizontal_plane_in_cam;
    normals_of_the_horizontal_plane_in_world.setZero(), normals_of_the_horizontal_plane_in_cam.setZero();

    normals_of_the_horizontal_plane_in_world(0) = 0;
    normals_of_the_horizontal_plane_in_world(1) = 0;
    normals_of_the_horizontal_plane_in_world(2) = 1;

    normals_of_the_horizontal_plane_in_cam = transformation_mat.transpose().eval() * normals_of_the_horizontal_plane_in_world;

    pcl::OrganizedMultiPlaneSegmentation< pcl::PointXYZRGB, pcl::Normal, pcl::Label > mps;
    mps.setMinInliers (500);
    mps.setAngularThreshold (0.017453 * 2.0); // 2 degrees
    mps.setDistanceThreshold (0.02); // 2cm
    mps.setInputNormals (point_normal);
    mps.setInputCloud (point_cloud);
    std::vector< pcl::PlanarRegion< pcl::PointXYZRGB >, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGB> > > regions;
    mps.segmentAndRefine (regions);

    for (size_t i = 0; i < regions.size (); i++)
    {
        Eigen::Vector3f centroid = regions[i].getCentroid ();
        Eigen::Vector4f model = regions[i].getCoefficients ();
        pcl::PointCloud<pcl::PointXYZRGB> boundary_cloud;
        boundary_cloud.points = regions[i].getContour ();
        //        printf ("Centroid: (%f, %f, %f)\n  Coefficients: (%f, %f, %f, %f)\n",
        //                centroid[0], centroid[1], centroid[2],
        //                model[0], model[1], model[2], model[3]);
        //        std::cout << "inliers " <<   boundary_cloud.points.size() << std::endl;

        cv::Mat final_pose_centroid;
        final_pose_centroid = cv::Mat::zeros(1, 8, CV_32F);

        //checking if the extract plane is a horizontal plane
        if(fabs(model[0] - normals_of_the_horizontal_plane_in_cam(0)) < 0.3 &&
                fabs(model[1] - normals_of_the_horizontal_plane_in_cam(1)) < 0.3 &&
                fabs(model[2] - normals_of_the_horizontal_plane_in_cam(2)) < 0.3)
        {
            //zero if its horizontal plane
            final_pose_centroid.at<float>(0,7) = 0;
            //std::cout << "Its horizontal plane " << std::endl;
        }
        else
        {
            //one if its a vertical plane
            final_pose_centroid.at<float>(0,7) = 1;
            //std::cout << "Its a vertical plane " << std::endl;
        }

        final_pose_centroid.at<float>(0,0) = centroid[0];
        final_pose_centroid.at<float>(0,1) = centroid[1];
        final_pose_centroid.at<float>(0,2) = centroid[2];
        final_pose_centroid.at<float>(0,3) = model[0];
        final_pose_centroid.at<float>(0,4) = model[1];
        final_pose_centroid.at<float>(0,5) = model[2];
        final_pose_centroid.at<float>(0,6) = model[4];

        final_pose_centroids_vec.push_back(final_pose_centroid);

    }

    //std::cout << "end of multiplane segmentation " << std::endl;
    return final_pose_centroids_vec;
}

double plane_segmentation::computeKmeans(cv::Mat points,
                                         const int num_centroids,
                                         cv::Mat &labels,
                                         cv::Mat &centroids)
{
    double compactness = 0.0;
    cv::TermCriteria criteria_kmeans(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 0.01);
    compactness = cv::kmeans(points, num_centroids, labels, criteria_kmeans, 10, cv::KMEANS_RANDOM_CENTERS, centroids);

    return compactness;

}

float plane_segmentation::computeDotProduct(Eigen::Vector4f vector_a, Eigen::Vector4f vector_b)
{
    float product =0;

    for (int i = 0; i < 3; i++)
        product = product + vector_a[i] * vector_b[i];

    return product;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_segmentation::preprocessPointCloud(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud)
{
    point_cloud = this->downsamplePointcloud(point_cloud);
    point_cloud = this->removeOutliers(point_cloud);
    point_cloud = this->distance_filter(point_cloud);

    return point_cloud;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_segmentation::downsamplePointcloud(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (point_cloud);
    sor.setLeafSize (0.1f, 0.1f, 0.1f);
    sor.filter (*cloud_filtered);

    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_segmentation::removeOutliers(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (point_cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);

    return cloud_filtered;

}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_segmentation::distance_filter(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud_filtered->reserve(point_cloud->size());

    std::copy_if(point_cloud->begin(), point_cloud->end(), std::back_inserter(cloud_filtered->points),
                 [&](pcl::PointXYZRGB& p) {
        double d = p.getVector3fMap().norm();
        return d > 0.3 && d < 3;
    }
    );

    cloud_filtered->width = cloud_filtered->size();
    cloud_filtered->height = 1;
    cloud_filtered->is_dense = false;

    cloud_filtered->header = point_cloud->header;

    return cloud_filtered;
}






