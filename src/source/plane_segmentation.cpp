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

plane_segmentation::segmented_objects plane_segmentation::segmentPointCloudData(semantic_SLAM::ObjectInfo object_info, sensor_msgs::PointCloud2 point_cloud,
                                                                                sensor_msgs::PointCloud2& segmented_point_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_point_cloud_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);
    plane_segmentation::segmented_objects segmented_objects_to_return;
    //for (int i =0; i < object_info.size(); ++i)
    //{
    //if(object_info[i].type == "chair")
    //{
    //transforming the color camera to depth cam frame
    //    Eigen::Matrix3f h_mat, k_depth, k_color;
    //    Eigen::Vector3f point_in_depth, point_in_color;

    //    point_in_color(0) = object_info.tl_x;
    //    point_in_color(1) = object_info.tl_y;
    //    point_in_color(2) = 1;

    //    h_mat(0,0) = 1; h_mat(0,1) = 0; h_mat(0,2) = 0.05;
    //    h_mat(1,0) = 0; h_mat(1,1) = 1; h_mat(1,2) = 0;
    //    h_mat(2,0) = 0; h_mat(2,1) = 0; h_mat(2,2) = 1;

    //    k_depth(0,0) = 312.6761474609375; k_depth(0,1) = 0; k_depth(0,2) = 161.23934936523438;
    //    k_depth(1,0) = 0; k_depth(1,1) = 312.6761474609375; k_depth(1,2) = 118.19078826904297;
    //    k_depth(2,0) = 0; k_depth(2,1) = 0; k_depth(2,2) = 1;

    //    k_color(0,0) = 314.9288024902344; k_color(0,1) = 0; k_color(0,2) = 159.6829071044922;
    //    k_color(1,0) = 0; k_color(1,1) = 317.964111328125; k_color(1,2) = 115.64295196533203;
    //    k_color(2,0) = 0; k_color(2,1) = 0; k_color(2,2) = 1;


    //point_in_depth = (k_depth * h_mat * k_color.inverse().eval()) * point_in_color;

    //    std::cout << "point in color  " << point_in_color << std::endl;
    //    std::cout << "point in depth  " << point_in_depth << std::endl;

    float start_u = object_info.tl_x, start_v = object_info.tl_y;
    float height = object_info.height, width = object_info.width;

    std::cout << "height " << object_info.height << std::endl;
    std::cout << "weight " << object_info.width  << std::endl;

    if(object_info.height < 0 || object_info.width < 0)
    {
        std::cout << "returning as spurious bb " << std::endl;
        segmented_objects_to_return.type = "spurious";
        return segmented_objects_to_return;
    }
    segmented_point_cloud_pcl->resize(object_info.height * object_info.width);
    segmented_point_cloud_pcl->height = object_info.height;
    segmented_point_cloud_pcl->width = object_info.width;
    segmented_point_cloud_pcl->is_dense = false;

    //    std::cout << "start u " << start_u << std::endl;
    //    std::cout << "start v " << start_v << std::endl;
    //    std::cout << "height "  << height << std::endl;
    //    std::cout << "width "   << width  << std::endl;

    int p_u =0;
    for(size_t u = start_u; u < (start_u+width); ++u)
    {
        //                std::cout << "pointcloud row  step " << point_cloud.row_step << std::endl;
        //                std::cout << "pointcloud point step " << point_cloud.point_step << std::endl;
        int p_v =0;
        for (size_t v = start_v; v < (start_v+height); ++v)
        {

            int arrayPosition = v*point_cloud.row_step + u*point_cloud.point_step;

            int arrayPosX, arrayPosY, arrayPosZ, arrayPosrgb;

            arrayPosX   = arrayPosition + point_cloud.fields[0].offset;
            arrayPosY   = arrayPosition + point_cloud.fields[1].offset;
            arrayPosZ   = arrayPosition + point_cloud.fields[2].offset;
            arrayPosrgb = arrayPosition + point_cloud.fields[3].offset;

            //std::cout << "arrayPosrgb " << point_cloud.fields[3].offset << std::endl;

            float X, Y, Z, RGB;
            memcpy(&X,   &point_cloud.data[arrayPosX], sizeof(float));
            memcpy(&Y,   &point_cloud.data[arrayPosY], sizeof(float));
            memcpy(&Z,   &point_cloud.data[arrayPosZ], sizeof(float));
            memcpy(&RGB, &point_cloud.data[arrayPosrgb], sizeof(float));

            //                    for(int f =0; f < point_cloud.fields.size(); ++f)
            //                        std::cout << "point cloud fields " << point_cloud.fields[f].name  << std::endl;

            //                    std::cout << "X position " << X << std::endl;
            //     /camera/color/image_raw               std::cout << "Y position " << Y << std::endl;
            //                    std::cout << "Z position " << Z << std::endl;
            //                    std::cout << "RGB " << RGB << std::endl;

            pcl::PointXYZRGB point;
            point.x   = X;
            point.y   = Y;
            point.z   = Z;
            point.rgb = RGB;

            //segmented_point_cloud_pcl->push_back(point);
            segmented_point_cloud_pcl->at(p_u,p_v) = point;
            p_v++;
        }
        p_u++;
    }
    //}
    //}

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

cv::Mat plane_segmentation::computeHorizontalPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud, pcl::PointCloud<pcl::Normal>::Ptr point_normal, Eigen::Matrix4f transformation_mat,
                                                   Eigen::MatrixXf final_pose, float& point_size)
{
    Eigen::Vector4f normals_of_the_horizontal_plane_in_world, normals_of_the_horizontal_plane_in_cam;

    normals_of_the_horizontal_plane_in_world.setZero(), normals_of_the_horizontal_plane_in_cam.setZero();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_without_nans (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmeted_points_using_first_kmeans (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_segmented_points (new pcl::PointCloud<pcl::PointXYZRGB>);

    cv::Mat empty_final_pose_mat;
    empty_final_pose_mat = cv::Mat::zeros(0, 3, CV_32F);

    //all the horizontal plane normals have this orientations
    normals_of_the_horizontal_plane_in_world(0) = 0;
    normals_of_the_horizontal_plane_in_world(1) = 0;
    normals_of_the_horizontal_plane_in_world(2) = 1;

    //finding the orientation of all the horizontal planes in the cam frame
    normals_of_the_horizontal_plane_in_cam = transformation_mat.transpose().eval() * normals_of_the_horizontal_plane_in_world;


    //---------------------removing the nans from the normals and the 3d points----------------------//
    cv::Mat normal_points, normal_filtered_points;
    normal_points = cv::Mat(point_normal->size(),3, CV_32F);
    normal_filtered_points = cv::Mat::zeros(0,3, CV_32F);

    for (size_t i =0; i < point_normal->size(); ++i)
    {
        if(!std::isnan(point_normal->points[i].normal_x) && !std::isnan(point_normal->points[i].normal_y)
                && !std::isnan(point_normal->points[i].normal_z))
        {
            normal_points.at<float>(i,0) =   static_cast<float>(point_normal->points[i].normal_x);
            normal_points.at<float>(i,1) =   static_cast<float>(point_normal->points[i].normal_y);
            normal_points.at<float>(i,2) =   static_cast<float>(point_normal->points[i].normal_z);

            points_without_nans->points.push_back(point_cloud->points[i]);
            normal_filtered_points.push_back(normal_points.row(i));
        }

    }
    //------------------------------------------------------------------------------//

    //-------------applying the kmeans for finding the centroids of all the points-----------------------//
    if(normal_filtered_points.rows <= 10)
    {
        std::cout << "returning as not enough normals points for kmeans" << std::endl;
        return empty_final_pose_mat;
    }
    cv::Mat centroids, labels;
    double compactness = 0.0;
    compactness = this->computeKmeans(normal_filtered_points,
                                      num_centroids_normals,
                                      labels,
                                      centroids);

    std::cout << "centroids of normals kmeans " << centroids << std::endl;

    float dot_product;
    for(int i =0; i < num_centroids_normals; ++i)
    {
        Eigen::Vector4f centroids_eigen;
        centroids_eigen.setZero();
        centroids_eigen(0) = centroids.at<float>(i,0);
        centroids_eigen(1) = centroids.at<float>(i,1);
        centroids_eigen(2) = centroids.at<float>(i,2);

        dot_product = this->computeDotProduct(normals_of_the_horizontal_plane_in_cam, centroids_eigen);

        std::cout << "dot product " << dot_product << std::endl;
    }

    //---------------------------------------------------------------------------------------------------//

    //---------------segmenting the normals and obtaining the corresponding 3D points--------------------//
    cv::Mat filtered_centroids = cv::Mat::zeros(0, 3, CV_32F);
    int filtered_centroids_counter = -1;
    for(int i = 0; i < num_centroids_normals; i++)
    {
        if(centroids.at<float>(i,0) < normals_of_the_horizontal_plane_in_cam(0)+0.3 && centroids.at<float>(i,0) > normals_of_the_horizontal_plane_in_cam(0)-0.3
                && centroids.at<float>(i,1) < normals_of_the_horizontal_plane_in_cam(1)+0.3 && centroids.at<float>(i,1) > normals_of_the_horizontal_plane_in_cam(1)-0.3
                && centroids.at<float>(i,2) < normals_of_the_horizontal_plane_in_cam(2)+0.3 && centroids.at<float>(i,2) > normals_of_the_horizontal_plane_in_cam(2)-0.3)
        {
            filtered_centroids.push_back(centroids.row(i));
            filtered_centroids_counter = i;
        }

    }

    std::cout << "filtered centroids " << filtered_centroids << std::endl;

    if(filtered_centroids.empty())
    {
        std::cout << "returning as no centroids of horizontal planes found " << std::endl;
        return empty_final_pose_mat;
    }

    //segmenting the pointcloud using the labels from the 1st kmeans
    for(size_t i = 0; i < points_without_nans->size(); ++i)
    {
        if(labels.at<int>(i,0) == filtered_centroids_counter)
            segmeted_points_using_first_kmeans->points.push_back(points_without_nans->points[i]);
    }
    //---------------------------------------------------------------------------------------------------------------//

    //-------------find the heights of each 3D point wrt to the camera and getting the height centroids--------------//
    std::vector<float> height_vector;
    float height;

    for (size_t i = 0; i < segmeted_points_using_first_kmeans->size(); ++i)
    {
        height = segmeted_points_using_first_kmeans->points[i].x * normals_of_the_horizontal_plane_in_cam(0) +
                segmeted_points_using_first_kmeans->points[i].y *  normals_of_the_horizontal_plane_in_cam(1) +
                segmeted_points_using_first_kmeans->points[i].z *  normals_of_the_horizontal_plane_in_cam(2);

        height = -1*height;
        height_vector.push_back(height);
    }

    //returning if the heigh vector size is less that 5 for avoiding failure of kmeans
    if(height_vector.size() <= 5)
    {
        std::cout << "height vector doesnt have enough data " << std::endl;
        return empty_final_pose_mat;
    }

    //applying second kmeans for height rejection
    cv::Mat height_points;
    height_points = cv::Mat(height_vector.size(),1, CV_32F);

    for(size_t i =0; i < height_vector.size(); ++i)
    {
        height_points.at<float>(i,0) = (float) height_vector.at(i);
    }

    cv::Mat height_centroids, height_labels;
    double height_compactness = 0.0;
    height_compactness = this->computeKmeans(height_points,
                                             num_centroids_height,
                                             height_labels,
                                             height_centroids);


    //std::cout << "height centroids " << height_centroids << std::endl;
    //std::cout << "height labels " << height_labels << std::endl;

    std::vector<float> height_centroids_vec;
    for(int i = 0; i < height_centroids.rows; ++i)
    {
        height_centroids_vec.push_back(height_centroids.at<float>(i,0));
    }

    std::sort(height_centroids_vec.begin(), height_centroids_vec.end(), comparator);

    std::cout << "final pose " << final_pose(2) << std::endl;
    for(int i =0; i < height_centroids_vec.size(); ++i)
    {
        std::cout << "height centroids sorted " << height_centroids_vec[i] << std::endl;
    }
    //-------------------------------------------------------------------------------------------------------------------//

    //------------------------obtaining the points corresponding to the segmented heights--------------------------------//
    int segmented_height_label = -1;
    for(int i = 0; i < height_centroids.rows; ++i)
    {
        //getting the label of the second height centroid
        if(height_centroids.at<float>(i,0) == height_centroids_vec[1] && (height_centroids_vec[1] < 2.1))
            segmented_height_label = i;
    }

    //std::cout << "segmented height label " << segmented_height_label << std::endl;

    if(segmented_height_label == -1)
    {
        std::cout << "returning as no height labels found" << std::endl;
        return empty_final_pose_mat;
    }

    //final segmented point cloud
    for(size_t i = 0; i < segmeted_points_using_first_kmeans->size(); ++i)
    {
        if(height_labels.at<int>(i,0) == segmented_height_label)
            final_segmented_points->points.push_back(segmeted_points_using_first_kmeans->points[i]);
    }

    point_size = final_segmented_points->size();

    //computer the convex hull
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_points_convex_hull;
    final_points_convex_hull = this->compute2DConvexHull(final_segmented_points);

    double x,y,z;
    std::vector<double> x_vec,y_vec,z_vec;
    x_vec.clear(); y_vec.clear(); z_vec.clear();

    for(int i = 0; i < final_points_convex_hull->size(); ++i)
    {
        x += final_points_convex_hull->points[i].x;
        y += final_points_convex_hull->points[i].y;
        z += final_points_convex_hull->points[i].z;


    }

    std::cout << "x" << x << std::endl;
    double x_final, y_final, z_final;

    x_final = x / final_points_convex_hull->size();
    y_final = y / final_points_convex_hull->size();
    z_final = z / final_points_convex_hull->size();

    //std::cout << "final_points_convex_hull->size() " << final_points_convex_hull->size() << std::endl;
    //std::cout << "Final pose from from convex hull " << std::endl
    //          <<  "X final : " << x_final << std::endl << "Y final : " << y_final << std::endl << "Z final: " << z_final << std::endl;

    cv::Mat final_pose_centroid;
    final_pose_centroid = cv::Mat::zeros(1, 3, CV_32F);
    if(!std::isnan(x_final) && !std::isnan(y_final) && !std::isnan(z_final))
    {
        final_pose_centroid.at<float>(0,0) = x_final;
        final_pose_centroid.at<float>(0,1) = y_final;
        final_pose_centroid.at<float>(0,2) = z_final;

        std::cout << "final pose centroid " << final_pose_centroid << std::endl;

        return final_pose_centroid;
    }
    else
    {
        std::cout << "returning as the final pose has nans "<< std::endl;
        return empty_final_pose_mat;
    }


    //    if(final_points_convex_hull->size() < 10)
    //    {
    //        std::cout << "returning as convex hull has too less points "<< std::endl;
    //        return empty_final_pose_mat;
    //    }

    //    cv::Mat final_segmented_points_mat;
    //    final_segmented_points_mat = cv::Mat(final_points_convex_hull->size(),3, CV_32F);
    //    for(size_t i =0; i < final_points_convex_hull->size(); ++i)
    //    {
    //        final_segmented_points_mat.at<float>(i,0) = final_points_convex_hull->points[i].x;
    //        final_segmented_points_mat.at<float>(i,1) = final_points_convex_hull->points[i].y;
    //        final_segmented_points_mat.at<float>(i,2) = final_points_convex_hull->points[i].z;

    //        //        std::cout << "final_segmented points pose "
    //        //                  << "X: " << final_segmented_points->points[i].x
    //        //                  << "Y: " << final_segmented_points->points[i].y
    //        //                  << "Z: " << final_segmented_points->points[i].z << std::endl;
    //    }

    //    //last kmeans for obtaining the final pose of the segmented horizontal plane
    //    cv::Mat kmeans_final_pose_centroids, final_pose_labels, final_pose_centroid;
    //    double final_pose_compactness = 0.0;

    //    final_pose_compactness = this->computeKmeans(final_segmented_points_mat,
    //                                                 num_centroids_pose,
    //                                                 final_pose_labels,
    //                                                 kmeans_final_pose_centroids);


    //    final_pose_centroid = cv::Mat::zeros(1, 3, CV_32F);
    //    if(kmeans_final_pose_centroids.at<float>(0,2) < kmeans_final_pose_centroids.at<float>(1,2) && kmeans_final_pose_centroids.at<float>(0,2) > 0)
    //    {
    //        final_pose_centroid.at<float>(0,0) = kmeans_final_pose_centroids.at<float>(0,0);
    //        final_pose_centroid.at<float>(0,1) = kmeans_final_pose_centroids.at<float>(0,1);
    //        final_pose_centroid.at<float>(0,2) = kmeans_final_pose_centroids.at<float>(0,2);
    //    }
    //    else
    //    {
    //        final_pose_centroid.at<float>(0,0) = kmeans_final_pose_centroids.at<float>(1,0);
    //        final_pose_centroid.at<float>(0,1) = kmeans_final_pose_centroids.at<float>(1,1);
    //        final_pose_centroid.at<float>(0,2) = kmeans_final_pose_centroids.at<float>(1,2);
    //    }

    //    //point_size = final_pose_labels.rows;
    //    point_size = final_segmented_points->size();
    //    //std::cout << "kmeans centroids from segmented horizontal plane" << kmeans_final_pose_centroids  << std::endl;
    //    std::cout << "final_pose from segmented horizontal plane" << final_pose_centroid  << std::endl;

    //    return final_pose_centroid;

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
    //point_cloud = this->compute2DConvexHull(point_cloud);

    return point_cloud;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_segmentation::downsamplePointcloud(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (point_cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
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
        return d > 0.3 && d < 6;
    }
    );

    cloud_filtered->width = cloud_filtered->size();
    cloud_filtered->height = 1;
    cloud_filtered->is_dense = false;

    cloud_filtered->header = point_cloud->header;

    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_segmentation::compute2DConvexHull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_point_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (filtered_point_cloud);
    seg.segment (*inliers, *coefficients);


    // Project the model inliers
    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (filtered_point_cloud);
    proj.setIndices (inliers);
    proj.setModelCoefficients (coefficients);
    proj.filter (*projected_cloud);

    // Create a Convex Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ConvexHull<pcl::PointXYZRGB> chull;
    chull.setInputCloud (projected_cloud);
    chull.reconstruct (*cloud_hull);

    //std::cout << "Final point cloud size " << filtered_point_cloud->size() << std::endl;
    //std::cout << "cloud size after convex hull " << cloud_hull->size() << std::endl;


    return cloud_hull;
}
