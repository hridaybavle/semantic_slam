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

sensor_msgs::PointCloud2 plane_segmentation::segmentPointCloudData(std::vector<semantic_SLAM::ObjectInfo> object_info, sensor_msgs::PointCloud2 point_cloud,
                                                                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr& segmented_point_cloud_pcl)
{
    sensor_msgs::PointCloud2 segmented_point_cloud;
    //pcl::PointCloud<pcl::PointXYZRGB> segmented_point_cloud_pcl;

    for (int i =0; i < object_info.size(); ++i)
    {
        if(object_info[i].type == "chair")
        {
            float start_u = object_info[i].tl_x, start_v = object_info[i].tl_y;
            float height = object_info[i].height, width = object_info[i].width;

            segmented_point_cloud_pcl->resize(object_info[i].height * object_info[i].width);
            segmented_point_cloud_pcl->height = object_info[i].height;
            segmented_point_cloud_pcl->width = object_info[i].width;
            segmented_point_cloud_pcl->is_dense = false;

            std::cout << "start u " << start_u << std::endl;
            std::cout << "start v " << start_v << std::endl;
            std::cout << "height "  << height << std::endl;
            std::cout << "width "   << width  << std::endl;

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
        }
    }

    std::cout << "here 2 " << std::endl;
    pcl::toROSMsg(*segmented_point_cloud_pcl, segmented_point_cloud);

    return segmented_point_cloud;
}

pcl::PointCloud<pcl::Normal>::Ptr plane_segmentation::computeNormalsFromPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud)
{
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.SIMPLE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor (0.03f);
    ne.setNormalSmoothingSize (20.0f);

    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
    normal_cloud->clear();

    ne.setInputCloud(point_cloud);
    ne.compute (*normal_cloud);

    return normal_cloud;

}

cv::Mat plane_segmentation::computeHorizontalPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud, pcl::PointCloud<pcl::Normal>::Ptr point_normal, Eigen::Matrix4f transformation_mat)
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

    std::cout << "normals in the cam frame " << normals_of_the_horizontal_plane_in_cam << std::endl;

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
    cv::TermCriteria criteria_kmeans(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 0.01);
    compactness = cv::kmeans(normal_filtered_points, num_centroids_normals, labels, criteria_kmeans, 10, cv::KMEANS_RANDOM_CENTERS, centroids);

    std::cout << "centroids of normals kmeans " << centroids << std::endl;
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
    cv::TermCriteria height_criteria_kmeans(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 0.01);
    height_compactness = cv::kmeans(height_points, num_centroids_height, height_labels, height_criteria_kmeans, 10, cv::KMEANS_RANDOM_CENTERS, height_centroids);

    std::cout << "height centroids " << height_centroids << std::endl;
    //std::cout << "height labels " << height_labels << std::endl;

    std::vector<float> height_centroids_vec;
    for(int i = 0; i < height_centroids.rows; ++i)
    {
        height_centroids_vec.push_back(height_centroids.at<float>(i,0));
    }

    std::sort(height_centroids_vec.begin(), height_centroids_vec.end(), comparator);

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
        if(height_centroids.at<float>(i,0) == height_centroids_vec[1])
            segmented_height_label = i;
    }

    std::cout << "segmented height label " << segmented_height_label << std::endl;

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

    cv::Mat final_segmented_points_mat;
    final_segmented_points_mat = cv::Mat(final_segmented_points->size(),3, CV_32F);
    for(size_t i =0; i < final_segmented_points->size(); ++i)
    {
        final_segmented_points_mat.at<float>(i,0) = final_segmented_points->points[i].x;
        final_segmented_points_mat.at<float>(i,1) = final_segmented_points->points[i].y;
        final_segmented_points_mat.at<float>(i,2) = final_segmented_points->points[i].z;
    }

    //last kmeans for obtaining the final pose of the segmented horizontal plane
    cv::Mat final_pose_centroid, final_pose_labels;
    double final_pose_compactness = 0.0;
    cv::TermCriteria final_pose_criteria_kmeans(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 0.01);
    final_pose_compactness = cv::kmeans(final_segmented_points_mat, 1, final_pose_labels, final_pose_criteria_kmeans, 10, cv::KMEANS_RANDOM_CENTERS, final_pose_centroid);

    std::cout << "final_pose from segmented horizontal plane" << final_pose_centroid  << std::endl;
    return final_pose_centroid;

}















