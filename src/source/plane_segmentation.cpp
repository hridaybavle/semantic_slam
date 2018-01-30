#include "plane_segmentation.h"


plane_segmentation::plane_segmentation()
{
    std::cout << "initialized plane segmentation " << std::endl;

}

plane_segmentation::~plane_segmentation()
{
    std::cout << "destructing plane segmentation " << std::endl;


}


sensor_msgs::PointCloud2 plane_segmentation::segmentPointCloudData(std::vector<semantic_SLAM::ObjectInfo> object_info, sensor_msgs::PointCloud2 point_cloud)
{
    sensor_msgs::PointCloud2 segmented_point_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> segmented_point_cloud_pcl;


    for (int i =0; i < object_info.size(); ++i)
    {
        if(object_info[i].type == "chair")
        {
            float start_u = object_info[i].tl_x, start_v = object_info[i].tl_y;
            float height = object_info[i].height, width = object_info[i].width;

            for(size_t u = start_u; u < (start_u+width); ++u)
            {
                //                std::cout << "pointcloud row  step " << point_cloud.row_step << std::endl;
                //                std::cout << "pointcloud point step " << point_cloud.point_step << std::endl;
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

                    //std::cout << "X position " << X << std::endl;
                    //std::cout << "Y position " << Y << std::endl;
                    //std::cout << "Z position " << Z << std::endl;
                    //std::cout << "RGB " << RGB << std::endl;

                    pcl::PointXYZRGB point;
                    point.x   = X;
                    point.y   = Y;
                    point.z   = Z;
                    point.rgb = RGB;

                    segmented_point_cloud_pcl.push_back(point);
                }
            }
        }
    }

    pcl::toROSMsg(segmented_point_cloud_pcl, segmented_point_cloud);

    return segmented_point_cloud;
}
