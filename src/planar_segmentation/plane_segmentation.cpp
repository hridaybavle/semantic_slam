#include "planar_segmentation/plane_segmentation.h"

plane_segmentation::plane_segmentation(bool verbose) {
  std::cout << "Initialized plane segmentation " << std::endl;

  verbose_ = verbose;
  ros::param::param<double>("~num_point_seg", num_points_seg_, 500);
  ros::param::param<double>("~norm_point_thres", normal_point_thres_, 5000);
  ros::param::param<double>("~planar_area", planar_area_, 0.1);

  std::cout << "Min num points for segmentation " << num_points_seg_
            << std::endl;
  std::cout << "Min normal points threshold " << normal_point_thres_
            << std::endl;
  std::cout << "planar area " << planar_area_ << std::endl;
}

plane_segmentation::~plane_segmentation() {
  std::cout << "Destructing plane segmentation " << std::endl;
}

inline bool comparator(const float lhs, const float rhs) { return lhs > rhs; }

plane_segmentation::segmented_objects plane_segmentation::segmentPointCloudData(
    semantic_SLAM::ObjectInfo object_info, sensor_msgs::PointCloud2 point_cloud,
    sensor_msgs::PointCloud2 &segmented_point_cloud) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_point_cloud_pcl(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  plane_segmentation::segmented_objects segmented_objects_to_return;

  float start_u = object_info.tl_x, start_v = object_info.tl_y;
  float height = object_info.height, width = object_info.width;

  if (object_info.height < 0 || object_info.width < 0 ||
      (start_u + width) > 640 || (start_v + height) > 480) {
    segmented_objects_to_return.type = "spurious";
    return segmented_objects_to_return;
  }
  segmented_point_cloud_pcl->resize(object_info.height * object_info.width);
  segmented_point_cloud_pcl->height = object_info.height;
  segmented_point_cloud_pcl->width = object_info.width;
  segmented_point_cloud_pcl->is_dense = false;

  int p_u = 0;
  for (size_t u = start_u; u < (start_u + width); ++u) {
    int p_v = 0;
    for (size_t v = start_v; v < (start_v + height); ++v) {
      int arrayPosition = v * point_cloud.row_step + u * point_cloud.point_step;

      int arrayPosX, arrayPosY, arrayPosZ, arrayPosrgb;

      arrayPosX = arrayPosition + point_cloud.fields[0].offset;
      arrayPosY = arrayPosition + point_cloud.fields[1].offset;
      arrayPosZ = arrayPosition + point_cloud.fields[2].offset;
      arrayPosrgb = arrayPosition + point_cloud.fields[3].offset;

      float X, Y, Z, RGB;
      memcpy(&X, &point_cloud.data[arrayPosX], sizeof(float));
      memcpy(&Y, &point_cloud.data[arrayPosY], sizeof(float));
      memcpy(&Z, &point_cloud.data[arrayPosZ], sizeof(float));
      memcpy(&RGB, &point_cloud.data[arrayPosrgb], sizeof(float));

      pcl::PointXYZRGB point;
      point.x = X;
      point.y = Y;
      point.z = Z;
      point.rgb = RGB;

      segmented_point_cloud_pcl->at(p_u, p_v) = point;
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

pcl::PointCloud<pcl::Normal>::Ptr
plane_segmentation::computeNormalsFromPointCloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
    pcl::PointIndices::Ptr inliers) {

  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(
      new pcl::PointCloud<pcl::Normal>);
  normal_cloud->clear();

  if (point_cloud->points.size() < normal_point_thres_) {
    return normal_cloud;
  }

  pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor(0.03f);
  ne.setNormalSmoothingSize(20.0f);

  ne.setInputCloud(point_cloud);
  // ne.setIndices(inliers);
  ne.compute(*normal_cloud);
  return normal_cloud;
}

std::vector<plane_segmentation::segmented_planes>
plane_segmentation::multiPlaneSegmentation(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
    pcl::PointCloud<pcl::Normal>::Ptr point_normal,
    pcl::PointIndices::Ptr inliers, Eigen::Matrix4f transformation_mat) {
  std::vector<cv::Mat> final_pose_centroids_vec;
  final_pose_centroids_vec.clear();

  std::vector<plane_segmentation::segmented_planes> planes_vec;
  planes_vec.clear();

  // first get the normals of horizontal planes using the transformation mat
  Eigen::Vector4f normals_of_the_horizontal_plane_in_world,
      normals_of_the_horizontal_plane_in_cam;
  normals_of_the_horizontal_plane_in_world.setZero(),
      normals_of_the_horizontal_plane_in_cam.setZero();

  normals_of_the_horizontal_plane_in_world(0) = 0;
  normals_of_the_horizontal_plane_in_world(1) = 0;
  normals_of_the_horizontal_plane_in_world(2) = 1;

  normals_of_the_horizontal_plane_in_cam =
      transformation_mat.transpose().eval() *
      normals_of_the_horizontal_plane_in_world;
  if (verbose_)
    std::cout << "normals_of_the_horizontal_plane_in_cam "
              << normals_of_the_horizontal_plane_in_cam << std::endl;

  pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGB, pcl::Normal,
                                       pcl::Label>
      mps;
  mps.setMinInliers(num_points_seg_);
  mps.setAngularThreshold(0.017453 * 2.0); // 2 degrees
  mps.setDistanceThreshold(0.02);          // 2cm
  mps.setInputNormals(point_normal);
  mps.setInputCloud(point_cloud);
  // mps.setIndices(inliers);
  // mps.setComparator();

  std::vector<pcl::PlanarRegion<pcl::PointXYZRGB>,
              Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGB>>>
      regions;
  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<pcl::PointIndices> inlier_indices;
  pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);
  std::vector<pcl::PointIndices> label_indices;
  std::vector<pcl::PointIndices> boundary_indices;
  mps.segmentAndRefine(regions, model_coefficients, inlier_indices, labels,
                       label_indices, boundary_indices);

  for (size_t i = 0; i < regions.size(); i++) {
    plane_segmentation::segmented_planes planar_surf;
    Eigen::Vector3f centroid = regions[i].getCentroid();
    Eigen::Vector4f model = regions[i].getCoefficients();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundary_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    boundary_cloud->points = regions[i].getContour();
    // this->downsamplePointcloud(point_cloud);

    if (boundary_cloud->points.size() > 100) {
      if (verbose_)
        std::cout << "Centroids from multiplane: " << centroid[0] << ","
                  << centroid[1] << "," << centroid[2] << std::endl
                  << "Coefficients from multiplane: " << model[0] << ","
                  << model[1] << "," << model[2] << "," << model[3]
                  << std::endl;

      cv::Mat final_pose_centroid;
      final_pose_centroid = cv::Mat::zeros(1, 8, CV_32F);

      Eigen::Vector4f normals_extracted;
      normals_extracted.setZero();
      normals_extracted(0) = model[0];
      normals_extracted(1) = model[1];
      normals_extracted(2) = model[2];

      float dot_product = this->computeDotProduct(
          normals_of_the_horizontal_plane_in_cam, normals_extracted);

      float area = pcl::calculatePolygonArea(*boundary_cloud);
      if (verbose_)
        std::cout << "polygon area is: " << area << std::endl;

      // height of the object should never be above the camera
      // if(centroid[2] < 6.0)
      if (area >= planar_area_) {
        // checking if the extract plane is a horizontal plane or vertical
        if (fabs(model[0]) - fabs(normals_of_the_horizontal_plane_in_cam(0)) <
                0.3 &&
            fabs(model[1]) - fabs(normals_of_the_horizontal_plane_in_cam(1)) <
                0.3 &&
            fabs(model[2]) - fabs(normals_of_the_horizontal_plane_in_cam(2)) <
                0.3) {
          // zero if its horizontal plane
          final_pose_centroid.at<float>(0, 7) = 0;
          final_pose_centroid.at<float>(0, 0) = centroid[0];
          final_pose_centroid.at<float>(0, 1) = centroid[1];
          final_pose_centroid.at<float>(0, 2) = centroid[2];

          // this is for ensuring all the horizontal planes have upwards normals
          if (model[1] > 0) {
            final_pose_centroid.at<float>(0, 3) = -model[0];
            final_pose_centroid.at<float>(0, 4) = -model[1];
            final_pose_centroid.at<float>(0, 5) = -model[2];
            final_pose_centroid.at<float>(0, 6) = -model[3];
          } else {
            final_pose_centroid.at<float>(0, 3) = model[0];
            final_pose_centroid.at<float>(0, 4) = model[1];
            final_pose_centroid.at<float>(0, 5) = model[2];
            final_pose_centroid.at<float>(0, 6) = model[3];
          }

          planar_surf.final_pose_mat = final_pose_centroid;
          planar_surf.planar_points.points = boundary_cloud->points;
          planes_vec.push_back(planar_surf);
          final_pose_centroids_vec.push_back(final_pose_centroid);
        } else if (dot_product < 0.5) {
          // std::cout << "Its a vertical plane " << std::endl;
          // one if its a vertical plane
          final_pose_centroid.at<float>(0, 7) = 1;
          final_pose_centroid.at<float>(0, 0) = centroid[0];
          final_pose_centroid.at<float>(0, 1) = centroid[1];
          final_pose_centroid.at<float>(0, 2) = centroid[2];

          // this is for ensuring all vert normals face towards the left
          if (model[0] > 0) {
            final_pose_centroid.at<float>(0, 3) = -model[0];
            final_pose_centroid.at<float>(0, 4) = -model[1];
            final_pose_centroid.at<float>(0, 5) = -model[2];
            final_pose_centroid.at<float>(0, 6) = -model[3];
            // std::cout << "\033[1;31m BAD NORMAL\033[0m\n" << std::endl;
          } else {
            final_pose_centroid.at<float>(0, 3) = model[0];
            final_pose_centroid.at<float>(0, 4) = model[1];
            final_pose_centroid.at<float>(0, 5) = model[2];
            final_pose_centroid.at<float>(0, 6) = model[3];
            // std::cout << "\033[1;34m GOOD NORMAL\033[0m\n" << std::endl;
          }

          planar_surf.final_pose_mat = final_pose_centroid;
          planar_surf.planar_points.points = boundary_cloud->points;
          planes_vec.push_back(planar_surf);
          final_pose_centroids_vec.push_back(final_pose_centroid);
        }
      }
    }
  }

  return planes_vec;
}

std::vector<cv::Mat> plane_segmentation::clusterAndSegmentAllPlanes(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
    pcl::PointCloud<pcl::Normal>::Ptr point_normal,
    Eigen::Matrix4f transformation_mat) {

  std::vector<cv::Mat> empty_final_pose_vec;

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
      segmented_points_using_first_kmeans;
  segmented_points_using_first_kmeans.clear();
  cv::Mat filtered_centroids = this->NormalBasedClusteringAndSegmentation(
      point_cloud, point_normal, transformation_mat,
      segmented_points_using_first_kmeans);

  if (filtered_centroids.empty())
    return empty_final_pose_vec;

  std::vector<cv::Mat> final_normals_with_distances;
  final_normals_with_distances.clear();
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
      segmented_points_using_second_kmeans;
  segmented_points_using_second_kmeans.clear();
  final_normals_with_distances = this->distanceBasedSegmentation(
      segmented_points_using_first_kmeans, filtered_centroids,
      segmented_points_using_second_kmeans);

  if (final_normals_with_distances.empty())
    return empty_final_pose_vec;

  std::vector<cv::Mat> final_pose_vec = this->getFinalPoseWithNormals(
      segmented_points_using_second_kmeans, final_normals_with_distances);

  return final_pose_vec;
}

cv::Mat plane_segmentation::NormalBasedClusteringAndSegmentation(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
    pcl::PointCloud<pcl::Normal>::Ptr point_normal,
    Eigen::Matrix4f transformation_mat,
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
        &segmented_points_using_first_kmeans) {
  cv::Mat empty_filtered_pose;
  empty_filtered_pose = cv::Mat::zeros(0, 3, CV_32F);

  //---------------------removing the nans from the normals and the 3d
  // points----------------------//
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_without_nans(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  cv::Mat normal_filtered_points_mat;
  normal_filtered_points_mat = cv::Mat::zeros(0, 3, CV_32F);

  this->removeNans(point_cloud, point_normal, normal_filtered_points_mat,
                   points_without_nans);
  //------------------------------------------------------------------------------------------------//

  //-------------applying the kmeans for finding the centroids of all the
  // points-----------------------//
  if (normal_filtered_points_mat.rows <= 10) {
    // std::cout << "returning as not enough normals points for kmeans" <<
    // std::endl;
    return empty_filtered_pose;
  }

  cv::Mat normal_centroids, labels;
  double compactness = 0.0;
  compactness =
      this->computeKmeans(normal_filtered_points_mat, num_centroids_normals,
                          labels, normal_centroids);

  // std::cout << "normal centroids from clustering " << normal_centroids <<
  // std::endl;
  //----------------------------------------------------------------------------------------------------//
  Eigen::Vector4f normals_of_the_horizontal_plane_in_world,
      normals_of_the_horizontal_plane_in_cam;
  normals_of_the_horizontal_plane_in_world.setZero(),
      normals_of_the_horizontal_plane_in_cam.setZero();
  // all the horizontal plane normals have this orientations in world
  normals_of_the_horizontal_plane_in_world(0) = 0;
  normals_of_the_horizontal_plane_in_world(1) = 0;
  normals_of_the_horizontal_plane_in_world(2) = 1;

  // finding the orientation of all the horizontal planes in the cam frame
  normals_of_the_horizontal_plane_in_cam =
      transformation_mat.transpose().eval() *
      normals_of_the_horizontal_plane_in_world;

  std::vector<int> filtered_centroids_id;
  cv::Mat filtered_centroids = this->filterCentroids(
      normal_centroids, normals_of_the_horizontal_plane_in_cam,
      filtered_centroids_id);


  for (int i = 0; i < filtered_centroids_id.size(); ++i) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    point_cloud->clear();
    for (size_t j = 0; j < points_without_nans->size(); ++j) {
      if (labels.at<int>(j, 0) == filtered_centroids_id[i])
        point_cloud->points.push_back(points_without_nans->points[j]);
    }

    segmented_points_using_first_kmeans.push_back(point_cloud);
  }
  //----------------------------------------------------------------------------------------------------------//

  return filtered_centroids;
}

std::vector<cv::Mat> plane_segmentation::distanceBasedSegmentation(
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
        segmented_points_using_first_kmeans,
    cv::Mat filtered_centroids,
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
        &final_segmented_vec_of_points_from_distances) {
  std::vector<cv::Mat> final_normals_with_distances;

  for (int i = 0; i < filtered_centroids.rows; ++i) {
    cv::Mat distance_mat = cv::Mat::zeros(0, 1, CV_32F);
    float distance;

    for (size_t j = 0; j < segmented_points_using_first_kmeans[i]->size();
         ++j) {
      distance = segmented_points_using_first_kmeans[i]->points[j].x *
                     filtered_centroids.at<float>(i, 0) +
                 segmented_points_using_first_kmeans[i]->points[j].y *
                     filtered_centroids.at<float>(i, 1) +
                 segmented_points_using_first_kmeans[i]->points[j].z *
                     filtered_centroids.at<float>(i, 2);

      distance = -1 * distance;
      distance_mat.push_back(distance);
    }

    cv::Mat distance_centroids, distance_labels;
    double distance_compactness = 0.0;
    distance_compactness =
        this->computeKmeans(distance_mat, num_centroids_distance,
                            distance_labels, distance_centroids);

    for (int d = 0; d < distance_centroids.rows; ++d) {
      cv::Mat final_normal_with_distance_mat = cv::Mat::zeros(1, 4, CV_32F);
      final_normal_with_distance_mat.at<float>(0, 0) =
          filtered_centroids.at<float>(i, 0);
      final_normal_with_distance_mat.at<float>(0, 1) =
          filtered_centroids.at<float>(i, 1);
      final_normal_with_distance_mat.at<float>(0, 2) =
          filtered_centroids.at<float>(i, 2);
      final_normal_with_distance_mat.at<float>(0, 3) =
          distance_centroids.at<float>(d, 0);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(
          new pcl::PointCloud<pcl::PointXYZRGB>);
      point_cloud->clear();
      for (size_t c = 0; c < segmented_points_using_first_kmeans[i]->size();
           ++c) {
        if (distance_labels.at<int>(c, 0) == d)
          point_cloud->points.push_back(
              segmented_points_using_first_kmeans[i]->points[c]);
      }

      if (point_cloud->size() > 500) {
        final_normals_with_distances.push_back(
            final_normal_with_distance_mat.row(0));
        final_segmented_vec_of_points_from_distances.push_back(point_cloud);
      }
    }
  }

  return final_normals_with_distances;
}

std::vector<cv::Mat> plane_segmentation::getFinalPoseWithNormals(
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
        segmented_points_vec_using_second_kmeans,
    std::vector<cv::Mat> final_normals_with_distances) {
  std::vector<cv::Mat> final_pose_centroids_vec;
  final_pose_centroids_vec.clear();

  for (int i = 0; i < segmented_points_vec_using_second_kmeans.size(); ++i) {
    // computer the convex hull
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_points_convex_hull;
    final_points_convex_hull =
        this->compute2DConvexHull(segmented_points_vec_using_second_kmeans[i]);

    double x = 0, y = 0, z = 0;

    for (int j = 0; j < final_points_convex_hull->size(); ++j) {
      cv::Mat final_pose_centroid;
      final_pose_centroid = cv::Mat::zeros(1, 8, CV_32F);
      // if(!std::isnan(x_final) && !std::isnan(y_final) &&
      // !std::isnan(z_final))
      {
        final_pose_centroid.at<float>(0, 0) =
            final_points_convex_hull->points[j].x;
        final_pose_centroid.at<float>(0, 1) =
            final_points_convex_hull->points[j].y;
        final_pose_centroid.at<float>(0, 2) =
            final_points_convex_hull->points[j].z;
        final_pose_centroid.at<float>(0, 3) =
            final_normals_with_distances[i].at<float>(0, 0);
        final_pose_centroid.at<float>(0, 4) =
            final_normals_with_distances[i].at<float>(0, 1);
        final_pose_centroid.at<float>(0, 5) =
            final_normals_with_distances[i].at<float>(0, 2);
        final_pose_centroid.at<float>(0, 6) =
            final_normals_with_distances[i].at<float>(0, 3);
        final_pose_centroid.at<float>(0, 7) = 0;

        // std::cout << "final pose centroid " << final_pose_centroid <<
        // std::endl;
        final_pose_centroids_vec.push_back(final_pose_centroid);
      }
    }
  }

  return final_pose_centroids_vec;
}

void plane_segmentation::removeNans(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
    pcl::PointCloud<pcl::Normal>::Ptr point_normal,
    cv::Mat &normal_filtered_points_mat,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points_without_nans) {
  cv::Mat normal_points_mat;
  normal_points_mat = cv::Mat(point_normal->size(), 3, CV_32F);

  for (size_t i = 0; i < point_normal->size(); ++i) {
    if (!std::isnan(point_normal->points[i].normal_x) &&
        !std::isnan(point_normal->points[i].normal_y) &&
        !std::isnan(point_normal->points[i].normal_z)) {
      normal_points_mat.at<float>(i, 0) =
          static_cast<float>(point_normal->points[i].normal_x);
      normal_points_mat.at<float>(i, 1) =
          static_cast<float>(point_normal->points[i].normal_y);
      normal_points_mat.at<float>(i, 2) =
          static_cast<float>(point_normal->points[i].normal_z);

      points_without_nans->points.push_back(point_cloud->points[i]);
      normal_filtered_points_mat.push_back(normal_points_mat.row(i));
    }
  }
}

cv::Mat plane_segmentation::filterCentroids(
    cv::Mat centroids, Eigen::Vector4f normals_horizontal_plane_in_cam,
    std::vector<int> &filtered_centroids_id) {
  cv::Mat filtered_centroids = cv::Mat::zeros(0, 3, CV_32F);
  filtered_centroids_id.clear();

  for (int i = 0; i < centroids.rows; i++) {
    if (centroids.at<float>(i, 0) < normals_horizontal_plane_in_cam(0) + 0.3 &&
        centroids.at<float>(i, 0) > normals_horizontal_plane_in_cam(0) - 0.3 &&
        centroids.at<float>(i, 1) < normals_horizontal_plane_in_cam(1) + 0.3 &&
        centroids.at<float>(i, 1) > normals_horizontal_plane_in_cam(1) - 0.3 &&
        centroids.at<float>(i, 2) < normals_horizontal_plane_in_cam(2) + 0.3 &&
        centroids.at<float>(i, 2) > normals_horizontal_plane_in_cam(2) - 0.3) {
      filtered_centroids.push_back(centroids.row(i));
      filtered_centroids_id.push_back(i);
    }
  }

  return filtered_centroids;
}

double plane_segmentation::computeKmeans(cv::Mat points,
                                         const int num_centroids,
                                         cv::Mat &labels, cv::Mat &centroids) {
  double compactness = 0.0;
  cv::TermCriteria criteria_kmeans(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10,
                                   0.01);
  compactness = cv::kmeans(points, num_centroids, labels, criteria_kmeans, 10,
                           cv::KMEANS_RANDOM_CENTERS, centroids);

  return compactness;
}

cv::Mat plane_segmentation::findNearestNeighbours(cv::Mat centroids) {
  cv::Mat filtered_centroids = cv::Mat::zeros(0, 3, CV_32F);
  ;

  for (int i = 0; i < centroids.rows; ++i) {
    for (int j = 1; j < centroids.rows; ++j) {
    }
  }
}

float plane_segmentation::computeDotProduct(Eigen::Vector4f vector_a,
                                            Eigen::Vector4f vector_b) {
  float product = 0;

  for (int i = 0; i < 3; i++)
    product = product + vector_a[i] * vector_b[i];

  return product;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_segmentation::preprocessPointCloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
    pcl::PointIndices::Ptr &inliers) {
  point_cloud = this->downsamplePointcloud(point_cloud, inliers);
  return point_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_segmentation::downsamplePointcloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
    pcl::PointIndices::Ptr &inliers) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud(point_cloud);
  sor.setLeafSize(0.1f, 0.1f, 0.1f);
  sor.filter(*cloud_filtered);

  boost::shared_ptr<std::vector<int>> IndicesPtr;
  IndicesPtr = sor.getIndices();

  for (int i = 0; i < IndicesPtr->size(); ++i) {
    inliers->indices.push_back(IndicesPtr->at(i));
  }

  return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_segmentation::removeOutliers(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
    pcl::PointIndices::Ptr &inliers) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud(point_cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.setIndices(inliers);
  sor.filter(*cloud_filtered);

  boost::shared_ptr<std::vector<int>> IndicesPtr;
  IndicesPtr = sor.getIndices();
  pcl::PointIndices::Ptr new_inliers(new pcl::PointIndices());
  for (int i = 0; i < IndicesPtr->size(); ++i) {
    new_inliers->indices.push_back(IndicesPtr->at(i));
  }
  inliers = new_inliers;
  // sor.setKeepOrganized(true);

  return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_segmentation::distance_filter(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud) {

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZRGB>());
  cloud_filtered->reserve(point_cloud->size());

  std::copy_if(point_cloud->begin(), point_cloud->end(),
               std::back_inserter(cloud_filtered->points),
               [&](pcl::PointXYZRGB &p) {
                 double d = p.getVector3fMap().norm();
                 return d > 0.3 && d < 3;
               });

  cloud_filtered->width = cloud_filtered->size();
  cloud_filtered->height = 1;
  cloud_filtered->is_dense = false;

  cloud_filtered->header = point_cloud->header;

  return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_segmentation::compute2DConvexHull(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_point_cloud) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(filtered_point_cloud);
  seg.segment(*inliers, *coefficients);

  //    // Project the model inliers
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(filtered_point_cloud);
  proj.setIndices(inliers);
  proj.setModelCoefficients(coefficients);
  proj.filter(*projected_cloud);

  // Create a Convex Hull representation of the projected inliers
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ConvexHull<pcl::PointXYZRGB> chull;
  chull.setInputCloud(projected_cloud);
  chull.reconstruct(*cloud_hull);
  
  return cloud_hull;
}
