#include <ps_graph_slam/keyframe.hpp>

#include <boost/filesystem.hpp>

#include <g2o/types/slam3d/vertex_se3.h>
#include <pcl/io/pcd_io.h>

namespace ps_graph_slam {

KeyFrame::KeyFrame(const ros::Time &stamp, const Eigen::Isometry3d &odom,
                   const Eigen::Isometry3d &robot_pose,
                   const Eigen::MatrixXf &odom_cov, double accum_distance,
                   const sensor_msgs::PointCloud2 &cloud_msg,
                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                   std::vector<semantic_SLAM::ObjectInfo> &obj_info)
    : stamp(stamp), odom(odom), robot_pose(robot_pose), odom_cov(odom_cov),
      accum_distance(accum_distance), cloud_msg(cloud_msg), cloud(cloud),
      obj_info(obj_info), node(nullptr) {}

KeyFrame::~KeyFrame() {}

void KeyFrame::dump(const std::string &directory) {
  if (!boost::filesystem::is_directory(directory)) {
    boost::filesystem::create_directory(directory);
  }

  std::ofstream ofs(directory + "/data");
  ofs << "stamp " << stamp.sec << " " << stamp.nsec << "\n";

  ofs << "odom\n";
  ofs << odom.matrix() << "\n";

  ofs << "accum_distance " << accum_distance << "\n";

  if (node) {
    ofs << "id " << node->id() << "\n";
  }

  // pcl::io::savePCDFileBinary(directory + "/cloud.pcd", *cloud);
}

} // namespace ps_graph_slam
