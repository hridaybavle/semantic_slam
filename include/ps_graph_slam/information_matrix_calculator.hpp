#ifndef INFORMATION_MATRIX_CALCULATOR_HPP
#define INFORMATION_MATRIX_CALCULATOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

namespace ps_graph_slam {

class InformationMatrixCalculator {
public:
  using PointT = pcl::PointXYZRGB;

  InformationMatrixCalculator();
  ~InformationMatrixCalculator();

  Eigen::MatrixXd calc_information_matrix() const;

private:
  double weight(double a, double max_x, double min_y, double max_y,
                double x) const {
    double y = (1.0 - std::exp(-a * x)) / (1.0 - std::exp(-a * max_x));
    return min_y + (max_y - min_y) * y;
  }

private:
  bool use_const_inf_matrix;
  double const_stddev_x;
  double const_stddev_q;

  double var_gain_a;
  double min_stddev_x;
  double max_stddev_x;
  double min_stddev_q;
  double max_stddev_q;
  double fitness_score_thresh;
};

} // namespace ps_graph_slam

#endif // INFORMATION_MATRIX_CALCULATOR_HPP
