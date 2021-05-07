#include <ps_graph_slam/information_matrix_calculator.hpp>

#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>

namespace ps_graph_slam {

InformationMatrixCalculator::InformationMatrixCalculator() {
    ros::param::get("~use_const_inf_matrix", use_const_inf_matrix);

    ros::param::get("~const_stddev_x", const_stddev_x);
    if(const_stddev_x == 0)
        const_stddev_x = 0.0667;

    ros::param::get("~const_stddev_q", const_stddev_q);
    if(const_stddev_q == 0)
        const_stddev_q = 0.0667;

    std::cout << "use_const_inf_matrix: " << use_const_inf_matrix << std::endl;
    std::cout << "const_stddev_x: " << const_stddev_x << std::endl;
    std::cout << "const_stddev_q: " << const_stddev_q << std::endl;
}

InformationMatrixCalculator::~InformationMatrixCalculator() {

}

Eigen::MatrixXd InformationMatrixCalculator::calc_information_matrix() const {
    if(use_const_inf_matrix) {
        Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
        inf.topLeftCorner(3, 3).array() /= const_stddev_x;
        inf.bottomRightCorner(3, 3).array() /= const_stddev_q;
        //inf(2,2) = 9;
        return inf;
    }

    double fitness_score = 0.9 ;//calc_fitness_score(cloud1, cloud2, relpose);

    double min_var_x = std::pow(min_stddev_x, 2);
    double max_var_x = std::pow(max_stddev_x, 2);
    double min_var_q = std::pow(min_stddev_q, 2);
    double max_var_q = std::pow(max_stddev_q, 2);

    float w_x = weight(var_gain_a, fitness_score_thresh, min_var_x, max_var_x, fitness_score);
    float w_q = weight(var_gain_a, fitness_score_thresh, min_var_q, max_var_q, fitness_score);

    Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
    inf.topLeftCorner(3, 3).array() /= w_x;
    inf.bottomRightCorner(3, 3).array() /= w_q;
    return inf;
}

}

