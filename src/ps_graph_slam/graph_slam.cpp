#include <ps_graph_slam/graph_slam.hpp>

#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/marginal_covariance_cholesky.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/stuff/macros.h>
#include <g2o/types/slam2d/types_slam2d.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d/parameter_se3_offset.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

G2O_USE_OPTIMIZATION_LIBRARY(csparse)

namespace g2o {
// G2O_REGISTER_TYPE(EDGE_SE3_PLANE, EdgeSE3Plane)
//        G2O_REGISTER_TYPE(EDGE_SE3_PRIORXY, EdgeSE3PriorXY)
//        G2O_REGISTER_TYPE(EDGE_SE3_PRIORXYZ, EdgeSE3PriorXYZ)
}

namespace ps_graph_slam {

/**
 * @brief constructor
 */
GraphSLAM::GraphSLAM(bool verbose) {

  verbose_ = verbose;
  graph.reset(new g2o::SparseOptimizer());

  std::cout << "construct solver... " << std::endl;

  //    typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> >
  //    BlockSolver_3_3; BlockSolver_3_3::LinearSolverType* linearSolver
  //            = new
  //            g2o::LinearSolverCSparse<BlockSolver_3_3::PoseMatrixType>();
  //    BlockSolver_3_3* blockSolver
  //            = new BlockSolver_3_3(linearSolver);
  //    g2o::OptimizationAlgorithmGaussNewton* solver
  //            = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
  //    graph->setAlgorithm(solver);

  // Create the block solver - the dimensions are specified because
  // 3D observations marginalise to a 6D estimate
  //    std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver (new
  //    g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>());
  //    std::unique_ptr<g2o::BlockSolverX> solver_ptr (new
  //    g2o::BlockSolverX(std::move(linearSolver)));
  //    g2o::OptimizationAlgorithmLevenberg * solver = new
  //    g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
  //    graph->setAlgorithm(solver);

  std::string g2o_solver_name = "lm_var";
  g2o::OptimizationAlgorithmFactory *solver_factory =
      g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property;
  g2o::OptimizationAlgorithm *solver =
      solver_factory->construct(g2o_solver_name, solver_property);
  graph->setAlgorithm(solver);

  g2o::ParameterSE3Offset *cameraOffset(new g2o::ParameterSE3Offset);
  Eigen::Quaterniond quat;
  quat.setIdentity();
  Eigen::Vector3d trans;
  trans.setZero();
  g2o::SE3Quat offset_quat(quat, trans);
  cameraOffset->setId(0);
  cameraOffset->setOffset(offset_quat);
  graph->addParameter(cameraOffset);

  if (!graph->solver()) {
    std::cerr << std::endl;
    std::cerr << "error : failed to allocate solver!!" << std::endl;
    // solver_factory->listSolvers(std::cerr);
    std::cerr << "-------------" << std::endl;
    std::cin.ignore(1);
    return;
  }
  std::cout << "done" << std::endl;

  // floor_plane_node = add_plane_node(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0));
  // floor_plane_node->setFixed(true);
}

/**
 * @brief destructor
 */
GraphSLAM::~GraphSLAM() { graph.reset(); }

g2o::VertexSE3 *GraphSLAM::add_se3_node(const Eigen::Isometry3d &pose) {
  g2o::VertexSE3 *vertex(new g2o::VertexSE3());
  vertex->setId(graph->vertices().size());
  vertex->setEstimate(pose);
  // fixing the first node pose
  if (graph->vertices().size() == 0) {
    vertex->setFixed(true);
  }
  graph->addVertex(vertex);

  return vertex;
}

// g2o::VertexPlane* GraphSLAM::add_plane_node(const Eigen::Vector4d&
// plane_coeffs) {
//    g2o::VertexPlane* vertex(new g2o::VertexPlane());
//    vertex->setId(graph->vertices().size());
//    vertex->setEstimate(plane_coeffs);
//    graph->addVertex(vertex);

//    return vertex;
//}

g2o::VertexPointXYZ *GraphSLAM::add_point_xyz_node(const Eigen::Vector3d &xyz) {
  g2o::VertexPointXYZ *vertex(new g2o::VertexPointXYZ());
  vertex->setId(graph->vertices().size());
  vertex->setEstimate(xyz);
  graph->addVertex(vertex);

  return vertex;
}

g2o::EdgeSE3 *
GraphSLAM::add_se3_edge(g2o::VertexSE3 *v1, g2o::VertexSE3 *v2,
                        const Eigen::Isometry3d &relative_pose,
                        const Eigen::MatrixXd &information_matrix) {
  g2o::EdgeSE3 *edge(new g2o::EdgeSE3());
  edge->setMeasurement(relative_pose);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3PointXYZ *GraphSLAM::add_se3_point_xyz_edge(
    g2o::VertexSE3 *v_se3, g2o::VertexPointXYZ *v_xyz,
    const Eigen::Vector3d &xyz, const Eigen::MatrixXd &information_matrix) {
  g2o::EdgeSE3PointXYZ *edge(new g2o::EdgeSE3PointXYZ());

  g2o::RobustKernelDCS *dcs_robust_kernel;

  edge->setMeasurement(xyz);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  edge->vertices()[1] = v_xyz;
  edge->setRobustKernel(dcs_robust_kernel);
  edge->setParameterId(0, 0);
  graph->addEdge(edge);

  return edge;
}

g2o::EdgePointXYZ *GraphSLAM::add_point_xyz_point_xyz_edge(
    g2o::VertexPointXYZ *v1_xyz, g2o::VertexPointXYZ *v2_xyz,
    const Eigen::Vector3d &xyz, const Eigen::MatrixXd &information_matrix) {
  g2o::EdgePointXYZ *edge(new g2o::EdgePointXYZ());
  edge->setMeasurement(xyz);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v1_xyz;
  edge->vertices()[1] = v2_xyz;
  edge->setParameterId(0, 0);
  graph->addEdge(edge);

  return edge;
}

bool GraphSLAM::optimize() {

  if (graph->edges().size() < 10) {
    return false;
  }

  if (verbose_) {
    std::cout << std::endl;
    std::cout << "--- pose graph optimization ---" << std::endl;
    std::cout << "nodes: " << graph->vertices().size()
              << "   edges: " << graph->edges().size() << std::endl;
    std::cout << "optimizing... " << std::flush;
  }

  //    graph->setComputeBatchStatistics(true);
  //    graph->computeActiveErrors();

  graph->initializeOptimization();
  graph->setVerbose(false);

  double chi2 = graph->chi2();

  auto t1 = ros::Time::now();
  int iterations = graph->optimize(1024);

  auto t2 = ros::Time::now();

  if (verbose_) {
    std::cout << "done" << std::endl;
    std::cout << "iterations: " << iterations << std::endl;
    std::cout << "chi2: (before)" << chi2 << " -> (after)" << graph->chi2()
              << std::endl;
    std::cout << "time: " << boost::format("%.3f") % (t2 - t1).toSec()
              << "[sec]" << std::endl;
  }

  return true;
}

bool GraphSLAM::computeLandmarkMarginals(
    g2o::SparseBlockMatrix<Eigen::MatrixXd> &spinv,
    std::vector<std::pair<int, int>> vert_pairs_vec) {

  if (graph->computeMarginals(spinv, vert_pairs_vec)) {
    if (verbose_)
      std::cout << "computed marginals " << std::endl;
    return true;
  } else {
    if (verbose_)
      std::cout << "not computing marginals " << std::endl;
    return false;
  }
}

void GraphSLAM::save(const std::string &filename) {
  std::ofstream ofs(filename);
  graph->save(ofs);
}

} // namespace ps_graph_slam
