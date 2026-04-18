#include <gtest/gtest.h>

#include <dual_pose_graph/optimizer_g2o.hpp>

namespace
{
OptimizerG2OParameters makeParams()
{
  OptimizerG2OParameters params;
  params.main_graph_odometry_distance_threshold = 0.5;
  params.main_graph_odometry_orientation_threshold = 10.0;
  params.temp_graph_odometry_distance_threshold = 0.1;
  params.temp_graph_odometry_orientation_threshold = 10.0;
  params.use_dual_graph = false;  // exercise the single-graph path
  return params;
}
}  // namespace

TEST(OptimizerOdom, FeedFiveKeyframesProducesChain)
{
  OptimizerG2O optimizer;
  optimizer.set_parameters(makeParams());

  Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(6, 6) * 0.01;
  int accepted = 0;
  for (int i = 0; i < 5; ++i) {
    OdometryWithCovariance odom;
    odom.odometry = Eigen::Isometry3d::Identity();
    odom.odometry.translation().x() = static_cast<double>(i);
    odom.covariance = cov;
    if (optimizer.handle_new_odom(odom)) {
      ++accepted;
    }
  }
  EXPECT_GE(accepted, 4);
  EXPECT_GE(optimizer.main_graph->graph_->vertices().size(), 5u);
}

TEST(OptimizerOdom, FinalPoseFollowsInput)
{
  OptimizerG2O optimizer;
  optimizer.set_parameters(makeParams());

  Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(6, 6) * 0.01;
  for (int i = 0; i < 5; ++i) {
    OdometryWithCovariance odom;
    odom.odometry = Eigen::Isometry3d::Identity();
    odom.odometry.translation().x() = static_cast<double>(i);
    odom.covariance = cov;
    optimizer.handle_new_odom(odom);
  }
  auto optimized = optimizer.get_optimized_pose();
  EXPECT_NEAR(optimized.translation().x(), 4.0, 0.1);
}
