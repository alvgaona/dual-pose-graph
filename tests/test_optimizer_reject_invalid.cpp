#include <gtest/gtest.h>

#include <dual_pose_graph/optimizer_g2o.hpp>

TEST(OptimizerReject, ZeroCovarianceIsRejected)
{
  OptimizerG2O optimizer;
  OptimizerG2OParameters p;
  p.main_graph_odometry_distance_threshold = 0.5;
  p.main_graph_odometry_orientation_threshold = 10.0;
  optimizer.set_parameters(p);

  OdometryWithCovariance odom;
  odom.odometry = Eigen::Isometry3d::Identity();
  odom.covariance = Eigen::MatrixXd::Zero(6, 6);
  EXPECT_FALSE(optimizer.handle_new_odom(odom));
}

TEST(OptimizerReject, BelowDistanceThresholdRejectedAfterInit)
{
  OptimizerG2O optimizer;
  OptimizerG2OParameters p;
  p.main_graph_odometry_distance_threshold = 10.0;  // very large
  p.main_graph_odometry_orientation_threshold = 10.0;
  p.use_dual_graph = false;
  optimizer.set_parameters(p);

  Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(6, 6) * 0.01;
  OdometryWithCovariance odom;
  odom.odometry = Eigen::Isometry3d::Identity();
  odom.covariance = cov;

  // First call: accepted as the init of the main graph.
  EXPECT_TRUE(optimizer.handle_new_odom(odom));
  // Second call with no motion and huge threshold: rejected.
  EXPECT_FALSE(optimizer.handle_new_odom(odom));
}

TEST(OptimizerReject, LargeMotionAcceptedEvenWithLargeThreshold)
{
  OptimizerG2O optimizer;
  OptimizerG2OParameters p;
  p.main_graph_odometry_distance_threshold = 0.5;
  p.main_graph_odometry_orientation_threshold = 10.0;
  p.use_dual_graph = false;
  optimizer.set_parameters(p);

  Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(6, 6) * 0.01;
  OdometryWithCovariance o1;
  o1.odometry = Eigen::Isometry3d::Identity();
  o1.covariance = cov;
  EXPECT_TRUE(optimizer.handle_new_odom(o1));

  OdometryWithCovariance o2;
  o2.odometry = Eigen::Isometry3d::Identity();
  o2.odometry.translation().x() = 2.0;
  o2.covariance = cov;
  EXPECT_TRUE(optimizer.handle_new_odom(o2));
}
