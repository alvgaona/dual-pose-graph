#include <gtest/gtest.h>

#include <dual_pose_graph/object_detection_types.hpp>
#include <dual_pose_graph/optimizer_g2o.hpp>

namespace
{
OptimizerG2OParameters params()
{
  OptimizerG2OParameters p;
  p.main_graph_odometry_distance_threshold = 1.0;
  p.main_graph_odometry_orientation_threshold = 10.0;
  p.main_graph_odometry_distance_threshold_if_detections = 0.2;
  p.temp_graph_odometry_distance_threshold = 0.05;
  p.temp_graph_odometry_orientation_threshold = 10.0;
  p.use_dual_graph = true;
  return p;
}
}  // namespace

TEST(OptimizerDualGraph, TempGraphCreatedOnFirstDetection)
{
  OptimizerG2O optimizer;
  optimizer.set_parameters(params());

  Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(6, 6) * 0.01;
  OdometryWithCovariance odom;
  odom.odometry = Eigen::Isometry3d::Identity();
  odom.covariance = cov;
  optimizer.handle_new_odom(odom);

  OdometryInfo det_info;
  ASSERT_TRUE(optimizer.check_adding_new_detection(odom, det_info));

  Eigen::Isometry3d lm_pose = Eigen::Isometry3d::Identity();
  lm_pose.translation() << 3.0, 0.0, 0.0;
  SE3ObjectDetection det(
    "lm", "a", lm_pose, Eigen::MatrixXd::Identity(6, 6) * 0.05, true);
  optimizer.handle_new_object_detection(&det, det_info);

  // The temp graph should have at least the initial keyframe + the landmark.
  EXPECT_GE(optimizer.temp_graph->graph_->vertices().size(), 2u);
  // No detection has been merged into the main graph yet.
  const auto & main_objs = optimizer.main_graph->get_object_nodes();
  EXPECT_EQ(main_objs.find("a"), main_objs.end());
}

TEST(OptimizerDualGraph, TempGraphDrainedAndResetOnKeyframe)
{
  OptimizerG2O optimizer;
  optimizer.set_parameters(params());

  Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(6, 6) * 0.01;

  auto push_odom = [&](double x) {
      OdometryWithCovariance o;
      o.odometry = Eigen::Isometry3d::Identity();
      o.odometry.translation().x() = x;
      o.covariance = cov;
      optimizer.handle_new_odom(o);
    };

  auto push_det = [&](double x) {
      OdometryWithCovariance o;
      o.odometry = Eigen::Isometry3d::Identity();
      o.odometry.translation().x() = x;
      o.covariance = cov;
      OdometryInfo info;
      if (!optimizer.check_adding_new_detection(o, info)) {return;}
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.translation() << 5.0, 0.0, 0.0;
      SE3ObjectDetection det("lm", "a", pose,
        Eigen::MatrixXd::Identity(6, 6) * 0.05, true);
      optimizer.handle_new_object_detection(&det, info);
    };

  push_odom(0.0);
  push_det(0.0);
  push_det(0.05);
  push_det(0.10);
  push_odom(0.30);  // crosses the if_detections threshold, triggers merge

  // After merge, the temp graph should have been reset (empty of user objects).
  EXPECT_TRUE(optimizer.temp_graph->get_object_nodes().empty());
  // And the landmark should appear in the main graph.
  EXPECT_NE(optimizer.main_graph->get_object_nodes().find("a"),
    optimizer.main_graph->get_object_nodes().end());
}
