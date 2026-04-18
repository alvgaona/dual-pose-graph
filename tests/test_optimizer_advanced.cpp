#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <string>

#include <dual_pose_graph/object_detection_types.hpp>
#include <dual_pose_graph/optimizer_g2o.hpp>
#include <dual_pose_graph/utils/csv_logger.hpp>

namespace fs = std::filesystem;

namespace
{
OptimizerG2OParameters base_params()
{
  OptimizerG2OParameters p;
  p.main_graph_odometry_distance_threshold = 0.5;
  p.main_graph_odometry_orientation_threshold = 10.0;
  p.main_graph_odometry_distance_threshold_if_detections = 0.2;
  p.temp_graph_odometry_distance_threshold = 0.05;
  p.temp_graph_odometry_orientation_threshold = 10.0;
  return p;
}

OdometryWithCovariance identity_odom_at(double x)
{
  OdometryWithCovariance o;
  o.odometry = Eigen::Isometry3d::Identity();
  o.odometry.translation().x() = x;
  o.covariance = Eigen::MatrixXd::Identity(6, 6) * 0.01;
  return o;
}
}  // namespace

TEST(OptimizerAdvanced, NaNOdometryIsRejected)
{
  OptimizerG2O optimizer;
  auto p = base_params();
  p.use_dual_graph = false;
  optimizer.set_parameters(p);

  // First call establishes a baseline.
  EXPECT_TRUE(optimizer.handle_new_odom(identity_odom_at(0.0)));

  OdometryWithCovariance nan_odom;
  nan_odom.odometry = Eigen::Isometry3d::Identity();
  nan_odom.odometry.translation().x() = std::nan("");
  nan_odom.covariance = Eigen::MatrixXd::Identity(6, 6) * 0.01;

  EXPECT_FALSE(optimizer.handle_new_odom(nan_odom));
}

TEST(OptimizerAdvanced, RelativeOdometryFlagIsRejected)
{
  OptimizerG2O optimizer;
  auto p = base_params();
  p.odometry_is_relative = true;
  optimizer.set_parameters(p);

  EXPECT_FALSE(optimizer.handle_new_odom(identity_odom_at(0.0)));
}

TEST(OptimizerAdvanced, CalculateOdomCovarianceBranchAccepts)
{
  OptimizerG2O optimizer;
  auto p = base_params();
  p.calculate_odom_covariance = true;
  p.use_dual_graph = false;
  optimizer.set_parameters(p);

  // Two distinct odom samples so the (new-last) covariance delta is nonzero.
  OdometryWithCovariance o1 = identity_odom_at(0.0);
  o1.covariance = Eigen::MatrixXd::Identity(6, 6) * 0.05;
  EXPECT_TRUE(optimizer.handle_new_odom(o1));

  OdometryWithCovariance o2 = identity_odom_at(1.0);
  o2.covariance = Eigen::MatrixXd::Identity(6, 6) * 0.10;
  EXPECT_TRUE(optimizer.handle_new_odom(o2));
}

TEST(OptimizerAdvanced, GenerateOdomMapTransformUpdatesAfterOptimization)
{
  OptimizerG2O optimizer;
  auto p = base_params();
  p.use_dual_graph = false;
  p.generate_odom_map_transform = true;
  p.map_odom_transform_alpha = 0.5;  // forces filter_transform path
  optimizer.set_parameters(p);

  for (int i = 0; i < 3; ++i) {
    optimizer.handle_new_odom(identity_odom_at(1.0 * i));
  }

  // Just check the transform getters are callable and don't NaN out.
  Eigen::Isometry3d t = optimizer.get_map_odom_transform();
  EXPECT_FALSE(std::isnan(t.translation().x()));
}

TEST(OptimizerAdvanced, FilterTransformInterpolatesWithAlpha)
{
  OptimizerG2O optimizer;
  auto p = base_params();
  p.map_odom_transform_alpha = 0.25;
  optimizer.set_parameters(p);

  Eigen::Isometry3d a = Eigen::Isometry3d::Identity();
  a.translation() << 0.0, 0.0, 0.0;
  Eigen::Isometry3d b = Eigen::Isometry3d::Identity();
  b.translation() << 4.0, 0.0, 0.0;

  Eigen::Isometry3d out = optimizer.filter_transform(a, b);
  // alpha=0.25 → 25% toward b
  EXPECT_NEAR(out.translation().x(), 1.0, 1e-9);
}

TEST(OptimizerAdvanced, CsvLoggerReceivesKeyframeLogs)
{
  auto dir = fs::temp_directory_path() / "dpg_opt_csv";
  fs::create_directories(dir);

  {
    CsvLogger logger(dir.string());
    OptimizerG2O optimizer;
    optimizer.set_csv_logger(&logger);
    auto p = base_params();
    p.use_dual_graph = false;
    optimizer.set_parameters(p);

    for (int i = 0; i < 5; ++i) {
      optimizer.handle_new_odom(identity_odom_at(1.0 * i));
    }
  }  // flushes on logger destruction

  auto kf = dir / "slam_keyframes.csv";
  ASSERT_TRUE(fs::exists(kf));
  EXPECT_GT(fs::file_size(kf), 100u);
  fs::remove_all(dir);
}

TEST(OptimizerAdvanced, GenerateDetectionOdometryInfoMatchesLastOdometry)
{
  OptimizerG2O optimizer;
  auto p = base_params();
  p.use_dual_graph = false;
  optimizer.set_parameters(p);
  optimizer.handle_new_odom(identity_odom_at(2.0));

  OdometryInfo info;
  EXPECT_TRUE(optimizer.generate_detection_odometry_info(identity_odom_at(2.0), info));
  EXPECT_NEAR(info.odom_ref.translation().x(), 2.0, 1e-9);
}

TEST(OptimizerAdvanced, HandleNewObjectDetectionSingleGraphThrottlesDuplicates)
{
  OptimizerG2O optimizer;
  auto p = base_params();
  p.use_dual_graph = false;
  p.throttle_detections = true;
  optimizer.set_parameters(p);
  optimizer.handle_new_odom(identity_odom_at(0.0));

  OdometryInfo info;
  optimizer.generate_detection_odometry_info(identity_odom_at(0.0), info);

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() << 5.0, 0.0, 0.0;
  SE3ObjectDetection det1("t", "same_id", pose,
    Eigen::MatrixXd::Identity(6, 6) * 0.05, true);
  SE3ObjectDetection det2("t", "same_id", pose,
    Eigen::MatrixXd::Identity(6, 6) * 0.05, true);

  optimizer.handle_new_object_detection(&det1, info);
  auto node_count = optimizer.main_graph->get_object_nodes().size();
  optimizer.handle_new_object_detection(&det2, info);
  // Duplicate ID in the same keyframe window should be throttled: no new object.
  EXPECT_EQ(optimizer.main_graph->get_object_nodes().size(), node_count);
}

TEST(OptimizerAdvanced, BelowThresholdInDualGraphRejectedAfterInit)
{
  OptimizerG2O optimizer;
  auto p = base_params();
  p.main_graph_odometry_distance_threshold = 10.0;
  p.use_dual_graph = true;
  optimizer.set_parameters(p);

  // First call accepted (init), second below threshold rejected.
  EXPECT_TRUE(optimizer.handle_new_odom(identity_odom_at(0.0)));
  EXPECT_FALSE(optimizer.handle_new_odom(identity_odom_at(0.0)));
}
