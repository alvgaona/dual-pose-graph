#include <gtest/gtest.h>

#include <dual_pose_graph/object_detection_types.hpp>
#include <dual_pose_graph/optimizer_g2o.hpp>

namespace
{
OptimizerG2OParameters makeParams(bool use_dual_graph)
{
  OptimizerG2OParameters params;
  params.main_graph_odometry_distance_threshold = 0.2;
  params.main_graph_odometry_orientation_threshold = 10.0;
  params.main_graph_odometry_distance_threshold_if_detections = 0.1;
  params.temp_graph_odometry_distance_threshold = 0.05;
  params.temp_graph_odometry_orientation_threshold = 10.0;
  params.use_dual_graph = use_dual_graph;
  return params;
}
}  // namespace

TEST(OptimizerDetection, GenericSE3DetectionMergesIntoMain)
{
  OptimizerG2O optimizer;
  optimizer.set_parameters(makeParams(/*use_dual_graph=*/false));

  Eigen::MatrixXd odom_cov = Eigen::MatrixXd::Identity(6, 6) * 0.01;
  Eigen::MatrixXd det_cov = Eigen::MatrixXd::Identity(6, 6) * 0.05;

  for (int i = 0; i < 5; ++i) {
    OdometryWithCovariance odom;
    odom.odometry = Eigen::Isometry3d::Identity();
    odom.odometry.translation().x() = 0.3 * i;
    odom.covariance = odom_cov;
    optimizer.handle_new_odom(odom);

    Eigen::Isometry3d lm = Eigen::Isometry3d::Identity();
    lm.translation() = Eigen::Vector3d(5.0, 0.0, 0.0);

    OdometryInfo det_info;
    OdometryWithCovariance det_odom = odom;
    ASSERT_TRUE(optimizer.generate_detection_odometry_info(det_odom, det_info));

    SE3ObjectDetection detection("my_landmark", "lm0", lm, det_cov, /*is_absolute=*/true);
    optimizer.handle_new_object_detection(&detection, det_info);
  }

  const auto & objs = optimizer.main_graph->get_object_nodes();
  EXPECT_NE(objs.find("lm0"), objs.end());
}

TEST(OptimizerDetection, GenericPoint3DDetectionMergesIntoMain)
{
  OptimizerG2O optimizer;
  optimizer.set_parameters(makeParams(/*use_dual_graph=*/false));

  Eigen::MatrixXd odom_cov = Eigen::MatrixXd::Identity(6, 6) * 0.01;
  Eigen::MatrixXd det_cov = Eigen::MatrixXd::Identity(3, 3) * 0.05;

  for (int i = 0; i < 5; ++i) {
    OdometryWithCovariance odom;
    odom.odometry = Eigen::Isometry3d::Identity();
    odom.odometry.translation().x() = 0.3 * i;
    odom.covariance = odom_cov;
    optimizer.handle_new_odom(odom);

    Eigen::Vector3d lm(3.0, 1.0, 0.0);

    OdometryInfo det_info;
    OdometryWithCovariance det_odom = odom;
    ASSERT_TRUE(optimizer.generate_detection_odometry_info(det_odom, det_info));

    Point3DObjectDetection detection("beacon", "b0", lm, det_cov, /*is_absolute=*/true);
    optimizer.handle_new_object_detection(&detection, det_info);
  }

  const auto & objs = optimizer.main_graph->get_object_nodes();
  EXPECT_NE(objs.find("b0"), objs.end());
}
