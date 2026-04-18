#include <gtest/gtest.h>

#include <dual_pose_graph/graph_node_types.hpp>
#include <dual_pose_graph/object_detection_types.hpp>

TEST(ObjectDetection, InformationMatrixIsInverseOfCovariance)
{
  Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(6, 6) * 0.04;
  SE3ObjectDetection detection(
    "t", "i", Eigen::Isometry3d::Identity(), cov, /*is_absolute=*/true);

  Eigen::MatrixXd info = detection.get_information_matrix();
  Eigen::MatrixXd expected = cov.inverse();
  EXPECT_TRUE(info.isApprox(expected, 1e-12));
}

TEST(ObjectDetection, SE3AbsolutePrepareMeasurementsAccepted)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() << 5.0, 0.0, 0.0;
  SE3ObjectDetection detection(
    "t", "i", pose, Eigen::MatrixXd::Identity(6, 6) * 0.1, true);

  OdometryInfo info;
  info.odom_ref = Eigen::Isometry3d::Identity();
  info.map_ref = Eigen::Isometry3d::Identity();
  info.increment = Eigen::Isometry3d::Identity();
  info.covariance_matrix = Eigen::MatrixXd::Identity(6, 6) * 0.01;

  EXPECT_TRUE(detection.prepare_measurements(info));
}

TEST(ObjectDetection, Point3DAbsolutePrepareMeasurementsAccepted)
{
  Point3DObjectDetection detection(
    "t", "i", Eigen::Vector3d(1.0, 2.0, 3.0),
    Eigen::MatrixXd::Identity(3, 3) * 0.1, true);

  OdometryInfo info;
  info.odom_ref = Eigen::Isometry3d::Identity();
  info.map_ref = Eigen::Isometry3d::Identity();
  info.increment = Eigen::Isometry3d::Identity();
  info.covariance_matrix = Eigen::MatrixXd::Identity(6, 6) * 0.01;

  EXPECT_TRUE(detection.prepare_measurements(info));
}

TEST(ObjectDetection, SE3CreateEdgeWithPoint3DReferenceReturnsNullptr)
{
  SE3ObjectDetection detection(
    "t", "i", Eigen::Isometry3d::Identity(),
    Eigen::MatrixXd::Identity(6, 6) * 0.1, true);

  OdometryInfo info;
  info.odom_ref = Eigen::Isometry3d::Identity();
  info.map_ref = Eigen::Isometry3d::Identity();
  info.increment = Eigen::Isometry3d::Identity();
  info.covariance_matrix = Eigen::MatrixXd::Identity(6, 6) * 0.01;
  detection.prepare_measurements(info);

  // Passing a Point3D node where SE3 is expected must fail cleanly.
  Point3DObjectNode wrong_ref("t", "i", Eigen::Vector3d::Zero());
  SE3ObjectNode target("t", "i", Eigen::Isometry3d::Identity());
  GraphEdge * edge = detection.create_edge(&wrong_ref, &target);
  EXPECT_EQ(edge, nullptr);
}
