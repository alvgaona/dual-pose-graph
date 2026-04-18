#include <gtest/gtest.h>

#include <dual_pose_graph/graph_g2o.hpp>
#include <dual_pose_graph/object_detection_types.hpp>

namespace
{
OdometryInfo identity_info()
{
  OdometryInfo info;
  info.odom_ref = Eigen::Isometry3d::Identity();
  info.map_ref = Eigen::Isometry3d::Identity();
  info.increment = Eigen::Isometry3d::Identity();
  info.covariance_matrix = Eigen::MatrixXd::Identity(6, 6) * 0.01;
  return info;
}
}  // namespace

TEST(GraphG2OObjects, SameDetectionIdReusesExistingNode)
{
  GraphG2O graph("g");
  graph.init_graph();

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() << 2.0, 0.0, 0.0;
  Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(6, 6) * 0.1;

  SE3ObjectDetection d1("t", "shared", pose, cov, true);
  d1.prepare_measurements(identity_info());
  graph.add_new_object_detection(&d1);

  SE3ObjectDetection d2("t", "shared", pose, cov, true);
  d2.prepare_measurements(identity_info());
  graph.add_new_object_detection(&d2);

  EXPECT_EQ(graph.get_object_nodes().size(), 1u);
  // Two edges were created (one per detection call), but only one object node.
  EXPECT_GE(graph.get_edges().size(), 2u);
}

TEST(GraphG2OObjects, DistinctIdsCreateDistinctNodes)
{
  GraphG2O graph("g");
  graph.init_graph();

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(6, 6) * 0.1;

  for (int i = 0; i < 3; ++i) {
    SE3ObjectDetection d("t", "id_" + std::to_string(i), pose, cov, true);
    d.prepare_measurements(identity_info());
    graph.add_new_object_detection(&d);
  }

  EXPECT_EQ(graph.get_object_nodes().size(), 3u);
}
