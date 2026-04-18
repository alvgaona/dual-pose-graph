#include <gtest/gtest.h>

#include <dual_pose_graph/graph_node_types.hpp>
#include <dual_pose_graph/object_detection_types.hpp>

TEST(NodeAbsoluteDetection, OdomNodeDefaultReturnsNullptr)
{
  OdomNode node(Eigen::Isometry3d::Identity());
  auto detection = node.build_absolute_detection(Eigen::MatrixXd::Identity(6, 6));
  EXPECT_EQ(detection, nullptr);
}

TEST(NodeAbsoluteDetection, SE3ObjectNodeEmitsMatchingDetection)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() << 2.0, 3.0, 4.0;
  SE3ObjectNode node("my_type", "inst_42", pose);

  Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(6, 6) * 0.25;
  auto detection = node.build_absolute_detection(cov);

  ASSERT_NE(detection, nullptr);
  EXPECT_EQ(detection->get_id(), "inst_42");
  EXPECT_EQ(detection->get_type(), "my_type");
  EXPECT_TRUE(detection->get_covariance_matrix().isApprox(cov, 1e-12));
  auto * se3 = dynamic_cast<SE3ObjectDetection *>(detection.get());
  EXPECT_NE(se3, nullptr);
}

TEST(NodeAbsoluteDetection, Point3DObjectNodeEmitsMatchingDetection)
{
  Eigen::Vector3d position(1.5, -2.0, 0.75);
  Point3DObjectNode node("beacon", "b_7", position);

  Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(3, 3) * 0.1;
  auto detection = node.build_absolute_detection(cov);

  ASSERT_NE(detection, nullptr);
  EXPECT_EQ(detection->get_id(), "b_7");
  EXPECT_EQ(detection->get_type(), "beacon");
  auto * p3d = dynamic_cast<Point3DObjectDetection *>(detection.get());
  EXPECT_NE(p3d, nullptr);
}

TEST(NodeAbsoluteDetection, TypeAndInstanceIdsAreCarriedThrough)
{
  SE3ObjectNode node("alpha", "beta", Eigen::Isometry3d::Identity());
  EXPECT_EQ(node.type_id(), "alpha");
  EXPECT_EQ(node.instance_id(), "beta");

  Point3DObjectNode pnode("gamma", "delta", Eigen::Vector3d::Zero());
  EXPECT_EQ(pnode.type_id(), "gamma");
  EXPECT_EQ(pnode.instance_id(), "delta");
}
