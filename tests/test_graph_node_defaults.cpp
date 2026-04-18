#include <gtest/gtest.h>

#include <dual_pose_graph/graph_node_types.hpp>

TEST(GraphNodeDefaults, OdomNodeHasEmptyTypeAndInstanceIds)
{
  OdomNode node(Eigen::Isometry3d::Identity());
  EXPECT_EQ(node.type_id(), "");
  EXPECT_EQ(node.instance_id(), "");
}

TEST(GraphNodeDefaults, OdomNodeVertexIsSE3)
{
  OdomNode node(Eigen::Isometry3d::Identity());
  EXPECT_NE(node.get_vertex_se3(), nullptr);
  EXPECT_NE(node.get_vertex(), nullptr);
}

TEST(GraphNodeDefaults, SetFixedPropagatesToG2oVertex)
{
  OdomNode node(Eigen::Isometry3d::Identity());
  EXPECT_FALSE(node.get_vertex_se3()->fixed());
  node.set_fixed();
  EXPECT_TRUE(node.get_vertex_se3()->fixed());
}

TEST(GraphNodeDefaults, SetGetCovarianceRoundTrips)
{
  OdomNode node(Eigen::Isometry3d::Identity());
  Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(6, 6) * 3.14;
  node.set_covariance(cov);
  EXPECT_TRUE(node.get_covariance().isApprox(cov, 1e-12));
}
