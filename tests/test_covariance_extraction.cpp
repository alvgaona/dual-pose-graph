#include <gtest/gtest.h>

#include <dual_pose_graph/graph_g2o.hpp>

TEST(CovarianceExtraction, NonFixedSE3NodeProducesBlock)
{
  GraphG2O graph("test");
  graph.init_graph();

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation().x() = 1.0;
  Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(6, 6) * 0.01;
  graph.add_new_keyframe(pose, pose, cov);
  graph.add_new_keyframe(pose, Eigen::Isometry3d::Identity(), cov);

  ASSERT_TRUE(graph.optimize_graph());

  auto nodes = graph.get_nodes();
  ASSERT_GE(nodes.size(), 2u);
  Eigen::MatrixXd m = graph.compute_node_covariance(nodes.back());
  EXPECT_GT(m.size(), 0);
  if (m.size() > 0) {
    EXPECT_EQ(m.rows(), 6);
    EXPECT_EQ(m.cols(), 6);
  }
}
