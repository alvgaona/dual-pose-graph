#include <gtest/gtest.h>

#include <dual_pose_graph/graph_g2o.hpp>

TEST(GraphG2O, InitGraphAddsFixedOrigin)
{
  GraphG2O graph("test");
  graph.init_graph();
  EXPECT_EQ(graph.graph_->vertices().size(), 1u);
  EXPECT_NE(graph.get_last_odom_node(), nullptr);
}

TEST(GraphG2O, AddNewKeyframeAddsEdgeAndNode)
{
  GraphG2O graph("test");
  graph.init_graph();

  Eigen::Isometry3d absolute = Eigen::Isometry3d::Identity();
  absolute.translation().x() = 1.0;
  Eigen::Isometry3d relative = absolute;
  Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(6, 6) * 0.01;

  graph.add_new_keyframe(absolute, relative, cov);
  EXPECT_EQ(graph.graph_->vertices().size(), 2u);
  EXPECT_EQ(graph.graph_->edges().size(), 1u);
}

TEST(GraphG2O, OptimizeEmptyGraphFails)
{
  GraphG2O graph("test");
  EXPECT_FALSE(graph.optimize_graph());
}
