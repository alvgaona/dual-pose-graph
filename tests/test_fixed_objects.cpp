#include <gtest/gtest.h>

#include <dual_pose_graph/graph_g2o.hpp>
#include <dual_pose_graph/optimizer_g2o.hpp>

TEST(FixedObjects, SE3FixedObjectAppearsInObjectNodes)
{
  OptimizerG2O optimizer;
  OptimizerG2OParameters params;
  params.main_graph_odometry_distance_threshold = 0.5;

  FixedObject fo;
  fo.id = "landmark_a";
  fo.kind = FixedObject::Kind::SE3;
  fo.isometry = Eigen::Isometry3d::Identity();
  fo.isometry.translation() << 1.0, 2.0, 3.0;
  params.fixed_objects.push_back(fo);

  optimizer.set_parameters(params);

  const auto & objs = optimizer.main_graph->get_object_nodes();
  ASSERT_NE(objs.find("landmark_a"), objs.end());
  auto * node = dynamic_cast<SE3ObjectNode *>(objs.at("landmark_a"));
  ASSERT_NE(node, nullptr);
  EXPECT_TRUE(node->get_pose().translation().isApprox(fo.isometry.translation(), 1e-9));
}

TEST(FixedObjects, Point3DFixedObjectAppearsInObjectNodes)
{
  OptimizerG2O optimizer;
  OptimizerG2OParameters params;
  params.main_graph_odometry_distance_threshold = 0.5;

  FixedObject fo;
  fo.id = "beacon_1";
  fo.kind = FixedObject::Kind::Point3D;
  fo.isometry = Eigen::Isometry3d::Identity();
  fo.isometry.translation() << 7.0, -2.0, 0.5;
  params.fixed_objects.push_back(fo);

  optimizer.set_parameters(params);

  const auto & objs = optimizer.main_graph->get_object_nodes();
  ASSERT_NE(objs.find("beacon_1"), objs.end());
  auto * node = dynamic_cast<Point3DObjectNode *>(objs.at("beacon_1"));
  ASSERT_NE(node, nullptr);
  EXPECT_TRUE(node->get_position().isApprox(fo.isometry.translation(), 1e-9));
}
