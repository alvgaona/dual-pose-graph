#include <gtest/gtest.h>

#include <dual_pose_graph/optimizer_g2o.hpp>

TEST(OptimizerTransforms, MapOdomTransformIdentityInitially)
{
  OptimizerG2O optimizer;
  OptimizerG2OParameters p;
  p.main_graph_odometry_distance_threshold = 0.5;
  optimizer.set_parameters(p);

  Eigen::Isometry3d t = optimizer.get_map_odom_transform();
  EXPECT_TRUE(t.translation().isApprox(Eigen::Vector3d::Zero(), 1e-12));
  EXPECT_TRUE(t.linear().isApprox(Eigen::Matrix3d::Identity(), 1e-12));
}

TEST(OptimizerTransforms, EarthToMapTransformPassedThrough)
{
  OptimizerG2O optimizer;
  OptimizerG2OParameters p;
  p.main_graph_odometry_distance_threshold = 0.5;
  p.earth_to_map_transform = Eigen::Isometry3d::Identity();
  p.earth_to_map_transform.translation() << 10.0, 20.0, 0.0;
  optimizer.set_parameters(p);

  Eigen::Isometry3d m = optimizer.get_map_transform();
  EXPECT_TRUE(m.translation().isApprox(Eigen::Vector3d(10.0, 20.0, 0.0), 1e-9));
}
