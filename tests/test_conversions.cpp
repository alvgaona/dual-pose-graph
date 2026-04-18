#include <gtest/gtest.h>

#include <dual_pose_graph/utils/conversions.hpp>

TEST(Conversions, PoseSE3FromVecQuatPreservesValues)
{
  Eigen::Vector3d position(1.0, 2.0, 3.0);
  Eigen::Quaterniond orientation(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
  PoseSE3 pose = convert_to_pose_se3(position, orientation);

  EXPECT_DOUBLE_EQ(pose.position.x(), 1.0);
  EXPECT_DOUBLE_EQ(pose.position.y(), 2.0);
  EXPECT_DOUBLE_EQ(pose.position.z(), 3.0);
  EXPECT_DOUBLE_EQ(pose.orientation.w(), orientation.w());
  EXPECT_DOUBLE_EQ(pose.orientation.z(), orientation.z());
}

TEST(Conversions, PoseSE3FromIsometryRoundTrips)
{
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.translation() << 4.0, -1.0, 2.5;
  iso.linear() = Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitY()).toRotationMatrix();

  PoseSE3 pose = convert_to_pose_se3(iso);
  Eigen::Isometry3d iso2 = convert_to_isometry_3d(pose.position, pose.orientation);

  EXPECT_TRUE(iso.translation().isApprox(iso2.translation(), 1e-12));
  EXPECT_TRUE(iso.linear().isApprox(iso2.linear(), 1e-12));
}

TEST(Conversions, IsometryFromVecQuatMatchesManualCompose)
{
  Eigen::Vector3d position(0.0, 5.0, -3.0);
  Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 3.0, Eigen::Vector3d::UnitX()));
  Eigen::Isometry3d iso = convert_to_isometry_3d(position, q);

  EXPECT_TRUE(iso.translation().isApprox(position, 1e-12));
  EXPECT_TRUE(iso.linear().isApprox(q.toRotationMatrix(), 1e-12));
}
