#include <gtest/gtest.h>

#include <dual_pose_graph/graph_g2o.hpp>
#include <dual_pose_graph/object_detection_types.hpp>

namespace
{
OdometryInfo make_info(const Eigen::Vector3d & map_offset)
{
  OdometryInfo info;
  info.odom_ref = Eigen::Isometry3d::Identity();
  info.odom_ref.translation() = map_offset;
  info.map_ref = info.odom_ref;
  info.increment = Eigen::Isometry3d::Identity();
  info.covariance_matrix = Eigen::MatrixXd::Identity(6, 6) * 0.01;
  return info;
}
}  // namespace

TEST(ObjectDetectionRelative, SE3RelativePrepareMeasurementsAccepted)
{
  Eigen::Isometry3d relative = Eigen::Isometry3d::Identity();
  relative.translation() << 2.0, 0.0, 0.0;
  SE3ObjectDetection detection(
    "t", "i", relative, Eigen::MatrixXd::Identity(6, 6) * 0.1,
    /*is_absolute=*/false);

  EXPECT_TRUE(detection.prepare_measurements(make_info(Eigen::Vector3d(3.0, 0.0, 0.0))));
}

TEST(ObjectDetectionRelative, Point3DRelativePrepareMeasurementsAccepted)
{
  Point3DObjectDetection detection(
    "t", "i", Eigen::Vector3d(1.0, 0.0, 0.0),
    Eigen::MatrixXd::Identity(3, 3) * 0.1,
    /*is_absolute=*/false);

  EXPECT_TRUE(detection.prepare_measurements(make_info(Eigen::Vector3d(4.0, 0.0, 0.0))));
}

TEST(ObjectDetectionRelative, Point3DCreateEdgeWithWrongReferenceReturnsNullptr)
{
  Point3DObjectDetection detection(
    "t", "i", Eigen::Vector3d::Zero(),
    Eigen::MatrixXd::Identity(3, 3) * 0.1, true);
  detection.prepare_measurements(make_info(Eigen::Vector3d::Zero()));

  // A Point3D node for the reference is incompatible (reference must be SE3).
  Point3DObjectNode wrong_ref("t", "i", Eigen::Vector3d::Zero());
  Point3DObjectNode target("t", "i", Eigen::Vector3d::Zero());
  GraphEdge * edge = detection.create_edge(&wrong_ref, &target);
  EXPECT_EQ(edge, nullptr);
}

TEST(ObjectDetectionRelative, SE3RelativeDetectionMergesIntoGraph)
{
  GraphG2O graph("g");
  graph.init_graph();

  Eigen::Isometry3d relative = Eigen::Isometry3d::Identity();
  relative.translation() << 2.0, 0.0, 0.0;
  SE3ObjectDetection detection(
    "t", "i", relative, Eigen::MatrixXd::Identity(6, 6) * 0.1,
    /*is_absolute=*/false);
  detection.prepare_measurements(make_info(Eigen::Vector3d(3.0, 0.0, 0.0)));

  graph.add_new_object_detection(&detection);
  EXPECT_EQ(graph.get_object_nodes().size(), 1u);
}
