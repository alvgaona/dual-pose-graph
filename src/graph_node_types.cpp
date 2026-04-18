#include "dual_pose_graph/graph_node_types.hpp"
#include "dual_pose_graph/object_detection_types.hpp"

std::unique_ptr<ObjectDetection> GraphNode::build_absolute_detection(
  const Eigen::MatrixXd & /*covariance*/) const
{
  return nullptr;
}

std::unique_ptr<ObjectDetection> SE3ObjectNode::build_absolute_detection(
  const Eigen::MatrixXd & covariance) const
{
  return std::make_unique<SE3ObjectDetection>(
    type_id_, instance_id_, vertex_->estimate(), covariance, /*is_absolute=*/true);
}

std::unique_ptr<ObjectDetection> Point3DObjectNode::build_absolute_detection(
  const Eigen::MatrixXd & covariance) const
{
  return std::make_unique<Point3DObjectDetection>(
    type_id_, instance_id_, vertex_->estimate(), covariance, /*is_absolute=*/true);
}
