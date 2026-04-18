// Acceptance test for the polymorphic merge refactor:
// define a brand-new detection kind entirely in this translation unit,
// without touching the library. It must round-trip through OptimizerG2O.

#include <gtest/gtest.h>

#include <memory>

#include <dual_pose_graph/object_detection_types.hpp>
#include <dual_pose_graph/optimizer_g2o.hpp>

namespace
{

class MyCustomDetection;

class MyCustomNode : public GraphNodeSE3
{
public:
  explicit MyCustomNode(const std::string & id, const Eigen::Isometry3d & pose)
  : GraphNodeSE3(pose), instance_id_(id) {}

  std::string type_id() const override {return "my_custom";}
  std::string instance_id() const override {return instance_id_;}
  std::unique_ptr<ObjectDetection> build_absolute_detection(
    const Eigen::MatrixXd & covariance) const override;

private:
  std::string instance_id_;
};

class MyCustomDetection : public ObjectDetectionSE3
{
public:
  MyCustomDetection(
    const std::string & id, const Eigen::Isometry3d & pose,
    const Eigen::MatrixXd & covariance, bool is_absolute)
  : ObjectDetectionSE3(id, pose, covariance, is_absolute) {}

  std::string get_type() const override {return "my_custom";}

  GraphNode * create_node() override
  {
    auto * n = new MyCustomNode(id_, node_estimation_);
    n->set_covariance(covariance_matrix_);
    return n;
  }

  GraphEdge * create_edge(GraphNode * _node, GraphNode * _det) override
  {
    auto * a = dynamic_cast<GraphNodeSE3 *>(_node);
    auto * b = dynamic_cast<GraphNodeSE3 *>(_det);
    return new GraphEdgeSE3(a, b, edge_measurement_, information_matrix_);
  }
};

std::unique_ptr<ObjectDetection> MyCustomNode::build_absolute_detection(
  const Eigen::MatrixXd & covariance) const
{
  return std::make_unique<MyCustomDetection>(
    instance_id_, vertex_->estimate(), covariance, /*is_absolute=*/true);
}

}  // namespace

TEST(CustomDetectionExtension, UserDefinedKindRoundTripsThroughOptimizer)
{
  OptimizerG2O optimizer;
  OptimizerG2OParameters params;
  params.main_graph_odometry_distance_threshold = 0.5;
  params.main_graph_odometry_orientation_threshold = 10.0;
  params.main_graph_odometry_distance_threshold_if_detections = 0.1;
  params.temp_graph_odometry_distance_threshold = 0.05;
  params.temp_graph_odometry_orientation_threshold = 10.0;
  params.use_dual_graph = true;
  optimizer.set_parameters(params);

  Eigen::MatrixXd odom_cov = Eigen::MatrixXd::Identity(6, 6) * 0.01;
  Eigen::MatrixXd det_cov = Eigen::MatrixXd::Identity(6, 6) * 0.05;

  for (int i = 0; i < 10; ++i) {
    OdometryWithCovariance odom;
    odom.odometry = Eigen::Isometry3d::Identity();
    odom.odometry.translation().x() = 0.2 * i;
    odom.covariance = odom_cov;
    optimizer.handle_new_odom(odom);

    Eigen::Isometry3d lm = Eigen::Isometry3d::Identity();
    lm.translation() = Eigen::Vector3d(4.0, 0.5, 0.0);

    OdometryInfo det_info;
    if (!optimizer.check_adding_new_detection(odom, det_info)) {
      continue;
    }
    MyCustomDetection detection("custom0", lm, det_cov, /*is_absolute=*/true);
    optimizer.handle_new_object_detection(&detection, det_info);
  }

  const auto & main_objs = optimizer.main_graph->get_object_nodes();
  EXPECT_NE(main_objs.find("custom0"), main_objs.end());
}
