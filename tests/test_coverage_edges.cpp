// Targeted tests for remaining uncovered reachable branches.

#include <gtest/gtest.h>

#include <cmath>

#include <dual_pose_graph/graph_g2o.hpp>
#include <dual_pose_graph/object_detection_types.hpp>
#include <dual_pose_graph/optimizer_g2o.hpp>

namespace
{
OdometryInfo make_info(const Eigen::Vector3d & p)
{
  OdometryInfo info;
  info.odom_ref = Eigen::Isometry3d::Identity();
  info.odom_ref.translation() = p;
  info.map_ref = info.odom_ref;
  info.increment = Eigen::Isometry3d::Identity();
  info.covariance_matrix = Eigen::MatrixXd::Identity(6, 6) * 0.01;
  return info;
}

OdometryWithCovariance odom_at(double x, double cov_scale = 0.01)
{
  OdometryWithCovariance o;
  o.odometry = Eigen::Isometry3d::Identity();
  o.odometry.translation().x() = x;
  o.covariance = Eigen::MatrixXd::Identity(6, 6) * cov_scale;
  return o;
}
}  // namespace

TEST(CoverageEdges, OptimizeGraphWithNoEdgesFails)
{
  GraphG2O graph("g");
  graph.init_graph();   // adds only a fixed vertex, no edges
  EXPECT_FALSE(graph.optimize_graph());
}

TEST(CoverageEdges, ComputeNodeCovariancePoint3DViaDualGraphMerge)
{
  // Drive the optimizer in dual-graph mode with Point3D detections so the
  // merge loop calls compute_node_covariance on a VertexPointXYZ and hits
  // the Point3D branch of that function.
  OptimizerG2O optimizer;
  OptimizerG2OParameters p;
  p.main_graph_odometry_distance_threshold = 1.0;
  p.main_graph_odometry_orientation_threshold = 10.0;
  p.main_graph_odometry_distance_threshold_if_detections = 0.2;
  p.temp_graph_odometry_distance_threshold = 0.05;
  p.temp_graph_odometry_orientation_threshold = 10.0;
  p.use_dual_graph = true;
  optimizer.set_parameters(p);

  auto push_odom = [&](double x) {
      OdometryWithCovariance o = odom_at(x);
      optimizer.handle_new_odom(o);
    };

  auto push_det = [&](double x) {
      OdometryWithCovariance o = odom_at(x);
      OdometryInfo info;
      if (!optimizer.check_adding_new_detection(o, info)) {return;}
      Point3DObjectDetection det("beacon", "b0", Eigen::Vector3d(5.0, 0.0, 0.0),
        Eigen::MatrixXd::Identity(3, 3) * 0.05, true);
      optimizer.handle_new_object_detection(&det, info);
    };

  push_odom(0.0);
  push_det(0.0);
  push_det(0.05);
  push_det(0.10);
  push_odom(0.30);  // triggers the merge → compute_node_covariance on point3d

  EXPECT_NE(optimizer.main_graph->get_object_nodes().find("b0"),
    optimizer.main_graph->get_object_nodes().end());
}

TEST(CoverageEdges, AddObjectDetectionWithIncompatibleEdgeIsNoop)
{
  // A detection whose create_edge() returns nullptr must not corrupt the graph.
  class BadDetection : public ObjectDetectionSE3
  {
public:
    BadDetection()
    : ObjectDetectionSE3("x", Eigen::Isometry3d::Identity(),
        Eigen::MatrixXd::Identity(6, 6) * 0.01, true) {}
    GraphNode * create_node() override
    {
      return new SE3ObjectNode("x", "x", Eigen::Isometry3d::Identity());
    }
    GraphEdge * create_edge(GraphNode *, GraphNode *) override {return nullptr;}
  };

  GraphG2O graph("g");
  graph.init_graph();

  BadDetection d;
  d.prepare_measurements(make_info(Eigen::Vector3d::Zero()));
  graph.add_new_object_detection(&d);

  // Object node was added, but no edge.
  EXPECT_EQ(graph.get_object_nodes().size(), 1u);
}

TEST(CoverageEdges, GenerateOdomMapTransformAlphaOneTakesNewDirectly)
{
  OptimizerG2O optimizer;
  OptimizerG2OParameters p;
  p.main_graph_odometry_distance_threshold = 0.1;
  p.main_graph_odometry_orientation_threshold = 10.0;
  p.use_dual_graph = false;
  p.generate_odom_map_transform = true;
  p.map_odom_transform_alpha = 1.0;  // exercises the non-filter branch
  optimizer.set_parameters(p);

  for (int i = 0; i < 3; ++i) {
    optimizer.handle_new_odom(odom_at(1.0 * i));
  }

  auto t = optimizer.get_map_odom_transform();
  EXPECT_FALSE(std::isnan(t.translation().x()));
}

TEST(CoverageEdges, CalculateOdomCovarianceZeroDiffRejectsSecondSample)
{
  OptimizerG2O optimizer;
  OptimizerG2OParameters p;
  p.main_graph_odometry_distance_threshold = 0.1;
  p.main_graph_odometry_orientation_threshold = 10.0;
  p.use_dual_graph = false;
  p.calculate_odom_covariance = true;
  optimizer.set_parameters(p);

  // First sample accepted (initializes last_odometry_added_).
  optimizer.handle_new_odom(odom_at(0.0, 0.02));

  // Second sample with identical covariance: delta on the diagonal is zero.
  // With rotation diag still nonzero, the generated matrix is NOT zero, but
  // if we zero the rotation components too, the matrix is all-zero → rejected.
  OdometryWithCovariance o;
  o.odometry = Eigen::Isometry3d::Identity();
  o.odometry.translation().x() = 1.0;
  o.covariance = Eigen::MatrixXd::Zero(6, 6);
  EXPECT_FALSE(optimizer.handle_new_odom(o));
}

TEST(CoverageEdges, DualGraphBelowIfDetectionsThresholdRejects)
{
  OptimizerG2O optimizer;
  OptimizerG2OParameters p;
  p.main_graph_odometry_distance_threshold = 0.5;
  p.main_graph_odometry_orientation_threshold = 10.0;
  p.main_graph_odometry_distance_threshold_if_detections = 10.0;  // very large
  p.temp_graph_odometry_distance_threshold = 0.01;
  p.temp_graph_odometry_orientation_threshold = 10.0;
  p.use_dual_graph = true;
  optimizer.set_parameters(p);

  optimizer.handle_new_odom(odom_at(0.0));

  // Seed the temp graph with a detection so temp_graph_generated_ = true.
  OdometryInfo det_info;
  ASSERT_TRUE(optimizer.check_adding_new_detection(odom_at(0.0), det_info));
  Eigen::Isometry3d lm = Eigen::Isometry3d::Identity();
  lm.translation() << 5.0, 0.0, 0.0;
  SE3ObjectDetection det("lm", "a", lm, Eigen::MatrixXd::Identity(6, 6) * 0.05, true);
  optimizer.handle_new_object_detection(&det, det_info);

  // Next odom: dual-graph mode uses the "if_detections" threshold, which is
  // huge, so this odom is rejected by that branch.
  EXPECT_FALSE(optimizer.handle_new_odom(odom_at(0.1)));
}

TEST(CoverageEdges, CheckAddingNewDetectionZeroCovarianceRejects)
{
  OptimizerG2O optimizer;
  OptimizerG2OParameters p;
  p.main_graph_odometry_distance_threshold = 0.5;
  optimizer.set_parameters(p);

  OdometryWithCovariance bad;
  bad.odometry = Eigen::Isometry3d::Identity();
  bad.covariance = Eigen::MatrixXd::Zero(6, 6);

  OdometryInfo info;
  EXPECT_FALSE(optimizer.check_adding_new_detection(bad, info));
}

TEST(CoverageEdges, CheckAddingNewDetectionBelowTempThresholdRejects)
{
  OptimizerG2O optimizer;
  OptimizerG2OParameters p;
  p.main_graph_odometry_distance_threshold = 0.5;
  p.main_graph_odometry_orientation_threshold = 10.0;
  p.temp_graph_odometry_distance_threshold = 10.0;  // very large
  p.temp_graph_odometry_orientation_threshold = 10.0;
  p.use_dual_graph = true;
  optimizer.set_parameters(p);

  optimizer.handle_new_odom(odom_at(0.0));

  // First detection initializes temp graph.
  OdometryInfo det_info;
  ASSERT_TRUE(optimizer.check_adding_new_detection(odom_at(0.0), det_info));

  // Second call: temp graph exists, but increment is tiny vs huge threshold.
  OdometryInfo det_info2;
  EXPECT_FALSE(optimizer.check_adding_new_detection(odom_at(0.0), det_info2));
}

TEST(CoverageEdges, ObjectDetectionBaseDefaultGetTypeIsEmpty)
{
  class MinimalSE3 : public ObjectDetectionSE3
  {
public:
    MinimalSE3()
    : ObjectDetectionSE3("id", Eigen::Isometry3d::Identity(),
        Eigen::MatrixXd::Identity(6, 6) * 0.01, true) {}
    GraphNode * create_node() override {return nullptr;}
    GraphEdge * create_edge(GraphNode *, GraphNode *) override {return nullptr;}
  };
  MinimalSE3 d;
  EXPECT_EQ(d.get_type(), "");
}

TEST(CoverageEdges, CalculateOdomCovarianceProducesZeroOnIdenticalDiagAndZeroRotation)
{
  OptimizerG2O optimizer;
  OptimizerG2OParameters p;
  p.main_graph_odometry_distance_threshold = 0.1;
  p.main_graph_odometry_orientation_threshold = 10.0;
  p.use_dual_graph = false;
  p.calculate_odom_covariance = true;
  optimizer.set_parameters(p);

  // Translation diagonal present (so receive-side isZero check passes); rotation
  // rows zero. On the second identical sample, the generated matrix is all zero.
  Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(6, 6);
  cov(0, 0) = cov(1, 1) = cov(2, 2) = 0.02;

  OdometryWithCovariance o1;
  o1.odometry = Eigen::Isometry3d::Identity();
  o1.covariance = cov;
  EXPECT_TRUE(optimizer.handle_new_odom(o1));

  OdometryWithCovariance o2;
  o2.odometry = Eigen::Isometry3d::Identity();
  o2.odometry.translation().x() = 1.0;
  o2.covariance = cov;  // identical → computed diagonal difference is zero
  EXPECT_FALSE(optimizer.handle_new_odom(o2));
}

TEST(CoverageEdges, DualGraphMergeSkipsNodeThatReturnsNullAbsoluteDetection)
{
  // A bespoke node whose build_absolute_detection returns nullptr (the base
  // default). The merge loop must handle that case cleanly.
  class NullDetection;

  class NullNode : public GraphNodeSE3
  {
public:
    explicit NullNode(const std::string & id, const Eigen::Isometry3d & pose)
    : GraphNodeSE3(pose), id_(id) {}
    std::string type_id() const override {return "null";}
    std::string instance_id() const override {return id_;}
    // Inherits default GraphNode::build_absolute_detection returning nullptr.

private:
    std::string id_;
  };

  class NullDetection : public ObjectDetectionSE3
  {
public:
    NullDetection()
    : ObjectDetectionSE3("null_id", Eigen::Isometry3d::Identity(),
        Eigen::MatrixXd::Identity(6, 6) * 0.01, true) {}
    GraphNode * create_node() override
    {
      return new NullNode(id_, Eigen::Isometry3d::Identity());
    }
    GraphEdge * create_edge(GraphNode * a, GraphNode * b) override
    {
      auto * na = dynamic_cast<GraphNodeSE3 *>(a);
      auto * nb = dynamic_cast<GraphNodeSE3 *>(b);
      return new GraphEdgeSE3(na, nb, edge_measurement_, information_matrix_);
    }
  };

  OptimizerG2O optimizer;
  OptimizerG2OParameters p;
  p.main_graph_odometry_distance_threshold = 1.0;
  p.main_graph_odometry_orientation_threshold = 10.0;
  p.main_graph_odometry_distance_threshold_if_detections = 0.2;
  p.temp_graph_odometry_distance_threshold = 0.05;
  p.temp_graph_odometry_orientation_threshold = 10.0;
  p.use_dual_graph = true;
  optimizer.set_parameters(p);

  optimizer.handle_new_odom(odom_at(0.0));
  OdometryInfo info;
  ASSERT_TRUE(optimizer.check_adding_new_detection(odom_at(0.0), info));
  NullDetection d;
  optimizer.handle_new_object_detection(&d, info);

  // Cross the threshold → merge runs; the nullptr branch in the merge is taken.
  optimizer.handle_new_odom(odom_at(0.30));
  // The null-returning node is skipped; no "null_id" object in the main graph.
  EXPECT_EQ(optimizer.main_graph->get_object_nodes().find("null_id"),
    optimizer.main_graph->get_object_nodes().end());
}

TEST(CoverageEdges, HandleNewObjectDetectionPrepareMeasurementsFailBranch)
{
  class FailingDetection : public ObjectDetectionSE3
  {
public:
    FailingDetection()
    : ObjectDetectionSE3("x", Eigen::Isometry3d::Identity(),
        Eigen::MatrixXd::Identity(6, 6) * 0.01, true) {}
    bool prepare_measurements(const OdometryInfo &) override {return false;}
    GraphNode * create_node() override {return nullptr;}
    GraphEdge * create_edge(GraphNode *, GraphNode *) override {return nullptr;}
  };

  OptimizerG2O optimizer;
  OptimizerG2OParameters p;
  p.main_graph_odometry_distance_threshold = 0.5;
  p.use_dual_graph = false;
  optimizer.set_parameters(p);
  optimizer.handle_new_odom(odom_at(0.0));

  FailingDetection d;
  OdometryInfo info;
  optimizer.generate_detection_odometry_info(odom_at(0.0), info);
  optimizer.handle_new_object_detection(&d, info);  // should early-return

  // Nothing was added.
  EXPECT_EQ(optimizer.main_graph->get_object_nodes().size(), 0u);
}
