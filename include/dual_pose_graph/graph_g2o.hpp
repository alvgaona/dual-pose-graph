#ifndef DUAL_POSE_GRAPH__GRAPH_G2O_HPP_
#define DUAL_POSE_GRAPH__GRAPH_G2O_HPP_

#include <g2o/core/optimizable_graph.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "dual_pose_graph/graph_edge_types.hpp"
#include "dual_pose_graph/graph_node_types.hpp"
#include "dual_pose_graph/object_detection_types.hpp"
#include "dual_pose_graph/utils/general_utils.hpp"

class GraphG2O
{
public:
  explicit GraphG2O(const std::string & _name);
  ~GraphG2O() {}

  const std::string & get_name() const {return name_;}
  std::vector<GraphNode *> get_nodes() const {return graph_nodes_;}
  std::vector<GraphEdge *> get_edges() const {return graph_edges_;}
  std::unordered_map<std::string, GraphNode *> get_object_nodes() const {return obj_id2node_;}
  OdomNode * get_last_odom_node() const {return last_odom_node_;}
  OdomNode * get_map_node() const {return map_node_;}
  void set_map_node(OdomNode * _map_node);

  void add_node(GraphNode & _node);
  void add_edge(GraphEdge & _edge);
  void add_new_keyframe(
    const Eigen::Isometry3d & _absolute_pose,
    const Eigen::Isometry3d & _relative_pose,
    const Eigen::MatrixXd & _relative_covariance);
  void add_new_object_detection(ObjectDetection * _object);
  Eigen::MatrixXd compute_node_covariance(GraphNode * _node);

  bool optimize_graph();
  void set_fixed_objects(const std::vector<FixedObject> & _fixed_objects);
  void init_graph(const Eigen::Isometry3d & _initial_pose = Eigen::Isometry3d::Identity());

  std::shared_ptr<g2o::SparseOptimizer> graph_;
  std::unordered_map<std::string, GraphNode *> obj_id2node_;

private:
  int n_vertices_ = 0;
  int n_edges_ = 0;
  std::string name_;
  OdomNode * last_odom_node_ = nullptr;
  OdomNode * map_node_ = nullptr;
  std::vector<GraphNode *> graph_nodes_;
  std::vector<GraphEdge *> graph_edges_;
};

#endif  // DUAL_POSE_GRAPH__GRAPH_G2O_HPP_
