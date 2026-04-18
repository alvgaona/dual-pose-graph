# dual_pose_graph

[![CI](https://github.com/alvgaona/dual-pose-graph/actions/workflows/ci.yaml/badge.svg)](https://github.com/alvgaona/dual-pose-graph/actions/workflows/ci.yaml)
[![License: BSD-3-Clause](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](LICENSE)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://isocpp.org/std/the-standard)
[![pixi](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/prefix-dev/pixi/main/assets/badge/v0.json)](https://pixi.sh)

Generic dual pose-graph SLAM library. Extracted from the AeroStack2
`as2_semantic_slam` package and stripped of every ROS, drone-racing, and
visualization dependency.

The library fuses odometry with semantic object detections by maintaining two
pose graphs: a long-term **main** graph for state estimation and a short-lived
**temp** graph that validates new detections before merging. Any detection kind
works: subclass `ObjectDetection` + `GraphNode`, implement one virtual, and the
optimizer integrates it without further changes.

## Build

```sh
pixi install -e test
pixi run -e test build
pixi run -e test test
pixi run -e test example
```

## Coverage

```sh
pixi run -e coverage coverage
```

Builds with `--coverage`, runs the full test suite, and emits an HTML report at
`build-coverage/coverage.html` plus a Cobertura `coverage.xml` for CI.

## Packaging as a conda package

```sh
pixi build
```

Uses pixi's preview `pixi-build-cmake` backend to wrap the library as a conda
package. The resulting `.conda` artifact installs headers, the shared library,
and CMake config files, so a downstream project can depend on it and use
`find_package(dual_pose_graph CONFIG REQUIRED)`.

The `libg2o` binary is pulled from the `robostack-humble` conda channel. The
package name is ROS-prefixed for packaging reasons only — the library itself is
plain C++ with no ROS code.

## Using it

Minimum viable usage — feed odometry + generic SE3 landmark detections:

```cpp
#include <dual_pose_graph/optimizer_g2o.hpp>
#include <dual_pose_graph/object_detection_types.hpp>

OptimizerG2O optimizer;
OptimizerG2OParameters params;
params.main_graph_odometry_distance_threshold = 0.5;
params.use_dual_graph = true;
optimizer.set_parameters(params);

OdometryWithCovariance odom;
odom.odometry = /* Eigen::Isometry3d */;
odom.covariance = /* Eigen::MatrixXd 6x6 */;
optimizer.handle_new_odom(odom);

SE3ObjectDetection det("my_kind", "instance_7", pose, cov_6x6, /*is_absolute=*/true);
OdometryInfo info;
if (optimizer.check_adding_new_detection(odom, info)) {
  optimizer.handle_new_object_detection(&det, info);
}

Eigen::Isometry3d corrected = optimizer.get_optimized_pose();
```

See `examples/minimal_example.cpp` for a runnable version.

## Extending with a custom detection kind

The optimizer's temp→main merge is polymorphic: each concrete `GraphNode`
rebuilds its own absolute-frame `ObjectDetection` via `buildAbsoluteDetection`.
Ship your own kind without touching the library:

```cpp
class MyDetection : public ObjectDetectionSE3 {
  // override create_node() to return new MyNode{...}
  // override create_edge() to wire into the graph
};

class MyNode : public GraphNodeSE3 {
  // override build_absolute_detection() to return std::make_unique<MyDetection>(...)
};
```

Then feed `MyDetection` through `handle_new_object_detection` exactly like the
generic types. The optimizer will merge it the same way.

See `tests/test_custom_detection_extension.cpp` for a worked example.

## API surface

- **`OptimizerG2O`** — dual-graph orchestrator. Entry points: `set_parameters`,
  `handle_new_odom`, `check_adding_new_detection`, `handle_new_object_detection`,
  `get_optimized_pose`, `get_optimized_map_pose`, `get_map_odom_transform`.
- **`GraphG2O`** — thin wrapper over `g2o::SparseOptimizer`.
- **`ObjectDetection`** — base for detections. Concrete helpers:
  `SE3ObjectDetection`, `Point3DObjectDetection`. Key virtuals: `create_node`,
  `create_edge`, `prepare_measurements`.
- **`GraphNode`** — base for graph vertices. Concrete helpers: `OdomNode`,
  `SE3ObjectNode`, `Point3DObjectNode`. Key virtual: `build_absolute_detection`.
- **`GraphEdge`** — base for graph edges. Concrete helpers: `OdomEdge`,
  `GraphEdgeSE3`, `GraphEdgeSE3Point3D`.

## What this library intentionally does NOT ship

- ROS integration (subscribers, TF, message conversions) — build it on top.
- Visualization (`visualization_msgs::Marker` or equivalent) — use public
  getters (`get_pose()`, `get_position()`, `type_id()`) to drive any visualizer.
- Drone-specific landmark types (ArUco markers, racing gates) — subclass the
  generic types in your downstream package.

## License

BSD 3-Clause. See `LICENSE`.
