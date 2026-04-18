find_package(GTest REQUIRED)

set(TEST_SOURCES
  test_graph_g2o.cpp
  test_graph_g2o_objects.cpp
  test_graph_node_defaults.cpp
  test_conversions.cpp
  test_csv_logger.cpp
  test_fixed_objects.cpp
  test_node_absolute_detection.cpp
  test_object_detection.cpp
  test_object_detection_relative.cpp
  test_optimizer_odom.cpp
  test_optimizer_detection_generic.cpp
  test_optimizer_dual_graph.cpp
  test_optimizer_reject_invalid.cpp
  test_optimizer_transforms.cpp
  test_optimizer_advanced.cpp
  test_coverage_edges.cpp
  test_temp_to_main_merge.cpp
  test_covariance_extraction.cpp
  test_custom_detection_extension.cpp
)

foreach(test_src ${TEST_SOURCES})
  get_filename_component(test_name ${test_src} NAME_WE)
  add_executable(${test_name} ${test_src})
  target_link_libraries(${test_name}
    PRIVATE
      dual_pose_graph::dual_pose_graph
      GTest::gtest
      GTest::gtest_main
  )
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
