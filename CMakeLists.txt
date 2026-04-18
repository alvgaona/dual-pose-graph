cmake_minimum_required(VERSION 3.22)
project(dual_pose_graph
  VERSION 0.1.0
  DESCRIPTION "Generic dual pose-graph SLAM library"
  LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(ENABLE_COVERAGE "Build with coverage instrumentation (--coverage)" OFF)
if(ENABLE_COVERAGE)
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(--coverage -O0 -g)
    add_link_options(--coverage)
  else()
    message(FATAL_ERROR "ENABLE_COVERAGE requires GCC or Clang")
  endif()
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(glog REQUIRED)
find_package(G2O REQUIRED)

set(${PROJECT_NAME}_SOURCES
  src/conversions.cpp
  src/graph_g2o.cpp
  src/graph_node_types.cpp
  src/optimizer_g2o.cpp
)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  $<BUILD_INTERFACE:${G2O_INCLUDE_DIR}>)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Eigen3::Eigen
    ${G2O_CORE_LIBRARY}
    ${G2O_STUFF_LIBRARY}
    ${G2O_TYPES_SLAM3D}
    ${G2O_TYPES_SLAM3D_ADDONS}
    ${G2O_TYPES_DATA}
    ${G2O_SOLVER_PCG}
    ${G2O_SOLVER_CSPARSE}
    ${G2O_SOLVER_CHOLMOD}
    ${G2O_SOLVER_EIGEN}
    fmt::fmt
    glog::glog
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  POSITION_INDEPENDENT_CODE ON
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.hpp"
)

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

configure_package_config_file(
  cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  cmake/FindG2O.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(tests)
endif()

add_subdirectory(examples)
