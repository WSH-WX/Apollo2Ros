cmake_minimum_required(VERSION 3.5)
project(nav2_smac_planner)

set(CMAKE_BUILD_TYPE Release) #Debug, Release

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(angles REQUIRED)
find_package(ompl REQUIRED)
find_package(OpenMP REQUIRED)
find_package(absl REQUIRED)
# 查找 Protocol Buffers 包
find_package(Protobuf REQUIRED)
# 查找ADOL-C库
find_package(adolc REQUIRED)
# 查找OSQP库
# find_package(OSQP REQUIRED)
# 查找 OSQP 库
find_library(OSQP_LIB osqp HINTS /usr/local/lib)
# 查找OSQP头文件
# 查找Ipopt库
find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)
# 查找 gflags
find_package(gflags REQUIRED)

# 查找 proto_files 目录中的所有 .proto 文件
file(GLOB PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/proto_files/*.proto")

# 生成所有 .proto 文件对应的 .h 和 .cc 文件
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

# 设置 OSQP 的头文件路径和库文件路径
set(OSQP_INCLUDE_DIRS /usr/local/include)
set(OSQP_LIB /usr/local/lib/libosqp.so)

# 如果没有找到OSQP库或头文件，报错
if(NOT OSQP_LIB OR NOT OSQP_INCLUDE_DIRS)
  message(FATAL_ERROR "OSQP library or include directory not found")
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
else()
  add_compile_options(-O3 -Wextra -Wdeprecated -fPIC)
endif()

include_directories(
  include
  math
  gflags
  util
  ${OMPL_INCLUDE_DIRS}
  ${OpenMP_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${OSQP_INCLUDE_DIRS}
  ${IPOPT_INCLUDE_DIRS}
  ${ADOLC_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIRS}
)

# 遍历 math 目录及其子目录中的所有 .cc 源文件
# file(GLOB_RECURSE MATH_SOURCES
#    ${CMAKE_SOURCE_DIR}/math/*.cc
# )

find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(library_name nav2_smac_planner)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  visualization_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  nav2_costmap_2d
  nav2_core
  pluginlib
  angles
  eigen3_cmake_module
)

# Hybrid plugin
add_library(${library_name} SHARED
  src/smac_planner_hybrid.cpp
  src/a_star.cpp
  src/collision_checker.cpp
  src/smoother.cpp
  src/analytic_expansion.cpp
  src/node_hybrid.cpp
  src/node_lattice.cpp
  src/costmap_downsampler.cpp
  src/node_2d.cpp
  src/node_basic.cpp

  math/vec2d.cc
  math/discrete_points_math.cc
  math/aabox2d.cc
  math/box2d.cc
  math/discretized_path.cc
  math/line_segment2d.cc
  math/math_utils.cc
  math/polygon2d.cc
  math/linear_interpolation.cc

  math/discretized_points_smoothing/fem_pos_deviation_sqp_osqp_interface.cc
  math/discretized_points_smoothing/fem_pos_deviation_osqp_interface.cc
  math/discretized_points_smoothing/fem_pos_deviation_smoother.cc

  gflags/planning_gflags.cc

  util/string_util.cc

  ${PROTO_SRCS}  # 添加生成的 .pb.cc 文件
)

target_link_libraries(${library_name} ${OMPL_LIBRARIES} ${OpenMP_LIBRARIES} ${GFLAGS_LIBRARIES} ${ADOLC_LIBRARIES} ${IPOPT_LIBRARIES} ${OSQP_LIB} ${Protobuf_LIBRARIES} OpenMP::OpenMP_CXX absl::strings)
target_include_directories(${library_name} PUBLIC math proto_files util gflags ${Eigen3_INCLUDE_DIRS})

ament_target_dependencies(${library_name}
  ${dependencies}
)

# 2D plugin
add_library(${library_name}_2d SHARED
  src/smac_planner_2d.cpp
  src/a_star.cpp
  src/smoother.cpp
  src/collision_checker.cpp
  src/analytic_expansion.cpp
  src/node_hybrid.cpp
  src/node_lattice.cpp
  src/costmap_downsampler.cpp
  src/node_2d.cpp
  src/node_basic.cpp
  math/vec2d.cc
  math/discrete_points_math.cc
  math/aabox2d.cc
  math/box2d.cc
  math/discretized_path.cc
  math/line_segment2d.cc
  math/math_utils.cc
  math/polygon2d.cc
  math/linear_interpolation.cc

  math/discretized_points_smoothing/fem_pos_deviation_osqp_interface.cc
  math/discretized_points_smoothing/fem_pos_deviation_smoother.cc

  gflags/planning_gflags.cc

  util/string_util.cc
  ${PROTO_SRCS}  # 添加生成的 .pb.cc 文件
)

target_link_libraries(${library_name}_2d ${OMPL_LIBRARIES} ${ADOLC_LIBRARIES} ${GFLAGS_LIBRARIES} ${IPOPT_LIBRARIES} ${OSQP_LIB} ${Protobuf_LIBRARIES} absl::strings)
target_include_directories(${library_name}_2d PUBLIC math proto_files util gflags ${Eigen3_INCLUDE_DIRS})

ament_target_dependencies(${library_name}_2d
  ${dependencies}
)

# Lattice plugin
add_library(${library_name}_lattice SHARED
  src/smac_planner_lattice.cpp
  src/a_star.cpp
  src/smoother.cpp
  src/collision_checker.cpp
  src/analytic_expansion.cpp
  src/node_hybrid.cpp
  src/node_lattice.cpp
  src/costmap_downsampler.cpp
  src/node_2d.cpp
  src/node_basic.cpp
  math/vec2d.cc
  math/discrete_points_math.cc
  math/aabox2d.cc
  math/box2d.cc
  math/discretized_path.cc
  math/line_segment2d.cc
  math/math_utils.cc
  math/polygon2d.cc
  math/linear_interpolation.cc

  math/discretized_points_smoothing/fem_pos_deviation_osqp_interface.cc
  math/discretized_points_smoothing/fem_pos_deviation_smoother.cc

  gflags/planning_gflags.cc

  util/string_util.cc

  ${PROTO_SRCS}  # 添加生成的 .pb.cc 文件
)

target_link_libraries(${library_name}_lattice ${OMPL_LIBRARIES} ${GFLAGS_LIBRARIES} ${ADOLC_LIBRARIES} ${IPOPT_LIBRARIES} ${OSQP_LIB} ${Protobuf_LIBRARIES} absl::strings)
target_include_directories(${library_name}_lattice PUBLIC math proto_files util gflags ${Eigen3_INCLUDE_DIRS})

ament_target_dependencies(${library_name}_lattice
  ${dependencies}
)

pluginlib_export_plugin_description_file(nav2_core smac_plugin_hybrid.xml)
pluginlib_export_plugin_description_file(nav2_core smac_plugin_2d.xml)
pluginlib_export_plugin_description_file(nav2_core smac_plugin_lattice.xml)

install(TARGETS ${library_name} ${library_name}_2d ${library_name}_lattice
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ math/ proto_files/ util/ gflags/
  DESTINATION include/
)

install(DIRECTORY lattice_primitives/sample_primitives DESTINATION share/${PROJECT_NAME})


ament_export_include_directories(include ${OMPL_INCLUDE_DIRS})
ament_export_libraries(${library_name} ${library_name}_2d ${library_name}_lattice)
ament_export_dependencies(${dependencies})
ament_package()
