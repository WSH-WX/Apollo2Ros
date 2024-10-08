cmake_minimum_required(VERSION 3.5)
project(wheeltec_robot_kcf)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED )
find_package(cv_bridge REQUIRED )
find_package(image_transport REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
include_directories(include/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)ImageConverter::ImageConverter()':
  ament_lint_auto_find_test_dependencies()
endif()



add_executable( run_tracker_node  src/run_tracker.cpp src/kcftracker.cpp src/fhog.cpp src/PID.cpp)

ament_target_dependencies(run_tracker_node  OpenCV rclcpp sensor_msgs geometry_msgs std_msgs cv_bridge image_transport)

install(
        TARGETS
        run_tracker_node  
        DESTINATION lib/${PROJECT_NAME}
        )
        
install(
	DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}
	)      
        
ament_package()
