# CMake generated Testfile for 
# Source directory: /home/wsh/wheeltec/wheeltec_ros2/src/navigation2-humble/nav2_rotation_shim_controller/test
# Build directory: /home/wsh/wheeltec/wheeltec_ros2/build/nav2_rotation_shim_controller/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_shim_controller "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/wsh/wheeltec/wheeltec_ros2/build/nav2_rotation_shim_controller/test_results/nav2_rotation_shim_controller/test_shim_controller.gtest.xml" "--package-name" "nav2_rotation_shim_controller" "--output-file" "/home/wsh/wheeltec/wheeltec_ros2/build/nav2_rotation_shim_controller/ament_cmake_gtest/test_shim_controller.txt" "--command" "/home/wsh/wheeltec/wheeltec_ros2/build/nav2_rotation_shim_controller/test/test_shim_controller" "--gtest_output=xml:/home/wsh/wheeltec/wheeltec_ros2/build/nav2_rotation_shim_controller/test_results/nav2_rotation_shim_controller/test_shim_controller.gtest.xml")
set_tests_properties(test_shim_controller PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/wsh/wheeltec/wheeltec_ros2/build/nav2_rotation_shim_controller/test/test_shim_controller" TIMEOUT "60" WORKING_DIRECTORY "/home/wsh/wheeltec/wheeltec_ros2/build/nav2_rotation_shim_controller/test" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/wsh/wheeltec/wheeltec_ros2/src/navigation2-humble/nav2_rotation_shim_controller/test/CMakeLists.txt;4;ament_add_gtest;/home/wsh/wheeltec/wheeltec_ros2/src/navigation2-humble/nav2_rotation_shim_controller/test/CMakeLists.txt;0;")
subdirs("../gtest")
