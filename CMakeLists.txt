cmake_minimum_required(VERSION 3.8)
project(inverse_kinematics_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(robot_kinematics SHARED src/robot_kinematics.cpp)

ament_auto_add_executable(
  inverse_kinematics_node
  src/inverse_kinematics_node.cpp)
target_link_libraries(inverse_kinematics_node robot_kinematics)

ament_auto_add_executable(
  generate_joint_targets
  tools/generate_joint_targets.cpp)
target_link_libraries(generate_joint_targets robot_kinematics)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_executable(
    test_transform_consistency
    test/test_transform_consistency.cpp)
  target_link_libraries(test_transform_consistency robot_kinematics)
  add_test(
    NAME test_transform_consistency
    COMMAND test_transform_consistency)

  add_executable(
    test_target_pose_csv
    test/test_target_pose_csv.cpp)
  target_link_libraries(test_target_pose_csv robot_kinematics)
  add_test(
    NAME test_target_pose_csv
    COMMAND test_target_pose_csv
      "${CMAKE_CURRENT_SOURCE_DIR}/config/flange_targets")
endif()

ament_auto_package(INSTALL_TO_SHARE config)