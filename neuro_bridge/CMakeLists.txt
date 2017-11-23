cmake_minimum_required(VERSION 2.8.3)
project(neuro_bridge)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
    message(STATUS "No build type selected, default to ${CMAKE_BUILD_TYPE}")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(left_gripper_trajectory_pub  src/left_gripper_trajectory_pub.cpp)
target_link_libraries(left_gripper_trajectory_pub ${catkin_LIBRARIES} )
add_dependencies(left_gripper_trajectory_pub neuro_bridge_generate_message_cpp)

add_executable(left_arm_trajectory_pub  src/left_arm.cpp)
target_link_libraries(left_arm_trajectory_pub  ${catkin_LIBRARIES} )
add_dependencies(left_arm_trajectory_pub  neuro_bridge_generate_message_cpp)

add_executable(right_arm_trajectory_pub  src/right_arm_trajectory_pub.cpp)
target_link_libraries(right_arm_trajectory_pub  ${catkin_LIBRARIES} )
add_dependencies(right_arm_trajectory_pub  neuro_bridge_generate_message_cpp)

add_executable(both_arm_pos_pub  src/both_arm.cpp)
target_link_libraries(both_arm_pos_pub  ${catkin_LIBRARIES} )
add_dependencies(both_arm_pos_pub  neuro_bridge_generate_message_cpp)
