cmake_minimum_required(VERSION 3.0.2)
project(test_dji)

find_package(catkin REQUIRED COMPONENTS
  dji_osdk_ros
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_dji
#  CATKIN_DEPENDS dji_osdk_ros roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/mbzirc_demo_search.cpp)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    djiosdk-core.a
    dji-linker.a
)
