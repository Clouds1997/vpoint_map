cmake_minimum_required(VERSION 3.0.2)
project(point_cloud)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES point_cloud
#  CATKIN_DEPENDS roscpp tf
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_library(point_cloud_lib

# )

# target_link_libraries(point_cloud_lib
#     ${catkin_LIBRARIES}
# 		)

add_executable(point_cloud src/point_node.cpp )
target_link_libraries(point_cloud 
  ${catkin_LIBRARIES}
  )
