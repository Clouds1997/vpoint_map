cmake_minimum_required(VERSION 3.0.2)
project(lane_find)

catkin_package()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  cv_bridge
  std_msgs
  geometry_msgs
  nav_msgs
)

find_package(OpenCV REQUIRED)


include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lane_find
#  CATKIN_DEPENDS roscpp tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(lane_find_lib 
        src/lane_find.cpp
)

target_link_libraries(lane_find_lib
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS} 
)

add_executable(lane_find src/lane_node.cpp)
target_link_libraries(lane_find 
        lane_find_lib
)

add_executable(map_grace src/map_grace.cpp)
target_link_libraries(map_grace
        lane_find_lib
)


