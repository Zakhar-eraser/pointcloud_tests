find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    sensor_msgs
  INCLUDE_DIRS include
)

add_library(mat_pointcloud2_conerts src/PointCloud2MatConverts.cpp)