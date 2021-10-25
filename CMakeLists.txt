cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_tests)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

find_package( OpenCV REQUIRED )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
  INCLUDE_DIRS include
)

add_executable(mean_distances_node src/mean_distances.cpp src/PointCloud2MatConverts.cpp)
target_link_libraries(mean_distances_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(mean_distances_node ${catkin_EXPORTED_TARGETS})

install(
    TARGETS
    mean_distances_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})