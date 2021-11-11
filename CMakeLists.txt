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

add_subdirectory(converts)

add_executable(mean_distances_node src/mean_distances.cpp)
target_include_directories(mean_distances_node PUBLIC converts/include)
target_link_directories(mean_distances_node PRIVATE converts/)
target_link_libraries(mean_distances_node ${catkin_LIBRARIES} ${OpenCV_LIBS} mat_pointcloud2_conerts)
add_dependencies(mean_distances_node ${catkin_EXPORTED_TARGETS})

add_executable(print_angles_node src/print_angles.cpp)
target_link_libraries(print_angles_node ${catkin_LIBRARIES})

add_executable(front_ranges_node src/front_ranges.cpp)
target_link_libraries(front_ranges_node ${catkin_LIBRARIES})

install(
    TARGETS
    mean_distances_node
    print_angles_node
    front_ranges_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})