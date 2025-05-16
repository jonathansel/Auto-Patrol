cmake_minimum_required(VERSION 3.0.2)
project(auto_patrol)

## Compile as C++17
add_compile_options(-std=c++17)

## Find catkin and required components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  rosbag
  message_generation
  image_transport
  pcl_conversions
  pcl_ros
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

## Find system dependencies
find_package(Boost REQUIRED COMPONENTS system)  # system for potential compiled needs
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

## Declare ROS messages
add_message_files(
  FILES
  CarDetection.msg
  CarDetectionArray.msg
  Line.msg
  LineArray.msg
)

## Generate messages with dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Catkin package configuration
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    sensor_msgs 
    cv_bridge 
    rosbag 
    message_runtime 
    geometry_msgs
)

## Specify include directories
include_directories(
  include  # For ipm.h in include/auto_patrol/
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Link directories for Boost
link_directories(${Boost_LIBRARY_DIRS})

## Declare executables
add_executable(parking-detect src/parking-detect.cpp)
target_link_libraries(parking-detect ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(parking-detect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(parking-offline src/parking-offline.cpp)
target_link_libraries(parking-offline ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(parking-offline ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ipm src/ipm.cpp)
target_link_libraries(ipm ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})  # No Boost needed
add_dependencies(ipm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})