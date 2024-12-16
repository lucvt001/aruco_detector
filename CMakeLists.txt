cmake_minimum_required(VERSION 3.0.2)
project(fiducial_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)
find_package(YAML-CPP REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fiducial_detector)
 CATKIN_DEPENDS roscpp sensor_msgs std_msgs image_transport cv_bridge
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/*.cpp
# )

add_executable(fiducial_detector src/main.cpp src/detector.cpp src/quaternion.cpp)
target_link_libraries(fiducial_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(fiducial_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(detect_one_image scripts/detect_one_image.cpp)
target_link_libraries(detect_one_image ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(find_all_aruco_offsets scripts/find_all_aruco_offsets.cpp)
target_link_libraries(find_all_aruco_offsets ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(add_common_offset scripts/add_common_offset.cpp)
target_link_libraries(add_common_offset ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(scale_common_offset scripts/scale_common_offset.cpp)
target_link_libraries(scale_common_offset ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES})