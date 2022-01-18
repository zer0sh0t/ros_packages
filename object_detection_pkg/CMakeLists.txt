cmake_minimum_required(VERSION 3.0.2)
project(object_detection_pkg)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/obj_det_sub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(webcam_pub src/webcam_pub.cpp)
target_link_libraries(webcam_pub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
