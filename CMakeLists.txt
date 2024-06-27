find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${LIBUSB_INCLUDE_DIRS}
)

add_executable(fpv_liberator_node src/fpv_liberator_node.cpp src/fpv_liberator.cpp)
target_link_libraries(fpv_liberator_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${LIBUSB_LIBRARIES}
  rt
  pthread
)