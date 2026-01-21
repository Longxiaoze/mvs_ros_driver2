cmake_minimum_required(VERSION 3.0.2)
project(mvs_ros2_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs cv_bridge image_transport
)

include_directories(
  /opt/MVS/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 根据架构设置MVS库路径
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  link_directories(/opt/MVS/lib/aarch64/)
  message(STATUS "Running on aarch64")
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  link_directories(/opt/MVS/lib/64/)
  message(STATUS "Running on x86_64")
endif()

add_executable(mvs_camera_node src/grab_trigger.cc)
add_dependencies(mvs_camera_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(mvs_camera_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  MvCameraControl
)

install(TARGETS mvs_camera_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz_cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
