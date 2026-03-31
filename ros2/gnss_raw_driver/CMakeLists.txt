cmake_minimum_required(VERSION 3.15)
project(gnss_raw_driver)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GnssRawObservation.msg"
  "msg/GnssRawEpoch.msg"
  DEPENDENCIES std_msgs
)

# libgnss++ sources needed (compile directly to avoid complex find_package)
set(LIBGNSS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(LIBGNSS_SOURCES
  ${LIBGNSS_ROOT}/src/io/ubx.cpp
  ${LIBGNSS_ROOT}/src/io/sbf.cpp
  ${LIBGNSS_ROOT}/src/io/serial_port.cpp
  ${LIBGNSS_ROOT}/src/io/protocol_detector.cpp
  ${LIBGNSS_ROOT}/src/core/types.cpp
  ${LIBGNSS_ROOT}/src/core/observation.cpp
  ${LIBGNSS_ROOT}/src/core/solution.cpp
  ${LIBGNSS_ROOT}/src/core/navigation.cpp
)

# Node executable
add_executable(gnss_raw_driver_node
  src/gnss_raw_driver_node.cpp
  ${LIBGNSS_SOURCES}
)
target_include_directories(gnss_raw_driver_node PRIVATE
  ${LIBGNSS_ROOT}/include
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(gnss_raw_driver_node
  rclcpp
  sensor_msgs
  std_msgs
)

# Link generated message typesupport
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(gnss_raw_driver_node "${cpp_typesupport_target}")

# KML sources for bag processor
set(KML_SOURCES
  ${LIBGNSS_ROOT}/src/kml/types.cpp
  ${LIBGNSS_ROOT}/src/kml/utils.cpp
  ${LIBGNSS_ROOT}/src/kml/elements.cpp
  ${LIBGNSS_ROOT}/src/kml/animation.cpp
  ${LIBGNSS_ROOT}/src/kml/camera.cpp
  ${LIBGNSS_ROOT}/src/kml/tour.cpp
  ${LIBGNSS_ROOT}/src/kml/writer.cpp
)

# Bag processor node (offline replay + KML/pos output)
add_executable(gnss_bag_processor_node
  src/gnss_bag_processor_node.cpp
  ${LIBGNSS_SOURCES}
  ${KML_SOURCES}
)
target_include_directories(gnss_bag_processor_node PRIVATE
  ${LIBGNSS_ROOT}/include
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(gnss_bag_processor_node
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(gnss_bag_processor_node "${cpp_typesupport_target}")

# Install
install(TARGETS gnss_raw_driver_node gnss_bag_processor_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
