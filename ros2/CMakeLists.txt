add_executable(gnss_solution_node gnss_solution_node.cpp)
target_link_libraries(gnss_solution_node gnss_lib gnss_lib_noopt Threads::Threads)
ament_target_dependencies(gnss_solution_node rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs)

install(TARGETS gnss_solution_node
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
