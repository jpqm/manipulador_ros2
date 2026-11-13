cmake_minimum_required(VERSION 3.10)
project(arctos_urdf_description)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_library(manipulador_rviz_panel SHARED src/manipulador_panel.cpp)
target_link_libraries(manipulador_rviz_panel
  pluginlib::pluginlib
  rclcpp::rclcpp
  rviz_common::rviz_common
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  Qt6::Widgets
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS manipulador_rviz_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY urdf meshes launch DESTINATION share/${PROJECT_NAME})

ament_package()
