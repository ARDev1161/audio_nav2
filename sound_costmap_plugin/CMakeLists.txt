cmake_minimum_required(VERSION 3.8)
project(sound_costmap_plugin)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(odas_classifier_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/sound_layer.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pluginlib
  nav2_costmap_2d
  odas_classifier_msgs
)

pluginlib_export_plugin_description_file(nav2_costmap_2d sound_layer.xml)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES sound_layer.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
