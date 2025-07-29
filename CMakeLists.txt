cmake_minimum_required(VERSION 3.8)
project(vrobot_plc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(THIS_PACKAGES_DEPENDENCIES
  rclcpp
  rclcpp_components
  std_msgs
)

foreach(dependency ${THIS_PACKAGES_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()


# Service definition
file(GLOB ${PROJECT_NAME}_interface_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*)
file(GLOB ${PROJECT_NAME}_srv_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} srv/*)
file(GLOB ${PROJECT_NAME}_action_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} action/*)
rosidl_generate_interfaces(
   ${PROJECT_NAME}
   ${${PROJECT_NAME}_interface_files}
   ${${PROJECT_NAME}_srv_files}
   ${${PROJECT_NAME}_action_files}
   DEPENDENCIES ${THIS_PACKAGES_DEPENDENCIES}
)
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")



include_directories(
  include
)

add_library(${PROJECT_NAME}_lib SHARED
  src/vrobot_plc.cpp
)
target_link_libraries(${PROJECT_NAME}_lib -lmodbus)
ament_target_dependencies(${PROJECT_NAME}_lib  ${THIS_PACKAGES_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}")


add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

install(TARGETS
  ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  include/${PROJECT_NAME}
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

 ament_export_dependencies(${THIS_PACKAGE_DEPENDENCIES})
 ament_package()
