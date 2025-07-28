cmake_minimum_required(VERSION 3.8)
project(vrobot_plc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGES_DEPENDENCIES
  rclcpp
  rclcpp_components
  std_msgs
)

foreach(dependency ${THIS_PACKAGES_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()


include_directories(
  include
)

add_library(vrobot_plc SHARED
  src/vrobot_plc.cpp
)
target_link_libraries(vrobot_plc -lmodbus)
ament_target_dependencies(vrobot_plc  ${THIS_PACKAGES_DEPENDENCIES})



add_executable(test_read_write src/test_read_write.cpp)
target_link_libraries(test_read_write vrobot_plc)

install(TARGETS
  vrobot_plc
  DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS
  test_read_write
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  include/${PROJECT_NAME}
  DESTINATION include
)


ament_package()
