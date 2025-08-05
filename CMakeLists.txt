cmake_minimum_required(VERSION 3.8)
project(gs_launchpad)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_subdirectory(gs_interfaces)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/gs_interfaces)

include_directories(include/launchpad)

# set(SOURCES_LAUNCHBOX
#   src/main_launchpad.cpp
#   include/launchpad/Launchbox.cpp
#   include/launchpad/ArduinoWyrzutnia.cpp
#   include/launchpad/ArduinoZawory.cpp
#   include/launchpad/PowerMonitor.cpp
#   include/launchpad/GSUART.cpp
# )

set(SOURCES_TANKING
  src/main_tanking.cpp
  include/launchpad/Tanking.cpp
  include/launchpad/PowerMonitor.cpp
  include/launchpad/GSUART.cpp
)

# add_executable(gs_launchbox ${SOURCES_LAUNCHBOX})
add_executable(gs_tanking ${SOURCES_TANKING})

# ament_target_dependencies(gs_launchbox rclcpp)
ament_target_dependencies(gs_tanking rclcpp)

# target_link_libraries(gs_launchbox i2c 
#   gs_interfaces__rosidl_typesupport_cpp
#   gs_interfaces__rosidl_typesupport_fastrtps_cpp
#   gs_interfaces__rosidl_generator_cpp)

target_link_libraries(gs_tanking i2c 
  gs_interfaces__rosidl_typesupport_cpp
  gs_interfaces__rosidl_typesupport_fastrtps_cpp
  gs_interfaces__rosidl_generator_cpp)

# install(TARGETS gs_launchbox
#   DESTINATION lib/${PROJECT_NAME})

install(TARGETS gs_tanking 
  DESTINATION lib/${PROJECT_NAME})

ament_package()