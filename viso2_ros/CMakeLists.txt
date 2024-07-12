cmake_minimum_required(VERSION 3.5)
project(viso2_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(libviso2 REQUIRED)
find_package(viso2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)


option(ARM_CROSS_COMPILATION "ARM Cross Compilation" OFF)

if(ARM_CROSS_COMPILATION)
  SET(CMAKE_SYSTEM_PROCESSOR arm)
  SET(CMAKE_CXX_FLAGS -mfpu=neon)
else(ARM_CROSS_COMPILATION)
  add_definitions(-msse3)
endif(ARM_CROSS_COMPILATION)


add_executable(stereo_odometer src/stereo_odometer.cpp)
ament_target_dependencies(stereo_odometer
  rclcpp
  nav_msgs
  sensor_msgs
  image_transport
  viso2_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  image_geometry
  cv_bridge
)
target_link_libraries(stereo_odometer
  libviso2::viso2
)

add_executable(mono_odometer
  src/mono_odometer.cpp)
ament_target_dependencies(mono_odometer
  rclcpp
  nav_msgs
  sensor_msgs
  image_transport
  viso2_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  image_geometry
  cv_bridge
)
target_link_libraries(mono_odometer
  libviso2::viso2
)

add_executable(mono_odometer_omnidirectional src/mono_odometer_omnidirectional.cpp)
ament_target_dependencies(mono_odometer_omnidirectional
  rclcpp
  nav_msgs
  sensor_msgs
  image_transport
  viso2_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  image_geometry
  cv_bridge
)
target_link_libraries(mono_odometer_omnidirectional
  libviso2::viso2
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS
  mono_odometer
  mono_odometer_omnidirectional
  stereo_odometer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
