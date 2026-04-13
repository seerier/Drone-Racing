cmake_minimum_required(VERSION 3.8)
project(crazyradio_driver_cpp)

# set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(jirl_interfaces REQUIRED)
find_package(crazyflie_cpp REQUIRED)
find_package(crazyflie-link-cpp REQUIRED)

# Configure RO-SLAM node source files
file(GLOB CRAZYDRIVER_NODE_SRCS
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "src/crazyradio_driver/*.cpp")

# CrazyradioDriver node
add_library(crazyradio_driver SHARED ${CRAZYDRIVER_NODE_SRCS})
target_compile_definitions(crazyradio_driver PRIVATE COMPOSITION_BUILDING_DLL)
target_include_directories(crazyradio_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(crazyradio_driver
  rclcpp
  rclcpp_components
  jirl_interfaces
)

target_link_libraries(crazyradio_driver
  crazyflieLinkCpp::crazyflieLinkCpp # Link the link library
  crazyflie_cpp::crazyflie_cpp       # Link the higher-level library (adjust target name if needed)
)

rclcpp_components_register_nodes(crazyradio_driver "CrazyradioDriver::CrazyradioDriverNode")

# CrazyradioDriver standalone application
add_executable(crazyradio_driver_app
  src/crazyradio_driver_app.cpp)
target_include_directories(crazyradio_driver_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(crazyradio_driver_app crazyradio_driver)
ament_target_dependencies(crazyradio_driver_app
  rclcpp)

# Install CrazyradioDriver component
install(TARGETS crazyradio_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install CrazyradioDriver app
install(TARGETS crazyradio_driver_app
  DESTINATION lib/${PROJECT_NAME})

# Install Config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Install Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Make sure that the library path is exported even if the library itself is not
ament_environment_hooks("${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")

ament_package()
