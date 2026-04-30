cmake_minimum_required(VERSION 3.5)
project(crazyflie_cpp VERSION 1.0)

# Enable C++14 and warnings
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Options
option(BUILD_PYTHON_BINDINGS "Build Python bindings using pybind11" OFF)
option(BUILD_CPP_EXAMPLES "Build C++ example executables" ON) # Default ON to build examples

# Include the low-level link library dependency
add_subdirectory(crazyflie-link-cpp EXCLUDE_FROM_ALL)

# Define the main crazyflie_cpp library target
# NOTE: Based on user-provided file list, only these two source files exist.
# This is likely INCOMPLETE and will cause linker errors later.
add_library(crazyflie_cpp
  src/Crazyflie.cpp
  src/crtp.cpp
)

# Setup include directories for the library
target_include_directories(crazyflie_cpp
  PUBLIC
    # For installation
    $<INSTALL_INTERFACE:include/crazyflie_cpp>
    # For build tree users (like the examples)
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/crazyflie_cpp>
    # Also need headers from the link library dependency
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/crazyflie-link-cpp/include>
)

# Link the library against its internal dependency
target_link_libraries(crazyflie_cpp
  PRIVATE
    crazyflieLinkCpp # Link against the target built from the crazyflie-link-cpp subdirectory
)

# --- Build C++ Examples ---
if(BUILD_CPP_EXAMPLES)
    message(STATUS "Building C++ Examples")

    # Find required packages for examples
    find_package(Threads REQUIRED)

    # Define the executable target for the control example
    # Assumes the code is saved as 'examples/control.cpp'
    add_executable(cf_control examples/control.cpp)

    # Link the executable against the main library and other dependencies
    # crazyflieLinkCpp and its dependencies (like libusb) should be linked transitively
    # because cf_control links against crazyflie_cpp which links against crazyflieLinkCpp.
    target_link_libraries(cf_control PRIVATE
        crazyflie_cpp      # Link against the library target defined above
        Threads::Threads   # Link against C++ standard threads support
        # No need to explicitly link crazyflieLinkCpp here if crazyflie_cpp links it properly
    )

    # Optional: Install the example executable if 'make install' is run
    # install(TARGETS cf_control DESTINATION bin)

else()
    message(STATUS "Skipping C++ Examples (BUILD_CPP_EXAMPLES is OFF)")
endif()
# --- End of Example Building Logic ---


# --- Installation Rules (Commented out - Keep as is or enable if needed) ---
# include(CMakePackageConfigHelpers)
# write_basic_package_version_file(...)
# install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/crazyflie_cpp ...)
# install(TARGETS crazyflie_cpp ...)
# install(EXPORT crazyflie_cpp-targets ...)
# install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/cmake/crazyflie_cpp-config.cmake" ...)
# install(FILES package.xml ...)