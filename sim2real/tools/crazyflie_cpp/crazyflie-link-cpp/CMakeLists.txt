cmake_minimum_required (VERSION 3.8) # Bumped for newer CMake features
project(crazyflie-link-cpp VERSION 1.0.0 LANGUAGES CXX) # Added VERSION and LANGUAGES

# define some options
option(BUILD_PYTHON_BINDINGS "Generate Python Bindings" ON)
option(BUILD_CPP_EXAMPLES "Generate C++ Examples" ON)
option(CFLINK_INSTALL_EXAMPLES "Install C++ Examples" OFF) # Added option to install examples

# Enable C++14 and warnings
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compile static libraries with hidden visibility
set(CMAKE_CXX_VISIBILITY_PRESET default) # Make symbols public by default
# set(CMAKE_VISIBILITY_INLINES_HIDDEN ON) # Can comment this out too, less critical

if (MSVC)
  add_definitions(-DNOMINMAX)
  add_compile_options(/W4)
else()
  add_compile_options(-Wall -Wextra -Werror)
endif()

# ============================================================
# Dependencies
# ============================================================
message(STATUS ">>> Starting crazyflie-link-cpp CMakeLists.txt <<<") # DEBUG Start

# --- LibUSB ---
if (WIN32)
  # If building libusb from source, treat it more like an internal dependency
  message(STATUS "Using bundled libusb source for Windows")
  include_directories(libusb/msvc)
  add_library(cflink_libusb STATIC # Keep it internal if bundled
    libusb/libusb/core.c
    libusb/libusb/descriptor.c
    libusb/libusb/os/events_windows.c
    libusb/libusb/hotplug.c
    libusb/libusb/io.c
    libusb/libusb/strerror.c
    libusb/libusb/sync.c
    libusb/libusb/os/threads_windows.c
    libusb/libusb/os/windows_common.c
    libusb/libusb/os/windows_usbdk.c
    libusb/libusb/os/windows_winusb.c
  )
  set(LIBUSB_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/libusb/libusb)
  set(LIBUSB_LIBRARIES cflink_libusb) # Link the internal static lib
else()
  # Find system-installed libusb EXPLICITLY using pkg-config
  message(STATUS "DEBUG: Handling Linux/Mac LibUSB case.") # DEBUG Point 3 LinMac
  message(STATUS "DEBUG: Attempting: find_package(PkgConfig REQUIRED)") # DEBUG Point 4
  find_package(PkgConfig REQUIRED)
  message(STATUS "DEBUG: find_package(PkgConfig) successful.") # DEBUG Point 5
  message(STATUS "DEBUG: Attempting: pkg_check_modules(LIBUSB REQUIRED libusb-1.0)") # DEBUG Point 6
  pkg_check_modules(LIBUSB REQUIRED libusb-1.0) # Use pkg-config name
  message(STATUS "DEBUG: pkg_check_modules(LIBUSB) successful.") # DEBUG Point 7
  message(STATUS "Found libusb via pkg-config: LIBRARIES='${LIBUSB_LIBRARIES}' INCLUDE_DIRS='${LIBUSB_INCLUDE_DIRS}'")
endif()

# --- PThreads (only if not Windows) ---
if (NOT WIN32)
  message(STATUS "DEBUG: Attempting: find_package(Threads REQUIRED)") # DEBUG Point 8
  set(THREADS_PREFER_PTHREAD_FLAG ON)
  find_package(Threads REQUIRED)
  message(STATUS "DEBUG: find_package(Threads) successful.") # DEBUG Point 9
  message(STATUS "Found Threads: ${CMAKE_THREAD_LIBS_INIT}")
endif()

# ============================================================
# Core C++ Library Target: crazyflieLinkCpp
# ============================================================
message(STATUS "DEBUG: Defining crazyflieLinkCpp target.") # DEBUG Point 10
add_library(crazyflieLinkCpp SHARED # Changed to SHARED, usually better for ROS
  src/USBManager.cpp
  src/Connection.cpp
  src/USBDevice.cpp
  src/Crazyradio.cpp
  src/CrazyradioThread.cpp
  src/CrazyflieUSB.cpp
  src/CrazyflieUSBThread.cpp
  src/Version.cpp
)
set_property(TARGET crazyflieLinkCpp PROPERTY POSITION_INDEPENDENT_CODE ON)

# Add include directories needed to BUILD the library
target_include_directories(crazyflieLinkCpp
  PRIVATE
    ${LIBUSB_INCLUDE_DIRS} # Use variable from pkg_check_modules
  PUBLIC
    # Headers needed by users of this library
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> # Path relative to CMAKE_INSTALL_PREFIX
)

# Link dependencies needed BY the library
target_link_libraries(crazyflieLinkCpp
  PRIVATE # Dependencies not exposed in public headers
    # Use the variable confirmed by the message() STATUS printout above.
    # It's most likely LIBUSB_LIBRARIES for pkg_check_modules.
    ${LIBUSB_LIBRARIES}
)
if (NOT WIN32)
  target_link_libraries(crazyflieLinkCpp
    PRIVATE
      Threads::Threads # Use imported target from find_package(Threads)
  )
endif()

# Apple macOS frameworks (if needed, apply to the target)
if (APPLE)
  target_link_libraries(crazyflieLinkCpp
    PRIVATE
     "-lobjc -framework IOKit -framework CoreFoundation -framework Security"
  )
endif()


# ============================================================
# Installation Rules
# ============================================================
include(GNUInstallDirs) # Provides CMAKE_INSTALL_LIBDIR, CMAKE_INSTALL_INCLUDEDIR etc.
include(CMakePackageConfigHelpers)

# --- Install Library Target ---
# Creates an export set named "crazyflieLinkCppTargets"
install(TARGETS crazyflieLinkCpp
  EXPORT crazyflieLinkCppTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} # For DLLs on Windows
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} # For .so / .lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} # For .a / static .lib
  # INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} # Public headers installed separately
)

# --- Install Public Headers ---
# Installs the contents of include/crazyflieLinkCpp to <prefix>/include/crazyflieLinkCpp
# Ensure this directory structure exists and contains your public headers
install(DIRECTORY include/crazyflieLinkCpp/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/crazyflieLinkCpp
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# --- Configure and Install CMake Package Files ---
# 1. Create the version file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion # Or ExactVersion, SameMajorVersion etc.
)

# 2. Configure the main config file from the template
#    Make sure cmake/crazyflie-link-cppConfig.cmake.in exists
set(LIBUSB_INSTALL_INTERFACE_LIBRARIES "${LIBUSB_LIBRARIES}") # Variable holding found libs
set(LIBUSB_INSTALL_INTERFACE_INCLUDE_DIRS "${LIBUSB_INCLUDE_DIRS}") # Variable holding found includes

configure_package_config_file(
  "cmake/${PROJECT_NAME}Config.cmake.in" # Input template path (e.g. cmake/crazyflie-link-cppConfig.cmake.in)
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" # Output path
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}" # Common location: lib/cmake/Project
  PATH_VARS LIBUSB_INSTALL_INTERFACE_LIBRARIES LIBUSB_INSTALL_INTERFACE_INCLUDE_DIRS # Pass the found paths
)

# 3. Install the config, version, and targets files
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}" # Match INSTALL_DESTINATION above
)

# 4. Install the export set (creates ${PROJECT_NAME}Targets.cmake)
# The NAMESPACE crazyflieLinkCpp:: makes the imported target look like crazyflieLinkCpp::crazyflieLinkCpp
install(EXPORT crazyflieLinkCppTargets
  FILE        ${PROJECT_NAME}Targets.cmake
  NAMESPACE   crazyflieLinkCpp::
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}" # Install alongside config files
)

# ============================================================
# C++ Examples (Build only, install optional)
# ============================================================
if (BUILD_CPP_EXAMPLES)
  set(CFLINK_EXAMPLES
      console
      control
      scan
      benchmark
      broadcast
      broadcast_and_console
  )
  foreach(EXAMPLE_NAME ${CFLINK_EXAMPLES})
      add_executable(example_${EXAMPLE_NAME} examples/${EXAMPLE_NAME}.cpp)
      # --- THIS LINE WAS MISSING/COMMENTED ---
      target_link_libraries(example_${EXAMPLE_NAME} PRIVATE crazyflieLinkCpp)
      # -------------------------------------

      if (CFLINK_INSTALL_EXAMPLES)
          install(TARGETS example_${EXAMPLE_NAME}
              RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/crazyflieLinkCpp_examples
          )
      endif()
  endforeach()
endif()

# ============================================================
# Python Bindings (Build only, installation handled by Python tools usually)
# ============================================================
if (BUILD_PYTHON_BINDINGS)
  # Ensure pybind11 submodule is initialized if using git submodules
  # Or find_package(pybind11 REQUIRED) if installed separately
  find_package(PythonLibs REQUIRED) # Needed by pybind11
  find_package(pybind11 REQUIRED) # Assumes pybind11 is installed or findable
  # add_subdirectory(pybind11 EXCLUDE_FROM_ALL) # Use if pybind11 is a submodule

  pybind11_add_module(cflinkcpp MODULE # Use MODULE for standard python extension build
    src/python_bindings.cpp
  )

  target_link_libraries(cflinkcpp
    PRIVATE
      crazyflieLinkCpp # Link against our library
  )
endif()

message(STATUS ">>> Finished crazyflie-link-cpp CMakeLists.txt <<<") # DEBUG End