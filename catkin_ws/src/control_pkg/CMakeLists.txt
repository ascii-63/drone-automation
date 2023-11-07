cmake_minimum_required(VERSION 3.0.2)
project(control_pkg)

# set(CMAKE_PREFIX_PATH "~/drone-automation/build/install")
set(nav_msgs_DIR "/opt/ros/noetic/share/nav_msgs/cmake")
set(roscpp_DIR "/opt/ros/noetic/share/roscpp/cmake")
set(sensor_msgs_DIR "/opt/ros/noetic/share/sensor_msgs/cmake")
set(std_msgs_DIR "/opt/ros/noetic/share/std_msgs/cmake")
set(wfov_camera_msgs_DIR "/opt/ros/noetic/share/wfov_camera_msgs/cmake")

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Find catkin macros and libraries
# if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  wfov_camera_msgs
)

# find_package(drone-automation REQUIRED)
find_package(PahoMqttCpp REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Add OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Add cv_bridge
find_package(cv_bridge REQUIRED)
include_directories(${cv_bridge_INCLUDE_DIRS})

# The catkin_package macro generates cmake config files for your package
# Declare things to be passed to dependent projects
# INCLUDE_DIRS: uncomment this if your package contains header files
# LIBRARIES: libraries you create in this project that dependent projects also need
# CATKIN_DEPENDS: catkin_packages dependent projects also need
# DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES control_lib
  CATKIN_DEPENDS nav_msgs roscpp sensor_msgs std_msgs wfov_camera_msgs
)

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Specify library directories for jsoncpp and yaml-cpp
link_directories(/usr/lib/x86_64-linux-gnu)

# Declare a C++ library
add_library(control_lib SHARED
  ./include/control_lib.cpp
)
target_link_libraries(control_lib
  jsoncpp
  Threads::Threads
  PahoMqttCpp::paho-mqttpp3
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)

# Declare a C++ executable
# With catkin_make all packages are built within a single CMake context
# The recommended prefix ensures that target names across packages don't collide
add_executable(log_node src/log.cpp)
add_executable(peripherals_node src/peripherals.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(log_node
  ${catkin_LIBRARIES}
  control_lib
)
target_link_libraries(peripherals_node
  ${catkin_LIBRARIES}
  control_lib
)
