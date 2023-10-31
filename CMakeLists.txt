cmake_minimum_required(VERSION 3.12...3.18)
set(CMAKE_CXX_STANDARD 11)

project(drone-automation)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)

# Add -pthread flag for multi-threading
find_package(Threads REQUIRED)

include_directories(
    ./include
)

# Specify library directories for jsoncpp and yaml-cpp
link_directories(/usr/lib/x86_64-linux-gnu)
include_directories(
    ${catkin_INCLUDE_DIRS}
)

set(MISSION_LIB_SRC ./include/mission_v1.cpp)
add_library(mission_lib SHARED ${MISSION_LIB_SRC})
target_link_libraries(
    mission_lib
    ${catkin_LIBRARIES}
    jsoncpp
    yaml-cpp
    Threads::Threads
)

set(DRONE_LIB_SRC ./include/drone-automation-lib.cpp)
add_library(drone_lib SHARED ${DRONE_LIB_SRC})
target_link_libraries(
    drone_lib
    ${catkin_LIBRARIES}
    jsoncpp
    yaml-cpp
    Threads::Threads
    mission_lib
)

add_executable(mid-man ./src/mid-man.cpp)
target_link_libraries(
    mid-man
    ${catkin_LIBRARIES}
    jsoncpp
    yaml-cpp
    Threads::Threads
    drone_lib
)