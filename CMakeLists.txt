cmake_minimum_required(VERSION 3.12...3.18)
set(CMAKE_CXX_STANDARD 11)

project(drone-automation)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)

# Add -pthread flag for multi-threading
find_package(Threads REQUIRED)

# PAHO.MQTT.CPP
find_package(PahoMqttCpp REQUIRED)

include_directories(
    include
)

# Specify library directories for jsoncpp and yaml-cpp
link_directories(/usr/lib/x86_64-linux-gnu)
include_directories(
    ${catkin_INCLUDE_DIRS}
)

# Mission library
set(MISSION_LIB_SRC ./include/mission_v1.cpp)
add_library(mission_lib SHARED ${MISSION_LIB_SRC})
target_link_libraries(mission_lib
    jsoncpp
    Threads::Threads
    PahoMqttCpp::paho-mqttpp3
)

# MQTT library
set(MQTT_LIB_SRC ./include/mqtt-lib.cpp)
add_library(mqtt_lib SHARED ${MQTT_LIB_SRC})
target_link_libraries(mqtt_lib
    Threads::Threads
    PahoMqttCpp::paho-mqttpp3
)

# Drone automation library
set(DRONE_LIB_SRC ./include/drone-automation-lib.cpp)
add_library(drone_lib SHARED ${DRONE_LIB_SRC})
target_link_libraries(drone_lib
    jsoncpp
    yaml-cpp
    Threads::Threads

    mission_lib
)

# Mid-man
add_executable(mid-man ./src/mid-man.cpp)
target_link_libraries(mid-man
    ${catkin_LIBRARIES}
    jsoncpp
    yaml-cpp
    Threads::Threads
    PahoMqttCpp::paho-mqttpp3

    mission_lib
    mqtt_lib
    drone_lib
)

# ###########################################
add_executable(async_publish ./temp/async_publish.cpp)
target_link_libraries(async_publish
    PahoMqttCpp::paho-mqttpp3
)

add_executable(async_consume ./temp/async_consume.cpp)
target_link_libraries(async_consume
    PahoMqttCpp::paho-mqttpp3
)

add_executable(test_a_p ./temp/test_lib_a_p.cpp)
target_link_libraries(test_a_p
    jsoncpp
    yaml-cpp
    Threads::Threads
    PahoMqttCpp::paho-mqttpp3

    mission_lib
    mqtt_lib
    drone_lib
)

add_executable(test_a_c ./temp/test_lib_a_c.cpp)
target_link_libraries(test_a_c
    jsoncpp
    yaml-cpp
    Threads::Threads
    PahoMqttCpp::paho-mqttpp3

    mission_lib
    mqtt_lib
    drone_lib
)