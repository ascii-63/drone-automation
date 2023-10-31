#ifndef DRONE_AUTOMATION_LIB_H
#define DRONE_AUTOMATION_LIB_H

#include <iostream>
#include <sstream>
#include <fstream>

#include <vector>
#include <string>
#include <queue>
#include <map>

#include <chrono>
#include <thread>
#include <algorithm>

// #include <yaml-cpp/yaml.h>

#include "./mission_v1.cpp"
#include "./UTM.h"

#define vector3 std::vector<double>

#define LOCAL_HOST "127.0.0.1"

enum PERIPHERAL_STATUS : int
{
    UNSPECIFIED = -1,   // The peripheral remains unused.
    ACTIVE,             // Obtain the ACTIVE status once the message has been present for a continuous duration of 5 seconds.
    INACTIVE,           // Retrieve the INACTIVE status when the message has been absent for a consecutive duration of 1 second.
    WAITING_FOR_ACTIVE, // Acquire the WAITING_FOR_ACTIVE status if the message persists for less than 5 consecutive seconds.
    NOT_FOUND           // The peripheral is required but cannot be located
};

enum DEVICE : int
{
    FLIR = 0,  // FLIR
    D455,      // D455
    T265,      // T265
    LIDAR,     // LIDAR
    TERABEE,   // Range Finder
    RTK,       // RTK
    FCU_STATE, // MAV State
    FCU_IMU,   // MAV IMU
    FCU_ODOM,  // MAV Odometry
    FCU_MAG,   // MAV Magnetometer
    FCU_PRES,  // MAV Absolute Pressure
    FCU_BAT,   // MAV Battery
    FCU_MOTOR, // MAV Motor outputs, control
    FCU_AHRS,  // MAV Accelerometer
    FCU_TELE,  // MAV Telemetry (P900?)
    FCU_GPS    // MAV GPS
};

/****************************************** DEFINES ******************************************/

// Mission ID
std::string mission_id;
// Home GPS
vector3 home_gps;
// Home Local (After add offset calculation)
vector3 home_local;

namespace System
{
    // Run a command with given argv in std::vector<std::string> type (use for rosservice), using std::system().
    void runCommand_system(const std::string &_command, const std::vector<std::string> _argv);

    // Sleep current thread.
    void threadSleeper(const int _time);

    // Get vector of PID with given name
    std::vector<std::string> getPIDList(const std::string _command_name);

    // Parsing the json file into mission object.
    bool jsonParsingToObject(const std::string _path_to_json_file, MissionRequest &_mission);

    // Send image to communication service
    void sendImage(const int _device, const std::string &_drone_id);

    // From int, return enum name in DEVICE
    std::string DEVICE_enumToString(const int _num);

    // From int, return enum name in PERIPHERAL_STATUS
    std::string PERIPHERAL_STATUS_enumToString(const int _num);

    bool flagChecker(const std::string &_msg);

    // Check every FLAG in a vector, only return true when every FLAGs is "ALLOW"
    bool flagChecker(const std::vector<std::string> &_flag_vector);

    // Sleep for less than a second, in seconds.
    void sleepLessThanASecond(const float _time);

    // Get the JSON mission file name in mission directory
    std::string getMissionFile(const std::string &_mission_dir);

    // Using netcat to get new message from _port to _flag_handle string
    bool getNewestFLAG(const int _port, std::string &_flag_handle, const int _timeout);

    // Rename a file by add drone_id (mission_id) before it, _path_to_dir MUST has '/' at the end
    void renameWithID(const std::string &_path_to_dir, const std::string &_file_name);

    // Lauching seq_controller package node
    bool seqControllerLauching();
}

namespace Communication
{
    namespace netcat
    {
        // Send a string message to localhost/port using echo and netcat command
        void sendMessage_echo_netcat(const std::string &_message, const int _port);
        // Receive message from a TCP port in localhost, return in std::string
        std::string receiveMessage_netcat(const int _port, const int _timeout);
    };

    namespace mqtt
    {

    };
};

#endif