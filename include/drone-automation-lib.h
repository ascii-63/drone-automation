#ifndef DRONE_AUTOMATION_LIB_H
#define DRONE_AUTOMATION_LIB_H

#include <unistd.h>

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

#include <yaml-cpp/yaml.h>
#include "mqtt/async_client.h"

#include "./mission_v1.h"
#include "./UTM.h"
#include "./mqtt-lib.h"

#define vector3 std::vector<double>

#define LOCAL_HOST "127.0.0.1"       // Localhost IP
#define DEFAULT_COMM_IMAGE_PORT 5000 // Send image here
#define DEFAULT_COMM_MSG_PORT 5100   // Send messages here

#define DEFAULT_SEQ_YAML_FILE_PATH "/"
#define DEFAULT_MISSION_DIR_PATH "/"
#define DEFAULT_IMAGE_DIR_PATH "~/image/"
#define DEFAULT_FLIR_PNG "flir.png"
#define DEFAULT_D455_PNG "d455.png"

#define DEFAULT_GCS_CONFIRM_TIMEOUT 120

#define FLAG_CAM_ALLOW "FLAG_CAM_ALLOW"
#define FLAG_CAM_REJECT "FLAG_CAM_REJECT"
#define FLAG_ALLOW_TO_FLY "FLAG_ALLOW_TO_FLY"
#define FLAG_DENY_TO_FLY "FLAG_DENY_TO_FLY"

#define MQTT_DEVICE_LIST_TOPIC "/device/list"
#define MQTT_DEVICE_STATUS_TOPIC "/device/status"
#define MQTT_MAV_STATE_TOPIC "/mav_state"

enum PERIPHERAL_STATUS : int
{
    UNSPECIFIED = -1, // The peripheral remains unused.
    ACTIVE,           // Obtain the ACTIVE status once the message has been present for a continuous duration of 5 seconds.
    INACTIVE          // Retrieve the INACTIVE status when the message has been absent for a consecutive duration of 1 second.
};

enum DEVICE : int
{
    FLIR = 0,  // FLIR
    D455,      // D455
    FCU_STATE, // MAV State
    FCU_IMU,   // MAV IMU
    FCU_ODOM,  // MAV Odometry
    FCU_MAG,   // MAV Magnetometer
    FCU_PRES,  // MAV Absolute Pressure
    FCU_BAT,   // MAV Battery
    FCU_GPS    // MAV GPS
};

/****************************************** DEFINES ******************************************/

// Mission ID
std::string mission_id;
// Home GPS
vector3 home_gps;

// This is mission object in drone_lib
MissionRequest mission;

namespace Communication
{
    namespace netcat
    {
        // Send a string message to localhost/port using echo and netcat command
        void sendMessage_echo_netcat(const std::string &_message, const int _port);
        // Receive message from a TCP port in localhost, return in std::string
        std::string receiveMessage_netcat(const int _port, const int _timeout, bool &_result);
    };

    namespace MQTT
    {
        const std::string ERROR_CONSUME_MESSAGE = "ERROR_CONSUME_MESSAGE";

        class Publisher
        {
        public:
            Publisher();
            Publisher(const std::string &_server_addr,
                      const std::string &_client_id,
                      const std::string _topic);
            ~Publisher();

            bool connect();
            void publish(const std::string &_message);
            void disconnect();

        public:
            std::string server_address;
            std::string client_id;
            std::string topic;

            mqtt::async_client *client;
        };

        class Consumer
        {
        public:
            Consumer();
            Consumer(const std::string &_server_addr,
                     const std::string &_client_id,
                     const std::string _topic);
            ~Consumer();

            bool connect();
            std::string consume();
            void disconnect();

        public:
            std::string server_address;
            std::string client_id;
            std::string topic;

            mqtt::async_client *client;
        };
    };
};

namespace System
{
    void command_sys(const std::string &_command);

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

    // Check every FLAG in a vector, only return true when every FLAGs is "ALLOW"
    bool camFlagChecker(const std::vector<std::string> &_flag_vector);

    // Sleep for less than a second, in seconds.
    void sleep_msecs(const float _time);

    // Get the JSON mission file name in mission directory
    std::string getMissionFile(const std::string &_mission_dir);

    // Using netcat to get new message from _port to _flag_handle string
    bool getNewestFLAG(const int _port, std::string &_flag_handle);

    // Rename a file by add drone_id (mission_id) before it
    void renameWithID(const std::string &_path_to_dir, const std::string &_file_name);

    // Lauching seq_controller package node
    bool seqControllerLauching();
}

/* Use this namespace to convert from mission object to .yaml file*/
namespace YAMLConvert
{
    /* From Terminator type, return the failsafe index:
    Ex: TERMINATION_AUTO -> Autoland -> count
        TERMINATION_STD -> Return Home -> count - 1
    */
    int terminatorToFailsafe(const int _terminator);

    // Handle the Single Instruction then write it to YAML::Emitter
    bool seqHandle(SingleInstruction *_instruction, YAML::Emitter &_emitter);

    // Handle the Action Instruction
    bool actionSeqHandle(SingleInstruction *_action_instruction, YAML::Emitter &_emitter);

    // Handle the Travel Instruction
    bool travelSeqHandle(SingleInstruction *_travel_instruction, YAML::Emitter &_emitter);

    /* Add 2 last sequences to yaml file:
    The prev last: return to home
    The last: autoland.
    */
    void addAdditionSeqToYAML(YAML::Emitter &_emitter);

    /* Convert from mission object to .yaml file
     */
    bool fromMisisonToYAML(const std::string &_yaml_file_path);

    namespace ActionHandle
    {
        bool arucoHandle(SingleInstruction *_action_instruction, YAML::Emitter &_emitter);

        bool autoLandHandle(SingleInstruction *_action_instruction, YAML::Emitter &_emitter);

        bool disArmHandle(SingleInstruction *_action_instruction, YAML::Emitter &_emitter);

        bool releaseHandle(SingleInstruction *_action_instruction, YAML::Emitter &_emitter);

        bool returnHomeHandle(SingleInstruction *_action_instruction, YAML::Emitter &_emitter);

        bool selfCheckHandle(SingleInstruction *_action_instruction, YAML::Emitter &_emitter);

        bool takeOffHandle(SingleInstruction *_action_instruction, YAML::Emitter &_emitter);

        bool whyconHandle(SingleInstruction *_action_instruction, YAML::Emitter &_emitter);
    };

};

#endif