#ifndef MISSION_V1_H
#define MISSION_V1_H

#include <cstdlib>
#include <cstdbool>
#include <cstring>
#include <cstdint>

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include <fstream>

#include <jsoncpp/json/json.h>

#define vector3 std::vector<double> // Vector3 contains 3 double variables

#define DOUBLE_MIN (double)-999999999 // Min value of double type

#define DOUBLE_MAX (double)999999999 // Max value of double type

enum Peripheral : int
{
    PERIPHERAL_UNSPECIFIED,  // -1
    PERIPHERAL_CAM_FORWARD,  // D455
    PERIPHERAL_CAM_DOWNWARD, // FLIR
    PERIPHERAL_LIDAR,        // LIDAR
    PERIPHERAL_CAM_ODOM,     // T265
    PERIPHERAL_FCU,          // PIXHARK 6
    PERIPHERAL_RANGE_FINDER  // TERABEE
};

enum Controller : int
{
    CONTROLLER_UNSPECIFIED,
    CONTROLLER_PX4_VELO_FB,
    CONTROLLER_A_FB,
    CONTROLLER_A_FW,
    CONTROLLER_A_ADRJ
};

enum Planner : int
{
    PLANNER_UNSPECIFIED,
    PLANNER_EGO,
    PLANNER_FAST,
    PLANNER_MARKER,
    PLANNER_SAFELAND
};

enum Terminator : int
{
    TERMINATION_UNSPECIFIED,
    TERMINATION_AUTO,
    TERMINATION_STD
};

enum Exit : int
{
    EXIT_UNSPECIFIED = -1,
    EXIT_PASSED = EXIT_SUCCESS, // 0
    EXIT_FAILED = EXIT_FAILURE  // 1
};

enum Action : int
{
    ACTION_UNSPECIFIED,
    ACTION_TAKEOFF,
    ACTION_DISARM,
    ACTION_SELFCHECK,
    ACTION_RELEASE,
    ACTION_RTLHOME,
    ACTION_HOLD,
    ACTION_AUTOLAND,
    ACTION_ARUCO,
    ACTION_WHYCON
};

enum Response : int
{
    RESPONSE_UNSPECIFIED = -1,
    RESPONSE_SUCCESS,
    RESPONSE_SYNTAX_ERROR,
    RESPONSE_MISSING_INSTRUCTION,
    RESPONSE_ENCODING_ERROR,
};

/**************************************************** DEFINES ****************************************************/

/************** ENUM THINGS **************/

namespace enumConvert
{
    int stringToPeripheral(const std::string inputString);
    int stringToController(const std::string inputString);
    int stringToPlanner(const std::string inputString);
    int stringToTerminator(const std::string inputString);
    int stringToExit(const std::string inputString);
    int stringToAction(const std::string inputString);
    int stringToResponse(const std::string inputString);
};

/*****************************************/

// Instruction Template
class SingleInstruction
{
public: // Base functions
    virtual ~SingleInstruction();

public: // InitInstruction base functions
    virtual void Init_getPeripherals(std::vector<int> &_peripherals_vector);

    virtual void Init_getHomePosition(vector3 &_home_pose);

    virtual int Init_getController();

    virtual int Init_getTerminator();

    virtual void Init_getExit(std::vector<int> &_exit_vector);

    virtual double Init_getTimeOut();

public: // TravelInstruction base functions
    virtual int Travel_getPlanner();

    virtual void Travel_getWaypoints(std::vector<vector3> &_waypoints_vector);

    virtual void Travel_getConstraints(std::vector<vector3> &_const_vector);

    virtual int Travel_getTerminator();

    virtual void Travel_getExit(std::vector<int> &_exit_vector);

    virtual double Travel_getTimeOut();

public: // ActionInstruction base functions
    virtual int Action_getAction();

    virtual double Action_getParam();

    virtual int Action_getTerminator();

    virtual void Action_getExit(std::vector<int> &_exit_vector);

    virtual double Action_getTimeOut();

public:
    std::string name; // Name of the instruction.
};

// Init Instruction, child class of the SingleInstruction class.
class InitInstruction : public SingleInstruction
{
public:
    ~InitInstruction() override;

    int Init_getController() override;

    int Init_getTerminator() override;

    void Init_getPeripherals(std::vector<int> &_peripherals_vector) override;

    void Init_getExit(std::vector<int> &_exit_vector) override;

    void Init_getHomePosition(vector3 &_home_pose) override;

    double Init_getTimeOut() override;

public:
    std::vector<int> peripherals; // Vector contains peripherals that need to be turned on
    int controller;               // Controller using in the flight process
    int terminator;               // Terminator
    std::vector<int> exit;        // Vector contains exit codes
    vector3 home_position;        // Home position (Lat, Long, Alt)
    double timeout;               // Time-out for this Instruction
};

// Travel Instruction, child class of the SingleInstruction class
class TravelInstruction : public SingleInstruction
{
public:
    ~TravelInstruction() override;

    int Travel_getPlanner() override;

    int Travel_getTerminator() override;

    void Travel_getWaypoints(std::vector<vector3> &_waypoints_vector) override;

    void Travel_getConstraints(std::vector<vector3> &_const_vector) override;

    void Travel_getExit(std::vector<int> &_exit_vector) override;

    double Travel_getTimeOut() override;

public:
    int planner;                      // Planner using in the travel process
    std::vector<vector3> waypoints;   // Vector contains waypoints in travel process in Lat, Long, Alt
    std::vector<vector3> constraints; // Vector contains one or more constants including: maximum velocity, maximum acceleration, geofence in 3 directions
    int terminator;                   // Terminator
    std::vector<int> exit;            // Vector contains exit codes
    double timeout;                   // Time-out for this Instruction
};

// Action Instruction, child class of the SingleInstruction class
class ActionInstruction : public SingleInstruction
{
public:
    ~ActionInstruction() override;

    int Action_getAction() override;

    double Action_getParam() override;

    int Action_getTerminator() override;

    void Action_getExit(std::vector<int> &_exit_vector) override;

    double Action_getTimeOut() override;

public:
    int action;            // Action
    double param;          // Parameter in current action
    int terminator;        // Terminator
    std::vector<int> exit; // Vector contains exit codes
    double timeout;        // Time-out for this Instruction
};

// Mission received from GCS
class MissionRequest
{
public:
    std::string id;                                        // ID of the mission
    int number_sequence_items;                             // Number of items in the sequence
    std::string description;                               // Mission description
    std::vector<SingleInstruction *> sequence_istructions; // Vector of instruction pointers
    std::vector<std::string> sequence_names;               // Vector of instruction name in order.
    std::vector<int> sequence_terminator;                  // Vector of instruction terminator in order
};

// Response message when received mission from GCS
class MissionResponse
{
public:
    MissionResponse();
    ~MissionResponse();

public:
    int responseCode; // Response Code
};

/********************* JSON PARSING NAMESPACE *********************/

namespace jsonParsing
{
    // Function to handle init_sequence
    bool handleInitSequence(const Json::Value &_sequence, InitInstruction &_init_instruction);

    // Function to handle action_sequence
    bool handleActionSequence(const Json::Value &_sequence, ActionInstruction &_action_instruction);

    // Function to handle travel_sequence
    bool handleTravelSequence(const Json::Value &_sequence, TravelInstruction &_travel_instruction);

    // Parsing from JSON file to MissionRequest class object
    bool parsing(const std::string _path_to_json_file, MissionRequest &_mission);
};
#endif