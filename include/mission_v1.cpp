#include "./mission_v1.h"

/*************************************************** IMPLEMENTS ***************************************************/

/************** ENUM THINGS **************/

int enumConvert::stringToPeripheral(const std::string inputString)
{
    if (inputString == "PERIPHERAL_UNSPECIFIED")
    {
        return PERIPHERAL_UNSPECIFIED;
    }
    else if (inputString == "PERIPHERAL_CAM_FLIR")
    {
        return PERIPHERAL_CAM_FLIR;
    }
    else if (inputString == "PERIPHERAL_CAM_D455")
    {
        return PERIPHERAL_CAM_D455;
    }
    else if (inputString == "PERIPHERAL_LIDAR")
    {
        return PERIPHERAL_LIDAR;
    }
    else if (inputString == "PERIPHERAL_FCU")
    {
        return PERIPHERAL_FCU;
    }
    else if (inputString == "PERIPHERAL_CAM_T265")
    {
        return PERIPHERAL_CAM_T265;
    }
    else
    {
        return INT8_MIN;
    }
}

int enumConvert::stringToController(const std::string inputString)
{
    if (inputString == "CONTROLLER_UNSPECIFIED")
    {
        return CONTROLLER_UNSPECIFIED;
    }
    else if (inputString == "CONTROLLER_PX4_VELO_FB")
    {
        return CONTROLLER_PX4_VELO_FB;
    }
    else if (inputString == "CONTROLLER_A_FB")
    {
        return CONTROLLER_A_FB;
    }
    else if (inputString == "CONTROLLER_A_FW")
    {
        return CONTROLLER_A_FW;
    }
    else if (inputString == "CONTROLLER_A_ADRJ")
    {
        return CONTROLLER_A_ADRJ;
    }
    else
    {
        return INT8_MIN;
    }
}

int enumConvert::stringToPlanner(const std::string inputString)
{
    if (inputString == "PLANNER_UNSPECIFIED")
    {
        return PLANNER_UNSPECIFIED;
    }
    else if (inputString == "PLANNER_EGO")
    {
        return PLANNER_EGO;
    }
    else if (inputString == "PLANNER_FAST")
    {
        return PLANNER_FAST;
    }
    else if (inputString == "PLANNER_MARKER")
    {
        return PLANNER_MARKER;
    }
    else if (inputString == "PLANNER_SAFELAND")
    {
        return PLANNER_SAFELAND;
    }
    else
    {
        return INT8_MIN;
    }
}

int enumConvert::stringToTerminator(const std::string inputString)
{
    if (inputString == "TERMINATION_UNSPECIFIED")
    {
        return TERMINATION_UNSPECIFIED;
    }
    else if (inputString == "TERMINATION_AUTO")
    {
        return TERMINATION_AUTO;
    }
    else if (inputString == "TERMINATION_STD")
    {
        return TERMINATION_STD;
    }
    else
    {
        return INT8_MIN;
    }
}

int enumConvert::stringToExit(const std::string inputString)
{
    if (inputString == "EXIT_UNSPECIFIED")
    {
        return EXIT_UNSPECIFIED;
    }
    else if (inputString == "EXIT_PASSED")
    {
        return EXIT_PASSED;
    }
    else if (inputString == "EXIT_FAILED")
    {
        return EXIT_FAILED;
    }
    else
    {
        return INT8_MIN;
    }
}

int enumConvert::stringToAction(const std::string inputString)
{
    if (inputString == "ACTION_UNSPECIFIED")
    {
        return ACTION_UNSPECIFIED;
    }
    else if (inputString == "ACTION_TAKEOFF")
    {
        return ACTION_TAKEOFF;
    }
    else if (inputString == "ACTION_DISARM")
    {
        return ACTION_DISARM;
    }
    else if (inputString == "ACTION_SELFCHECK")
    {
        return ACTION_SELFCHECK;
    }
    else if (inputString == "ACTION_RELEASE")
    {
        return ACTION_RELEASE;
    }
    else if (inputString == "ACTION_RTLHOME")
    {
        return ACTION_RTLHOME;
    }
    else if (inputString == "ACTION_HOLD")
    {
        return ACTION_HOLD;
    }
    else if (inputString == "ACTION_AUTOLAND")
    {
        return ACTION_AUTOLAND;
    }
    else if (inputString == "ACTION_ARUCO")
    {
        return ACTION_ARUCO;
    }
    else if (inputString == "ACTION_WHYCON")
    {
        return ACTION_WHYCON;
    }
    else
    {
        return INT8_MIN;
    }
}

int enumConvert::stringToResponse(const std::string inputString)
{
    if (inputString == "RESPONSE_UNSPECIFIED")
    {
        return RESPONSE_UNSPECIFIED;
    }
    else if (inputString == "RESPONSE_SUCCESS")
    {
        return RESPONSE_SUCCESS;
    }
    else if (inputString == "RESPONSE_SYNTAX_ERROR")
    {
        return RESPONSE_SYNTAX_ERROR;
    }
    else if (inputString == "RESPONSE_MISSING_INSTRUCTION")
    {
        return RESPONSE_MISSING_INSTRUCTION;
    }
    else if (inputString == "RESPONSE_ENCODING_ERROR")
    {
        return RESPONSE_ENCODING_ERROR;
    }
    else
    {
        return INT8_MIN;
    }
}

/*******************************************/

SingleInstruction::~SingleInstruction() {}

int SingleInstruction::Init_getController()
{
    return INT8_MIN;
}

int SingleInstruction::Init_getTerminator()
{
    return INT8_MIN;
}

void SingleInstruction::Init_getPeripherals(std::vector<int> &_peripherals_vector)
{
    return;
}

void SingleInstruction::Init_getExit(std::vector<int> &_exit_vector)
{
    return;
}

void SingleInstruction::Init_getHomePosition(vector3 &_home_position)
{
    return;
}

double SingleInstruction::Init_getTimeOut()
{
    return DOUBLE_MIN;
}

int SingleInstruction::Travel_getPlanner()
{
    return INT8_MIN;
}

int SingleInstruction::Travel_getTerminator()
{
    return INT8_MIN;
}

void SingleInstruction::Travel_getExit(std::vector<int> &_exit_vector)
{
    return;
}

void SingleInstruction::Travel_getWaypoints(std::vector<vector3> &_waypoints_vector)
{
    return;
}

void SingleInstruction::Travel_getConstraints(std::vector<vector3> &_const_vector)
{
    return;
}

double SingleInstruction::Travel_getTimeOut()
{
    return DOUBLE_MIN;
}

int SingleInstruction::Action_getAction()
{
    return INT8_MIN;
}

double SingleInstruction::Action_getParam()
{
    return DOUBLE_MIN;
}

int SingleInstruction::Action_getTerminator()
{
    return INT8_MIN;
}

void SingleInstruction::Action_getExit(std::vector<int> &_exit_vector)
{
    return;
}

double SingleInstruction::Action_getTimeOut()
{
    return DOUBLE_MIN;
}

/*******************************************/

InitInstruction::~InitInstruction() {}

int InitInstruction::Init_getController()
{
    return controller;
}

int InitInstruction::Init_getTerminator()
{
    return terminator;
}

void InitInstruction::Init_getPeripherals(std::vector<int> &_peripherals_vector)
{
    _peripherals_vector = peripherals;
}

void InitInstruction::Init_getExit(std::vector<int> &_exit_vector)
{
    _exit_vector = exit;
}

void InitInstruction::Init_getHomePosition(vector3 &_home_position)
{
    _home_position = home_position;
}

double InitInstruction::Init_getTimeOut()
{
    return timeout;
}

/*******************************************/

TravelInstruction::~TravelInstruction()
{
}

int TravelInstruction::Travel_getPlanner()
{
    return planner;
}

int TravelInstruction::Travel_getTerminator()
{
    return terminator;
}

void TravelInstruction::Travel_getWaypoints(std::vector<vector3> &_waypoints_vector)
{
    _waypoints_vector = waypoints;
}

void TravelInstruction::Travel_getConstraints(std::vector<vector3> &_const_vector)
{
    _const_vector = constraints;
}

void TravelInstruction::Travel_getExit(std::vector<int> &_exit_vector)
{
    _exit_vector = exit;
}

double TravelInstruction::Travel_getTimeOut()
{
    return timeout;
}

/*******************************************/

ActionInstruction::~ActionInstruction() {}

int ActionInstruction::Action_getAction()
{
    return action;
}

double ActionInstruction::Action_getParam()
{
    return param;
}

int ActionInstruction::Action_getTerminator()
{
    return terminator;
}

void ActionInstruction::Action_getExit(std::vector<int> &_exit_vector)
{
    _exit_vector = exit;
}

double ActionInstruction::Action_getTimeOut()
{
    return timeout;
}

/********************* JSON PARSING NAMESPACE *********************/

bool jsonParsing::handleInitSequence(const Json::Value &_sequence, InitInstruction &_init_instruction)
{
    const Json::Value &peripheral = _sequence["peripheral"];
    if (!peripheral.isArray())
    {
        std::cerr << "The format of the peripheral array is incorrect." << std::endl;
        return false;
    }
    std::vector<int> this_peripherals;
    for (const auto &value : peripheral)
    {
        int peripheralValue = value.asInt();
        this_peripherals.push_back(peripheralValue);
    }
    _init_instruction.peripherals = this_peripherals;

    std::string this_controller = _sequence["controller"].asString();
    _init_instruction.controller = enumConvert::stringToController(this_controller);

    std::string this_terminator = _sequence["terminate"].asString();
    _init_instruction.terminator = enumConvert::stringToTerminator(this_terminator);

    const Json::Value &home_gps = _sequence["home"];
    if (!home_gps.isArray())
    {
        std::cerr << "The format of the home vector3 is incorrect." << std::endl;
        return false;
    }
    std::vector<double> this_home_gps;
    for (const auto &value : home_gps)
    {
        double gpsValue = value.asDouble();
        this_home_gps.push_back(gpsValue);
    }
    _init_instruction.home_position = this_home_gps;

    double this_timeout = _sequence["timeout"].asDouble();
    _init_instruction.timeout = this_timeout;

    return true;
}

bool jsonParsing::handleActionSequence(const Json::Value &_sequence, ActionInstruction &_action_instruction)
{
    std::string this_action = _sequence["action"].asString();
    _action_instruction.action = enumConvert::stringToAction(this_action);

    double this_param = _sequence["param"].asDouble();
    _action_instruction.param = this_param;

    std::string this_terminator = _sequence["terminate"].asString();
    _action_instruction.terminator = enumConvert::stringToTerminator(this_terminator);

    double this_timeout = _sequence["timeout"].asDouble();
    _action_instruction.timeout = this_timeout;

    return true;
}

bool jsonParsing::handleTravelSequence(const Json::Value &_sequence, TravelInstruction &_travel_instruction)
{
    std::string this_planner = _sequence["planner"].asString();
    _travel_instruction.planner = enumConvert::stringToPlanner(this_planner);

    const Json::Value &waypoints = _sequence["waypoint"];
    if (!waypoints.isArray())
    {
        std::cerr << "The format of the waypoints array is incorrect." << std::endl;
        return false;
    }
    std::vector<vector3> this_waypoints;
    for (const auto &point : waypoints)
    {
        std::vector<double> temp_vector3;
        temp_vector3.push_back(point[0].asDouble());
        temp_vector3.push_back(point[1].asDouble());
        temp_vector3.push_back(point[2].asDouble());

        this_waypoints.push_back(temp_vector3);
    }
    _travel_instruction.waypoints = this_waypoints;

    const Json::Value &constraints = _sequence["constraint"];
    if (!constraints.isArray())
    {
        std::cerr << "The format of the constraints array is incorrect." << std::endl;
        return false;
    }
    std::vector<vector3> this_constraints;
    for (const auto &constr : constraints)
    {
        std::vector<double> temp_vector3;
        temp_vector3.push_back(constr[0].asDouble());
        temp_vector3.push_back(constr[1].asDouble());
        temp_vector3.push_back(constr[2].asDouble());

        this_constraints.push_back(temp_vector3);
    }
    _travel_instruction.constraints = this_constraints;

    std::string this_terminator = _sequence["terminate"].asString();
    _travel_instruction.terminator = enumConvert::stringToTerminator(this_terminator);

    double this_timeout = _sequence["timeout"].asDouble();
    _travel_instruction.timeout = this_timeout;

    return true;
}

bool jsonParsing::parsing(const std::string _path_to_json_file, MissionRequest &_mission)
{
    // Json file
    std::ifstream file(_path_to_json_file);
    if (!file.is_open())
    {
        std::cerr << "Unable to open json file" << std::endl;
        return false;
    }

    // Read the file content into a string
    std::string jsonContent((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    file.close();

    // Parse the JSON string using a Json::Reader object
    Json::Value jsonData;
    Json::Reader jsonReader;
    if (!jsonReader.parse(jsonContent, jsonData))
    {
        std::cerr << "Cannot Parse jsonData using jsonReader" << std::endl;
        return false;
    }

    _mission.id = jsonData["id"].asString();
    _mission.number_sequence_items = jsonData["number_sequence_items"].asInt();
    _mission.description = jsonData["description"].asString();

    // Access the sequences field
    const Json::Value &sequences = jsonData["sequences"];

    // Access the sequence_items array
    const Json::Value &sequence_items = sequences["sequenceItems"];
    if (!sequence_items.isArray())
    {
        std::cerr << "The format of the sequence_items is incorrect." << std::endl;
        return false;
    }

    int sequence_index = 0;

    for (auto const &item : sequence_items)
    {
        const std::vector<std::string> &member_names = item.getMemberNames();
        if (member_names.empty())
        {
            std::cerr << "Can not get the object name of no." << sequence_index << " sequence item." << std::endl;
            return false;
        }
        std::cout << member_names[0] << std::endl;
        std::string current_item_name = member_names[0];

        if (current_item_name == "initSequence")
        {
            const Json::Value &init_sequence = sequence_items[sequence_index]["initSequence"];
            InitInstruction *init_instruction = new InitInstruction();
            if (!handleInitSequence(init_sequence, *init_instruction))
            {
                std::cerr << "Failed to parse the no." << sequence_index << " sequence item: init_sequence." << std::endl;
                return false;
            }
            init_instruction->name = "init_sequence";
            _mission.sequence_istructions.push_back(init_instruction);
            _mission.sequence_names.push_back("init_sequence");
            _mission.sequence_terminator.push_back(init_instruction->Init_getTerminator());
        }

        else if (current_item_name == "actionSequence")
        {
            const Json::Value &action_sequence = sequence_items[sequence_index]["actionSequence"];
            ActionInstruction *action_instruction = new ActionInstruction();
            if (!handleActionSequence(action_sequence, *action_instruction))
            {
                std::cerr << "Failed to parse the no." << sequence_index << " sequence item: action_instruction." << std::endl;
                return false;
            }
            action_instruction->name = "action_sequence";
            _mission.sequence_istructions.push_back(action_instruction);
            _mission.sequence_names.push_back("action_sequence");
            _mission.sequence_terminator.push_back(action_instruction->Action_getTerminator());
        }

        else if (current_item_name == "travelSequence")
        {
            const Json::Value &travel_sequence = sequence_items[sequence_index]["travelSequence"];
            TravelInstruction *travel_instruction = new TravelInstruction();
            if (!handleTravelSequence(travel_sequence, *travel_instruction))
            {
                std::cerr << "Failed to parse the no." << sequence_index << " sequence item: travel_instruction." << std::endl;
                return false;
            }
            travel_instruction->name = "travel_sequence";
            _mission.sequence_istructions.push_back(travel_instruction);
            _mission.sequence_names.push_back("travel_sequence");
            _mission.sequence_terminator.push_back(travel_instruction->Travel_getTerminator());
        }

        sequence_index++;
    }

    return true;
}