#include "./drone-automation-lib.h"

void System::command_sys(const std::string &_command)
{
    std::system(_command.c_str());
}

void System::runCommand_system(const std::string &_command, const std::vector<std::string> _argv)
{
    std::string command_with_args;

    std::stringstream ss;
    ss << _command << " ";
    for (std::string arg : _argv)
        ss << arg << " ";

    command_with_args = ss.str();
    command_with_args.pop_back();

    std::thread thr(command_sys, command_with_args); // Create a new thread to run the command

    thr.detach(); // Detach the thread to run independently
}

void System::threadSleeper(const int _time)
{
    std::chrono::seconds pauseTime(_time);
    std::this_thread::sleep_for(pauseTime);
}

std::vector<std::string> System::getPIDList(const std::string _command_name)
{
    std::vector<std::string> result;

    std::string command = "pgrep -f " + _command_name;
    FILE *pipe = popen(command.c_str(), "r");
    if (!pipe)
    {
        return result;
    }

    char buffer[2048];
    while (fgets(buffer, 2048, pipe))
    {
        std::string temp_string = std::string(buffer);
        temp_string.pop_back();
        result.push_back(temp_string);
    }
    // The last PID is the PID of "pgrep" command.
    result.pop_back();

    return result;
}

bool System::jsonParsingToObject(const std::string _path_to_json_file, MissionRequest &_mission)
{
    if (jsonParsing::parsing(_path_to_json_file, _mission))
    {
        Communication::netcat::sendMessage_echo_netcat("[ INFO] Parsing successful.", DEFAULT_COMM_MSG_PORT);
        return true;
    }
    return false;
}

void System::sendImage(const int _device, const std::string &_drone_id)
{
    std::string curl_cmd = "curl";
    std::vector<std::string> curl_argv;
    curl_argv.push_back("-X POST -F");

    std::stringstream ss;
    ss << "\"image=@" << DEFAULT_IMAGE_DIR_PATH;
    switch (_device)
    {
    case Peripheral::PERIPHERAL_CAM_FLIR:
        ss << _drone_id << "-" << DEFAULT_FLIR_PNG << "\"";
        break;

    case Peripheral::PERIPHERAL_CAM_D455:
        ss << _drone_id << "-" << DEFAULT_D455_PNG << "\"";
        break;

    default:
        Communication::netcat::sendMessage_echo_netcat("[ WARN] Invalid device id, this image will not be send.", DEFAULT_COMM_MSG_PORT);
        break;
    }
    curl_argv.push_back(ss.str());

    std::stringstream ss1;
    ss1 << "http://localhost:" << DEFAULT_COMM_IMAGE_PORT << "/upload";
    curl_argv.push_back(ss1.str());

    System::runCommand_system(curl_cmd, curl_argv);
}

std::string System::DEVICE_enumToString(const int _num)
{
    switch (_num)
    {
    case DEVICE::FLIR:
        return "FLIR";
    case DEVICE::D455:
        return "D455";
    case DEVICE::FCU_STATE:
        return "MAV State";
    case DEVICE::FCU_IMU:
        return "MAV IMU";
    case DEVICE::FCU_ODOM:
        return "MAV Odometry";
    case DEVICE::FCU_MAG:
        return "MAV Magnetometer";
    case DEVICE::FCU_PRES:
        return "MAV Absolute Pressure";
    case DEVICE::FCU_BAT:
        return "MAV Battery";
    case DEVICE::FCU_GPS:
        return "MAV GPS";
    default:
        return "UNKNOWN";
    }
}

std::string System::PERIPHERAL_STATUS_enumToString(const int _num)
{
    switch (_num)
    {
    case PERIPHERAL_STATUS::UNSPECIFIED:
        return "UNSPECIFIED";
    case PERIPHERAL_STATUS::ACTIVE:
        return "ACTIVE";
    case PERIPHERAL_STATUS::INACTIVE:
        return "INACTIVE";
    default:
        return "UNKNOWN";
    }
}

bool System::camFlagChecker(const std::vector<std::string> &_flag_vector)
{
    for (auto flag : _flag_vector)
    {
        if (flag != FLAG_CAM_ALLOW)
            return false;
    }
    return true;
}

void System::sleep_msecs(const float _time)
{
    struct timespec sleepTime;
    sleepTime.tv_sec = 0;
    sleepTime.tv_nsec = static_cast<long>(_time * 1000000000); // nanoseconds

    int result = nanosleep(&sleepTime, NULL);
}

std::string System::getMissionFile(const std::string &_mission_dir)
{
    std::string cmd = "ls " + _mission_dir;
    std::string result;

    while (result.empty())
    {
        FILE *pipe = popen(cmd.c_str(), "r");

        if (!pipe)
        {
            Communication::netcat::sendMessage_echo_netcat("[ERROR] Can not find any JSON mission file in " + _mission_dir + " directory.", DEFAULT_COMM_MSG_PORT);
            return result;
        }
        char buffer[256];
        try
        {
            while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
                result += buffer;
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << std::endl;
        }

        pclose(pipe);
    }
    result.pop_back();

    return result;
}

bool System::getNewestFLAG(const int _port, std::string &_flag_handle)
{
    std::string line;
    bool result;
    do
    {
        line = Communication::netcat::receiveMessage_netcat(_port, DEFAULT_GCS_CONFIRM_TIMEOUT, result);
    } while (!result);

    if (line != FLAG_ALLOW_TO_FLY && line != FLAG_DENY_TO_FLY)
    {
        Communication::netcat::sendMessage_echo_netcat("[ERROR] Not received any CONFIRM FLAGs after TIMEOUT duration, stop listening.", DEFAULT_COMM_MSG_PORT);
        _flag_handle = "";
        return false;
    }

    _flag_handle = line;
    return true;
}

void System::renameWithID(const std::string &_path_to_dir, const std::string &_file_name)
{
    std::string path = _path_to_dir;
    std::string cmd = "mv";
    std::vector<std::string> argv;

    char const &back_char = path.back();
    if (back_char != '/')
        path = path + "/";

    std::string old_name = path + _file_name;
    std::string new_name = path + mission_id + "-" + _file_name;
    argv.push_back(old_name);
    argv.push_back(new_name);
    System::runCommand_system(cmd, argv);
}

bool System::seqControllerLauching()
{
    try
    {
        std::string cmd = "bash";
        std::vector<std::string> argv;
        std::string path = get_current_dir_name();
        path = path + "/../bash/seq_controller.sh";
        System::runCommand_system(cmd, argv);
    }
    catch (const std::exception &e)
    {
        return false;
    }
    return true;
}

/******************************************************************************/

void Communication::netcat::sendMessage_echo_netcat(const std::string &_message, const int _port)
{
    std::string command = "echo";
    std::vector<std::string> argv;

    std::stringstream ss;
    ss << "\"" << _message << "\"";
    argv.push_back(ss.str());
    argv.push_back("|");
    argv.push_back("nc -q 1 localhost");
    argv.push_back(std::to_string(_port));

    System::runCommand_system(command, argv);
    System::sleep_msecs(0.1);
}

std::string Communication::netcat::receiveMessage_netcat(const int _port, const int _timeout, bool &_result)
{
    std::string command = "nc -l -p 24000";
    std::string line;
    std::string result = "";

    // Record the starting time
    auto startTime = std::chrono::high_resolution_clock::now();
    // Set the timeout duration
    auto timeoutDuration = std::chrono::seconds(_timeout);

    do
    {
        // Check if the timeout has occurred
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto elapsedDuration = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime);
        if (elapsedDuration >= timeoutDuration)
        {
            _result = false;
            return result;
        }

        // Open a pipe and execute the command
        FILE *pipe = popen(command.c_str(), "r");
        if (!pipe)
        {
            _result = false;
            return result;
        }
        char buffer[1024];
        if (fgets(buffer, sizeof(buffer), pipe) != nullptr)
        {
            line = buffer;
            // line.pop_back(); // Remove the newline character
        }
        pclose(pipe); // Close the pipe
    } while (line.empty());

    // For testing using echo, uncomment this below line
    line.pop_back();

    result = line;
    _result = true;

    return result;
}

//////////////////////////////////////////////

Communication::MQTT::Publisher::Publisher() {}

Communication::MQTT::Publisher::Publisher(const std::string &_server_addr,
                                          const std::string &_client_id,
                                          const std::string _topic) : server_address(_server_addr),
                                                                      client_id(_client_id),
                                                                      topic(_topic)
{
}

Communication::MQTT::Publisher::~Publisher() {}

bool Communication::MQTT::Publisher::connect()
{
    client = new mqtt::async_client(server_address, client_id, DEFAULT_PERSIST_DIR);
    try
    {
        mqtt::token_ptr connection_token = client->connect(connection_options);
        connection_token->wait();
    }
    catch (const mqtt::exception &exc)
    {
        std::cerr << exc.what() << '\n';
        return false;
    }
    return true;
}

void Communication::MQTT::Publisher::publish(const std::string &_message)
{
    try
    {
        mqtt::message_ptr pub_msg = mqtt::make_message(topic, _message);
        pub_msg->set_qos(DEFAULT_QOS);
        client->publish(pub_msg)->wait_for(DEFAULT_TIMEOUT);
    }
    catch (const mqtt::exception &exc)
    {
        std::cerr << exc.what() << std::endl
                  << "Message droped: "
                  << _message
                  << std::endl;
    }
    System::sleep_msecs(0.1);
}

void Communication::MQTT::Publisher::disconnect()
{
    try
    {
        client->disconnect()->wait();
    }
    catch (const mqtt::exception &exc)
    {
        std::cerr << exc.what() << '\n';
    }
}

//////////////////////////////////////////////

Communication::MQTT::Consumer::Consumer() {}

Communication::MQTT::Consumer::Consumer(const std::string &_server_addr,
                                        const std::string &_client_id,
                                        const std::string _topic) : server_address(_server_addr),
                                                                    client_id(_client_id),
                                                                    topic(_topic)
{
}

Communication::MQTT::Consumer::~Consumer() {}

bool Communication::MQTT::Consumer::connect()
{
    client = new mqtt::async_client(server_address, client_id);
    try
    {
        // Start consumer before connecting to make sure to not miss messages
        client->start_consuming();

        // Connect to the server
        auto token = client->connect(connection_options);

        // Getting the connect response will block waiting for the connection to complete
        auto response = token->get_connect_response();

        // If there is no session present, then we need to subscribe, but if
        // there is a session, then the server remembers us and our
        // subscriptions.
        if (!response.is_session_present())
            client->subscribe(topic, DEFAULT_QOS)->wait();
    }
    catch (const mqtt::exception &exc)
    {
        std::cerr << exc.what() << '\n';
        return false;
    }

    return true;
}

std::string Communication::MQTT::Consumer::consume()
{
    try
    {
        auto message = client->consume_message();
        return message->to_string();
    }
    catch (const std::system_error &e)
    {
        std::cout << "Caught system_error with code ["
                  << e.code() << "] meaning ["
                  << e.what() << "]\n";
    }
    return ERROR_CONSUME_MESSAGE;
}

void Communication::MQTT::Consumer::disconnect()
{
    try
    {
        client->disconnect()->wait();
    }
    catch (const mqtt::exception &exc)
    {
        std::cerr << exc.what() << '\n';
    }
}

/******************************************************************************/

int YAMLConvert::terminatorToFailsafe(const int _terminator)
{
    switch (_terminator)
    {
    case Terminator::TERMINATION_STD:
        return mission.number_sequence_items;

    case Terminator::TERMINATION_AUTO:
        return mission.number_sequence_items + 1;

    default:
        return -1;
    }
    return -1;
}

bool YAMLConvert::seqHandle(SingleInstruction *_instruction, YAML::Emitter &_emitter)
{
    if (_instruction->name == "action_sequence")
        return actionSeqHandle(_instruction, _emitter);
    else if (_instruction->name == "travel_sequence")
        return travelSeqHandle(_instruction, _emitter);
    return false;
}

bool YAMLConvert::actionSeqHandle(SingleInstruction *_action_instruction, YAML::Emitter &_emitter)
{
    const int action = _action_instruction->Action_getAction();
    switch (action)
    {
    case Action::ACTION_ARUCO:
        return YAMLConvert::ActionHandle::arucoHandle(_action_instruction, _emitter);

    case Action::ACTION_AUTOLAND:
        return YAMLConvert::ActionHandle::autoLandHandle(_action_instruction, _emitter);

    case Action::ACTION_DISARM:
        return YAMLConvert::ActionHandle::disArmHandle(_action_instruction, _emitter);

    case Action::ACTION_RELEASE:
        return YAMLConvert::ActionHandle::releaseHandle(_action_instruction, _emitter);

    case Action::ACTION_RTLHOME:
        return YAMLConvert::ActionHandle::returnHomeHandle(_action_instruction, _emitter);

    case Action::ACTION_SELFCHECK:
        return YAMLConvert::ActionHandle::selfCheckHandle(_action_instruction, _emitter);

    case Action::ACTION_TAKEOFF:
        return YAMLConvert::ActionHandle::takeOffHandle(_action_instruction, _emitter);

    case Action::ACTION_WHYCON:
        return YAMLConvert::ActionHandle::whyconHandle(_action_instruction, _emitter);

    default:
        Communication::netcat::sendMessage_echo_netcat("[ERROR] This Action Instruction has a invalid Action.", DEFAULT_COMM_MSG_PORT);
        return false;
    }
    return false;
}

bool YAMLConvert::travelSeqHandle(SingleInstruction *_travel_instruction, YAML::Emitter &_emitter)
{
    int planner;                      // PLanner
    std::vector<vector3> waypoints;   // Waypoints
    std::vector<vector3> constraints; // Constraints
    vector3 vmax, amax;               // Vmax and Amax
    int terminator;                   // Terminator
    /********************************************************************/
    std::string type;                             // Type field
    int count;                                    // Count field
    double timeout;                               // Timeout field
    int failsafe;                                 // Failsafe field
    std::map<std::string, vector3> waypoints_map; // Local points map

    planner = _travel_instruction->Travel_getPlanner();
    _travel_instruction->Travel_getWaypoints(waypoints);
    _travel_instruction->Travel_getConstraints(constraints);
    if (constraints.size() > 0)
        vmax = constraints[0];
    if (constraints.size() > 1)
        amax = constraints[1];
    terminator = _travel_instruction->Travel_getTerminator();

    /*************************************************/

    type = "m";
    count = waypoints.size();
    timeout = _travel_instruction->Travel_getTimeOut();

    // Update the waypoints map with waypoint and it's name.
    for (int i = 0; i < waypoints.size(); i++)
    {
        std::string name = "t" + std::to_string(i + 1);
        waypoints_map[name] = waypoints[i];
    }

    failsafe = terminatorToFailsafe(terminator);

    /*************************************************/

    _emitter << YAML::BeginMap;
    _emitter << YAML::Key << "type" << YAML::Value << type;
    _emitter << YAML::Key << "count" << YAML::Value << count;
    _emitter << YAML::Key << "timeout" << YAML::Value << timeout;
    for (int i = 0; i < waypoints.size(); i++)
    {
        std::string name = "t" + std::to_string(i + 1);
        _emitter << YAML::Key << name << YAML::Value << waypoints_map[name];
    }
    _emitter << YAML::Key << "failsafe" << YAML::Value << failsafe;
    _emitter << YAML::EndMap;

    return true;
}

void YAMLConvert::addAdditionSeqToYAML(YAML::Emitter &_emitter)
{
    int pre_last_seq_index = terminatorToFailsafe(Terminator::TERMINATION_STD);
    std::string pre_last_seq_str = "s" + std::to_string(pre_last_seq_index);

    _emitter << YAML::Key << pre_last_seq_str;
    _emitter << YAML::BeginMap;
    _emitter << YAML::Key << "type" << YAML::Value << "m";
    _emitter << YAML::Key << "count" << YAML::Value << int(1);
    _emitter << YAML::Key << "timeout" << YAML::Value << float(60.0);
    _emitter << YAML::Key << "t1" << YAML::Value << home_gps;
    // _emitter << YAML::Key << "t1" << YAML::Value << home_local;
    _emitter << YAML::Key << "failsafe" << YAML::Value << terminatorToFailsafe(Terminator::TERMINATION_AUTO);
    _emitter << YAML::EndMap;

    int last_seq_index = pre_last_seq_index + 1;
    std::string last_seq_str = "s" + std::to_string(last_seq_index);

    _emitter << YAML::Key << last_seq_str;
    _emitter << YAML::BeginMap;
    _emitter << YAML::Key << "type" << YAML::Value << "l";
    _emitter << YAML::Key << "timeout" << YAML::Value << float(60.0);
    _emitter << YAML::Key << "failsafe" << YAML::Value << int(0);
    _emitter << YAML::EndMap;
}

bool YAMLConvert::fromMisisonToYAML(const std::string &_yaml_file_path)
{
    int count = mission.number_sequence_items + 1; // count field in yaml
    int goal = mission.number_sequence_items - 1;  // goal field in yaml
    int seq_index = 1;                             // Sequence index in mission object, start at 1 (0 is init)
    std::ofstream seq_yaml(_yaml_file_path, std::ofstream::out | std::ofstream::trunc);
    if (!seq_yaml.is_open())
    {
        Communication::netcat::sendMessage_echo_netcat("[ERROR] Failed to open seq.yaml file.", DEFAULT_COMM_MSG_PORT);
        return false;
    }

    YAML::Emitter emitter; // Write to YAML file to emitter
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "count" << YAML::Value << count;
    emitter << YAML::Key << "goal" << YAML::Value << goal;

    // Write from SingleInstruction to YAML file one by one
    while (seq_index < mission.number_sequence_items)
    {
        std::string seq_name = "s" + std::to_string(seq_index);
        emitter << YAML::Key << seq_name;
        if (!seqHandle(mission.sequence_istructions[seq_index], emitter))
            return false;
        seq_index++;
    }

    // Add 2 last sequences to yaml file
    addAdditionSeqToYAML(emitter);

    emitter << YAML::EndMap;

    seq_yaml << emitter.c_str();
    seq_yaml.close();

    return true;
}

bool YAMLConvert::ActionHandle::arucoHandle(SingleInstruction *_action_instruction, YAML::Emitter &_emitter)
{
    double param;   // Param value in Action Instruction
    double timeout; // Time-out value in Action Instruction
    int terminator; // Terminator value in Action Instruction

    std::string type; // Type field
    double height;    // Height field
    int failsafe;     // Failsafe field

    /******************************************************/

    param = _action_instruction->Action_getParam();
    timeout = _action_instruction->Action_getTimeOut();
    terminator = _action_instruction->Action_getTerminator();

    /******************************************************/

    type = "a";
    height = param;
    failsafe = terminatorToFailsafe(terminator);

    /******************************************************/

    _emitter << YAML::BeginMap;
    _emitter << YAML::Key << "type" << YAML::Value << type;
    _emitter << YAML::Key << "height" << YAML::Value << height;
    _emitter << YAML::Key << "timeout" << YAML::Value << timeout;
    _emitter << YAML::Key << "failsafe" << YAML::Value << failsafe;
    _emitter << YAML::EndMap;

    return true;
}

bool YAMLConvert::ActionHandle::autoLandHandle(SingleInstruction *_action_instruction, YAML::Emitter &_emitter)
{
    double param;   // Param value in Action Instruction
    double timeout; // Time-out value in Action Instruction
    int terminator; // Terminator value in Action Instruction

    std::string type; // Type field
    double velocity;  // Velocity field
    int failsafe;     // Failsafe field

    /******************************************************/

    param = _action_instruction->Action_getParam();
    timeout = _action_instruction->Action_getTimeOut();
    terminator = _action_instruction->Action_getTerminator();

    /******************************************************/

    type = "l";
    velocity = param;
    failsafe = terminatorToFailsafe(terminator);

    /******************************************************/

    _emitter << YAML::BeginMap;
    _emitter << YAML::Key << "type" << YAML::Value << type;
    _emitter << YAML::Key << "velocity" << YAML::Value << velocity;
    _emitter << YAML::Key << "timeout" << YAML::Value << timeout;
    _emitter << YAML::Key << "failsafe" << YAML::Value << failsafe;
    _emitter << YAML::EndMap;

    return true;
}

bool YAMLConvert::ActionHandle::disArmHandle(SingleInstruction *_action_instruction, YAML::Emitter &_emitter)
{
    double timeout; // Time-out value in Action Instruction
    int terminator; // Terminator value in Action Instruction

    std::string type; // Type field
    int failsafe;     // Failsafe field

    /******************************************************/

    timeout = _action_instruction->Action_getTimeOut();
    terminator = _action_instruction->Action_getTerminator();

    /******************************************************/

    type = "d";
    failsafe = terminatorToFailsafe(terminator);

    /******************************************************/

    _emitter << YAML::BeginMap;
    _emitter << YAML::Key << "type" << YAML::Value << type;
    _emitter << YAML::Key << "timeout" << YAML::Value << timeout;
    _emitter << YAML::Key << "failsafe" << YAML::Value << failsafe;
    _emitter << YAML::EndMap;

    return true;
}

bool YAMLConvert::ActionHandle::releaseHandle(SingleInstruction *_action_instruction, YAML::Emitter &_emitter)
{
    double param;   // Param value in Action Instruction
    double timeout; // Time-out value in Action Instruction
    int terminator; // Terminator value in Action Instruction

    std::string type; // Type field
    int package;      // Package field
    int failsafe;     // Failsafe field

    /******************************************************/

    param = _action_instruction->Action_getParam();
    timeout = _action_instruction->Action_getTimeOut();
    terminator = _action_instruction->Action_getTerminator();

    /******************************************************/

    type = "r";
    package = static_cast<int>(param);
    failsafe = terminatorToFailsafe(terminator);

    /******************************************************/

    _emitter << YAML::BeginMap;
    _emitter << YAML::Key << "type" << YAML::Value << type;
    _emitter << YAML::Key << "package" << YAML::Value << package;
    _emitter << YAML::Key << "timeout" << YAML::Value << timeout;
    _emitter << YAML::Key << "failsafe" << YAML::Value << failsafe;
    _emitter << YAML::EndMap;

    return true;
}

bool YAMLConvert::ActionHandle::returnHomeHandle(SingleInstruction *_action_instruction, YAML::Emitter &_emitter)
{
    double timeout; // Time-out value in Action Instruction
    int terminator; // Terminator value in Action Instruction

    std::string type; // Type field
    int count;        // Count field
    vector3 t1;       // t1 filed (home position in local)
    int failsafe;     // Failsafe field

    /******************************************************/

    timeout = _action_instruction->Action_getTimeOut();
    terminator = _action_instruction->Action_getTerminator();

    /******************************************************/

    type = "m";
    count = 1;
    t1 = home_gps;
    failsafe = terminatorToFailsafe(terminator);

    /******************************************************/

    _emitter << YAML::BeginMap;
    _emitter << YAML::Key << "type" << YAML::Value << type;
    _emitter << YAML::Key << "count" << YAML::Value << count;
    _emitter << YAML::Key << "timeout" << YAML::Value << timeout;
    _emitter << YAML::Key << "t1" << YAML::Value << t1;
    _emitter << YAML::Key << "failsafe" << YAML::Value << failsafe;
    _emitter << YAML::EndMap;

    return true;
}

bool YAMLConvert::ActionHandle::selfCheckHandle(SingleInstruction *_action_instruction, YAML::Emitter &_emitter)
{
    double timeout; // Time-out value in Action Instruction
    int terminator; // Terminator value in Action Instruction

    std::string type; // Type field
    int failsafe;     // Failsafe field

    /******************************************************/

    timeout = _action_instruction->Action_getTimeOut();
    terminator = _action_instruction->Action_getTerminator();

    /******************************************************/

    type = "s";
    failsafe = terminatorToFailsafe(terminator);

    /******************************************************/

    _emitter << YAML::BeginMap;
    _emitter << YAML::Key << "type" << YAML::Value << type;
    _emitter << YAML::Key << "timeout" << YAML::Value << timeout;
    _emitter << YAML::Key << "failsafe" << YAML::Value << failsafe;
    _emitter << YAML::EndMap;

    return true;
}

bool YAMLConvert::ActionHandle::takeOffHandle(SingleInstruction *_action_instruction, YAML::Emitter &_emitter)
{
    double param;   // Param value in Action Instruction
    double timeout; // Time-out value in Action Instruction
    int terminator; // Terminator value in Action Instruction

    std::string type; // Type field
    double height;    // Velocity field
    int failsafe;     // Failsafe field

    /******************************************************/

    param = _action_instruction->Action_getParam();
    timeout = _action_instruction->Action_getTimeOut();
    terminator = _action_instruction->Action_getTerminator();

    /******************************************************/

    type = "t";
    height = param;
    failsafe = terminatorToFailsafe(terminator);

    /******************************************************/

    _emitter << YAML::BeginMap;
    _emitter << YAML::Key << "type" << YAML::Value << type;
    _emitter << YAML::Key << "height" << YAML::Value << height;
    _emitter << YAML::Key << "timeout" << YAML::Value << timeout;
    _emitter << YAML::Key << "failsafe" << YAML::Value << failsafe;
    _emitter << YAML::EndMap;

    return true;
}

bool YAMLConvert::ActionHandle::whyconHandle(SingleInstruction *_action_instruction, YAML::Emitter &_emitter)
{
    double param;   // Param value in Action Instruction
    double timeout; // Time-out value in Action Instruction
    int terminator; // Terminator value in Action Instruction

    std::string type; // Type field
    double height;    // Height field
    int failsafe;     // Failsafe field

    /******************************************************/

    param = _action_instruction->Action_getParam();
    timeout = _action_instruction->Action_getTimeOut();
    terminator = _action_instruction->Action_getTerminator();

    /******************************************************/

    type = "w";
    height = param;
    failsafe = terminatorToFailsafe(terminator);

    /******************************************************/

    _emitter << YAML::BeginMap;
    _emitter << YAML::Key << "type" << YAML::Value << type;
    _emitter << YAML::Key << "height" << YAML::Value << height;
    _emitter << YAML::Key << "timeout" << YAML::Value << timeout;
    _emitter << YAML::Key << "failsafe" << YAML::Value << failsafe;
    _emitter << YAML::EndMap;

    return true;
}
