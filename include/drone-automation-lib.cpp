#include "./drone-automation-lib.h"

inline void System::command_sys(const std::string &_command)
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

inline void System::threadSleeper(const int _time)
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

inline bool System::jsonParsingToObject(const std::string _path_to_json_file, MissionRequest &_mission)
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
    case Peripheral::PERIPHERAL_CAM_DOWNWARD:
        ss << _drone_id << "-" << DEFAULT_FLIR_PNG << "\"";
        break;

    case Peripheral::PERIPHERAL_CAM_FORWARD:
        ss << _drone_id << "-" << DEFAULT_D455_PNG << "\"";
        break;

    default:
        Communication::netcat::sendMessage_echo_netcat("[ WARN] Invalid device id, this image will not be send.", DEFAULT_COMM_MSG_PORT);
        System::sleepLessThanASecond(0.1);
        break;
    }
    curl_argv.push_back(ss.str());

    std::stringstream ss1;
    ss1 << "http://localhost:" << DEFAULT_COMM_IMAGE_PORT << "/upload";
    curl_argv.push_back(ss1.str());

    System::runCommand_system(curl_cmd, curl_argv);
}

inline std::string System::DEVICE_enumToString(const int _num)
{
    switch (_num)
    {
    case DEVICE::FLIR:
        return "FLIR";
    case DEVICE::D455:
        return "D455";
    case DEVICE::T265:
        return "T265";
    case DEVICE::LIDAR:
        return "LIDAR";
    case DEVICE::TERABEE:
        return "Range Finder";
    case DEVICE::RTK:
        return "RTK";
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
    case DEVICE::FCU_MOTOR:
        return "MAV Motor";
    case DEVICE::FCU_AHRS:
        return "MAV Accelerometer";
    case DEVICE::FCU_TELE:
        return "MAV Telemetry";
    case DEVICE::FCU_GPS:
        return "MAV GPS";
    default:
        return "Unknown";
    }
}

inline std::string System::PERIPHERAL_STATUS_enumToString(const int _num)
{
    switch (_num)
    {
    case PERIPHERAL_STATUS::UNSPECIFIED:
        return "UNSPECIFIED";
    case PERIPHERAL_STATUS::ACTIVE:
        return "ACTIVE";
    case PERIPHERAL_STATUS::INACTIVE:
        return "INACTIVE";
    case PERIPHERAL_STATUS::WAITING_FOR_ACTIVE:
        return "WAITING_FOR_ACTIVE";
    case PERIPHERAL_STATUS::NOT_FOUND:
        return "NOT_FOUND";
    default:
        return "Unknown";
    }
}

inline bool System::camFlagChecker(const std::vector<std::string> &_flag_vector)
{
    for (auto flag : _flag_vector)
    {
        if (flag != FLAG_CAM_ALLOW)
            return false;
    }
    return true;
}

inline void System::sleepLessThanASecond(const float _time)
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
        System::sleepLessThanASecond(0.1);
        _flag_handle = "";
        return false;
    }

    _flag_handle = line;
    return true;
}

inline void System::renameWithID(const std::string &_path_to_dir, const std::string &_file_name)
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

inline bool System::seqControllerLauching()
{
    try
    {
        std::string cmd = "rosrun";
        std::vector<std::string> argv;
        argv.push_back("sequence_controller");
        argv.push_back("parser");
        argv.push_back("&");
        System::runCommand_system(cmd, argv);
    }
    catch (const std::exception &e)
    {
        return false;
    }
    return true;
}

/******************************************************************************/

inline void Communication::netcat::sendMessage_echo_netcat(const std::string &_message, const int _port)
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

/******************************************************************************/

inline vector3 Drone::GPStoUTM(const vector3 &_gps)
{
    double UTM_X, UTM_Y;
    double GPS_X, GPS_Y;

    GPS_X = _gps[0];
    GPS_Y = _gps[1];
    double ALT = _gps[2];

    LatLonToUTMXY(GPS_X, GPS_Y, 48, UTM_X, UTM_Y);

    vector3 result;
    result.push_back(UTM_X);
    result.push_back(UTM_Y);
    result.push_back(ALT);

    return result;
}

inline vector3 Drone::UTMtoLocal(const vector3 &_home_utm, const vector3 &_point_utm)
{
    vector3 result;
    result.push_back(_point_utm[0] - _home_utm[0]);
    result.push_back(_point_utm[1] - _home_utm[1]);
    result.push_back(_point_utm[2] - _home_utm[2]);

    return result;
}