// #include "std_msgs/String.h"
#include "drone-automation-lib.h"

std::string mission_file; // Path to .json mission file
bool flir_exist = false;  // True if FLIR Camera is ACTIVE
bool d455_exist = false;  // True if D455 Camera is ACTIVE

Communication::MQTT::Publisher *device_pub = new Communication::MQTT::Publisher(DEFAULT_SERVER_ADDRESS,
                                                                                "mm_device_pub_client",
                                                                                MQTT_DEVICE_LIST_TOPIC);
Communication::MQTT::Consumer *device_sub = new Communication::MQTT::Consumer(DEFAULT_SERVER_ADDRESS,
                                                                              "mm_device_sub_client",
                                                                              MQTT_DEVICE_STATUS_TOPIC);
Communication::MQTT::Consumer *mav_state_sub = new Communication::MQTT::Consumer(DEFAULT_SERVER_ADDRESS,
                                                                                 "mm_mav_state_sub_client",
                                                                                 MQTT_MAV_STATE_TOPIC);

/* Lauch all requirement package in the bash file,
Connect to the MQTT Server
*/
inline bool requirementInit()
{
    std::string cmd = "bash";
    std::vector<std::string> argv;

    // std::string path_to_bash_scripts = get_current_dir_name();
    // path_to_bash_scripts = path_to_bash_scripts + "/../bash/requirement.sh";
    std::string path_to_bash_scripts = "/home/pino/drone-automation/bash/requirement.sh";

    try
    {

        // System::runCommand_system(cmd, argv);
        cmd.clear();
        argv.clear();
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        Communication::netcat::sendMessage_echo_netcat("[ERROR] Package lauching error.", DEFAULT_COMM_MSG_PORT);
        return false;
    }

    ////////////////////////////

    if (!device_pub->connect() || !device_sub->connect() || !mav_state_sub->connect())
    {
        Communication::netcat::sendMessage_echo_netcat("MQTT Startup error.", DEFAULT_COMM_MSG_PORT);
        return false;
    }

    return true;
}

// Search for new misison file
std::string missionFileWatcher()
{
    Communication::netcat::sendMessage_echo_netcat("[ INFO] Searching for mission file...", DEFAULT_COMM_MSG_PORT);

    while (true)
    {
        std::string file = System::getMissionFile(DEFAULT_MISSION_DIR_PATH);
        size_t found = file.find(".json");
        if (found != std::string::npos)
        {
            Communication::netcat::sendMessage_echo_netcat("[ INFO] Mission file has been found", DEFAULT_COMM_MSG_PORT);

            std::string found_path = DEFAULT_MISSION_DIR_PATH;
            found_path = found_path + file;
            return found_path;
        }
    }
}

// Send MAV state and Device status to communation service in human-readable format
void sendDeviceStatus(const std::string &_mav_state, const std::string &_status)
{
    std::stringstream ss(_status);
    std::vector<int> device_status;
    int status_num;
    while (ss >> status_num)
        device_status.push_back(status_num);

    ss.clear();
    ss << std::endl
       << "========================================" << std::endl; // 40 "="
    ss << std::setw(24) << "MAV_STATE: " << _mav_state << std::endl
       << std::endl;
    for (int i = 0; i < device_status.size(); i++)
        ss << std::setw(22) << System::DEVICE_enumToString(i) << ": " << System::PERIPHERAL_STATUS_enumToString(device_status[i]) << std::endl;
    ss << "========================================" << std::endl; // 40 "="

    Communication::netcat::sendMessage_echo_netcat(ss.str(), DEFAULT_COMM_MSG_PORT);
}

/* Peripherals check:
1. Send device ID to control_pkg
2. Get device status from control_pkg
3. Send image base on device status
*/
bool peripheralsCheck()
{
    std::vector<int> device_list;
    mission.sequence_istructions[0]->Init_getPeripherals(device_list);
    std::stringstream ss;
    ss << mission_id << " ";
    for (auto dev : device_list)
        ss << dev << " ";
    std::string msg = ss.str();
    msg.pop_back();

    device_pub->publish(msg);

    //////////////////////////////////

    std::string status_str = device_sub->consume();
    if (status_str == Communication::MQTT::ERROR_CONSUME_MESSAGE)
    {
        Communication::netcat::sendMessage_echo_netcat("[ERROR] Bad consume.", DEFAULT_COMM_MSG_PORT);
        return false;
    }

    std::string mav_state = mav_state_sub->consume();
    if (mav_state == Communication::MQTT::ERROR_CONSUME_MESSAGE)
    {
        Communication::netcat::sendMessage_echo_netcat("[ERROR] Bad consume.", DEFAULT_COMM_MSG_PORT);
        return false;
    }

    sendDeviceStatus(mav_state, status_str);
    std::vector<int> status_vec;
    std::stringstream ss1(status_str);
    int status_num;
    while (ss1 >> status_num)
    {
        status_vec.push_back(status_num);
        std::cout << "DEVICE STATUS: " << status_num << std::endl;
    }

    //////////////////////////////////

    int FLIR_status = status_vec[DEVICE::FLIR], D455_status = status_vec[DEVICE::D455];
    if (FLIR_status == PERIPHERAL_STATUS::ACTIVE)
    {
        flir_exist = true;
        System::sendImage(Peripheral::PERIPHERAL_CAM_FLIR, mission_id);
        Communication::netcat::sendMessage_echo_netcat("[ INFO] FLIR image sended.", DEFAULT_COMM_MSG_PORT);
    }
    if (D455_status == PERIPHERAL_STATUS::ACTIVE)
    {
        flir_exist = true;
        System::sendImage(Peripheral::PERIPHERAL_CAM_D455, mission_id);
        Communication::netcat::sendMessage_echo_netcat("[ INFO] D455 image sended.", DEFAULT_COMM_MSG_PORT);
    }

    return true;
}

/* This function performs the process of sending and receiving decisions between UAV and GCS:
1. Get image confirm
2. Send request to fly to comm
3. Receive fly confirm from comm
 */
bool confirmProcess()
{
    if (!flir_exist && !d455_exist)
        Communication::netcat::sendMessage_echo_netcat("[ INFO] Skip image confirm FLAGs checking: No image sended.", DEFAULT_COMM_MSG_PORT);
    else
    {
        const int num_img_confirm_req = flir_exist + d455_exist;
        int count = 0;
        while (count < num_img_confirm_req)
        {
            std::string flag;
            bool result = System::getNewestFLAG(DEFAULT_CONTROL_CONFIRM_PORT, flag);
            if (result)
            {
                if (flag == FLAG_CAM_REJECT)
                    return false;
                if (flag == FLAG_CAM_ALLOW)
                    count++;
            }
        }
    }

    ////////////////////////////////////////////

    Communication::netcat::sendMessage_echo_netcat("[ INFO] Waiting for permission to fly. Timeout: 120 seconds.", DEFAULT_COMM_MSG_PORT);

    ////////////////////////////////////////////

    std::string flag;
    while (flag.empty())
    {
        bool result = System::getNewestFLAG(DEFAULT_CONTROL_CONFIRM_PORT, flag);
        if (flag == FLAG_ALLOW_TO_FLY)
            return true;
        if (flag == FLAG_DENY_TO_FLY)
            return false;
    }

    return false;
}

// Init Intruction Execution
bool initInstructionExecution()
{
    /* PENDING */
    Communication::netcat::sendMessage_echo_netcat("[ INFO] Peripherals check starting...", DEFAULT_COMM_MSG_PORT);
    if (!peripheralsCheck())
    {
        Communication::netcat::sendMessage_echo_netcat("[ERROR] Peripherals check failed.", DEFAULT_COMM_MSG_PORT);
        return false;
    }

    Communication::netcat::sendMessage_echo_netcat("[ INFO] Waiting for confirmation from GCS...", DEFAULT_COMM_MSG_PORT);
    if (!confirmProcess())
    {
        Communication::netcat::sendMessage_echo_netcat("[ERROR] Errors when get confirmation from GCS.", DEFAULT_COMM_MSG_PORT);
        return false;
    }

    return true;
}

// Main control
bool missionExecution()
{
    mission_file = missionFileWatcher();
    std::cout << mission_file << std::endl;

    // .json mission file parsing into misison class object
    if (!System::jsonParsingToObject(mission_file, mission))
        return false;

    mission_id = mission.id;
    // vector3 home_gps; // Home GPS in the Init Instruction
    mission.sequence_istructions[0]->Init_getHomePosition(home_gps);

    if (!YAMLConvert::fromMisisonToYAML(DEFAULT_SEQ_YAML_FILE_PATH))
    {
        Communication::netcat::sendMessage_echo_netcat("[ERROR] Cannot transform mission object to YAML.", DEFAULT_COMM_MSG_PORT);
        return false;
    }
    Communication::netcat::sendMessage_echo_netcat("[ INFO] Transform mission object to YAML successful.", DEFAULT_COMM_MSG_PORT);

    Communication::netcat::sendMessage_echo_netcat("[ INFO] Init Instruction starting...", DEFAULT_COMM_MSG_PORT);
    if (!initInstructionExecution())
    {
        Communication::netcat::sendMessage_echo_netcat("[ERROR] Init Instruction return FALSE.", DEFAULT_COMM_MSG_PORT);
        return false;
    }
    Communication::netcat::sendMessage_echo_netcat("[ INFO] Init Instruction completed.", DEFAULT_COMM_MSG_PORT);

    Communication::netcat::sendMessage_echo_netcat("[ INFO] Mission Executing...", DEFAULT_COMM_MSG_PORT);
    if (!System::seqControllerLauching())
    {
        Communication::netcat::sendMessage_echo_netcat("[ERROR] Failed to start sequences_controller package.", DEFAULT_COMM_MSG_PORT);
        return false;
    }

    return true;
}

void cleaner()
{
    device_pub->disconnect();
    device_pub->disconnect();
    mav_state_sub->disconnect();
}

int main()
{
    if (!requirementInit())
        return -1;
    if (!missionExecution())
        return -2;

    while (true)
    {
        std::string status_msg = device_sub->consume();
        std::string mav_state_msg = mav_state_sub->consume();
        sendDeviceStatus(mav_state_msg, status_msg);

        if (flir_exist)
            System::sendImage(DEVICE::FLIR, mission_id);
        if (d455_exist)
            System::sendImage(DEVICE::D455, mission_id);

        System::threadSleeper(DEFAULT_TIME_BETWEEN_2_IMAGE);
    }

    cleaner();

    return 0;
}