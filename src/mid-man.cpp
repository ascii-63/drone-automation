#include "std_msgs/String.h"

#include <unistd.h>

#include "drone-automation-lib.h"

std::string mission_file; // Path to .json mission file

/* Lauch all requirement package in the bash file */
inline void requirementInit()
{
    std::string cmd = "bash";
    std::vector<std::string> argv;

    std::string path_to_bash_scripts = get_current_dir_name();
    path_to_bash_scripts = path_to_bash_scripts + "/../bash/requirement.sh";

    System::runCommand_system(cmd, argv);
    cmd.clear();
    argv.clear();
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

// Peripherals check
bool peripheralsCheck()
{
    /* PENDING */
    return true;
}

/* This function performs the process of sending and receiving decisions between UAV and GCS
 */
bool confirmProcess()
{
    /* PENDING */
    return true;
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
    vector3 this_home_gps; // Home GPS in the Init Instruction
    mission.sequence_istructions[0]->Init_getHomePosition(this_home_gps);

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

int main()
{
    return 0;
}