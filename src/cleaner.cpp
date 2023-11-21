#include "drone-automation-lib.h"

std::vector<std::string> pid_kill_list;

std::vector<std::string> getPIDList(const std::string _process)
{
    std::vector<std::string> result;

    std::string command = "pgrep -f " + _process;
    FILE *pipe = popen(command.c_str(), "r");
    if (!pipe)
        return result;

    char buffer[2048];
    while (fgets(buffer, 2048, pipe))
    {
        std::string temp_string = std::string(buffer);
        temp_string.pop_back();
        result.push_back(temp_string);
    }
    result.pop_back(); // The last PID is the PID of "pgrep" command.

    return result;
}

void addToKillList(const std::string _process)
{
    std::vector<std::string> pid_vec;
    pid_vec = getPIDList(_process);
    pid_kill_list.reserve(pid_kill_list.size() + pid_vec.size());
    pid_kill_list.insert(pid_kill_list.end(), pid_vec.begin(), pid_vec.end());
}

void killProcesses()
{
    addToKillList("rosmaster");
    addToKillList("mavros");
    addToKillList("px4");
    addToKillList("gazebo");
    addToKillList("spinnaker");
    addToKillList("realsense2");
    addToKillList("peripherals_node");
    addToKillList("log_node");
    addToKillList("control_pkg");
    addToKillList("requirement");
    addToKillList("seq_controller");
    addToKillList("mid-man");
    addToKillList("\"nc -l -k -p\"");
    addToKillList("\"nc -l -p\"");

    std::cout << std::endl
              << "PID List:";
    std::string cmd;
    for (auto pid : pid_kill_list)
    {
        cmd = "kill -9 " + pid;
        std::system(cmd.c_str());
        System::sleep_msecs(0.1);
        std::cout << " " << pid;
    }

    System::sleep_msecs(0.1);
}

void clearDir(const std::string &_dir)
{
    std::string cmd;
    std::vector<std::string> argv;
    cmd = "cd";
    argv.push_back(_dir);
    argv.push_back("&&");
    argv.push_back("rm -rf *");

    System::runCommand_system(cmd, argv);
    System::sleep_msecs(0.1);
}

int main()
{
    killProcesses();
    clearDir(DEFAULT_IMAGE_DIR_PATH);
    clearDir(DEFAULT_MISSION_DIR_PATH);

    std::cout << std::endl
              << "************************" << std::endl
              << "*  Cleaner completed!  *" << std::endl
              << "************************" << std::endl
              << std::endl;
    return 0;
}