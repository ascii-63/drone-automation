#include "control_lib.h"
#include <unistd.h>

MQTT::Consumer *device_sub = new MQTT::Consumer(DEFAULT_SERVER_ADDRESS, "ctr_device_sub_client", MQTT_DEVICE_LIST_TOPIC);
MQTT::Publisher *device_pub = new MQTT::Publisher(DEFAULT_SERVER_ADDRESS, "ctr_device_pub_client", MQTT_DEVICE_STATUS_TOPIC);
MQTT::Publisher *mav_state_pub = new MQTT::Publisher(DEFAULT_SERVER_ADDRESS, "ctr_mav_state_pub_client", MQTT_MAV_STATE_TOPIC);

std::string mission_id;
std::vector<int> dev_list;

bool firstRecive()
{
    if (!device_pub->connect() || !device_sub->connect() || !mav_state_pub->connect())
        return false;

    std::string firstMsg = device_sub->consume();
    std::stringstream ss(firstMsg);
    std::vector<int> temp_vec;
    int temp_var;
    while (ss >> temp_var)
        temp_vec.push_back(temp_var);
    mission_id = std::to_string(temp_vec[0]);
    for (int i = 1; i < temp_vec.size(); i++)
        dev_list.push_back(temp_vec[i]);

    return true;
}

int main(int argc, char **argv)
{
    if (!firstRecive())
        return -1;

    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh("");
    PeripheralsStatus *peripheral_status = new PeripheralsStatus(nh, mission_id, dev_list);

    auto start_time = ros::Time::now();
    auto first_status_duration = ros::Duration(10);

    while (ros::ok())
    {
        peripheral_status->callBack_exist();

        auto current_time = ros::Time::now();
        auto release_time = current_time - start_time;
        if (release_time > first_status_duration)
        {
            std::string mav_state_msg = peripheral_status->MAV_STATE;
            std::string status_msg = peripheral_status->getStatus();
            mav_state_pub->publish(mav_state_msg);
            device_pub->publish(status_msg);
        }

        // peripheral_status->debug();

        ros::spinOnce();
        ros::Duration(1).sleep();
    }

    device_sub->disconnect();
    device_pub->disconnect();
    mav_state_pub->disconnect();

    return 0;
}