#include "control_lib.h"
#include <unistd.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "log_node");
    ros::NodeHandle nh("");
    LogsHandler *logs_handler = new LogsHandler(nh);

    while (ros::ok())
    {
        ros::spinOnce();            // Process any incoming messages
        ros::Duration(0.1).sleep(); // Sleep to avoid consuming too much CPU (0.1 second)
    }

    return 0;
}