#include "drone-automation-lib.h"

MissionRequest misison;

int main()
{
    bool result = jsonParsing::parsing("/home/pino/drone-automation/gcs-comm-service/mission/10-mission.json", mission);
    std::cout << result << std::endl;
}