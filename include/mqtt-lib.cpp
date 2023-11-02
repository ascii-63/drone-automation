#include "./mqtt-lib.h"

void my_callback::connection_lost(const std::string &cause)
{
    std::cout << "\nConnection lost" << std::endl;
    if (!cause.empty())
        std::cout << "\tcause: " << cause << std::endl;
}