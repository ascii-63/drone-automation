#include "drone-automation-lib.h"

int main()
{
    auto *client = new Communication::MQTT::Consumer(DEFAULT_SERVER_ADDRESS, "test_a_c", "test/topic");

    if (!client->connect())
        return -1;

    for (;;)
    {
        std::string message = client->consume();
        std::cout << message << std::endl;
    }

    client->disconnect();
    delete client;

    return 0;
}