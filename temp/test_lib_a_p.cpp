#include "drone-automation-lib.h"

int main()
{
    auto *client = new Communication::MQTT::Publisher(DEFAULT_SERVER_ADDRESS, "test_a_p", "test/topic");

    if (!client->connect())
        return -1;

    for (int i = 0; i < 100; i++)
    {
        std::string message = "Message #" + std::to_string(i);
        client->publish(message);
        std::cout << "[x] Send: " << message << std::endl;
        System::sleep_msecs(0.1);
    }

    client->disconnect();
    delete client;

    return 0;
}