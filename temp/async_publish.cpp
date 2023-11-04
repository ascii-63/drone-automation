#include <iostream>
#include <cstdlib>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include "mqtt/async_client.h"

const std::string DEFAULT_SERVER_ADDRESS = "tcp://localhost:1883";
const std::string CLIENT_ID = "async_publish";
const std::string PERSIST_DIR = "./persist";

const std::string TOPIC = "test_topic";

const char *PAYLOAD1 = "Message #1";

const int QOS = 1;
const auto TIMEOUT = std::chrono::seconds(10);

/*******************************************************************/

// A callback class for use with the main MQTT client.
class callback : public virtual mqtt::callback
{
public:
    void connection_lost(const std::string &cause) override
    {
        std::cout << "\nConnection lost" << std::endl;
        if (!cause.empty())
            std::cout << "\tcause: " << cause << std::endl;
    }
};

int main()
{
    std::string address = DEFAULT_SERVER_ADDRESS;
    std::string client_id = CLIENT_ID;

    std::cout << "Initializing for server '" << address << "'..." << std::endl;
    mqtt::async_client client(address, client_id, PERSIST_DIR);

    callback cb;
    client.set_callback(cb);

    auto connOpts = mqtt::connect_options_builder()
                        .clean_session()
                        .finalize();

    std::cout << "...OK" << std::endl
              << std::endl;

    try
    {
        std::cout << "Connecting..." << std::endl;
        mqtt::token_ptr conntok = client.connect(connOpts);
        std::cout << "Waiting for the connection..." << std::endl;
        conntok->wait();
        std::cout << "...OK" << std::endl
                  << std::endl;

        std::cout << "Sending message..." << std::endl;
        std::string message = "hahahaha";
        mqtt::message_ptr pubmsg = mqtt::make_message(TOPIC, message);
        pubmsg->set_qos(QOS);
        client.publish(pubmsg)->wait_for(TIMEOUT);
        std::cout << "...OK" << std::endl
                  << std::endl;

        // Disconnect
        std::cout << "Disconnecting..." << std::endl;
        client.disconnect()->wait();
        std::cout << "...OK" << std::endl
                  << std::endl;
    }
    catch (const mqtt::exception &exc)
    {
        std::cerr << exc.what() << '\n';
    }

    return 0;
}