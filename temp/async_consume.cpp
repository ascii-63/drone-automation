#include <iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>
#include "mqtt/async_client.h"

const std::string SERVER_ADDRESS = "tcp://localhost:1883";
const std::string CLIENT_ID = "async_consume";
const std::string TOPIC = "test_topic";

const int QOS = 1;

int main()
{
    mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);

    auto connOpts = mqtt::connect_options_builder()
                        .clean_session(true)
                        // .clean_session(false)
                        .finalize();

    try
    {
        // Start consumer before connecting to make sure to not miss messages
        client.start_consuming();

        // Connect to the server
        std::cout << "Connecting to the MQTT server..." << std::endl;
        auto token = client.connect(connOpts);

        // Getting the connect response will block waiting for the connection to complete.
        auto rsp = token->get_connect_response();

        // If there is no session present, then we need to subscribe, but if
        // there is a session, then the server remembers us and our
        // subscriptions.
        if (!rsp.is_session_present())
            client.subscribe(TOPIC, QOS)->wait();

        std::cout << "...OK" << std::endl
                  << std::endl;

        std::cout << "Waiting for messages on topic: '" << TOPIC << "'" << std::endl;

        // // Consume messages
        // // This just exits if the client is disconnected.
        // while (true)
        // {
        //     std::cout << "debug 0" << std::endl;
        //     auto msg = client.consume_message();
        //     std::cout << "debug 1" << std::endl;
        //     if (!msg)
        //         break;
        //     std::cout << msg->get_topic() << ": " << msg->to_string() << std::endl;
        // }
        // std::cout << "debug 2" << std::endl;

        auto msg = client.consume_message();
        if (msg)
            std::cout << msg->get_topic() << ": " << msg->to_string() << std::endl;

        // If the code run to here, the client was almost certainly disconnected.
        // But still check it, just to make sure.
        if (client.is_connected())
        {
            std::cout << "\nShutting down and disconnecting from the MQTT server..." << std::endl;
            client.unsubscribe(TOPIC)->wait();
            client.stop_consuming();
            client.disconnect()->wait();
            std::cout << "...OK" << std::endl
                      << std::endl;
        }
        else
        {
            std::cout << "Client was disconnected" << std::endl;
        }
    }
    catch (const mqtt::exception &exc)
    {
        std::cerr << exc.what() << '\n';
    }

    return 0;
}