#ifndef MQTT_LIB_H
#define MQTT_LIB_H

#include <iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>
#include "mqtt/async_client.h"

const std::string DEFAULT_SERVER_ADDRESS = "tcp://localhost:1883";
// const std::string CLIENT_ID = "ansync_publish";
const std::string DEFAULT_PERSIST_DIR = "./persist";

const int DEFAULT_QOS = 1;
const auto DEFAULT_TIMEOUT = std::chrono::seconds(10);

// A callback class for use with the main MQTT client.
class my_callback : public virtual mqtt::callback
{
public:
    void connection_lost(const std::string &cause) override;
};

auto connOpts = mqtt::connect_options_builder()
                    .clean_session(false)
                    .finalize();

#endif