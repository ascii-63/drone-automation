#ifndef CONTROL_LIB_H
#define CONTROL_LIB_H

#include <ros/ros.h>
#include <wfov_camera_msgs/WFOVImage.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/GPSRTK.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/GPSINPUT.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <vector>
#include <string>
#include <iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>
#include "mqtt/async_client.h"

/*****************************************************************************/

const float min_voltage = 0;
const float max_voltage = 30;
const float min_percentage = 0;
const float max_percentage = 1;

const float GPS_max_horiz_accuracy = 0.1;
const float GPS_max_vert_accuracy = 0.1;
const float GPS_max_HDOP = 1;
const float GPS_max_VDOP = 1;

#define NUM_OF_DEVICES 9
#define IMAGE_DIR_PATH "~/image/"

enum PERIPHERAL_STATUS : int
{
    UNSPECIFIED = -1, // The peripheral remains unused.
    ACTIVE,           // Obtain the ACTIVE status once the message has been present for a continuous duration of 5 seconds.
    INACTIVE          // Retrieve the INACTIVE status when the message has been absent for a consecutive duration of 1 second.
};

enum DEVICE : int
{
    FLIR = 0,  // FLIR
    D455,      // D455
    FCU_STATE, // MAV State
    FCU_IMU,   // MAV IMU
    FCU_ODOM,  // MAV Odometry
    FCU_MAG,   // MAV Magnetometer
    FCU_PRES,  // MAV Absolute Pressure
    FCU_BAT,   // MAV Battery
    FCU_GPS    // MAV GPS
};

/********************************************/

class PeripheralsStatus
{
public:
    ros::NodeHandle nh;
    // ros::NodeHandle nh_private_;

    ros::Subscriber FLIR_sub;
    ros::Subscriber D455_sub;
    ros::Subscriber FCU_STATE_sub;
    ros::Subscriber FCU_IMU_sub;
    ros::Subscriber FCU_ODOM_sub;
    ros::Subscriber FCU_MAG_sub;
    ros::Subscriber FCU_PRES_sub;
    ros::Subscriber FCU_BAT_sub;
    ros::Subscriber FCU_GPS_sub;

    std::string MAV_STATE;
    std::vector<bool> callback_vec;
    std::vector<int> status_vec;
    std::vector<bool> used_vec;

    std::string mission_id;
    std::string flir_image_path;
    std::string d455_image_path;

public:
    PeripheralsStatus();
    PeripheralsStatus(const ros::NodeHandle &_nh, std::string _id, std::vector<int> _used); // FULL
    ~PeripheralsStatus();

    ////////////////////////////

    // If the callBack function of a subscriber is not be executed, change the current_status of it to NOT_FOUND
    void callBack_exist();

    std::string getStatus();

    ////////////////////////////

    void FLIR_CallBack(const wfov_camera_msgs::WFOVImage::ConstPtr &msg);

    void D455_CallBack(const sensor_msgs::Image::ConstPtr &msg);

    void FCU_STATE_CallBack(const mavros_msgs::State::ConstPtr &msg);

    void FCU_IMU_CallBack(const sensor_msgs::Imu::ConstPtr &msg);

    void FCU_ODOM_CallBack(const nav_msgs::Odometry::ConstPtr &msg);

    void FCU_MAG_CallBack(const sensor_msgs::MagneticField::ConstPtr &msg);

    void FCU_PRES_CallBack(const sensor_msgs::FluidPressure::ConstPtr &msg);

    void FCU_BAT_CallBack(const sensor_msgs::BatteryState::ConstPtr &msg);

    void FCU_GPS_CallBack(const mavros_msgs::GPSINPUT::ConstPtr &msg);

    ////////////////////////////

    bool FLIR_exist(const wfov_camera_msgs::WFOVImage::ConstPtr &msg);

    bool D455_exist(const sensor_msgs::Image::ConstPtr &msg);

    bool FCU_STATE_exist(const mavros_msgs::State::ConstPtr &msg);

    bool FCU_IMU_exist(const sensor_msgs::Imu::ConstPtr &msg);

    bool FCU_ODOM_exist(const nav_msgs::Odometry::ConstPtr &msg);

    bool FCU_MAG_exist(const sensor_msgs::MagneticField::ConstPtr &msg);

    bool FCU_PRES_exist(const sensor_msgs::FluidPressure::ConstPtr &msg);

    bool FCU_BAT_exist(const sensor_msgs::BatteryState::ConstPtr &msg);

    bool FCU_GPS_exist(const mavros_msgs::GPSINPUT::ConstPtr &msg);
};

/*****************************************************************************/

const std::string DEFAULT_SERVER_ADDRESS = "tcp://localhost:1883";
const std::string DEFAULT_PERSIST_DIR = "./persist";

const int DEFAULT_QOS = 1;
const auto DEFAULT_TIMEOUT = std::chrono::seconds(10);

// A callback class for use with the main MQTT client.
class my_callback : public virtual mqtt::callback
{
public:
    void connection_lost(const std::string &cause) override;
};

auto connection_options = mqtt::connect_options_builder()
                              .clean_session(true)
                              // .clean_session(false)
                              .finalize();

/*******************************************************************************/

#define MQTT_DEVICE_LIST_TOPIC "/device/list"
#define MQTT_DEVICE_STATUS_TOPIC "/device/status"
#define MQTT_MAV_STATE_TOPIC "/mav_state"

namespace MQTT
{
    const std::string ERROR_CONSUME_MESSAGE = "ERROR_CONSUME_MESSAGE";

    class Publisher
    {
    public:
        Publisher();
        Publisher(const std::string &_server_addr,
                  const std::string &_client_id,
                  const std::string _topic);
        ~Publisher();

        bool connect();
        void publish(const std::string &_message);
        void disconnect();

    public:
        std::string server_address;
        std::string client_id;
        std::string topic;

        mqtt::async_client *client;
    };

    class Consumer
    {
    public:
        Consumer();
        Consumer(const std::string &_server_addr,
                 const std::string &_client_id,
                 const std::string _topic);
        ~Consumer();

        bool connect();
        std::string consume();
        void disconnect();

    public:
        std::string server_address;
        std::string client_id;
        std::string topic;

        mqtt::async_client *client;
    };
};

#endif