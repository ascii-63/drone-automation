#include "./control_lib.h"

PeripheralsStatus::PeripheralsStatus()
{
}

PeripheralsStatus::PeripheralsStatus(const ros::NodeHandle &_nh, std::string _id, std::vector<int> _used) : nh(_nh),
                                                                                                            mission_id(_id)
{
    FLIR_sub = nh.subscribe("/camera/image", 5, &PeripheralsStatus::FLIR_CallBack, this);
    D455_sub = nh.subscribe("/d400/color/image_raw", 5, &PeripheralsStatus::D455_CallBack, this);
    FCU_STATE_sub = nh.subscribe("/mavros/state", 5, &PeripheralsStatus::FCU_STATE_CallBack, this);
    FCU_IMU_sub = nh.subscribe("/mavros/imu/data", 5, &PeripheralsStatus::FCU_IMU_CallBack, this);
    FCU_ODOM_sub = nh.subscribe("/mavros/odometry/in", 5, &PeripheralsStatus::FCU_ODOM_CallBack, this);
    FCU_MAG_sub = nh.subscribe("/mavros/imu/mag", 5, &PeripheralsStatus::FCU_MAG_CallBack, this);
    FCU_PRES_sub = nh.subscribe("/mavros/imu/static_pressure", 5, &PeripheralsStatus::FCU_PRES_CallBack, this);
    FCU_BAT_sub = nh.subscribe("/mavros/battery", 5, &PeripheralsStatus::FCU_BAT_CallBack, this);
    FCU_GPS_sub = nh.subscribe("/mavros/gps_input/gps_input", 5, &PeripheralsStatus::FCU_GPS_CallBack, this);

    status_vec.resize(NUM_OF_DEVICES, false);
    used_vec.resize(NUM_OF_DEVICES, true);
    callback_vec.resize(NUM_OF_DEVICES, false);
    used_vec[0] = false;
    used_vec[1] = false;
    for (auto idx : _used)
        used_vec[idx] = true;

    d455_image_path = IMAGE_DIR_PATH;
    d455_image_path = d455_image_path + mission_id + "-d455.png";
    flir_image_path = IMAGE_DIR_PATH;
    flir_image_path = flir_image_path + mission_id + "-flir.png";
}

PeripheralsStatus::~PeripheralsStatus()
{
}

/***********************************************************************/

void PeripheralsStatus::FLIR_CallBack(const wfov_camera_msgs::WFOVImage::ConstPtr &msg)
{
    callback_vec[DEVICE::FLIR] = true;
    const sensor_msgs::Image &ros_image = msg->image;

    if (!FLIR_exist(msg))
    {
        status_vec[DEVICE::FLIR] = PERIPHERAL_STATUS::INACTIVE;
        return;
    }

    //////////////////////////////////

    status_vec[DEVICE::FLIR] = PERIPHERAL_STATUS::ACTIVE;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(ros_image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    cv::Mat cv_image = cv_ptr->image;
    bool imwrite_status = false;
    try
    {
        imwrite_status = cv::imwrite(flir_image_path, cv_image);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void PeripheralsStatus::D455_CallBack(const sensor_msgs::Image::ConstPtr &msg)
{
    callback_vec[DEVICE::D455] = true;
    const sensor_msgs::Image &ros_image = *msg;

    if (!D455_exist(msg))
    {
        status_vec[DEVICE::D455] = PERIPHERAL_STATUS::INACTIVE;
        return;
    }

    //////////////////////////////////

    status_vec[DEVICE::D455] = PERIPHERAL_STATUS::ACTIVE;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(ros_image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    cv::Mat cv_image = cv_ptr->image;
    bool imwrite_status = false;
    try
    {
        imwrite_status = cv::imwrite(d455_image_path, cv_image);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void PeripheralsStatus::FCU_STATE_CallBack(const mavros_msgs::State::ConstPtr &msg)
{
    callback_vec[DEVICE::FCU_STATE] = true;

    if (!FCU_STATE_exist(msg))
        status_vec[DEVICE::FCU_STATE] = PERIPHERAL_STATUS::INACTIVE;
    else
        status_vec[DEVICE::FCU_STATE] = PERIPHERAL_STATUS::ACTIVE;
}

void PeripheralsStatus::FCU_IMU_CallBack(const sensor_msgs::Imu::ConstPtr &msg)
{
    callback_vec[DEVICE::FCU_IMU] = true;

    if (!FCU_IMU_exist(msg))
        status_vec[DEVICE::FCU_IMU] = PERIPHERAL_STATUS::INACTIVE;
    else
        status_vec[DEVICE::FCU_IMU] = PERIPHERAL_STATUS::ACTIVE;
}

void PeripheralsStatus::FCU_ODOM_CallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
    callback_vec[DEVICE::FCU_ODOM] = true;

    if (!FCU_ODOM_exist(msg))
        status_vec[DEVICE::FCU_ODOM] = PERIPHERAL_STATUS::INACTIVE;
    else
        status_vec[DEVICE::FCU_ODOM] = PERIPHERAL_STATUS::ACTIVE;
}

void PeripheralsStatus::FCU_MAG_CallBack(const sensor_msgs::MagneticField::ConstPtr &msg)
{
    callback_vec[DEVICE::FCU_MAG] = true;

    if (!FCU_MAG_exist(msg))
        status_vec[DEVICE::FCU_MAG] = PERIPHERAL_STATUS::INACTIVE;
    else
        status_vec[DEVICE::FCU_MAG] = PERIPHERAL_STATUS::ACTIVE;
}

void PeripheralsStatus::FCU_PRES_CallBack(const sensor_msgs::FluidPressure::ConstPtr &msg)
{
    callback_vec[DEVICE::FCU_PRES] = true;

    if (!FCU_PRES_exist(msg))
        status_vec[DEVICE::FCU_PRES] = PERIPHERAL_STATUS::INACTIVE;
    else
        status_vec[DEVICE::FCU_PRES] = PERIPHERAL_STATUS::ACTIVE;
}

void PeripheralsStatus::FCU_BAT_CallBack(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    callback_vec[DEVICE::FCU_BAT] = true;

    if (!FCU_BAT_exist(msg))
        status_vec[DEVICE::FCU_BAT] = PERIPHERAL_STATUS::INACTIVE;
    else
        status_vec[DEVICE::FCU_BAT] = PERIPHERAL_STATUS::ACTIVE;
}

void PeripheralsStatus::FCU_GPS_CallBack(const mavros_msgs::GPSINPUT::ConstPtr &msg)
{
    callback_vec[DEVICE::FCU_GPS] = true;

    if (!FCU_GPS_exist(msg))
        status_vec[DEVICE::FCU_GPS] = PERIPHERAL_STATUS::INACTIVE;
    else
        status_vec[DEVICE::FCU_GPS] = PERIPHERAL_STATUS::ACTIVE;
}

/***********************************************************************/

bool PeripheralsStatus::FLIR_exist(const wfov_camera_msgs::WFOVImage::ConstPtr &msg)
{
    const sensor_msgs::Image &ros_image = msg->image;
    return (ros_image.data.size() > 0) ? true : false;
}

bool PeripheralsStatus::D455_exist(const sensor_msgs::Image::ConstPtr &msg)
{
    const sensor_msgs::Image &ros_image = *msg;
    return (ros_image.data.size() > 0) ? true : false;
}

bool PeripheralsStatus::FCU_STATE_exist(const mavros_msgs::State::ConstPtr &msg)
{
    const mavros_msgs::State &state = *msg;
    std::string mode = msg->mode;
    MAV_STATE = mode;
    return !mode.empty();
}

bool PeripheralsStatus::FCU_IMU_exist(const sensor_msgs::Imu::ConstPtr &msg)
{

    const geometry_msgs::Quaternion &orientation = msg->orientation;
    const geometry_msgs::Vector3 &angular_velocity = msg->angular_velocity;
    const geometry_msgs::Vector3 &linear_acceleration = msg->linear_acceleration;

    bool orientation_check = (orientation.x != 0 || orientation.y != 0 || orientation.z != 0 || orientation.w != 0);
    bool angular_velocity_check = (angular_velocity.x != 0 || angular_velocity.y != 0 || angular_velocity.z != 0);
    bool linear_acceleration_check = (linear_acceleration.x != 0 || linear_acceleration.y != 0 || linear_acceleration.z != 0);

    return (orientation_check && angular_velocity_check && linear_acceleration_check);
}

bool PeripheralsStatus::FCU_ODOM_exist(const nav_msgs::Odometry::ConstPtr &msg)
{
    const geometry_msgs::PoseWithCovariance &pose_with_convariance = msg->pose;
    const geometry_msgs::Point &point = msg->pose.pose.position;

    return (point.x != 0 && point.y != 0 && point.z != 0);
}

bool PeripheralsStatus::FCU_MAG_exist(const sensor_msgs::MagneticField::ConstPtr &msg)
{
    const geometry_msgs::Vector3 &magnetic_field = msg->magnetic_field;
    return (magnetic_field.x != 0 || magnetic_field.y != 0 || magnetic_field.z != 0);
}

bool PeripheralsStatus::FCU_PRES_exist(const sensor_msgs::FluidPressure::ConstPtr &msg)
{
    const sensor_msgs::FluidPressure &fluid_pressure = *msg;
    return (fluid_pressure.fluid_pressure > 0);
}

bool PeripheralsStatus::FCU_BAT_exist(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    const sensor_msgs::BatteryState &battery_state = *msg;
    bool voltage_check = (battery_state.voltage > min_voltage && battery_state.voltage < max_voltage);
    bool percentage_check = (battery_state.percentage > min_percentage && battery_state.percentage < max_percentage);

    return (voltage_check && percentage_check);
}

bool PeripheralsStatus::FCU_GPS_exist(const mavros_msgs::GPSINPUT::ConstPtr &msg)
{
    const mavros_msgs::GPSINPUT &gps = *msg;
    bool GPS_check = (gps.lat != 0 && gps.lon != 0 && gps.alt >= 0);
    bool DOP_check = (gps.hdop < GPS_max_HDOP && gps.vdop < GPS_max_VDOP);
    bool accuracy_check = (gps.horiz_accuracy < GPS_max_horiz_accuracy && gps.vert_accuracy < GPS_max_vert_accuracy);

    return (GPS_check && DOP_check && accuracy_check);
}

/***********************************************************************/

void PeripheralsStatus::callBack_exist()
{
    for (int i = 0; i < NUM_OF_DEVICES; i++)
    {
        if (!callback_vec[i] && used_vec[i])
            status_vec[i] = PERIPHERAL_STATUS::INACTIVE;
    }
}

std::string PeripheralsStatus::getStatus()
{
    std::string result;
    for (int i = 0; i < status_vec.size(); i++)
    {
        result += std::to_string(status_vec[i]);
        if (i != status_vec.size() - 1)
            result += " ";
    }
    return result;
}

void PeripheralsStatus::debug()
{
    std::cout << std::endl;
    std::cout << "\nCALLBACK:";
    for (auto idx : callback_vec)
        std::cout << " " << idx;
    std::cout << "\n    USED:";
    for (auto idx : used_vec)
        std::cout << " " << idx;
    std::cout << "\n  STATUS:";
    for (auto idx : status_vec)
        std::cout << " " << idx;
}

////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                            //
//                                                                                            //
//                                                                                            //
//                                                                                            //
//                                                                                            //
//                                                                                            //
//                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////

void command_sys(const std::string &_command)
{
    std::system(_command.c_str());
}

LogsHandler::LogsHandler()
{
}

LogsHandler::LogsHandler(const ros::NodeHandle &_nh) : nh(_nh)
{
    ROSOUT_sub = nh.subscribe("/rosout", 1000, &LogsHandler::ROSOUT_callBack, this);
    EMB_sub = nh.subscribe("/emb", 1000, &LogsHandler::EMB_callBack, this);
}

LogsHandler::~LogsHandler()
{
}

void LogsHandler::ROSOUT_callBack(const rosgraph_msgs::Log::ConstPtr &_log)
{
    if (_log->level == rosgraph_msgs::Log::ERROR || _log->level == rosgraph_msgs::Log::FATAL)
    {
        std::string message = _log->msg;
        sendToComm(message);
    }
}

void LogsHandler::EMB_callBack(const std_msgs::String::ConstPtr &_msg)
{
    std::string message = _msg->data;
    sendToComm(message);
}

void LogsHandler::debug()
{
}

void LogsHandler::sendToComm(const std::string _msg)
{
    std::stringstream ss;
    ss << "echo \"" << _msg << "\" | nc -q 1 localhost " << DEFAULT_COMM_MSG_PORT;

    std::thread thr(command_sys, ss.str()); // Create a new thread to run the command
    thr.detach();                           // Detach the thread to run independently
}

////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                            //
//                                                                                            //
//                                                                                            //
//                                                                                            //
//                                                                                            //
//                                                                                            //
//                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////

void my_callback::connection_lost(const std::string &cause)
{
    std::cout << "\nConnection lost" << std::endl;
    if (!cause.empty())
        std::cout << "\tcause: " << cause << std::endl;
}

/************************************************************************/

MQTT::Publisher::Publisher() {}

MQTT::Publisher::Publisher(const std::string &_server_addr,
                           const std::string &_client_id,
                           const std::string _topic) : server_address(_server_addr),
                                                       client_id(_client_id),
                                                       topic(_topic)
{
}

MQTT::Publisher::~Publisher() {}

bool MQTT::Publisher::connect()
{
    client = new mqtt::async_client(server_address, client_id, DEFAULT_PERSIST_DIR);
    try
    {
        mqtt::token_ptr connection_token = client->connect(connection_options);
        connection_token->wait();
    }
    catch (const mqtt::exception &exc)
    {
        std::cerr << exc.what() << '\n';
        return false;
    }
    return true;
}

void MQTT::Publisher::publish(const std::string &_message)
{
    try
    {
        mqtt::message_ptr pub_msg = mqtt::make_message(topic, _message);
        pub_msg->set_qos(DEFAULT_QOS);
        client->publish(pub_msg)->wait_for(DEFAULT_TIMEOUT);
    }
    catch (const mqtt::exception &exc)
    {
        std::cerr << exc.what() << std::endl
                  << "Message droped: "
                  << _message
                  << std::endl;
    }
}

void MQTT::Publisher::disconnect()
{
    try
    {
        client->disconnect()->wait();
    }
    catch (const mqtt::exception &exc)
    {
        std::cerr << exc.what() << '\n';
    }
}

//////////////////////////////////////////////

MQTT::Consumer::Consumer() {}

MQTT::Consumer::Consumer(const std::string &_server_addr,
                         const std::string &_client_id,
                         const std::string _topic) : server_address(_server_addr),
                                                     client_id(_client_id),
                                                     topic(_topic)
{
}

MQTT::Consumer::~Consumer() {}

bool MQTT::Consumer::connect()
{
    client = new mqtt::async_client(server_address, client_id);
    try
    {
        // Start consumer before connecting to make sure to not miss messages
        client->start_consuming();

        // Connect to the server
        auto token = client->connect(connection_options);

        // Getting the connect response will block waiting for the connection to complete
        auto response = token->get_connect_response();

        // If there is no session present, then we need to subscribe, but if
        // there is a session, then the server remembers us and our
        // subscriptions.
        if (!response.is_session_present())
            client->subscribe(topic, DEFAULT_QOS)->wait();
    }
    catch (const mqtt::exception &exc)
    {
        std::cerr << exc.what() << '\n';
        return false;
    }

    return true;
}

std::string MQTT::Consumer::consume()
{
    try
    {
        auto message = client->consume_message();
        return message->to_string();
    }
    catch (const std::system_error &e)
    {
        std::cout << "Caught system_error with code ["
                  << e.code() << "] meaning ["
                  << e.what() << "]\n";
    }
    return ERROR_CONSUME_MESSAGE;
}

void MQTT::Consumer::disconnect()
{
    try
    {
        client->disconnect()->wait();
    }
    catch (const mqtt::exception &exc)
    {
        std::cerr << exc.what() << '\n';
    }
}