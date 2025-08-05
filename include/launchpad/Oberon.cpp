#include "Oberon.hpp"

Oberon::Oberon()
    : Node("oberon")
{
    loadCellsLaunchPadPublisher = this->create_publisher<gs_interfaces::msg::LoadCells>("/oberon/launch_tower/tenso", 10);
    loadCellsParamsPublisher = this->create_publisher<gs_interfaces::msg::LoadCellsParams>("/oberon/launch_tower/tenso_params", 3);
    temperatureLaunchPadPublisher = this->create_publisher<gs_interfaces::msg::Temperature>("/oberon/launch_tower/temperature", 3);
    loadCellsLaunchPadTareSubscription = this->create_subscription<gs_interfaces::msg::LoadCellsTare>("/oberon/launch_tower/tenso_tare", 3, std::bind(&Oberon::arduinoWyrzutniaTareCallback, this, std::placeholders::_1)); 
    
    powerMonitorPublisher = this->create_publisher<gs_interfaces::msg::Power>("/oberon/power", 5);
    uartStatsPub = this->create_publisher<gs_interfaces::msg::UartStatistics>("/oberon/launch_tower/uart_stats", 3);
    remoteUartStatsPub = this->create_publisher<gs_interfaces::msg::UartStatistics>("/oberon/launch_tower/remote_uart_stats", 3);

    valveServosPublisher = this->create_publisher<gs_interfaces::msg::ValveServos>("/oberon/fueling/valves", 3);
    temperatureZaworyPublisher = this->create_publisher<gs_interfaces::msg::Temperature>("/oberon/fueling/temperature", 3);
    pressureZaworyPublisher = this->create_publisher<gs_interfaces::msg::Pressure>("/oberon/fueling/pressure", 3);
    uartStatsZaworyPub = this->create_publisher<gs_interfaces::msg::UartStatistics>("/oberon/fueling/uart_stats", 3);
    remoteUartStatsZaworyPub = this->create_publisher<gs_interfaces::msg::UartStatistics>("/oberon/fueling/remote_uart_stats", 3);

    tankingControlSubscription = this->create_subscription<gs_interfaces::msg::TankingControl>("/oberon/fueling/control", 3, std::bind(&Oberon::fuelingControlCallback, this, std::placeholders::_1));

    oneSecondTimer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Oberon::oneSecondTimerCallback, this));

    // powerMonitor = std::make_unique<PowerMonitor>(std::bind(&Oberon::powerMonitorCallback, this));
    // RPI - always /dev/ttyS0  
    // arduinoWyrzutnia = std::make_unique<ArduinoWyrzutnia>("/dev/ttyS4", std::bind(&Oberon::arduinoWyrzutniaTensoCallback, this), std::bind(&Oberon::arduinoWyrzutniaTemperatureCallback, this), "/dev/ttyS4");
    // ubuntu - /dev/ttyUSB0 / /dev/ttyUSB1 ...
    // arduinoWyrzutnia = std::make_unique<ArduinoWyrzutnia>("/dev/ttyUSB0", std::bind(&Oberon::arduinoWyrzutniaTensoCallback, this),
    //                                                                       std::bind(&Oberon::arduinoWyrzutniaTemperatureCallback, this),
    //                                                                       std::bind(&Oberon::publishArduinoWyrzutniaRemoteUartStats, this));
    // arduinoZawory = std::make_unique<ArduinoZawory>("/dev/ttyUSB0", std::bind(&Oberon::publishArduinoZaworyZaworyPos, this),
    //                                                                 std::bind(&Oberon::publishArduinoZaworyTemperature, this),
    //                                                                 std::bind(&Oberon::publishArduinoZaworyPressure, this),
    //                                                                 std::bind(&Oberon::publishArduinoZaworyRemoteUartStats, this));
    createLiveConfigIfDoesNotExist();
    // loadLiveConfig();
}

Oberon::~Oberon()
{
}

void Oberon::powerMonitorCallback()
{
    gs_interfaces::msg::Power msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = this->get_fully_qualified_name();

    auto power = powerMonitor->getPower();
    msg.shunt_voltage = power.shunt_voltage;
    msg.bus_voltage = power.bus_voltage;
    msg.power = power.power;
    msg.current = power.current;

    powerMonitorPublisher->publish(msg);
}

void Oberon::oneSecondTimerCallback()
{
    publishUartStats();
    publishLoadCellsParams();
}

void Oberon::arduinoWyrzutniaTensoCallback()
{
    gs_interfaces::msg::LoadCells msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = this->get_fully_qualified_name();
    auto tL = arduinoWyrzutnia->getTensoL();
    auto tR = arduinoWyrzutnia->getTensoR();

    msg.tenso_l.raw_val = tL.raw_value;
    msg.tenso_l.raw_kg = tL.raw_kg;
    msg.tenso_l.vehicle_kg = tL.rocket_kg;
    msg.tenso_l.fuel_kg = tL.fuel_kg;

    msg.tenso_r.raw_val = tR.raw_value;
    msg.tenso_r.raw_kg = tR.raw_kg;
    msg.tenso_r.vehicle_kg = tR.rocket_kg;
    msg.tenso_r.fuel_kg = tR.fuel_kg;

    msg.combined_raw_kg = tL.raw_kg + tR.raw_kg;
    msg.combined_vehicle_kg = tL.rocket_kg + tR.rocket_kg;
    msg.combined_fuel_kg = tL.fuel_kg + tR.fuel_kg;

    loadCellsLaunchPadPublisher->publish(msg);
}

void Oberon::arduinoWyrzutniaTemperatureCallback()
{
    gs_interfaces::msg::Temperature msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = this->get_fully_qualified_name();
    msg.temperature_celsius = arduinoWyrzutnia->getTemperature();
    temperatureLaunchPadPublisher->publish(msg);
}

void Oberon::publishUartStats()
{
    // gs_interfaces::msg::UartStatistics msg;
    // msg.header.stamp = this->now();
    // msg.header.frame_id = this->get_fully_qualified_name();
    // const GSUART::UARTStatistics::Stats& stats = arduinoWyrzutnia->getUartStats();
    // msg.total_bytes_sent = stats.totalBytesSent;
    // msg.total_bytes_received = stats.totalBytesReceived;
    // msg.total_messages_sent = stats.totalMessagesSent;
    // msg.total_messages_received = stats.totalMessagesReceived;
    // msg.good_messages_received = stats.goodMessagesReceived;
    // msg.good_messages_received_per_second = stats.goodMessagesReceivedPerSec;
    // msg.messages_sent_per_second = stats.messagesSentPerSec;
    // msg.messages_received_per_second = stats.messagesRecPerSec;
    // msg.bytes_sent_per_second = stats.bytesSentPerSec;
    // msg.bytes_received_per_second = stats.bytesRecPerSec;
    // msg.good_messages_ratio_received_per_second = stats.goodMessagesReceivedPerSecRatio;
    // msg.messages_overwritten = stats.messagesOverwritten;
    // msg.buffor_overflows = stats.bufforOverflows;
    // uartStatsPub->publish(msg);

    // gs_interfaces::msg::UartStatistics msgZawory;
    // msgZawory.header.stamp = this->now();
    // msgZawory.header.frame_id = this->get_fully_qualified_name();
    // const GSUART::UARTStatistics::Stats& statsZawory = arduinoZawory->getUartStats();
    // msgZawory.total_bytes_sent = statsZawory.totalBytesSent;
    // msgZawory.total_bytes_received = statsZawory.totalBytesReceived;
    // msgZawory.total_messages_sent = statsZawory.totalMessagesSent;
    // msgZawory.total_messages_received = statsZawory.totalMessagesReceived;
    // msgZawory.good_messages_received = statsZawory.goodMessagesReceived;
    // msgZawory.good_messages_received_per_second = statsZawory.goodMessagesReceivedPerSec;
    // msgZawory.messages_sent_per_second = statsZawory.messagesSentPerSec;
    // msgZawory.messages_received_per_second = statsZawory.messagesRecPerSec;
    // msgZawory.bytes_sent_per_second = statsZawory.bytesSentPerSec;
    // msgZawory.bytes_received_per_second = statsZawory.bytesRecPerSec;
    // msgZawory.good_messages_ratio_received_per_second = statsZawory.goodMessagesReceivedPerSecRatio;
    // msgZawory.messages_overwritten = statsZawory.messagesOverwritten;
    // msgZawory.buffor_overflows = statsZawory.bufforOverflows;
    // uartStatsZaworyPub->publish(msgZawory);
}

void Oberon::publishArduinoWyrzutniaRemoteUartStats()
{
    gs_interfaces::msg::UartStatistics msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = this->get_fully_qualified_name();
    const GSUART::UARTStatistics::Stats& stats = arduinoWyrzutnia->getRemoteUartStats();
    msg.total_bytes_sent = stats.totalBytesSent;
    msg.total_bytes_received = stats.totalBytesReceived;
    msg.total_messages_sent = stats.totalMessagesSent;
    msg.total_messages_received = stats.totalMessagesReceived;
    msg.good_messages_received = stats.goodMessagesReceived;
    msg.good_messages_received_per_second = stats.goodMessagesReceivedPerSec;
    msg.messages_sent_per_second = stats.messagesSentPerSec;
    msg.messages_received_per_second = stats.messagesRecPerSec;
    msg.bytes_sent_per_second = stats.bytesSentPerSec;
    msg.bytes_received_per_second = stats.bytesRecPerSec;
    msg.good_messages_ratio_received_per_second = stats.goodMessagesReceivedPerSecRatio;
    msg.messages_overwritten = stats.messagesOverwritten;
    msg.buffor_overflows = stats.bufforOverflows;
    remoteUartStatsPub->publish(msg);
}

void Oberon::publishLoadCellsParams()
{
    // gs_interfaces::msg::LoadCellsParams msg;
    // msg.header.stamp = this->now();
    // msg.header.frame_id = this->get_fully_qualified_name();
    // auto tL = arduinoWyrzutnia->getTensoL();
    // auto tR = arduinoWyrzutnia->getTensoR();
    // msg.tenso_l_rocket_point = tL.rocket_point;
    // msg.tenso_l_empty_rocket_point = tL.empty_rocket_point;
    // msg.tenso_l_scale = tL.scale;
    // msg.tenso_r_rocket_point = tR.rocket_point;
    // msg.tenso_r_empty_rocket_point = tR.empty_rocket_point;
    // msg.tenso_r_scale = tR.scale;
    // msg.lean_angle = arduinoWyrzutnia->getLeanAngle();
    // loadCellsParamsPublisher->publish(msg);
}

void Oberon::arduinoWyrzutniaTareCallback(const gs_interfaces::msg::LoadCellsTare::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Wyrzutnia tare msg from %s", msg->header.frame_id.c_str());
    if (msg->tare_rocket_point)
        arduinoWyrzutnia->tareRocketPoint();
    if (msg->tare_empty_rocket_point)
        arduinoWyrzutnia->tareEmptyRocketPoint();
    if (msg->set_scale_left)
        arduinoWyrzutnia->setScaleLeft(msg->scale_left);
    if (msg->set_scale_right)
        arduinoWyrzutnia->setScaleRight(msg->scale_right);
    if (msg->set_lean_angle)
        arduinoWyrzutnia->setLeanAngle(msg->lean_angle);
    saveLiveConfig();
    publishLoadCellsParams();
}

void Oberon::createLiveConfigIfDoesNotExist()
{
    std::string filename = "." + std::string(this->get_fully_qualified_name()) + ".live.cfg";
    std::fstream file(filename, std::ios::in);
    if (!file.is_open())
    {
        file.close();
        saveLiveConfig();
    }
}

void Oberon::loadLiveConfig()
{
    std::string filename = "." + std::string(this->get_fully_qualified_name()) + ".live.cfg";
    std::fstream file(filename, std::ios::in);
    if (!file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot open file %s", filename.c_str());
        return;
    }

    auto tensoL = arduinoWyrzutnia->getTensoL();
    file >> tensoL.rocket_point;
    file >> tensoL.empty_rocket_point;
    file >> tensoL.scale;
    auto tensoR = arduinoWyrzutnia->getTensoR();
    file >> tensoR.rocket_point;
    file >> tensoR.empty_rocket_point;
    file >> tensoR.scale;
    float leanAngle;
    file >> leanAngle;
    arduinoWyrzutnia->setLeanAngle(leanAngle);
    file.close();
}

void Oberon::saveLiveConfig()
{
    std::string filename = "." + std::string(this->get_fully_qualified_name()) + ".live.cfg";
    std::fstream file(filename, std::ios::out);
    if (!file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot open file %s", filename.c_str());
        return;
    }

    auto tensoL = arduinoWyrzutnia->getTensoL();
    file << tensoL.rocket_point << std::endl;
    file << tensoL.empty_rocket_point << std::endl;
    file << tensoL.scale << std::endl;
    auto tensoR = arduinoWyrzutnia->getTensoR();
    file << tensoR.rocket_point << std::endl;
    file << tensoR.empty_rocket_point << std::endl;
    file << tensoR.scale << std::endl;
    file << arduinoWyrzutnia->getLeanAngle() << std::endl;
    file.close();
}

void Oberon::publishArduinoZaworyZaworyPos()
{
    gs_interfaces::msg::ValveServos msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = this->get_fully_qualified_name();
    auto pos = arduinoZawory->getZaworyPos();
    msg.valve_feed_position = pos.feed_percent;
    msg.valve_vent_position = pos.vent_percent;
    valveServosPublisher->publish(msg);
}

void Oberon::publishArduinoZaworyTemperature()
{
    gs_interfaces::msg::Temperature msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = this->get_fully_qualified_name();
    msg.temperature_celsius = arduinoZawory->getTemperature();
    temperatureZaworyPublisher->publish(msg);   
}

void Oberon::publishArduinoZaworyPressure()
{
    gs_interfaces::msg::Pressure msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = this->get_fully_qualified_name();
    msg.pressure_bar = arduinoZawory->getPressure();
    pressureZaworyPublisher->publish(msg);
}

void Oberon::publishArduinoZaworyRemoteUartStats()
{
    gs_interfaces::msg::UartStatistics msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = this->get_fully_qualified_name();
    const GSUART::UARTStatistics::Stats& stats = arduinoZawory->getRemoteUartStats();
    msg.total_bytes_sent = stats.totalBytesSent;
    msg.total_bytes_received = stats.totalBytesReceived;
    msg.total_messages_sent = stats.totalMessagesSent;
    msg.total_messages_received = stats.totalMessagesReceived;
    msg.good_messages_received = stats.goodMessagesReceived;
    msg.good_messages_received_per_second = stats.goodMessagesReceivedPerSec;
    msg.messages_sent_per_second = stats.messagesSentPerSec;
    msg.messages_received_per_second = stats.messagesRecPerSec;
    msg.bytes_sent_per_second = stats.bytesSentPerSec;
    msg.bytes_received_per_second = stats.bytesRecPerSec;
    msg.good_messages_ratio_received_per_second = stats.goodMessagesReceivedPerSecRatio;
    msg.messages_overwritten = stats.messagesOverwritten;
    msg.buffor_overflows = stats.bufforOverflows;
    remoteUartStatsZaworyPub->publish(msg);
}

void Oberon::fuelingControlCallback(const gs_interfaces::msg::TankingControl::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Fueling control msg from %s  %d  %d  %d", msg->header.frame_id.c_str(), msg->valve_feed, msg->valve_vent, msg->decouple);
    if (arduinoZawory)
        arduinoZawory->steerFueling(msg->valve_feed, msg->valve_vent, msg->decouple);
    else
        printf("arduinoZawory is nullptr\n");
}