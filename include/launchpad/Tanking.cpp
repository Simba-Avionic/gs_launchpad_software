#include "Tanking.hpp"

Tanking::Tanking(std::string serialPort) : Node("tanking"),
    messenger(serialPort)
{
    localUartStatsValvesPub = this->create_publisher<gs_interfaces::msg::UartStatistics>("/tanking/uart_stats", 3);
    remoteUartStatsValvesPub = this->create_publisher<gs_interfaces::msg::UartStatistics>("/tanking/remote_uart_stats", 3);
    temperatureValvesPub = this->create_publisher<gs_interfaces::msg::Temperature>("/tanking/temperature", 3);
    pressureValvesPub = this->create_publisher<gs_interfaces::msg::Pressure>("/tanking/pressure", 3);
    tankingActuatorsPub = this->create_publisher<gs_interfaces::msg::TankingActuators>("/tanking/valves/servos", 3);
    tankingPowerPub = this->create_publisher<gs_interfaces::msg::PowerTanking>("/tanking/power", 3);
    tankingCmdsSub = this->create_subscription<gs_interfaces::msg::TankingCommands>("/tanking/commands", 3, std::bind(&Tanking::tankingControlCallback, this, std::placeholders::_1));
    tankingAbortSub = this->create_subscription<gs_interfaces::msg::TankingAbort>("/tanking/abort", 3, std::bind(&Tanking::abortCallback, this, std::placeholders::_1));

    oneSecondTimer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Tanking::oneSecondTimerCallback, this));
    readT = std::thread(&Tanking::readingLoop, this);
}

Tanking::~Tanking()
{
    readT.join();
}

void Tanking::tankingControlCallback(const gs_interfaces::msg::TankingCommands::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Fueling control msg from %s  %d  %d  %d", msg->header.frame_id.c_str(), msg->valve_feed_oxidizer, msg->valve_feed_pressurizer, msg->decoupler_oxidizer);
    steerFueling(msg->valve_feed_oxidizer, msg->valve_feed_pressurizer, msg->valve_vent_oxidizer, msg->valve_vent_pressurizer, msg->decoupler_oxidizer, msg->decoupler_pressurizer);
}

void Tanking::abortCallback(const gs_interfaces::msg::TankingAbort::SharedPtr msg)
{
    RCLCPP_WARN(this->get_logger(), "ABORT command received from %s with value %d", msg->header.frame_id.c_str(), msg->abort);
    GSUART::MsgAbort abortMsg;
    abortMsg.abort = msg->abort;
    messenger.send(abortMsg);
}

void Tanking::readingLoop()
{
    while (true)
    {
        const GSUART::Message* msg = nullptr;
        while (!msg)
        {
            msg = messenger.receive();
        }

        switch (msg->getID())
        {
            case GSUART::MsgID::ZAWORY_POZYCJA:
            {
                const GSUART::MsgZaworyPozycja* msgZaworyPos = dynamic_cast<const GSUART::MsgZaworyPozycja*>(msg);
                if (tankingActuatorsPub)
                    publishValvesCurrPosition(msgZaworyPos);
                break;
            }
            case GSUART::MsgID::TEMPERATURE:
            {
                const GSUART::MsgTemperature* msgTemperature = dynamic_cast<const GSUART::MsgTemperature*>(msg);
                if (temperatureValvesPub)
                    publishValvesTemperature(msgTemperature);
                break;
            }
            case GSUART::MsgID::PRESSURE:
            {
                const GSUART::MsgPressure* msgPressure = dynamic_cast<const GSUART::MsgPressure*>(msg);
                if (pressureValvesPub)
                    publishValvesPressure(msgPressure);
                break;
            }
            case GSUART::MsgID::UART_STATS:
            {
                const GSUART::MsgUartStats* msgUARTStats = dynamic_cast<const GSUART::MsgUartStats*>(msg);
                if (localUartStatsValvesPub)
                    publishRemoteUartStats(msgUARTStats);
                break;
            }
            case GSUART::MsgID::POWER_TANKING:
            {
                const GSUART::MsgPowerTanking* msgPowerTanking = dynamic_cast<const GSUART::MsgPowerTanking*>(msg);
                if (tankingPowerPub)
                    publishTankingPower(msgPowerTanking);
                break;
            }
            default:
                // printf("Unknown message type\n");
                break;
        }
    }
}

void Tanking::steerFueling(
    bool valve_feed_oxidizer, bool valve_feed_pressurizer, 
    bool valve_vent_oxidizer, bool valve_vent_pressurizer, 
    bool decoupler_oxidizer, bool decoupler_pressurizer
)
{
    GSUART::MsgZaworySterowanie msg;
    msg.valve_feed_oxidizer = valve_feed_oxidizer;
    msg.valve_feed_pressurizer = valve_feed_pressurizer;
    msg.valve_vent_oxidizer = valve_vent_oxidizer;
    msg.valve_vent_pressurizer = valve_vent_pressurizer;
    msg.decoupler_oxidizer = decoupler_oxidizer;
    msg.decoupler_pressurizer = decoupler_pressurizer;
    messenger.send(msg);
}

const GSUART::UARTStatistics::Stats& Tanking::getUartStats()
{
    return messenger.getStats().stats;
}

const GSUART::UARTStatistics::Stats& Tanking::getRemoteUartStats()
{
    return remoteUartStats;
}

void Tanking::oneSecondTimerCallback()
{
    publishLocalUartStats();
}

void Tanking::publishLocalUartStats()
{
    gs_interfaces::msg::UartStatistics msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = this->get_fully_qualified_name();
    const GSUART::UARTStatistics::Stats& stats = messenger.getStats().stats;
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
    localUartStatsValvesPub->publish(msg);

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

void Tanking::publishValvesCurrPosition(const GSUART::MsgZaworyPozycja* msgZaworyPos)
{
    gs_interfaces::msg::TankingActuators msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = this->get_fully_qualified_name();
    msg.valve_feed_oxidizer = msgZaworyPos->valve_feed_oxidizer;
    msg.valve_feed_pressurizer = msgZaworyPos->valve_feed_pressurizer;
    msg.valve_vent_oxidizer = msgZaworyPos->valve_vent_oxidizer;
    msg.valve_vent_pressurizer = msgZaworyPos->valve_vent_pressurizer;
    msg.decoupler_oxidizer = msgZaworyPos->decoupler_oxidizer;
    msg.decoupler_pressurizer = msgZaworyPos->decoupler_pressurizer;
    tankingActuatorsPub->publish(msg);
}

void Tanking::publishValvesTemperature(const GSUART::MsgTemperature* msgTemperature)
{
    gs_interfaces::msg::Temperature msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = this->get_fully_qualified_name();
    msg.temperature_celsius = msgTemperature->temperature_celsius;
    temperatureValvesPub->publish(msg);
}

void Tanking::publishValvesPressure(const GSUART::MsgPressure* msgPressure)
{
    gs_interfaces::msg::Pressure msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = this->get_fully_qualified_name();
    msg.pressure_bar = msgPressure->pressure_bar;
    pressureValvesPub->publish(msg);
}

void Tanking::publishRemoteUartStats(const GSUART::MsgUartStats* msgUARTStats)
{
    gs_interfaces::msg::UartStatistics msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = this->get_fully_qualified_name();
    // const GSUART::UARTStatistics::Stats& stats = msgUARTStats;
    msg.total_bytes_sent = msgUARTStats->stats.totalBytesSent;
    msg.total_bytes_received = msgUARTStats->stats.totalBytesReceived;
    msg.total_messages_sent = msgUARTStats->stats.totalMessagesSent;
    msg.total_messages_received = msgUARTStats->stats.totalMessagesReceived;
    msg.good_messages_received = msgUARTStats->stats.goodMessagesReceived;
    msg.good_messages_received_per_second = msgUARTStats->stats.goodMessagesReceivedPerSec;
    msg.messages_sent_per_second = msgUARTStats->stats.messagesSentPerSec;
    msg.messages_received_per_second = msgUARTStats->stats.messagesRecPerSec;
    msg.bytes_sent_per_second = msgUARTStats->stats.bytesSentPerSec;
    msg.bytes_received_per_second = msgUARTStats->stats.bytesRecPerSec;
    msg.good_messages_ratio_received_per_second = msgUARTStats->stats.goodMessagesReceivedPerSecRatio;
    msg.messages_overwritten = msgUARTStats->stats.messagesOverwritten;
    msg.buffor_overflows = msgUARTStats->stats.bufforOverflows;
    remoteUartStatsValvesPub->publish(msg);
}

void Tanking::publishTankingPower(const GSUART::MsgPowerTanking* msgPower)
{
    gs_interfaces::msg::PowerTanking msg;
    msg.v7_4.bus_voltage_v = msgPower->v7_4.V;
    msg.v7_4.current_ma = msgPower->v7_4.mA;
    msg.v12.bus_voltage_v = msgPower->v12.V;
    msg.v12.current_ma = msgPower->v12.mA;
    tankingPowerPub->publish(msg);
}