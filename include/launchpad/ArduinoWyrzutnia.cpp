#include "ArduinoWyrzutnia.hpp"

#include <cstring>
#include <cmath>

ArduinoWyrzutnia::ArduinoWyrzutnia(std::string serialPort)
    : Node("weigtmodule"), messenger(serialPort)
{
    tensoR.scale = 1.097547806;
    tensoL.scale = 1.078646757;

    UartStatsPub = this->create_publisher<gs_interfaces::msg::UartStatistics>("/tanking/load_cells/remote_uart_stats", 3);
    localUartStatsPub = this->create_publisher<gs_interfaces::msg::UartStatistics>("/tanking/load_cells/local_uart_stats", 3);
    LoadCellsPub = this->create_publisher<gs_interfaces::msg::LoadCells>("/tanking/load_cells", 3);
    temperaturePub = this->create_publisher<gs_interfaces::msg::Temperature>("/tanking/load_cells/temperature", 3);
    ParamsCellsPub = this->create_publisher<gs_interfaces::msg::LoadCellsParams>("/tanking/load_cells/params", 3);

    tareSub = this->create_subscription<gs_interfaces::msg::LoadCellsTare>("/tanking/load_cells/tare", 3, std::bind(&ArduinoWyrzutnia::tareCallback, this, std::placeholders::_1));

    oneSecondTimer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ArduinoWyrzutnia::oneSecondTimerCallback, this));
    
    readT = std::thread(&ArduinoWyrzutnia::readingLoop, this);
}

ArduinoWyrzutnia::~ArduinoWyrzutnia()
{
    
}

void ArduinoWyrzutnia::readingLoop()
{
    while (true)
    {
        const GSUART::Message* msg = nullptr;
        while (!msg)
        {
            msg = messenger.receive();
        }
        // RCLCPP_INFO (this->get_logger(), "%d", (int)msg->getID());
        
        switch (msg->getID())
        {
            case GSUART::MsgID::TENSO:
            {
                const GSUART::MsgTenso* msgTenso = dynamic_cast<const GSUART::MsgTenso*>(msg);
                tensoL.raw_value = msgTenso->tenso_left_raw;
                tensoL.last_values[(tensoL.last_values_idx++) % 30] = tensoL.raw_value;
                tensoR.raw_value = msgTenso->tenso_right_raw;
                tensoR.last_values[(tensoR.last_values_idx++) % 30] = tensoR.raw_value;

                tensoL.raw_kg = (tensoL.raw_value - tensoL.no_decoupler_point) * tensoL.scale / 1000.0;
                tensoL.rocket_kg = (tensoL.raw_value - tensoL.rocket_point) * tensoL.scale / 1000.0;
                tensoL.fuel_kg = (tensoL.raw_value - tensoL.empty_rocket_point) * tensoL.scale / 1000.0;

                tensoR.raw_kg = (tensoR.raw_value - tensoR.no_decoupler_point) * tensoR.scale / 1000.0;
                tensoR.rocket_kg = (tensoR.raw_value - tensoR.rocket_point) * tensoR.scale / 1000.0;
                tensoR.fuel_kg = (tensoR.raw_value - tensoR.empty_rocket_point) * tensoR.scale / 1000.0;

                // uwzglednienie kata nachylenia
                tensoL.raw_kg /= lean.cosinus;
                tensoL.rocket_kg /= lean.cosinus;
                tensoL.fuel_kg /= lean.cosinus;
                tensoR.raw_kg /= lean.cosinus;
                tensoR.rocket_kg /= lean.cosinus;
                tensoR.fuel_kg /= lean.cosinus;
                
            

                gs_interfaces::msg::LoadCells msg_load_cells;
                msg_load_cells.header.stamp = this->now();
                msg_load_cells.header.frame_id = this->get_fully_qualified_name();
                msg_load_cells.tenso_l.raw_val = tensoL.raw_value;
                msg_load_cells.tenso_l.raw_mass_kg = tensoL.raw_kg;
                msg_load_cells.tenso_l.vehicle_mass_kg = tensoL.rocket_kg;
                msg_load_cells.tenso_l.fuel_mass_kg = tensoL.fuel_kg;
                msg_load_cells.tenso_r.raw_val = tensoR.raw_value;
                msg_load_cells.tenso_r.raw_mass_kg = tensoR.raw_kg;
                msg_load_cells.tenso_r.vehicle_mass_kg = tensoR.rocket_kg;
                msg_load_cells.tenso_r.fuel_mass_kg = tensoR.fuel_kg;

                msg_load_cells.combined_raw_mass_kg = tensoL.raw_kg + tensoR.raw_kg;
                msg_load_cells.combined_vehicle_mass_kg = tensoL.rocket_kg + tensoR.rocket_kg;
                msg_load_cells.combined_fuel_mass_kg = tensoL.fuel_kg + tensoR.fuel_kg;
                LoadCellsPub->publish(msg_load_cells);

                break;
            }
            case GSUART::MsgID::TEMPERATURE:
            {
                const GSUART::MsgTemperature* msgTemperature = dynamic_cast<const GSUART::MsgTemperature*>(msg);
                temperature = msgTemperature->temperature_celsius;

                gs_interfaces::msg::Temperature msgTemp;
                msgTemp.header.stamp = this->now();
                msgTemp.header.frame_id = this->get_fully_qualified_name();
                msgTemp.temperature_celsius = temperature;
                temperaturePub->publish(msgTemp);
                break;
            }
            case GSUART::MsgID::UART_STATS:
            {
                const GSUART::MsgUartStats* msgUARTStats = dynamic_cast<const GSUART::MsgUartStats*>(msg);
                remoteUARTStats = msgUARTStats->stats;
                

                gs_interfaces::msg::UartStatistics msgUart;
                msgUart.header.stamp = this->now();
                msgUart.header.frame_id = this->get_fully_qualified_name();
                msgUart.total_bytes_sent = msgUARTStats->stats.totalBytesSent;
                msgUart.total_bytes_received = msgUARTStats->stats.totalBytesReceived;
                msgUart.total_messages_sent = msgUARTStats->stats.totalMessagesSent;
                msgUart.total_messages_received = msgUARTStats->stats.totalMessagesReceived;
                msgUart.good_messages_received = msgUARTStats->stats.goodMessagesReceived;
                msgUart.good_messages_received_per_second = msgUARTStats->stats.goodMessagesReceivedPerSec;
                msgUart.messages_sent_per_second = msgUARTStats->stats.messagesSentPerSec;
                msgUart.messages_received_per_second = msgUARTStats->stats.messagesRecPerSec;
                msgUart.bytes_sent_per_second = msgUARTStats->stats.bytesSentPerSec;
                msgUart.bytes_received_per_second = msgUARTStats->stats.bytesRecPerSec;
                msgUart.good_messages_ratio_received_per_second = msgUARTStats->stats.goodMessagesReceivedPerSecRatio;
                msgUart.messages_overwritten = msgUARTStats->stats.messagesOverwritten;
                msgUart.buffor_overflows = msgUARTStats->stats.bufforOverflows;
                UartStatsPub->publish(msgUart);

            
                break;
            }
            default:
                // printf("Unknown message type\n");
                break;
        }
    }
}

ArduinoWyrzutnia::tenso& ArduinoWyrzutnia::getTensoL()
{
    return tensoL;
}

ArduinoWyrzutnia::tenso& ArduinoWyrzutnia::getTensoR()
{
    return tensoR;
}

const float& ArduinoWyrzutnia::getLeanAngle()
{
    return lean.angle;
}

const float& ArduinoWyrzutnia::getTemperature()
{
    return temperature;
}

const GSUART::UARTStatistics::Stats& ArduinoWyrzutnia::getUartStats()
{
    return messenger.getStats().stats;
}

const GSUART::UARTStatistics::Stats& ArduinoWyrzutnia::getRemoteUartStats()
{
    return remoteUARTStats;
}

void ArduinoWyrzutnia::tareRocketPoint()
{
    tensoL.rocket_point = tensoL.getAvgValue30();
    tensoR.rocket_point = tensoR.getAvgValue30();
    RCLCPP_INFO(this->get_logger(), "Tarowanie rakiety:  %d, %d", tensoL.rocket_point, tensoR.rocket_point);
}

void ArduinoWyrzutnia::tareEmptyRocketPoint()
{
    tensoL.empty_rocket_point = tensoL.getAvgValue30();
    tensoR.empty_rocket_point = tensoR.getAvgValue30();
    RCLCPP_INFO(this->get_logger(), "Tarowanie utleniacz: %d, %d", tensoL.empty_rocket_point, tensoR.empty_rocket_point);
}

void ArduinoWyrzutnia::tareNoDecouplerPoint()
{
    tensoL.no_decoupler_point = tensoL.getAvgValue30();
    tensoR.no_decoupler_point = tensoR.getAvgValue30();
    RCLCPP_INFO(this->get_logger(), "Tarowanie rakiety bez decouplera: %d, %d", tensoL.no_decoupler_point, tensoR.no_decoupler_point);
}

void ArduinoWyrzutnia::setScaleLeft(double scale)
{
    tensoL.scale = scale;
}

void ArduinoWyrzutnia::setScaleRight(double scale)
{
    tensoR.scale = scale;
}

void ArduinoWyrzutnia::setLeanAngle(float angle)
{
    lean.angle = angle;
    lean.cosinus = cos(angle * 3.14159265359 / 180.0);
}
void ArduinoWyrzutnia::tareCallback(const gs_interfaces::msg::LoadCellsTare::SharedPtr msg)
{
    if(msg->tare_rocket)
    {
        tareRocketPoint();
    }
    if(msg->tare_oxidizer)
    {
        tareEmptyRocketPoint();
    }
    if(msg->tare_pressurizer)
    {
        tareNoDecouplerPoint();
    }
}

void ArduinoWyrzutnia::oneSecondTimerCallback()
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
    localUartStatsPub->publish(msg);

    gs_interfaces::msg::LoadCellsParams msgParams;

    msgParams.header.stamp = this->now();
    msgParams.header.frame_id = this->get_fully_qualified_name();

    msgParams.tenso_l_empty_rocket_point = tensoL.empty_rocket_point;
    msgParams.tenso_l_rocket_point = tensoL.rocket_point;
    msgParams.tenso_l_scale = tensoL.scale;

    msgParams.tenso_r_empty_rocket_point = tensoR.empty_rocket_point;
    msgParams.tenso_r_rocket_point = tensoR.rocket_point;
    msgParams.tenso_r_scale = tensoR.scale;

    msgParams.lean_angle = lean.angle;

    ParamsCellsPub->publish(msgParams);

}