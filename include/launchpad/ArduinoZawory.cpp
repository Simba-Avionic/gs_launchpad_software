#include "ArduinoZawory.hpp"

ArduinoZawory::ArduinoZawory(std::string serialPort, std::function<void()> newZaworyPosCallback,
                                                     std::function<void()> newTemperatureCallback,
                                                     std::function<void()> newPressureCallback,
                                                     std::function<void()> newUARTStatsCallback)
    :   messenger(serialPort),
        newZaworyPosCallback(newZaworyPosCallback),
        newTemperatureCallback(newTemperatureCallback),
        newPressureCallback(newPressureCallback),
        newUARTStatsCallback(newUARTStatsCallback)
{
    readT = std::thread(&ArduinoZawory::readingLoop, this);
}

ArduinoZawory::~ArduinoZawory()
{
    
}

void ArduinoZawory::readingLoop()
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
                zaworyPos.feed_percent = msgZaworyPos->valve_feed;
                zaworyPos.vent_percent = msgZaworyPos->valve_vent;
                if (newZaworyPosCallback)
                    newZaworyPosCallback();
                break;
            }
            case GSUART::MsgID::TEMPERATURE:
            {
                const GSUART::MsgTemperature* msgTemperature = dynamic_cast<const GSUART::MsgTemperature*>(msg);
                temperature = msgTemperature->temperature_celsius;
                if (newTemperatureCallback)
                    newTemperatureCallback();
                break;
            }
            case GSUART::MsgID::PRESSURE:
            {
                const GSUART::MsgPressure* msgPressure = dynamic_cast<const GSUART::MsgPressure*>(msg);
                pressure = msgPressure->pressure_bar;
                if (newPressureCallback)
                    newPressureCallback();
                break;
            }
            case GSUART::MsgID::UART_STATS:
            {
                const GSUART::MsgUartStats* msgUARTStats = dynamic_cast<const GSUART::MsgUartStats*>(msg);
                remoteUartStats = msgUARTStats->stats;
                if (newUARTStatsCallback)
                    newUARTStatsCallback();
                break;
            }
            default:
                // printf("Unknown message type\n");
                break;
        }
    }
}

void ArduinoZawory::steerFueling(int8_t feed_percent, int8_t vent_percent, bool decouple)
{
    GSUART::MsgZaworySterowanie msg;
    msg.valve_feed = feed_percent;
    msg.valve_vent = vent_percent;
    msg.decouple = decouple;
    messenger.send(msg);
}

const ArduinoZawory::ZaworyPos& ArduinoZawory::getZaworyPos()
{
    return zaworyPos;
}

const float& ArduinoZawory::getTemperature()
{
    return temperature;
}

const double& ArduinoZawory::getPressure()
{
    return pressure;
}

const GSUART::UARTStatistics::Stats& ArduinoZawory::getUartStats()
{
    return messenger.getStats().stats;
}

const GSUART::UARTStatistics::Stats& ArduinoZawory::getRemoteUartStats()
{
    return remoteUartStats;
}