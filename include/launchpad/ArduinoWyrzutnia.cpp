#include "ArduinoWyrzutnia.hpp"

#include <cstring>
#include <cmath>

ArduinoWyrzutnia::ArduinoWyrzutnia(std::string serialPort, std::function<void()> newTensoCallback, std::function<void()> newTemperatureCallback, std::function<void()> newUARTStatsCallback)
    : messenger(serialPort), newTensoCallback(newTensoCallback), newTemperatureCallback(newTemperatureCallback), newUARTStatsCallback(newUARTStatsCallback)
{
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
        
        switch (msg->getID())
        {
            case GSUART::MsgID::TENSO:
            {
                const GSUART::MsgTenso* msgTenso = dynamic_cast<const GSUART::MsgTenso*>(msg);
                tensoL.raw_value = msgTenso->tenso_left_raw;
                tensoL.last_values[(tensoL.last_values_idx++) % 30] = tensoL.raw_value;
                tensoR.raw_value = msgTenso->tenso_right_raw;
                tensoR.last_values[(tensoR.last_values_idx++) % 30] = tensoR.raw_value;

                tensoL.raw_kg = tensoL.raw_value * tensoL.scale / 1000.0;
                tensoL.rocket_kg = (tensoL.raw_value - tensoL.rocket_point) * tensoL.scale / 1000.0;
                tensoL.fuel_kg = (tensoL.raw_value - tensoL.empty_rocket_point) * tensoL.scale / 1000.0;

                tensoR.raw_kg = tensoR.raw_value * tensoR.scale / 1000.0;
                tensoR.rocket_kg = (tensoR.raw_value - tensoR.rocket_point) * tensoR.scale / 1000.0;
                tensoR.fuel_kg = (tensoR.raw_value - tensoR.empty_rocket_point) * tensoR.scale / 1000.0;

                // uwzglednienie kata nachylenia
                tensoL.raw_kg /= lean.cosinus;
                tensoL.rocket_kg /= lean.cosinus;
                tensoL.fuel_kg /= lean.cosinus;
                tensoR.raw_kg /= lean.cosinus;
                tensoR.rocket_kg /= lean.cosinus;
                tensoR.fuel_kg /= lean.cosinus;

                if (newTensoCallback)
                    newTensoCallback();
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
            case GSUART::MsgID::UART_STATS:
            {
                const GSUART::MsgUartStats* msgUARTStats = dynamic_cast<const GSUART::MsgUartStats*>(msg);
                remoteUARTStats = msgUARTStats->stats;
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
}

void ArduinoWyrzutnia::tareEmptyRocketPoint()
{
    tensoL.empty_rocket_point = tensoL.getAvgValue30();
    tensoR.empty_rocket_point = tensoR.getAvgValue30();
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