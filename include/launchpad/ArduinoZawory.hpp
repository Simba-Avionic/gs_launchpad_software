#pragma once

#include <string>
#include <functional>
#include <thread>

#include "GSUART.hpp"

class ArduinoZawory
{
public:
    struct ZaworyPos
    {
        int8_t feed_percent = 0;
        int8_t vent_percent = 0;
    };

    ArduinoZawory(std::string serialPort, std::function<void()> newZaworyPosCallback,
                                          std::function<void()>  newTemperatureCallback,
                                          std::function<void()> newPressureCallback,
                                          std::function<void()> newUARTStatsCallback);
    ~ArduinoZawory();

    void steerFueling(int8_t feed_percent, int8_t vent_percent, bool decouple);

    const ZaworyPos& getZaworyPos();
    const float& getTemperature();
    const double& getPressure();
    const GSUART::UARTStatistics::Stats& getUartStats();
    const GSUART::UARTStatistics::Stats& getRemoteUartStats();
private:
    ZaworyPos zaworyPos;
    float temperature = 0.0;
    double pressure = 0.0;

    GSUART::UARTStatistics::Stats remoteUartStats;

    GSUART::Messenger messenger;
    std::thread readT;
    void readingLoop();

    std::function<void()> newZaworyPosCallback;
    std::function<void()> newTemperatureCallback;
    std::function<void()> newPressureCallback;
    std::function<void()> newUARTStatsCallback;
};