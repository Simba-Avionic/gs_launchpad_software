#pragma once

#include <unistd.h>
#include <termio.h>
#include <errno.h>
#include <fcntl.h>
#include <string>
#include <string.h>
#include <stdio.h>
#include <thread>
#include <vector>
#include <functional>

// #define GSUART_PLATFORM 0        //    0 - arduino - GSUART_PLATFORM_ARDUINO
                                    //    1 - rpi ubuntu - GSUART_PLATFORM_RPI_UBUNTU
#define GSUART_PLATFORM_ARDUINO     0
#define GSUART_PLATFORM_RPI_UBUNTU  1
#define GSUART_PLATFORM GSUART_PLATFORM_RPI_UBUNTU
#include "GSUART.hpp"

class ArduinoWyrzutnia
{
public:
    struct tenso
    {
        int32_t last_values[30] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
                                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
                                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0};   // 30 ostatnich wartosci do sredniej dla tarowania
        int32_t last_values_idx = 0;
        int32_t raw_value = 0;                  // bez tarowania
        int32_t rocket_point = 0;               // tarujemy i kladziemy rakiete
        int32_t empty_rocket_point = 0;         // tarujemy i tankujemy paliwo
        double scale = 1.0;
        float raw_kg = 0.0;
        float rocket_kg = 0.0;
        float fuel_kg = 0.0;

        int32_t getAvgValue30()
        {
            int32_t sum = 0;
            for (int i=0; i<30; i++)
                sum += last_values[i];
            return sum / 30;
        }
    };

    ArduinoWyrzutnia(std::string serialPort, std::function<void()> newTensoCallback, std::function<void()> newTemperatureCallback, std::function<void()> newUARTStatsCallback);
    ~ArduinoWyrzutnia();

    tenso& getTensoL();
    tenso& getTensoR();
    const float& getLeanAngle();
    const float& getTemperature();

    const GSUART::UARTStatistics::Stats& getUartStats();
    const GSUART::UARTStatistics::Stats& getRemoteUartStats();

    void tareRocketPoint();
    void tareEmptyRocketPoint();
    void setScaleLeft(double scale);
    void setScaleRight(double scale);
    void setLeanAngle(float angle);

private:
    tenso tensoL, tensoR;

    struct Lean {
        float angle = 0.0;
        float cosinus = 1.0;
    } lean;

    float temperature = 0.0;

    GSUART::UARTStatistics::Stats remoteUARTStats;

    GSUART::Messenger messenger;
    std::thread readT;
    void readingLoop();

    std::function<void()> newTensoCallback;
    std::function<void()> newTemperatureCallback;
    std::function<void()> newUARTStatsCallback;
};