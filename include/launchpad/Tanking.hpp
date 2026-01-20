#pragma once

#include <thread>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "gs_interfaces/msg/temperature.hpp"
#include "gs_interfaces/msg/uart_statistics.hpp"
#include "gs_interfaces/msg/power_tanking.hpp"
#include "gs_interfaces/msg/tanking_actuators.hpp"
// #include "gs_interfaces/msg/pressure.hpp"
#include "gs_interfaces/msg/tanking_commands.hpp"
#include "gs_interfaces/msg/tanking_abort.hpp"
#include "gs_interfaces/msg/tanking_sensors.hpp"

#include "GSUART.hpp"

class Tanking : public rclcpp::Node
{
public:
    Tanking(std::string serialPort);
    ~Tanking();

    struct ValvesPosition
    {
        int8_t feed_percent = 0;
        int8_t vent_percent = 0;
    };

private:
    // local is RaspberryPi, remote is Arduino
    rclcpp::Publisher<gs_interfaces::msg::UartStatistics>::SharedPtr localUartStatsValvesPub;
    rclcpp::Publisher<gs_interfaces::msg::UartStatistics>::SharedPtr remoteUartStatsValvesPub;
    rclcpp::Publisher<gs_interfaces::msg::Temperature>::SharedPtr temperatureValvesPub;
    // rclcpp::Publisher<gs_interfaces::msg::Pressure>::SharedPtr pressureValvesPub;
    rclcpp::Publisher<gs_interfaces::msg::TankingActuators>::SharedPtr tankingActuatorsPub;
    rclcpp::Publisher<gs_interfaces::msg::PowerTanking>::SharedPtr tankingPowerPub;
    rclcpp::Publisher<gs_interfaces::msg::TankingSensors>::SharedPtr tankingSensorsPub;
    rclcpp::Subscription<gs_interfaces::msg::TankingCommands>::SharedPtr tankingCmdsSub;
    rclcpp::Subscription<gs_interfaces::msg::TankingAbort>::SharedPtr tankingAbortSub;
    
    rclcpp::TimerBase::SharedPtr oneSecondTimer;
    
    GSUART::UARTStatistics::Stats remoteUartStats;

    GSUART::Messenger messenger;
    std::thread readT;
    void readingLoop();
    void steerFueling(
        bool valve_feed_oxidizer, bool valve_feed_pressurizer, 
        bool valve_vent_oxidizer, bool valve_vent_pressurizer, 
        bool decoupler_oxidizer, bool decoupler_pressurizer
    );
    
    const GSUART::UARTStatistics::Stats& getUartStats();
    const GSUART::UARTStatistics::Stats& getRemoteUartStats();

    void oneSecondTimerCallback();
    
    void publishLocalUartStats();
    void publishRemoteUartStats(const GSUART::MsgUartStats* msgUARTStats);

    void publishValvesCurrPosition(const GSUART::MsgZaworyPozycja* msgZaworyPos);
    
    void publishValvesTemperature(const GSUART::MsgTemperature* msgTemperature);
    // void publishValvesPressure(const GSUART::MsgPressure* msgPressure);

    void publishHydroSensors(const GSUART::MsgHydroSensors* msgHydroSensors);

    void publishTankingPower(const GSUART::MsgPowerTanking* msgPower);

    void tankingControlCallback(const gs_interfaces::msg::TankingCommands::SharedPtr msg);
    void abortCallback(const gs_interfaces::msg::TankingAbort::SharedPtr msg);
};

/*
- Read topic tanking commands
- przetworzyć je na gsuart (GSUART.hpp) i wysłać hen daleko!
- odebrać uzywajac GSUART pozycje zaworów i opublikować jako topic
- odebrac power GSUARTEm i opublikować jako topic
- odebrac UartStatistics GSUARTem i opublikować jako topic
- publikowac swoje lokalne UartStatistics co 1sekunde

- TODO na potem:  pressure i temperature GSUARTem i opublikować jako topic
*/