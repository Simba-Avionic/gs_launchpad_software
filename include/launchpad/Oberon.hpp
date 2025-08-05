#pragma once

#include <thread>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "gs_interfaces/msg/load_cells.hpp"
#include "gs_interfaces/msg/load_cells_params.hpp"
#include "gs_interfaces/msg/load_cells_tare.hpp"
#include "gs_interfaces/msg/temperature.hpp"
#include "gs_interfaces/msg/uart_statistics.hpp"
#include "gs_interfaces/msg/power.hpp"
#include "gs_interfaces/msg/valve_servos.hpp"
#include "gs_interfaces/msg/pressure.hpp"
#include "gs_interfaces/msg/tanking_control.hpp"

#include "ArduinoWyrzutnia.hpp"
#include "ArduinoZawory.hpp"
#include "PowerMonitor.hpp"

class Oberon : public rclcpp::Node
{
public:
    Oberon();
    ~Oberon();
private:
    rclcpp::Publisher<gs_interfaces::msg::LoadCells>::SharedPtr loadCellsLaunchPadPublisher;
    rclcpp::Publisher<gs_interfaces::msg::UartStatistics>::SharedPtr uartStatsPub;
    rclcpp::Publisher<gs_interfaces::msg::UartStatistics>::SharedPtr remoteUartStatsPub;
    rclcpp::Publisher<gs_interfaces::msg::Temperature>::SharedPtr temperatureLaunchPadPublisher;
    rclcpp::Publisher<gs_interfaces::msg::LoadCellsParams>::SharedPtr loadCellsParamsPublisher;
    rclcpp::Publisher<gs_interfaces::msg::Power>::SharedPtr powerMonitorPublisher;
    rclcpp::Publisher<gs_interfaces::msg::Temperature>::SharedPtr temperatureZaworyPublisher;
    rclcpp::Publisher<gs_interfaces::msg::Pressure>::SharedPtr pressureZaworyPublisher;
    rclcpp::Publisher<gs_interfaces::msg::ValveServos>::SharedPtr valveServosPublisher;
    rclcpp::Publisher<gs_interfaces::msg::UartStatistics>::SharedPtr uartStatsZaworyPub;
    rclcpp::Publisher<gs_interfaces::msg::UartStatistics>::SharedPtr remoteUartStatsZaworyPub;
    rclcpp::Subscription<gs_interfaces::msg::LoadCellsTare>::SharedPtr loadCellsLaunchPadTareSubscription;
    rclcpp::Subscription<gs_interfaces::msg::TankingControl>::SharedPtr tankingControlSubscription;
    
    void arduinoWyrzutniaTareCallback(const gs_interfaces::msg::LoadCellsTare::SharedPtr msg);

    std::unique_ptr<ArduinoWyrzutnia> arduinoWyrzutnia;
    std::unique_ptr<ArduinoZawory> arduinoZawory;
    std::unique_ptr<PowerMonitor> powerMonitor;

    void powerMonitorCallback();

    rclcpp::TimerBase::SharedPtr oneSecondTimer;
    void oneSecondTimerCallback();

    void publishUartStats();
    void publishLoadCellsParams();

    void arduinoWyrzutniaTensoCallback();
    void arduinoWyrzutniaTemperatureCallback();
    void publishArduinoWyrzutniaRemoteUartStats();

    void publishArduinoZaworyRemoteUartStats();
    void publishArduinoZaworyZaworyPos();
    void publishArduinoZaworyTemperature();
    void publishArduinoZaworyPressure();
    void fuelingControlCallback(const gs_interfaces::msg::TankingControl::SharedPtr msg);

    void createLiveConfigIfDoesNotExist();
    void loadLiveConfig();
    void saveLiveConfig();
};