#include "ServoValve.hpp"
#include "GSUART.hpp"
#include <Arduino.h>

#include "PressureConverter.hpp"

GSUART::MsgHydroSensors msgHydroSensors;

PressureConverter* pressure_sensor_oxidizer;

#define INTERVAL_SEND_HYDRAULIC_SENSORS 500
#define INTERVAL_SEND_UART_STATS 4000

unsigned long last_hydro_send_time = 0;
unsigned long last_uart_stats_send_time = 0;



GSUART::Messenger messenger(&Serial);

GSUART::MsgZaworyPozycja msgValves;

ServoValve* valve_feed_oxidizer;

Decoupler* decoupler_oxidizer;
ElectroValve* valve_vent_oxidizer;

#define INTERVAL_SEND_VALVES_POSITION 500
#define COMMAND_TIMEOUT_ms 3000
#define TIMEOUT_ABORT_ms 30000

unsigned long last_command_time = 0;
unsigned long last_send_time = 0;
bool is_aborted = false;
unsigned long last_abort_msg_time = 0;

void send_valves_position();
void send_hydraulic_sensors();
void send_uart_stats();
void goToSafeState();
void enterAbortState();

void setup()
{
    Serial.begin(9600);
    delay(100);
    analogReadResolution(12);

    // PWM values unchanged from main firmware
    valve_feed_oxidizer = new ServoValve(
        27,
        460,
        2280,
        A14,
        10,
        84
    );

    decoupler_oxidizer = new Decoupler(26, 25);
    valve_vent_oxidizer = new ElectroValve(14);

    pressure_sensor_oxidizer = new PressureConverter(34);

    goToSafeState();

    last_command_time = millis();
}

void loop()
{
    if (is_aborted && (millis() - last_abort_msg_time > TIMEOUT_ABORT_ms))
        is_aborted = false;

    auto msg = messenger.receive();

    if (msg != nullptr)
    {
        switch (msg->getID())
        {
            case GSUART::MsgID::ZAWORY_STEROWANIE:
            {
                if (is_aborted)
                    break;

                auto* cmd =
                    static_cast<const GSUART::MsgZaworySterowanie*>(msg);

                // direct PWM steering
                if (cmd->valve_feed_oxidizer)
                    valve_feed_oxidizer->open(cmd->valve_feed_oxidizer);
                else
                    valve_feed_oxidizer->close();

                // vent valve
                if (cmd->valve_vent_oxidizer)
                    valve_vent_oxidizer->open();
                else
                    valve_vent_oxidizer->close();

                // decoupler
                if (cmd->decoupler_oxidizer)
                    decoupler_oxidizer->open();
                else
                    decoupler_oxidizer->close();

                last_command_time = millis();

                send_valves_position();

                break;
            }

            case GSUART::MsgID::ABORT:
            {
                auto* abort_msg =
                    static_cast<const GSUART::MsgAbort*>(msg);

                if (abort_msg->abort)
                {
                    is_aborted = true;
                    last_abort_msg_time = millis();
                    last_command_time = millis();
                    enterAbortState();
                }

                break;
            }

            case GSUART::MsgID::MSG_PING:
            {
                auto* ping =
                    static_cast<const GSUART::MsgPing*>(msg);

                GSUART::MsgPong pong;
                pong.seq = ping->seq;

                messenger.send(pong);

                break;
            }

            default:
                break;
        }
    }

    // communication timeout
    if (!is_aborted && millis() - last_command_time > COMMAND_TIMEOUT_ms)
    {
        goToSafeState();
    }

    // periodic state feedback
    if (millis() - last_send_time > INTERVAL_SEND_VALVES_POSITION)
    {
        send_valves_position();
    }

    if (millis() - last_hydro_send_time > INTERVAL_SEND_HYDRAULIC_SENSORS)
    {
        send_hydraulic_sensors();
    }

    if (millis() - last_uart_stats_send_time > INTERVAL_SEND_UART_STATS)
    {
        send_uart_stats();
    }

    delay(1);
}

void send_valves_position()
{
    last_send_time = millis();

    msgValves.valve_feed_oxidizer =
        valve_feed_oxidizer->readPosition();

    msgValves.valve_vent_oxidizer =
        valve_vent_oxidizer->isOpen();

    msgValves.decoupler_oxidizer =
        decoupler_oxidizer->isOpen();

    messenger.send(msgValves);
}

void goToSafeState()
{
    valve_feed_oxidizer->close();

    valve_vent_oxidizer->close();

    decoupler_oxidizer->close();
}

void enterAbortState()
{
    valve_feed_oxidizer->close();

    valve_vent_oxidizer->open();

    decoupler_oxidizer->open();
}

void send_hydraulic_sensors()
{
    last_hydro_send_time = millis();

    msgHydroSensors.pressure_bar =
        pressure_sensor_oxidizer->readPressure();

    messenger.send(msgHydroSensors);
//    Serial.println();/
//    Serial.println(msgH/ydroSensors.pressure_bar);
}

void send_uart_stats()
{
    last_uart_stats_send_time = millis();
    messenger.sendUartStats();
}
