// arduino sterujące zaworami tankowania rakiety w projekcie SimBa
// także odczytuje czujniki i steruje decouplerem

#include <Wire.h>
#include "DFRobot_INA219.h"
#include "Servo.h"
#include "ServoValve.hpp"

// #define GSUART_PLATFORM_ARDUINO     1
// #define GSUART_PLATFORM_RPI_UBUNTU  0
// #define GSUART_PLATFORM GSUART_PLATFORM_ARDUINO
#include "GSUART.hpp"
GSUART::Messenger messenger(&Serial);
GSUART::MsgPowerTanking msgPower;
GSUART::MsgZaworyPozycja msgValves;

ServoValve* valve_feed_oxidizer;
ServoValve* valve_feed_pressurizer;

Decoupler* decoupler_oxidizer;
Decoupler* decoupler_pressurizer;

ElectroValve* valve_vent_oxidizer;
ElectroValve* valve_vent_pressurizer;

DFRobot_INA219_IIC czujnik_pradu_7v(&Wire, INA219_I2C_ADDRESS4);
DFRobot_INA219_IIC czujnik_pradu_12v(&Wire, INA219_I2C_ADDRESS1);

#define INTERVAL_CHECK_COMMANDS_INPUT 10
#define INTERVAL_SEND_VALVES_POSITION 700
#define INTERVAL_CHECK_VALVES_POSITION 100
#define INTERVAL_SEND_POWER_SENSORS 500
#define INTERVAL_CHECK_POWER_SENSORS 100
#define INTERVAL_SEND_UART_STATS 4000
#define INTERVAL_CHECK_BUTTONS 100
#define DELAY_MAIN_LOOP 1

#define THRESHOLD_VALVE_POS_CHANGE_SEND 4
#define THRESHOLD_CZUJNIK_PRADU_CURRENT_CHANGE_mA_SEND 85
#define THRESHOLD_OSTATNIA_KOMENDA_ARRIVED_TIMEOUT_ms 3000

#define TIMEOUT_ABORT_ms 30000 // 30s

#define PIN_BUTTON_DECOUPLERS 10
#define PIN_BUTTON_VENTS 8
#define PIN_BUTTON_FEEDS 7

#define PIN_DIODE 13

unsigned long time_last_check_commands_input = 0;
unsigned long time_last_sent_valves_position = 0;
unsigned long time_last_sent_power_sensors = 0;
unsigned long time_last_sent_uart_stats = 0;
unsigned long time_last_check_valves_position = 0;
unsigned long time_last_check_power_sensors = 0;
unsigned long time_last_check_buttons = 0;

unsigned long last_command_time = 0;

void check_commands_input();
void check_valves_position();
void check_power_sensors();
void check_buttons();
void send_valves_position();
void send_power_sensors();
void send_uart_stats();

void goToSafeState();
void abort();

void setup() {
  pinMode(PIN_DIODE, OUTPUT);
  digitalWrite(PIN_DIODE, LOW);
  pinMode(PIN_BUTTON_DECOUPLERS, INPUT_PULLUP);
  pinMode(PIN_BUTTON_VENTS, INPUT_PULLUP);
  pinMode(PIN_BUTTON_FEEDS, INPUT_PULLUP);

  Serial.begin(9600);
  while(!Serial);

  valve_feed_oxidizer = new ServoValve(9, 600, 2440, A0);
  valve_feed_pressurizer = new ServoValve(6, 600, 2440, A1);

  decoupler_oxidizer = new Decoupler(2, 3);
  decoupler_pressurizer = new Decoupler(4, 5);

  valve_vent_oxidizer = new ElectroValve(11);
  valve_vent_pressurizer = new ElectroValve(12);

  while(czujnik_pradu_7v.begin() != true) {
    // Serial.println("czujnik_pradu_7v begin failed");
    // tutaj wyslij blad zalaczenia czujnika
    delay(1000);
  }
  // Serial.println("czujnik_pradu_7v begin OK");

  while(czujnik_pradu_12v.begin() != true) {
    // Serial.println("czujnik_pradu_12v begin failed");
    // tutaj wyslij blad zalaczenia czujnika
    delay(1000);
  }
  // Serial.println("czujnik_pradu_12v begin OK");

  goToSafeState();
  digitalWrite(PIN_DIODE, HIGH);
}

void loop() {
  unsigned long time_now = millis();

  if (time_now >= time_last_check_buttons + INTERVAL_CHECK_BUTTONS)
    check_buttons();

  if(time_now >= time_last_check_commands_input + INTERVAL_CHECK_COMMANDS_INPUT)
    check_commands_input();

  if(time_now >= time_last_sent_valves_position + INTERVAL_SEND_VALVES_POSITION)
    send_valves_position();

  if(time_now >= time_last_sent_power_sensors + INTERVAL_SEND_POWER_SENSORS)
    send_power_sensors();

  if(time_now >= time_last_sent_uart_stats + INTERVAL_SEND_UART_STATS)
    send_uart_stats();

  if(time_now >= time_last_check_valves_position + INTERVAL_CHECK_VALVES_POSITION)
    check_valves_position();

  if(time_now >= time_last_check_power_sensors + INTERVAL_CHECK_POWER_SENSORS)
    check_power_sensors();

  delay(DELAY_MAIN_LOOP);
}

void check_commands_input()
{
  time_last_check_commands_input = millis();

  static bool is_aborted = false;
  static unsigned long last_abort_msg_time = 0;

  if (is_aborted && ( millis() - last_abort_msg_time > TIMEOUT_ABORT_ms )) is_aborted = false;

  // if there was no command for a certain ammount of time there could be some connection problem and we should go to safe state of the system
  if (!is_aborted && millis()-last_command_time > THRESHOLD_OSTATNIA_KOMENDA_ARRIVED_TIMEOUT_ms)
  {
    last_command_time = millis();
    goToSafeState();
  }
  
  auto msg = messenger.receive();

  if (msg == nullptr) return;

  // Serial.println((int)msg->getID());

  switch (msg->getID())
  {
    case GSUART::MsgID::ZAWORY_STEROWANIE:
    {
      if (is_aborted) break;
      const GSUART::MsgZaworySterowanie* msgZaworySterowanie = static_cast<const GSUART::MsgZaworySterowanie*>(msg);
      msgZaworySterowanie->valve_feed_oxidizer      ? valve_feed_oxidizer->openWithExtraSteps()     : valve_feed_oxidizer->close();
      msgZaworySterowanie->valve_feed_pressurizer   ? valve_feed_pressurizer->openWithExtraSteps()  : valve_feed_pressurizer->close();
      msgZaworySterowanie->valve_vent_oxidizer      ? valve_vent_oxidizer->open()                   : valve_vent_oxidizer->close();
      msgZaworySterowanie->valve_vent_pressurizer   ? valve_vent_pressurizer->open()                : valve_vent_pressurizer->close();
      msgZaworySterowanie->decoupler_oxidizer       ? decoupler_oxidizer->open()                    : decoupler_oxidizer->close();
      msgZaworySterowanie->decoupler_pressurizer    ? decoupler_pressurizer->open()                 : decoupler_pressurizer->close();
      send_valves_position();
      last_command_time = millis();
      break;
    }
    case GSUART::MsgID::ABORT:
    {
      const GSUART::MsgAbort* msgAbort = static_cast<const GSUART::MsgAbort*>(msg);
      if (msgAbort->abort)
      {
        is_aborted = true;
        last_abort_msg_time = millis();
        last_command_time = millis();
        abort();
      }
      break;
    }
    case GSUART::MsgID::PING:
    {
      const GSUART::MsgPing* msgPing = static_cast<const GSUART::MsgPing*>(msg);
      GSUART::MsgPong msgPong;
      msgPong.seq = msgPing->seq;
      messenger.send(msgPong);
      break;
    }
    default:
    // unknown message type
    break;
  }
}

void check_valves_position()
{
  time_last_check_valves_position = millis();
  auto pos1 = valve_feed_oxidizer->readPosition();
  auto pos2 = valve_feed_pressurizer->readPosition();
  if (valve_feed_oxidizer->posChange() >= THRESHOLD_VALVE_POS_CHANGE_SEND || valve_feed_pressurizer->posChange() >= THRESHOLD_VALVE_POS_CHANGE_SEND)
    send_valves_position();
}

void check_power_sensors()
{
  time_last_check_power_sensors = millis();
  static float czujnik_pradu_7v_last_current_mA = 0.0f;
  static float czujnik_pradu_12v_last_current_mA = 0.0f;

  auto current_1 = czujnik_pradu_7v.getCurrent_mA();
  auto current_2 = czujnik_pradu_12v.getCurrent_mA();
  auto diff1 = current_1 - czujnik_pradu_7v_last_current_mA;
  if (diff1 < 0) diff1 *= -1.0f;

  auto diff2 = current_2 - czujnik_pradu_12v_last_current_mA;
  if (diff2 < 0) diff2 *= -1.0f;

  if (diff1 >= THRESHOLD_CZUJNIK_PRADU_CURRENT_CHANGE_mA_SEND || diff2 >= THRESHOLD_CZUJNIK_PRADU_CURRENT_CHANGE_mA_SEND)
    send_power_sensors();

  czujnik_pradu_7v_last_current_mA = current_1;
  czujnik_pradu_12v_last_current_mA = current_2;
}

void check_buttons()
{
  time_last_check_buttons = millis();

  static bool button_decouplers_last_pressed = false;
  static bool button_vents_last_pressed = false;
  static bool button_feeds_last_pressed = false;

  bool anything_pressed = false;
  bool send_valves = false;

  if (digitalRead(PIN_BUTTON_DECOUPLERS) == LOW) {
    anything_pressed = true;
    
    decoupler_oxidizer->open();
    decoupler_pressurizer->open();

    if (!button_decouplers_last_pressed) send_valves = true;
    button_decouplers_last_pressed = true;
  }
  else if (button_decouplers_last_pressed) {
    button_decouplers_last_pressed = false;
    decoupler_oxidizer->close();
    decoupler_pressurizer->close();
    send_valves = true;
  }

  if (digitalRead(PIN_BUTTON_VENTS) == LOW) {
    anything_pressed = true;

    valve_vent_oxidizer->open();
    valve_vent_pressurizer->open();

    if (!button_vents_last_pressed) send_valves = true;
    button_vents_last_pressed = true;
  }
  else if (button_vents_last_pressed) {
    button_vents_last_pressed = false;
    valve_vent_oxidizer->close();
    valve_vent_pressurizer->close();
    send_valves = true;
  }

  if (digitalRead(PIN_BUTTON_FEEDS) == LOW) {
    anything_pressed = true;
    
    valve_feed_oxidizer->openWithExtraSteps();
    valve_feed_pressurizer->openWithExtraSteps();

    if (!button_feeds_last_pressed) send_valves = true;
    button_feeds_last_pressed = true;
  }
  else if (button_feeds_last_pressed) {
    button_feeds_last_pressed = false;
    valve_feed_oxidizer->close();
    valve_feed_pressurizer->close();
    send_valves = true;
  }

  if (send_valves)
    send_valves_position();
  if (anything_pressed)
    last_command_time = millis();
}

void send_valves_position()
{
  time_last_sent_valves_position = millis();
  msgValves.valve_feed_oxidizer = valve_feed_oxidizer->position();
  msgValves.valve_feed_pressurizer = valve_feed_pressurizer->position();
  msgValves.valve_vent_oxidizer = valve_vent_oxidizer->isOpen();
  msgValves.valve_vent_pressurizer = valve_vent_pressurizer->isOpen();
  msgValves.decoupler_oxidizer = decoupler_oxidizer->isOpen();
  msgValves.decoupler_pressurizer = decoupler_pressurizer->isOpen();
  // messenger.send(msgValves);
}

void send_power_sensors()
{
  time_last_sent_power_sensors = millis();
  msgPower.v7_4.V = czujnik_pradu_7v.getBusVoltage_V();
  msgPower.v7_4.mA = czujnik_pradu_7v.getCurrent_mA();
  msgPower.v12.V = czujnik_pradu_12v.getBusVoltage_V();
  msgPower.v12.mA = czujnik_pradu_12v.getCurrent_mA();
  // messenger.send(msgPower);
}

void send_uart_stats()
{
  time_last_sent_uart_stats = millis();
  messenger.sendUartStats();
}

void goToSafeState()
{
  valve_feed_oxidizer->close();
  valve_feed_pressurizer->close();

  valve_vent_oxidizer->close();
  valve_vent_pressurizer->close();

  decoupler_oxidizer->close();
  decoupler_pressurizer->close();
}

void abort()
{
  valve_feed_oxidizer->close();
  valve_feed_pressurizer->close();

  valve_vent_oxidizer->open();
  valve_vent_pressurizer->open();

  decoupler_oxidizer->open();
  decoupler_pressurizer->open();
}
