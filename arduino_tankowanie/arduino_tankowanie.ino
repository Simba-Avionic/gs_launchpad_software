// arduino sterujące zaworami tankowania rakiety w projekcie SimBa
// także odczytuje czujniki i steruje decouplerem

#include <Wire.h>
#include "DFRobot_INA219.h"
#include "Servo.h"
#include "ServoValve.hpp"

// #define GSUART_PLATFORM_ARDUINO     0
// #define GSUART_PLATFORM_RPI_UBUNTU  1
// #define GSUART_PLATFORM GSUART_PLATFORM_ARDUINO
// #include "../include/oberon/GSUART.hpp"

ServoValve* valve_feed_oxidizer;
ServoValve* valve_feed_pressurizer;

Decoupler* decoupler_oxidizer;
Decoupler* decoupler_pressurizer;

ElectroValve* valve_vent_oxidizer;
ElectroValve* valve_vent_pressurizer;

DFRobot_INA219_IIC czujnik_pradu_7v(&Wire, INA219_I2C_ADDRESS4);
DFRobot_INA219_IIC czujnik_pradu_12v(&Wire, INA219_I2C_ADDRESS1);

#define INTERVAL_CHECK_COMMANDS_INPUT 10
#define INTERVAL_SEND_VALVES_POSITION 1000
#define INTERVAL_CHECK_VALVES_POSITION 200
#define INTERVAL_SEND_POWER_SENSORS 1000
#define INTERVAL_CHECK_POWER_SENSORS 200
#define INTERVAL_SEND_UART_STATS 4000
#define INTERVAL_CHECK_BUTTONS 100
#define DELAY_MAIN_LOOP 1

#define THRESHOLD_VALVE_POS_CHANGE_SEND 7
#define THRESHOLD_CZUJNIK_PRADU_CURRENT_CHANGE_mA_SEND 85
#define THRESHOLD_OSTATNIA_KOMENDA_ARRIVED_TIMEOUT_ms 3000

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

void printPower();
void printPosition(int pos1, int pos2);

void setup() {
  pinMode(PIN_DIODE, OUTPUT);
  digitalWrite(PIN_DIODE, LOW);
  pinMode(PIN_BUTTON_DECOUPLERS, INPUT_PULLUP);
  pinMode(PIN_BUTTON_VENTS, INPUT_PULLUP);
  pinMode(PIN_BUTTON_FEEDS, INPUT_PULLUP);

  Serial.begin(9600);
  while(!Serial);

  valve_feed_oxidizer = new ServoValve(9, 600, 2440, A0);
  valve_feed_oxidizer->close();
  valve_feed_pressurizer = new ServoValve(6, 600, 2440, A1);
  valve_feed_pressurizer->close();

  decoupler_oxidizer = new Decoupler(2, 3);
  decoupler_oxidizer->close();
  decoupler_pressurizer = new Decoupler(4, 5);
  decoupler_pressurizer->close();

  valve_vent_oxidizer = new ElectroValve(11);
  valve_vent_oxidizer->close();
  valve_vent_pressurizer = new ElectroValve(12);
  valve_vent_pressurizer->close();

  while(czujnik_pradu_7v.begin() != true) {
    Serial.println("czujnik_pradu_7v begin failed");
    // tutaj wyslij blad zalaczenia czujnika
    delay(1000);
  }
  Serial.println("czujnik_pradu_7v begin OK");

  while(czujnik_pradu_12v.begin() != true) {
    Serial.println("czujnik_pradu_12v begin failed");
    // tutaj wyslij blad zalaczenia czujnika
    delay(1000);
  }
  Serial.println("czujnik_pradu_12v begin OK");

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
  static bool first = true;

  time_last_check_commands_input = millis();
  if(Serial.available() > 0)
  {
    int byte = Serial.read();
    if(byte == -1) return;
    if(byte == 10) return;

    byte -= '0';
    int pos_servo = byte*20;
    if (first)
      valve_feed_oxidizer->setPosition(pos_servo);
    else
      valve_feed_pressurizer->setPosition(pos_servo);

    first = !first;

    last_command_time = millis();
  }

  // if there was no command for a certain ammount of time there could be some connection problem and we should go to safe state of the system
  if (millis()-last_command_time > THRESHOLD_OSTATNIA_KOMENDA_ARRIVED_TIMEOUT_ms)
  {
    goToSafeState();
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

  if (digitalRead(PIN_BUTTON_DECOUPLERS) == LOW) {
    anything_pressed = true;
    // Serial.println("Otworzyc decouplery");
    button_decouplers_last_pressed = true;
    decoupler_oxidizer->open();
    decoupler_pressurizer->open();
  }
  else if (button_decouplers_last_pressed) {
    // Serial.println("Zamknac decouplery");
    button_decouplers_last_pressed = false;
    decoupler_oxidizer->close();
    decoupler_pressurizer->close();
  }

  if (digitalRead(PIN_BUTTON_VENTS) == LOW) {
    anything_pressed = true;
    // Serial.println("Otworzyc venty");
    button_vents_last_pressed = true;
    valve_vent_oxidizer->open();
    valve_vent_pressurizer->open();
  }
  else if (button_vents_last_pressed) {
    // Serial.println("Zamknac venty");
    button_vents_last_pressed = false;
    valve_vent_oxidizer->close();
    valve_vent_pressurizer->close();
  }

  if (digitalRead(PIN_BUTTON_FEEDS) == LOW) {
    anything_pressed = true;
    // Serial.println("Otworzyc feedy");
    button_feeds_last_pressed = true;
    valve_feed_oxidizer->openWithExtraSteps();
    valve_feed_pressurizer->openWithExtraSteps();
  }
  else if (button_feeds_last_pressed) {
    // Serial.println("Zamknac feedy");
    button_feeds_last_pressed = false;
    valve_feed_oxidizer->close();
    valve_feed_pressurizer->close();
  }

  if (anything_pressed)
    last_command_time = millis();
}

void send_valves_position()
{
  time_last_sent_valves_position = millis();
  auto pos1 = valve_feed_oxidizer->position();
  auto pos2 = valve_feed_pressurizer->position();
  printPosition(pos1, pos2);
}

void send_power_sensors()
{
  time_last_sent_power_sensors = millis();
  printPower();
}

void send_uart_stats()
{
  time_last_sent_uart_stats = millis();
}

void printPosition(int pos1, int pos2)
{
  // Serial.print(pos1);
  // Serial.print(" ");
  // Serial.println(pos2);
}

void printPower()
{
    // Serial.print(czujnik_pradu_1.getBusVoltage_V(), 2);
    // Serial.print("V ");
    // Serial.print(czujnik_pradu_1.getShuntVoltage_mV(), 3);
    // Serial.print("mV ");
    Serial.print(czujnik_pradu_7v.getCurrent_mA(), 1);
    Serial.print("mA ");
    Serial.print(czujnik_pradu_12v.getCurrent_mA(), 1);
    Serial.print("mA ");
    // Serial.print(czujnik_pradu_1.getPower_mW(), 1);
    // Serial.print("mW ");
    Serial.println("");
}

void goToSafeState()
{
  valve_feed_oxidizer->setPosition(160);
  valve_feed_pressurizer->setPosition(160);
  // tutaj także zawory ventów
  // tutaj nie decoupler chyba
}
