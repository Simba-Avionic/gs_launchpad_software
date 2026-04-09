#include "ServoValve.hpp"

#include "Arduino.h"
#include <Servo.h>

ServoValve::ServoValve(int pin_servo, int pwm_min, int pwm_max, int pin_potentiometer, int map_pos_close, int map_pos_open)
{
  // Serial.print("constructorXD ");
  // Serial.println(pin_servo);
  servo.attach(pin_servo, pwm_min, pwm_max);
  potentiometer_pin = pin_potentiometer;
  this->map_pos_close = map_pos_close;
  this->map_pos_open = map_pos_open;
}

int ServoValve::readPosition()
{
  position_last = position_current;
  int new_reading = analogRead(potentiometer_pin) / 10;  // 0-1023  ->  mapowanie np. 11-86 ->  0-100
  position_current = map(new_reading, map_pos_close, map_pos_open, 0, 100);

  return position_current;
}

int ServoValve::position()
{
  return position_current;
}

int ServoValve::posChange()
{
  if (position_current > position_last)
    return position_current - position_last;
  else
    return position_last - position_current;
}

void ServoValve::setPosition(int deg)
{
  servo.write(deg);
}

void ServoValve::openWithExtraSteps()
{
  
  // static int open_levels[] = { 180, 130, 90, 50, 25, 0 };
  static int open_levels[] = { 180, 155, 130, 90, 50, 0 };
  
  if (last_command == Command::STEP_OPEN)
  {
    if (millis() >= last_step_open_time + STEP_TIME)
    {
      open_idx++;
      if (open_idx > 5) open_idx = 5;
      servo.write(open_levels[open_idx]);
      last_step_open_time = millis();
    }
  }
  else
  {
    open_idx = 1;
    servo.write(open_levels[open_idx]);
    last_step_open_time = millis();
  }

  last_command = Command::STEP_OPEN;
}

void ServoValve::open(uint8_t position)
{
  if (position == 2)
    servo.write(0);
  else if (position == 1)
    servo.write(130);
  last_command = Command::SET_OPEN;
}

void ServoValve::close()
{
  servo.write(180);
  last_command = Command::HARD_CLOSE;
}


Decoupler::Decoupler(int pin_1, int pin_2)
  : pin_1(pin_1), pin_2(pin_2)
{
  pinMode(pin_1, OUTPUT);
  pinMode(pin_2, OUTPUT);
}

void Decoupler::open()
{
  digitalWrite(pin_1, HIGH);
  digitalWrite(pin_2, LOW);
  // Serial.println("Open decoupler!");
  is_open = true;
}

void Decoupler::close()
{
  digitalWrite(pin_1, LOW);
  digitalWrite(pin_2, HIGH);
  // Serial.println("Close decoupler!");
  is_open = false;
}

void Decoupler::dontMove()
{
  digitalWrite(pin_1, HIGH);
  digitalWrite(pin_2, HIGH);
  // Serial.println("Stop decoupler!");
}

bool Decoupler::isOpen()
{
  return is_open;
}


ElectroValve::ElectroValve(int pin)
  : pin(pin)
{
  pinMode(pin, OUTPUT);
}

void ElectroValve::open()
{
  digitalWrite(pin, HIGH);
  is_open = true;
}

void ElectroValve::close()
{
  digitalWrite(pin, LOW);
  is_open = false;
}

bool ElectroValve::isOpen()
{
  return is_open;
}