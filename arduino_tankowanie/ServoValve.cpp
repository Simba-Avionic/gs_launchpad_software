#include "ServoValve.hpp"

#include "Arduino.h"
#include <Servo.h>

ServoValve::ServoValve(int pin_servo, int pwm_min, int pwm_max, int pin_potentiometer)
{
  // Serial.print("constructorXD ");
  // Serial.println(pin_servo);
  servo.attach(pin_servo, pwm_min, pwm_max);
  potentiometer_pin = pin_potentiometer;
}

int ServoValve::readPosition()
{
  position_last = position_current;
  position_current = analogRead(potentiometer_pin);
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
  Serial.println("Open decoupler!");
}

void Decoupler::close()
{
  digitalWrite(pin_1, LOW);
  digitalWrite(pin_2, HIGH);
  Serial.println("Close decoupler!");
}

void Decoupler::dontMove()
{
  digitalWrite(pin_1, HIGH);
  digitalWrite(pin_2, HIGH);
  Serial.println("Stop decoupler!");
}


ElectroValve::ElectroValve(int pin)
  : pin(pin)
{
  pinMode(pin, OUTPUT);
}

void ElectroValve::open()
{
  digitalWrite(pin, HIGH);
}

void ElectroValve::close()
{
  digitalWrite(pin, LOW);
}