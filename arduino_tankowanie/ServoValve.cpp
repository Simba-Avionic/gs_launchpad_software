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