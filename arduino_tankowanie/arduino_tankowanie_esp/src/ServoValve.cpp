#include "ServoValve.hpp"

#include "Arduino.h"

ServoValve::ServoValve(int pin_servo, int pwm_min, int pwm_max, int pin_potentiometer, int map_pos_close, int map_pos_open)
{
  servo_pin = pin_servo;
  pwm_min_us = pwm_min;
  pwm_max_us = pwm_max;

  // ESP32Servo clamps pulse widths below 500 us. This valve is calibrated
  // down to 400 us, so drive the ESP32 LEDC peripheral directly.
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(servo_pin, PWM_CHANNEL);

  potentiometer_pin = pin_potentiometer;
  this->map_pos_close = map_pos_close;
  this->map_pos_open = map_pos_open;
}

int ServoValve::readPosition()
{
  position_last = position_current;
  int raw_reading = analogRead(potentiometer_pin);

  // Keep the existing 0..102 calibration scale, but avoid the preliminary
  // integer map that discarded resolution and introduced 1-2% steps.
  float calibrated_reading = static_cast<float>(raw_reading) * 102.0f / 4095.0f;
  float calibrated_position =
      (calibrated_reading - map_pos_close) * 100.0f /
      (map_pos_open - map_pos_close);
  position_current = static_cast<int>(roundf(calibrated_position));
  position_current = constrain(position_current, 0, 100);

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
  writeAngle(deg);
}

void ServoValve::writeAngle(int deg)
{
  int constrained_deg = constrain(deg, 0, 180);
  int pulse_width_us = map(constrained_deg, 0, 180, pwm_min_us, pwm_max_us);
  writePulseMicroseconds(pulse_width_us);
}

void ServoValve::writePulseMicroseconds(int pulse_width_us)
{
  int constrained_pulse = constrain(pulse_width_us, pwm_min_us, pwm_max_us);
  constexpr uint32_t PWM_MAX_DUTY = (1UL << PWM_RESOLUTION_BITS) - 1;
  uint32_t duty = static_cast<uint32_t>(
      static_cast<uint64_t>(constrained_pulse) * PWM_FREQUENCY_HZ * PWM_MAX_DUTY /
      1000000ULL);
  ledcWrite(PWM_CHANNEL, duty);
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
      writeAngle(open_levels[open_idx]);
      last_step_open_time = millis();
    }
  }
  else
  {
    open_idx = 1;
    writeAngle(open_levels[open_idx]);
    last_step_open_time = millis();
  }

  last_command = Command::STEP_OPEN;
}

void ServoValve::open(uint8_t position)
{
  if (position == 2)
    writeAngle(0);
  else if (position == 1)
    writeAngle(90);
  last_command = Command::SET_OPEN;
}

void ServoValve::close()
{
  writeAngle(180);
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
