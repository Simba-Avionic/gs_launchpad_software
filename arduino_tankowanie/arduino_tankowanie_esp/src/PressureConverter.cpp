#include "Arduino.h"
#include "PressureConverter.hpp"

PressureConverter::PressureConverter(int adc_pin)
{
  this->adc_pin = adc_pin;
}

float PressureConverter::readPressure()
{
  constexpr float ADC_MAX = 4095.0f;
  constexpr float PRESSURE_MAX = 100.0f;
  constexpr float VOLTAGE_MAX = 3.3f;
  
  int raw_value = analogRead(this->adc_pin);
  voltage_V = float(raw_value) / ADC_MAX * VOLTAGE_MAX;
  float new_pressure = float(raw_value) / ADC_MAX * PRESSURE_MAX;
  unsigned long new_time = millis();
  unsigned long time_distance = new_time - this->last_reading_time;

  if (time_distance > 0)
    this->pressure_change_per_second = (new_pressure - this->pressure_Bar) * 1000.0f / time_distance;
  
  this->last_reading_time = new_time;
  this->pressure_Bar = new_pressure;
  return this->pressure_Bar;
}

float PressureConverter::pressure()
{
  return this->pressure_Bar;
}

float PressureConverter::raw_voltage()
{
  return this->voltage_V;
}

float PressureConverter::deltaP()
{
  return this->pressure_change_per_second;
}
