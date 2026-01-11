#include "Arduino.h"
#include "PressureConverter.hpp"

PressureConverter::PressureConverter(int adc_pin)
{
  this->adc_pin = adc_pin;
}

float PressureConverter::readPressure()
{
  #define ADC_MAX       1023.0f
  #define PRESSURE_MAX  100.0f
  float new_pressure = float(analogRead(this->adc_pin)) / ADC_MAX * PRESSURE_MAX;
  float new_time = millis();
  unsigned long time_distance = new_time - this->last_reading_time;

  this->pressure_change_per_second = (new_pressure - this->pressure_Bar) * 1000.0f / time_distance; 
  
  this->last_reading_time = millis();
  this->pressure_Bar = new_pressure;
  return this->pressure_Bar;
}

float PressureConverter::pressure()
{
  return this->pressure_Bar;
}

float PressureConverter::deltaP()
{
  return this->pressure_change_per_second;
}