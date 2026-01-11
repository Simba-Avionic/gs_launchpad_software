#include "Arduino.h"
#include "PT100.hpp"

PT100::PT100(int spi_chip_select_pin, int rdy_pin)
{
  this->spi_chip_select_pin = spi_chip_select_pin;
  this->conversion_ready_pin = rdy_pin;

  pinMode(this->conversion_ready_pin, INPUT_PULLUP);

  MAX31865_sensor = new Adafruit_MAX31865(spi_chip_select_pin);
  MAX31865_sensor->begin(MAX31865_4WIRE);
  
  MAX31865_sensor->enableBias(true);
  MAX31865_sensor->enable50Hz(true);
  MAX31865_sensor->autoConvert(true);
}

PT100::~PT100()
{
  delete MAX31865_sensor;
}

float PT100::readTemperature()
{
  if (digitalRead(this->conversion_ready_pin) == LOW)
  {
    #define RNOMINAL  100.0
    #define RREF      430.0
    uint16_t rtd = MAX31865_sensor->readRegister16(MAX31865_RTDMSB_REG);
    rtd >>= 1;

    this->ratio = (float)rtd / 32768.0f;
    this->resistance = RREF * ratio;
    float new_temperature = MAX31865_sensor->calculateTemperature(rtd, RNOMINAL, RREF);
    
    unsigned long new_time = millis();
    unsigned long time_distance = new_time - this->last_reading_time;

    this->temperature_change_per_second = (new_temperature - this->temperature_C) * 1000.f / time_distance;

    this->temperature_C = new_temperature;
    this->last_reading_time = new_time;
    return this->temperature_C;
  }
  return -300.0f;
}

float PT100::temperature()
{
  return temperature_C;
}

float PT100::deltaT()
{
  return this->temperature_change_per_second;
}