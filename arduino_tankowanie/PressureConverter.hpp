#include "Arduino.h"

class PressureConverter
{
public:
  PressureConverter(int adc_pin);

  // performs a new reading from the correspoding adc, saves the value to private variable
  // returns the saved value
  float readPressure();

  // only returns last saved value of the pressure from the private variable, does not perform a new reading
  float pressure();

  // returns a pressure change - value represents a pressure change per second - calculation based on last reading
  float deltaP();

private:
  int adc_pin;

  float pressure_Bar;
  float pressure_change_per_second;
  unsigned long last_reading_time = 0;
};