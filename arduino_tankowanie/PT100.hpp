#include "Adafruit_MAX31865.h"
#include "Arduino.h"

class PT100
{
public:
  PT100(int spi_chip_select_pin, int rdy_pin);
  ~PT100();

  // performs a new reading from the SPI device, saves the value to private variable
  // returns the saved value
  float readTemperature();

  // only returns last saved value of the temperature from the private variable, does not perform a new reading
  float temperature();

  // returns a temperature change - value represents a temperature change per second - calculation based on last reading
  float deltaT();

private:
  int spi_chip_select_pin;
  int conversion_ready_pin;
  Adafruit_MAX31865* MAX31865_sensor;

  float ratio;
  float resistance;
  float temperature_C;

  float temperature_change_per_second;
  unsigned long last_reading_time = 0;
};