#include <Wire.h>
#include <Servo.h>

class ServoValve
{
public:
  ServoValve(int pin_servo, int pwm_min, int pwm_max, int pin_potentiometer);

  // read value from potentiometer, DOES perform new adc reading
  int readPosition();

  // returns last potentiometer position, DOES NOT perform new adc reading
  int position();

  // returns a absolute difference in last 2 position readings, DOES NOT perform new adc reading
  int posChange();

  // sets pwm signal width
  void setPosition(int deg);

private:
  Servo servo;
  int potentiometer_pin;
  int position_last = 0, position_current = 0;
};