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

  void openWithExtraSteps();

  void close();

  void check();

private:
  void hardClose();
  void softClose();

  Servo servo;
  int potentiometer_pin;
  int position_last = 0, position_current = 0;

  #define STEP_TIME 1000
  unsigned long last_step_open_time = 0;
  int open_idx = 0;

  enum Command {
    HARD_CLOSE = 0, SOFT_CLOSE, STEP_OPEN, SET_OPEN 
  } last_command = SET_OPEN;
};

class Decoupler
{
public:
  Decoupler(int pin_1, int pin_2);

  // set electric polarization to open decoupler, decoupler will be opening till it reaches full openness or phisical limit
  // opening decoupler is intended to decouple from the rocket
  void open();

  // set electric polarization to close decoupler, decoupler will be closing till it reaches full closeness or phisical limit
  // closing decoupler is intended to tighten the rocket connection
  void close();

  // disable electric polarization of the decoupler, decoupler will not move on its own
  // this state may be usefull later
  void dontMove();

  // returns current steering signal
  bool isOpen();

private:
  int pin_1;
  int pin_2;

  bool is_open = false;
};

class ElectroValve
{
public:
  ElectroValve(int pin);

  // set electric polarization to open the valve, sends current to the coil, which opens the valve using an electromagnet
  void open();

  // set electric polarization to close the valve, sends current to the coil, which closes the valve using an electromagnet
  // currently used valve is Normally Closed so this is the default state, also with no power at all
  void close();

  // returns current steering signal
  bool isOpen();

private:
  int pin;
  
  bool is_open = false;
};