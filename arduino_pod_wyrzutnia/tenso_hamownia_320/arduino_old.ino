#include "Wire.h"
#include <OneWire.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"

// include GSUART - these files should be in the same directory as this file, define GSUART_PLATFORM before
// #define GSUART_PLATFORM      //    0 - arduino - GSUART_PLATFORM_ARDUINO
                                //    1 - rpi ubuntu - GSUART_PLATFORM_RPI_UBUNTU
#define GSUART_PLATFORM_ARDUINO     0
#define GSUART_PLATFORM_RPI_UBUNTU  1
#define GSUART_PLATFORM GSUART_PLATFORM_ARDUINO
#include "GSUART.hpp"
GSUART::Messenger messenger(&Serial);
GSUART::MsgTenso msgTenso;
GSUART::MsgTemperature msgTemperature;

#define SEND_INTERVAL_TEMPERATURE 1000
#define SEND_INTERVAL_UART_STATS  4000

OneWire  ds(10);  // on pin 10 (a 4.7K resistor is necessary)
byte temp_addr[8] = {0x28, 0xC2, 0x18, 0x3A, 0x0A, 0x00, 0x00, 0x94};

enum class TENSO_SIDE
{
    LEFT, RIGHT
};
#define TCAADDR 0x70
#define nau_left_mpx_pin   4
#define nau_right_mpx_pin  6
NAU7802 nau_left;
NAU7802 nau_right;

bool connectTenso(NAU7802& nau);

bool readTenso(TENSO_SIDE tenso_side);
bool readTemperature();

void sendTenso();
void sendUartStats();
void sendTemperature();

void sendTensoText();
void sendTemperatureText();

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
  delay(2);
}

void setup() {
    Wire.begin();
    // Serial.begin(9600);
    Serial.begin(115200);
    
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    tcaselect(nau_left_mpx_pin);
    while (!connectTenso(nau_left))
        delay(100);
    
    tcaselect(nau_right_mpx_pin);
    while (!connectTenso(nau_right))
        delay(100);
        
    delay(666);
    digitalWrite(LED_BUILTIN, LOW);
    
    ds.reset();
    ds.select(temp_addr);
    ds.write(0x44);
    tcaselect(nau_right_mpx_pin);
}

void loop()
{
    static unsigned long temp_read_time = millis();
    static unsigned long uart_stats_read_time = millis();

    unsigned long tic = millis();
    // if (millis() >= temp_read_time)
    // { 
    //     temp_read_time = millis() + SEND_INTERVAL_TEMPERATURE;
    //     if (readTemperature())
    //         sendTemperature();
    //         // sendTemperatureText();
    //     // ~17 ms
    // }

    // if (millis() >= uart_stats_read_time)
    // {
    //     uart_stats_read_time = millis() + SEND_INTERVAL_UART_STATS;
    //     sendUartStats();
    // }

    // bool got_data = readTenso(TENSO_SIDE::LEFT) | readTenso(TENSO_SIDE::RIGHT);
    bool got_data = readTenso(TENSO_SIDE::RIGHT);
    if (got_data)
    {
        // sendTenso();
        sendTensoText();
        digitalWrite(LED_BUILTIN, HIGH);
    }

    // unsigned long duration = millis()-tic;
    // if (duration < 100)
    //     delay(100 - duration);
    // delay(1);

    digitalWrite(LED_BUILTIN, LOW);
}

bool connectTenso(NAU7802& nau_x)
{
    if (! nau_x.begin()) {
        return false;
    } else {
        bool result = true;
        result &= nau_x.setLDO(NAU7802_LDO_4V5);        //Set LDO to 4.5V
        result &= nau_x.setGain(NAU7802_GAIN_1);        //Set gain to 1
        result &= nau_x.setSampleRate(NAU7802_SPS_320);  //Set samples per second to 10

        delay(250); //Wait for LDO to stabilize - takes about 200ms

        result &= nau_x.calibrateAFE(); //Re-cal analog front end when we change gain, sample rate, or channel
        
        return result;
    }
}

void sendTenso()
{
    messenger.send(msgTenso);
}

void sendTemperature()
{
    messenger.send(msgTemperature);
}

void sendUartStats()
{
    messenger.sendUartStats();
}

void sendTensoText()
{
    // Serial.print(msgTenso.tenso_left_raw);
    // Serial.print(" ");
    Serial.println(msgTenso.tenso_right_raw);
    // Serial.println();
}

void sendTemperatureText()
{
    Serial.println(msgTemperature.temperature_celsius);
}

bool readTenso(TENSO_SIDE tenso_side)
{
    switch (tenso_side) {
        case TENSO_SIDE::LEFT:
        {
            // tcaselect(nau_left_mpx_pin);
            if (nau_left.available()) {
                msgTenso.tenso_left_raw = nau_left.getReading();
                return true;
            }
            break;
        }
        case TENSO_SIDE::RIGHT:
        {
            // tcaselect(nau_right_mpx_pin);
            if (nau_right.available()) {
                msgTenso.tenso_right_raw = nau_right.getReading();
                return true;
            }
            break;
        }
    }
    return false;
}

bool readTemperature()
{
    if (!ds.reset())
        return false;
    
    ds.select(temp_addr);                 // ~5ms
    ds.write(0xBE);                       // 0-1ms
    
    byte data[9];
    for (byte i = 0; i < 9; i++) {        // ~5ms
        data[i] = ds.read();
    }

    if (OneWire::crc8(data, 8) != data[8])                              // 0ms FROM HERE
    {
        return false;
    }
        
    int16_t raw = (data[1] << 8) | data[0];
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
    msgTemperature.temperature_celsius = (float)raw / 16.0;      // celsius                    // 0ms TO HERE

    // start next conversion - will take ~750 ms (on the termometer hardware)
    ds.reset();
    ds.select(temp_addr);                 // ~5ms
    ds.write(0x44);                       // 0-1ms
    return true;
}