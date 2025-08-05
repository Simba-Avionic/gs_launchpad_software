#pragma once

#ifndef _POWER_MONITOR_HPP
#define _POWER_MONITOR_HPP

#include <iostream>
#include <thread>
#include <functional>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>

// #define _I2C_ADDRESS1                      (0x40)    /// < I2C ADDRESS 1
// #define _I2C_ADDRESS2                      (0x41)    /// < I2C ADDRESS 2
// #define _I2C_ADDRESS3                      (0x44)    /// < I2C ADDRESS 3
// #define _I2C_ADDRESS4                      (0x45)    /// < I2C ADDRESS 4

#define DFROBOT__READ                      (0x01)   /// <Register Configuration

#define _REG_CONFIG                        (0x00)   /// <Config register
#define _CONFIG_RESET                      (0x8000) /// <Config reset register
#define _CONFIG_BUSVOLTAGERANGE_MASK       (0x2000) /// <Config bus voltage range
#define _REG_SHUNTVOLTAGE                  (0x01)   /// <Shunt Voltage Register
#define _REG_BUSVOLTAGE                    (0x02)   /// <Bus Voltage Register
#define _REG_POWER                         (0x03)   /// <Power Register
#define _REG_CURRENT                       (0x04)   /// <Current Register
#define _REG_CALIBRATION                   (0x05)   /// <Register Calibration

static const int I2C_ADDRESSES[4] = {0x45, 0x44, 0x41, 0x40}; // possible addresses of i2c device
#define I2C_BUS "/dev/i2c-1"

class PowerMonitor
{

public:

    typedef enum {
        e_ok,               /**<No error */
        e_InitError,        /**<Init error */
        e_WriteRegError,    /**<Write register error */
        e_ReadRegError,     /**<Read register error */
    } eStatus_t;

    typedef enum {
        eBusVolRange_16V,   /**< ±16V*/ 
        eBusVolRange_32V    /**< ±32V*/
    } eBusVolRange_t;

    typedef enum {
        ePGABits_1,   /**<GAIN:1,Range ±40 mV*/ 
        ePGABits_2,   /**<GAIN:/2,Range ±80 mV*/ 
        ePGABits_4,   /**<GAIN:/4,Range ±160 mV*/ 
        ePGABits_8    /**<GAIN:/8,Range ±320 mV*/ 
    } ePGABits_t;

    typedef enum {
        eAdcBits_9,
        eAdcBits_10,
        eAdcBits_11,
        eAdcBits_12
    } eAdcBits_t;

    typedef enum {
        eAdcSample_1,
        eAdcSample_2,
        eAdcSample_4,
        eAdcSample_8,
        eAdcSample_16,
        eAdcSample_32,
        eAdcSample_64,
        eAdcSample_128
    } eAdcSample_t;

    typedef enum{
        ePowerDown,   /**<Power-down*/ 
        eSVolTrig,    /**<Shunt voltage, triggered*/ 
        eBVolTrig,    /**<Bus voltage, triggered*/ 
        eSAndBVolTrig,/**<Shunt and bus, triggered*/ 
        eAdcOff,      /**<ADC off (disabled)*/ 
        eSVolCon,     /**<Shunt voltage, continuous*/ 
        eBVolCon,     /**<Bus voltage, continuous*/ 
        eSAndBVolCon  /**<Shunt and bus, continuous*/ 
    } eMode_t;

    struct power_monitor
    {
        float shunt_voltage;
        float bus_voltage;
        float power;
        float current;
    };

    PowerMonitor(std::function<void()> newReadingCallback);
    ~PowerMonitor() = default;

    void readingLoop();
    power_monitor& getPower();

private:
    power_monitor pm;
    int lastOperateStatus;
    uint16_t calValue;
    bool scan();
    void writeReg(uint8_t *pBuf, uint16_t len);
    void readReg(uint8_t reg, uint8_t *pBuf, uint16_t len);
    uint8_t BusRange, Pga, Badc, Sadc, Mode;
    int16_t readInaReg(uint8_t reg);
    uint16_t readInaRegUnsigned(uint8_t reg);
    void writeInaReg(uint8_t reg, uint16_t value);

    int i2c_address;
    int file_i2c;

    std::thread readT;
    bool begin();
    void reset();

    float getBusVoltage_V();
    float getShuntVoltage_mV();
    float getCurrent_mA();
    float getPower_mW();

    void setBRNG(eBusVolRange_t value);
    void setPGA(ePGABits_t bits);
    void setBADC(eAdcBits_t bits, eAdcSample_t sample);
    void setSADC(eAdcBits_t bits, eAdcSample_t sample);
    void setMode(eMode_t mode);

    void linearCalibrate(float reading_mA, float extMeterReading_mA);
    
    std::function<void()> newReadingCallback;
};

#endif