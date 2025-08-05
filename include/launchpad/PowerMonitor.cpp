#include "PowerMonitor.hpp"

void PowerMonitor::readingLoop()
{
    while(true)
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        pm.bus_voltage = getBusVoltage_V();
        pm.shunt_voltage = getShuntVoltage_mV();
        pm.current = getCurrent_mA();
        pm.power = getPower_mW();

        if(newReadingCallback) {
            newReadingCallback();
        }

        auto stop_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop_time - start_time).count();
        int delay = 200 - elapsed_time;
        if (delay > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    }
}

PowerMonitor::PowerMonitor(std::function<void()> newReadingCallback) :
    newReadingCallback(newReadingCallback)
{
    begin();
    readT = std::thread(&PowerMonitor::readingLoop, this);
}

PowerMonitor::power_monitor& PowerMonitor::getPower() {
    return pm;
}

bool PowerMonitor::begin()
{
    if (scan()) {
        setBRNG(eBusVolRange_32V);
        setPGA(ePGABits_8);
        setBADC(eAdcBits_12, eAdcSample_8);
        setSADC(eAdcBits_12, eAdcSample_8);
        setMode(eSAndBVolCon);

        calValue = 4096;
        linearCalibrate(1000, 1000);

        std::cout << "Successfully initialized!" << std::endl;
        return true;
    }
    else {
        std::cout << "Couldn't initialized!" << std::endl;
        return false;
    }
}

bool PowerMonitor::scan()
{
    if ((file_i2c = open(I2C_BUS, O_RDWR)) < 0) {
        std::cerr << "Failed to open the I2C bus" << std::endl;
        return false;
    }

    printf("Successfully opened the i2c bus");

    // find available device address
    for (auto addr : I2C_ADDRESSES) {
        if (ioctl(file_i2c, I2C_SLAVE, addr) >= 0) {
            i2c_address = addr;
            printf("Scan completed! Found working address: %d\n", i2c_address);
            return true;
        }
    }

    lastOperateStatus = e_InitError;
    return false;
}

void PowerMonitor::reset()
{
    writeInaReg(_REG_CONFIG, _CONFIG_RESET);
}

void PowerMonitor::writeReg(uint8_t *pBuf, uint16_t len)
{
    if (write(file_i2c, pBuf, len) != len) {
        std::cerr << "Failed to write to the I2C bus." << std::endl;
        lastOperateStatus = e_WriteRegError;
        close(file_i2c);
        return;
    }
    lastOperateStatus = e_ok;
}

void PowerMonitor::readReg(uint8_t reg, uint8_t *pBuf, uint16_t len)
{   
    pBuf[0] = reg;

    if (write(file_i2c, pBuf, len) != len) {
        std::cerr << "Failed to write to the I2C bus." << std::endl;
        lastOperateStatus = e_WriteRegError;
        close(file_i2c);
        return;
    }

    if (read(file_i2c, pBuf, len) != len) {
        std::cerr << "Failed to read from the I2C bus." << std::endl;
        lastOperateStatus = e_ReadRegError;
        close(file_i2c);
        return;
    }
    lastOperateStatus = e_ok;
}

int16_t PowerMonitor::readInaReg(uint8_t reg)
{
    uint8_t buf[2] = {0};
    readReg(reg, buf, sizeof(buf));
    return (buf[0] << 8) | buf[1];
}

uint16_t PowerMonitor::readInaRegUnsigned(uint8_t reg)
{
    uint8_t buf[2] = {0};
    readReg(reg, buf, sizeof(buf));
    return (buf[0] << 8) | buf[1];
}

void PowerMonitor::writeInaReg(uint8_t reg, uint16_t value)
{
    uint8_t buf[3] = {reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xff)};
    writeReg(buf, sizeof(buf));
}

float PowerMonitor::getBusVoltage_V()
{
    return (float) (readInaRegUnsigned(_REG_BUSVOLTAGE) >> 3) * 0.004;
}

float PowerMonitor::getShuntVoltage_mV()
{
    return (float) readInaReg(_REG_SHUNTVOLTAGE) * 0.01;
}

float PowerMonitor::getCurrent_mA()
{
    return (float) readInaReg(_REG_CURRENT);
}

float PowerMonitor::getPower_mW()
{
    return (float) readInaReg(_REG_POWER) * 20;
}

/*Sets Bus Voltage Range(default value is 32V)*/
void PowerMonitor::setBRNG(eBusVolRange_t value)/**/
{
    int16_t    conf;
    conf = readInaReg(_REG_CONFIG);
    conf &= ~((uint16_t) 1 << 13);
    conf |= (uint16_t) value << 13;
    writeInaReg(_REG_CONFIG, conf);
}
/*Sets PGA gain and range(default value is 320mV)*/
void PowerMonitor::setPGA(ePGABits_t bits)//Sets PGA gain and range(default value is 320mV)
{
    int16_t    conf;
    conf = readInaReg(_REG_CONFIG);
    conf &= ~((uint16_t) 0x03 << 11);
    conf |= (uint16_t) bits << 11;
    writeInaReg(_REG_CONFIG, conf);
}
/*
 *These bits adjust the Bus ADC resolution (9-, 10-, 11-, or 12-bit) 
 *or set the number of samples used when averaging results for the Bus Voltage Register
 */
void PowerMonitor::setBADC(eAdcBits_t bits, eAdcSample_t sample)
{
    int16_t    conf;
    int16_t    value = 0;
    if(bits < eAdcBits_12 && sample > eAdcSample_1)
       return;
    if(bits < eAdcBits_12)
        value = bits;
    else
        value = 0x08 | sample;
    conf = readInaReg(_REG_CONFIG);
    conf &= ~((uint16_t) 0x0f << 7);
    conf |= (uint16_t) value << 7;
    writeInaReg(_REG_CONFIG, conf);
}

/*
 *These bits adjust the Shunt ADC resolution (9-, 10-, 11-, or 12-bit) 
 *or set the number of samples used when averaging results for the Shunt Voltage Register
 */
void PowerMonitor::setSADC(eAdcBits_t bits, eAdcSample_t sample)
{
    int16_t    conf;
    int16_t    value = 0;
    if(bits < eAdcBits_12 && sample > eAdcSample_1)
       return;
    if(bits < eAdcBits_12)
        value = bits;
    else
        value = 0x08 | sample;
    conf = readInaReg(_REG_CONFIG);
    conf &= ~((uint16_t) 0x0f << 3);
    conf |= (uint16_t) value << 3;
    writeInaReg(_REG_CONFIG, conf);
}
/*Selects continuous, triggered, or power-down mode of operation*/
void PowerMonitor::setMode(eMode_t mode)
{
    int16_t    conf;
    conf = readInaReg(_REG_CONFIG);
    conf &= ~((uint16_t) 0x07);
    conf |= (uint16_t) mode;
    writeInaReg(_REG_CONFIG, conf);
}

void PowerMonitor::linearCalibrate(float reading_mA, float extMeterReading_mA)
{
    calValue = (uint16_t)((extMeterReading_mA / reading_mA) * calValue) & 0xFFFE;
    writeInaReg(_REG_CALIBRATION, calValue);
}