#ifndef __SensorBase__
#define __SensorBase__
#include <Arduino.h>

class SensorBase {
private:
    static bool _isI2CSetUp;

protected:
    // when doing multi-byte reads from registers, what bit
    // in the register address needs to be set to signal auto 
    // incrementing in device. This number is 0-based (e.g., the 
    // msb bit in a byte is bit 7). However, set to 0 if there
    // is no auto-increment bit.
    virtual uint8_t i2cAutoIncrementBit(void) const         { return 7; }

    // defines the i2c address for this sensor. Set to 0xFF if this
    // sensor does not have an i2c address.
    virtual uint8_t i2cDeviceAddress(void) const = 0;

    // normally you wouldn't call this directly. But in some cases you will. 
    int writeRegister(uint8_t slaveAddress, uint8_t address, uint8_t value);
    int readRegister(uint8_t slaveAddress, uint8_t address);
    int readRegisters(uint8_t slaveAddress, uint8_t address, uint8_t* data, size_t length);

public:
    SensorBase();
    virtual ~SensorBase();

    virtual void setup(void)    {}

    // This functions is expected to get a current sensor measurement
    // then fill a byte buffer that will be transmitted via the 
    // LoRaWAN radio. It is the sensor's responsibility to define the
    // the buffer format for it's measurement. 
    // IMPORTANT: Returns NULL if sensor read was not successful.
    virtual const uint8_t* getCurrentMeasurementBuffer(void) = 0;

    virtual uint8_t getMeasurementBufferSize() const = 0;
    virtual uint8_t getPort() const = 0;

    //
    // I2C methods
    //
    int writeRegister(uint8_t address, uint8_t value)                   { return writeRegister(i2cDeviceAddress(), address, value); }
    int readRegister(uint8_t address)                                   { return readRegister(i2cDeviceAddress(), address); }                          
    int readRegisters(uint8_t address, uint8_t* data, size_t length)    { return readRegisters(i2cDeviceAddress(), address, data, length); }
};



#endif // __SensorBase__