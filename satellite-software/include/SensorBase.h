#ifndef __SensorBase__
#define __SensorBase__
#include <Arduino.h>
#include "LoRaPayloadBase.h"

class PersistedConfiguration;

class SensorBase : public LoRaPayloadBase {
private:
    static bool _isI2CSetUp;
    bool _isFound;

protected:
    PersistedConfiguration& _config;

    // when doing multi-byte reads from registers, this indicates what bit
    // in the register address needs to be set to signal auto  incrementing in 
    // device. This number is 0-based (e.g., the msb bit in a byte is bit 7). 
    // Set to 0 if there is no auto-increment bit.
    virtual uint8_t i2cAutoIncrementBit(void) const         { return 7; }

    // defines the i2c address for this sensor. Set to 0xFF if this
    // sensor does not have an i2c address.
    virtual uint8_t i2cDeviceAddress(void) const = 0;

    // normally you wouldn't call this directly. But in some cases you will if the sensor has
    // multiple i2c addresses. 
    int writeRegister(uint8_t slaveAddress, uint8_t address, uint8_t value);
    int readRegister(uint8_t slaveAddress, uint8_t address);
    int readRegisters(uint8_t slaveAddress, uint8_t address, uint8_t* data, size_t length);

    void setIsFound(bool isFound)               { _isFound = isFound; }

public:
    SensorBase(PersistedConfiguration& config);
    virtual ~SensorBase();

    virtual void setup(void)    {}

    // the sensor hardware can be found and responds to queries
    bool isFound(void) const                    { return _isFound; }

    // the sensor is collecting measurements per configuration. Returns false if sensor is 
    // configured to be off. Returns false if sensor is not found.
    // Override to reflect specific sensor confiuration.  
    virtual bool isActive(void) const           { return isFound(); }

    //
    // I2C methods - use these to interact with the i2c bus.
    //
    int writeRegister(uint8_t address, uint8_t value)                   { return writeRegister(i2cDeviceAddress(), address, value); }
    int readRegister(uint8_t address)                                   { return readRegister(i2cDeviceAddress(), address); }                          
    int readRegisters(uint8_t address, uint8_t* data, size_t length)    { return readRegisters(i2cDeviceAddress(), address, data, length); }
};



#endif // __SensorBase__