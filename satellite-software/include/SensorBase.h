#ifndef __SensorBase__
#define __SensorBase__
#include <Arduino.h>
#include "LoRaPayloadBase.h"
#include "PersistedConfiguration.h"


class SensorBase : public LoRaPayloadBase, public SensorConfigurationDelegate {
private:
    static bool _isI2CSetUp;
    bool _isFound;

protected:

    // when doing multi-byte reads from registers, this indicates what bit
    // in the register address needs to be set to signal auto  incrementing in 
    // device. This number is 0-based (e.g., the msb bit in a byte is bit 7). 
    // Set to 0 if there is no auto-increment bit.
    virtual uint8_t i2cAutoIncrementBit(void) const         { return 0; }

    // defines the i2c address for this sensor. Set to 0xFF if this
    // sensor does not have an i2c address.
    virtual uint8_t i2cDeviceAddress(void) const = 0;

    // normally you wouldn't call this directly. But in some cases you will if the sensor has
    // multiple i2c addresses. 
    bool writeData(uint8_t deviceAddress, uint8_t data, bool sendStop = true);
    bool writeData(uint8_t deviceAddress, const uint8_t* data, size_t length, bool sendStop = true, bool acceptNACKAtEnd = false);
    bool writeRegister(uint8_t deviceAddress, uint8_t address, uint8_t value);
    bool readData(uint8_t deviceAddress, uint8_t* data, uint8_t length, bool sendStop = true );
    bool readRegister(uint8_t deviceAddress, uint8_t address, uint8_t& register_value);
    bool readRegisters(uint8_t deviceAddress, uint8_t address, uint8_t* data, size_t length);
    bool updateRegister(uint8_t deviceAddress, uint8_t address, uint8_t update_mask, uint8_t value);

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
    bool writeData(uint8_t data, bool sendStop = true)                              { return writeData(i2cDeviceAddress(), data, sendStop); }
    bool writeData(const uint8_t* data, size_t length, bool sendStop = true )       { return writeData(i2cDeviceAddress(), data, length, sendStop); }
    bool writeRegister(uint8_t address, uint8_t value)                              { return writeRegister(i2cDeviceAddress(), address, value); }
    bool readData(uint8_t* data, uint8_t length, bool sendStop = true )             { return readData(i2cDeviceAddress(), data, length, sendStop); }
    bool readRegister(uint8_t address, uint8_t& register_value)                     { return readRegister(i2cDeviceAddress(), address, register_value); }
    bool readRegisters(uint8_t address, uint8_t* data, size_t length)               { return readRegisters(i2cDeviceAddress(), address, data, length); }
    bool updateRegister(uint8_t address, uint8_t preserve_mask, uint8_t value)      { return updateRegister(i2cDeviceAddress(), address, preserve_mask, value); }
};



#endif // __SensorBase__