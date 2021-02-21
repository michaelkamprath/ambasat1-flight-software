#include <Wire.h>
#include "SensorBase.h"
#include "PersistedConfiguration.h"
#include "Logging.h"

bool SensorBase::_isI2CSetUp = false;

SensorBase::SensorBase(PersistedConfiguration& config)
    :   SensorConfigurationDelegate(config),
        _isFound(false)
{
    if (!_isI2CSetUp) {
        PRINTLN_INFO(F("Starting I2C"));
        Wire.begin(); //I'm the master
        delay(300); // Some sensors have a start up time of at least 25 ms
        // Global I2C reset
        PRINTLN_DEBUG(F("Global I2C reset"));
        Wire.beginTransmission(0x00); // global i2c reset address
        Wire.write(0x06);
        Wire.endTransmission();
        delay(50); // wait for everything to reboot
        PRINTLN_DEBUG(F("I2C has been set up"));
        _isI2CSetUp = true;
    }

}

SensorBase::~SensorBase()
{

}

bool SensorBase::writeData(uint8_t deviceAddress, uint8_t data, bool sendStop)
{
    return writeData(deviceAddress, &data, 1);
}

bool SensorBase::writeData(uint8_t deviceAddress, const uint8_t* data, size_t length, bool sendStop, bool acceptNACKAtEnd )
{
    if (deviceAddress == 0xFF) {
        // this is not a i2c sensor
        return false;
    }
    Wire.beginTransmission(deviceAddress);

    for (size_t i = 0; i < length; i++)
    {
        if (Wire.write(data[i]) != 1) {
            PRINT_ERROR(F("ERROR writing data byte "));
            PRINT_ERROR(i);
            PRINT_ERROR(F(" device 0x"));
            PRINT_HEX_ERROR(deviceAddress);
            PRINT_DEBUG(F(", data 0x"));
            PRINT_HEX_DEBUG(data[i]);
            PRINT_ERROR(F("\n"));
            return false;
        }
        delay(1);
    }
    uint8_t err = Wire.endTransmission(sendStop);
    if (err != 0 && ((!acceptNACKAtEnd) || (acceptNACKAtEnd&&(err != 3)) ) ) {
        PRINT_ERROR(F("ERROR endTransmission err: "));
        PRINT_ERROR(err);
        PRINT_ERROR(F(", device 0x"));
        PRINT_HEX_ERROR(deviceAddress);
        PRINT_ERROR(F("\n"));
        return false;
    }

    return true;
}

bool SensorBase::readData(uint8_t deviceAddress, uint8_t* data, uint8_t length, bool sendStop)
{
    if (deviceAddress == 0xFF) {
        // this is not a i2c sensor
        return false;
    }
    Wire.requestFrom(deviceAddress, length, (uint8_t)sendStop);

    // wait up to 100 milliseconds for the data
    if (Wire.available() < (int)length) {
        delay(100);
        if (Wire.available() != (int)length) {
            PRINT_ERROR(F("ERROR requesting data, device 0x"));
            PRINT_HEX_ERROR(deviceAddress);
            PRINT_DEBUG(F(", data len = "));
            PRINT_DEBUG(length);
            PRINT_DEBUG(F(", got "));
            PRINT_DEBUG(Wire.available());
            PRINT_ERROR(F("\n"));
            return false;
        }
    }

    for (size_t i = 0; i < length; i++) {
        *data++ = Wire.read();
    }

    Wire.endTransmission();

    return true;
}

bool SensorBase::readRegister(uint8_t deviceAddress, uint8_t address, uint8_t& register_value)
{
    if (deviceAddress == 0xFF) {
        // this is not a i2c sensor
        return -1;
    }
    Wire.beginTransmission(deviceAddress);
    Wire.write(address);
    if (Wire.endTransmission() != 0) {
        PRINT_ERROR(F("ERROR ending trans 1 device 0x"));
        PRINT_HEX_ERROR(deviceAddress);
        PRINT_ERROR(F("\n"));
       return false;
    }

    if (Wire.requestFrom(deviceAddress, (uint8_t)1) != 1) {
        PRINT_ERROR(F("ERROR requesting data, device 0x"));
        PRINT_HEX_ERROR(deviceAddress);
        PRINT_DEBUG(F(" at addr 0x"));
        PRINT_HEX_DEBUG(address);
        PRINT_ERROR(F("\n"));
        return false;
    }

    int value = Wire.read();

    if (Wire.endTransmission() != 0) {
        PRINT_ERROR(F("ERROR ending trans 2 device 0x"));
        PRINT_HEX_ERROR(deviceAddress);
        PRINT_ERROR(F("\n"));
        return false;
    }

    register_value = value;
    return true;
}

bool SensorBase::readRegisters(uint8_t deviceAddress, uint8_t address, uint8_t* data, size_t length)
{
    if (deviceAddress == 0xFF) {
        // this is not a i2c sensor
        return false;
    }
    Wire.beginTransmission(deviceAddress);
    uint8_t autoIncrementBit = 0x00;
    if (i2cAutoIncrementBit() > 0 ) {
        autoIncrementBit = 1 << i2cAutoIncrementBit();
    }
    Wire.write(autoIncrementBit | address);
    if (Wire.endTransmission(false) != 0) {
        PRINT_ERROR(F("ERROR ending trans 3 device 0x"));
        PRINT_HEX_ERROR(deviceAddress);
        PRINT_ERROR(F("\n"));
        return false;
    }

    if (Wire.requestFrom(deviceAddress, length) != length) {
        PRINT_ERROR(F("ERROR requesting data from device 0x"));
        PRINT_HEX_ERROR(deviceAddress);
        PRINT_DEBUG(F(" at addr 0x"));
        PRINT_HEX_DEBUG(address);
        PRINT_ERROR(F("\n"));
        return false;
    }

    for (size_t i = 0; i < length; i++) {
        *data++ = Wire.read();
    }

    if (Wire.endTransmission() != 0) {
        PRINT_ERROR(F("ERROR ending trans 4 device 0x"));
        PRINT_HEX_ERROR(deviceAddress);
        PRINT_ERROR(F("\n"));
        return false;
    }
    return true;
}

bool SensorBase::writeRegister(uint8_t deviceAddress, uint8_t address, uint8_t value)
{
    if (deviceAddress == 0xFF) {
        // this is not a i2c sensor
        return false;
    }
    Wire.beginTransmission(deviceAddress);
    Wire.write(address);
    Wire.write(value);
    if (Wire.endTransmission() != 0) {
        PRINT_ERROR(F("ERROR ending trans 5 device 0x"));
        PRINT_HEX_ERROR(deviceAddress);
        PRINT_ERROR(F("\n"));
        return false;
    }

    return true;
}

bool SensorBase::updateRegister(uint8_t deviceAddress, uint8_t address, uint8_t update_mask, uint8_t value)
{
    uint8_t curValue;
    if (!readRegister(deviceAddress, address, curValue)) {
        return false;
    }
    return writeRegister(deviceAddress, address, ((curValue&(~update_mask)) | (value&update_mask)) );
}