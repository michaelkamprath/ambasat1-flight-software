#include <Wire.h>
#include "SensorBase.h"

bool SensorBase::_isI2CSetUp = false;

SensorBase::SensorBase()
{
    if (!_isI2CSetUp) {
        Serial.println("Begin Wire");
        Wire.begin(); //I'm the master
        delay(50); // Sensor06 has a startup time of 25ms as per the data sheet
        // Global I2C reset
        Serial.println("Global I2C reset");
        Wire.beginTransmission(0x06); // global i2c reset address
        Wire.endTransmission(); 
        delay(50); // wait for everything to reboot

        _isI2CSetUp = true;
    }

}

SensorBase::~SensorBase()
{

}

int SensorBase::readRegister(uint8_t slaveAddress, uint8_t address)
{
    Wire.beginTransmission(slaveAddress);
    Wire.write(address);
    if (Wire.endTransmission() != 0) {
        return -1;
    }

    if (Wire.requestFrom(slaveAddress, (uint8_t)1) != 1) {
        return -1;
    }

    return Wire.read();
}

int SensorBase::readRegisters(uint8_t slaveAddress, uint8_t address, uint8_t* data, size_t length)
{
    Wire.beginTransmission(slaveAddress);
    Wire.write(0x80 | address);
    if (Wire.endTransmission(false) != 0) {
        return -1;
    }

    if (Wire.requestFrom(slaveAddress, length) != length) {
        return 0;
    }

    for (size_t i = 0; i < length; i++) {
        *data++ = Wire.read();
    }

    return 1;
}

int SensorBase::writeRegister(uint8_t slaveAddress, uint8_t address, uint8_t value)
{
    Wire.beginTransmission(slaveAddress);
    Wire.write(address);
    Wire.write(value);
    if (Wire.endTransmission() != 0) {
        return 0;
    }

    return 1;
}