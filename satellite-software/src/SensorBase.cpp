#include <Wire.h>
#include "SensorBase.h"
#include "PersistedConfiguration.h"

bool SensorBase::_isI2CSetUp = false;

SensorBase::SensorBase(PersistedConfiguration& config)
    : _isFound(false),
      _config(config)
{
    if (!_isI2CSetUp) {
        Serial.println(F("Begin Wire"));
        Wire.begin(); //I'm the master
        delay(300); // Some sensors have a start up time of at least 25 ms
        // // Global I2C reset
        // Serial.println(F("Global I2C reset"));
        // Wire.beginTransmission(0x00); // global i2c reset address
        // Wire.write(0x06);
        // Serial.println(F("    ... reset sent."));
        // Wire.endTransmission(false); 
        // Serial.println(F("    ... trnsmission ended"));
        // delay(50); // wait for everything to reboot
        Serial.println(F("I2C Wire has been set up."));
        _isI2CSetUp = true;
    }

}

SensorBase::~SensorBase()
{

}

uint8_t SensorBase::calculatCRC(const uint8_t* bytes, uint8_t nbrOfBytes, uint16_t polynomial)
{
    uint8_t crc = 0xFF; // calculated checksum
    // calculates 8-Bit checksum with given polynomial
    for(uint8_t byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++)
    {
        crc ^= (bytes[byteCtr]);
        for(uint8_t bit = 8; bit > 0; --bit)
        {
            if(crc & 0x80) crc = (crc << 1) ^ polynomial;
            else           crc = (crc << 1);
        }
    }
    return crc;
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
            Serial.print(F("ERROR writing data on byte "));
            Serial.print(i);
            Serial.print(F(" of data buffer. Device address = 0x"));
            Serial.print(deviceAddress, HEX);
            Serial.print(F(", data value = 0x"));
            Serial.print(data[i], HEX);
            Serial.print(F("\n"));
            return false;
        }
        delay(1);
    }
    uint8_t err = Wire.endTransmission(sendStop);
    if (err != 0 && ((!acceptNACKAtEnd) || (acceptNACKAtEnd&&(err != 3)) ) ) {
        Serial.print(F("ERROR ending transmission - writeData, error = "));
        Serial.print(err);
        Serial.print(F(", device address = 0x"));
        Serial.print(deviceAddress, HEX);
        Serial.print(F("\n"));
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
    uint32_t startTime = millis();
    while ((Wire.available() < (int)length) && (millis() - startTime < 100)) {
        // waitng for the data
        delay(1);
    }
    if (Wire.available() != length) {
        Serial.print(F("ERROR requesting data - readData( 0x"));
        Serial.print(deviceAddress, HEX);
        Serial.print(F(", 0x"));
        Serial.print((size_t)data, HEX);
        Serial.print(F(", "));
        Serial.print(length);
        Serial.print(F(" ), only got "));
        Serial.print(Wire.available());
        Serial.print(F(" bytes.\n"));
        return false;
    }

    for (size_t i = 0; i < length; i++) {
        *data++ = Wire.read();
    }

    Wire.endTransmission();

    return true;
}

int SensorBase::readRegister(uint8_t deviceAddress, uint8_t address)
{
    if (deviceAddress == 0xFF) {
        // this is not a i2c sensor
        return -1;
    }
    Wire.beginTransmission(deviceAddress);
    Wire.write(address);
    if (Wire.endTransmission() != 0) {
        Serial.print(F("ERROR ending transmission #1 - readRegister( 0x"));
        Serial.print(deviceAddress, HEX);
        Serial.print(F(", 0x"));
        Serial.print(address, HEX);
        Serial.print(F(" )\n"));
       return -1;
    }

    if (Wire.requestFrom(deviceAddress, (uint8_t)1) != 1) {
        Serial.print(F("ERROR requesting data - readRegister( 0x"));
        Serial.print(deviceAddress, HEX);
        Serial.print(F(", 0x"));
        Serial.print(address, HEX);
        Serial.print(F(" )\n"));
        return -1;
    }

    int value = Wire.read();

    if (Wire.endTransmission() != 0) {
        Serial.print(F("ERROR ending transmission #2 - readRegister( 0x"));
        Serial.print(deviceAddress, HEX);
        Serial.print(F(", 0x"));
        Serial.print(address, HEX);
        Serial.print(F(" )\n"));
        return -1;
    }

    return value;
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
        Serial.print(F("ERROR ending transmission #1 - readRegister( 0x"));
        Serial.print(deviceAddress, HEX);
        Serial.print(F(", 0x"));
        Serial.print(address, HEX);
        Serial.print(F(", 0x"));
        Serial.print((size_t)data, HEX);
        Serial.print(F(", "));
        Serial.print(length);
        Serial.print(F(" )\n"));
        return false;
    }

    if (Wire.requestFrom(deviceAddress, length) != length) {
        Serial.print(F("ERROR requesting data - readRegister( 0x"));
        Serial.print(deviceAddress, HEX);
        Serial.print(F(", 0x"));
        Serial.print(address, HEX);
        Serial.print(F(", 0x"));
        Serial.print((size_t)data, HEX);
        Serial.print(F(", "));
        Serial.print(length);
        Serial.print(F(" )\n"));
        return false;
    }

    for (size_t i = 0; i < length; i++) {
        *data++ = Wire.read();
    }

    if (Wire.endTransmission() != 0) {
        Serial.print(F("ERROR ending transmission #2 - readRegister( 0x"));
        Serial.print(deviceAddress, HEX);
        Serial.print(F(", 0x"));
        Serial.print(address, HEX);
        Serial.print(F(", 0x"));
        Serial.print((size_t)data, HEX);
        Serial.print(F(", "));
        Serial.print(length);
        Serial.print(F(" )\n"));
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
        Serial.print(F("ERROR ending transmission - writeRegister( 0x"));
        Serial.print(deviceAddress, HEX);
        Serial.print(F(", 0x"));
        Serial.print(address, HEX);
        Serial.print(F(", 0x"));
        Serial.print(value, HEX);
        Serial.print(F(" )\n"));
        return false;
    }

    return true;
}