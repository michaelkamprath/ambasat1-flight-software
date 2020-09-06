#include <Wire.h>
#include "SensorBase.h"

bool SensorBase::_isI2CSetUp = false;

SensorBase::SensorBase()
{
    if (!_isI2CSetUp) {
        Serial.println("Begin Wire");
        Wire.begin(); //I'm the master
        delay(25); // Sensor06 has a startup time of 25ms as per the data sheet
        // Global I2C reset
        Serial.println("Global I2C reset");
        Wire.beginTransmission(0x06); // global i2c reset address
        Wire.endTransmission(); 

        _isI2CSetUp = true;
    }

}

SensorBase::~SensorBase()
{

}
