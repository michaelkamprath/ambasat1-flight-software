#include <Wire.h>
#include "Sensors.h"
#include "Utilities.h"

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

//
// Voltage Sensor
//

VoltageSensor::VoltageSensor()
{

}

VoltageSensor::~VoltageSensor()
{

}

const uint8_t* 
VoltageSensor::getCurrentMeasurementBuffer(void)
{
    // Read 1.1V reference against AVcc
    // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
#else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

    delay(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA,ADSC)); // measuring

    uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
    uint8_t high = ADCH; // unlocks both

    int32_t volts = 1125300L / ((high<<8) | low);
    int16_t volts_int16 = volts < 32767 ? volts : 32767;

    // put results in buffer and return
    hton_int16(volts_int16, _buffer);

    return _buffer;
}

//
// LSM9DS1Sensor
//

LSM9DS1Sensor::LSM9DS1Sensor()
    :   _lsm()
{
    // Try to initialise and warn if we couldn't detect the chip
    if (!_lsm.begin())
    {
        Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
        while (1);
    }
    Serial.println("Found LSM9DS1 9DOF");

    // TODO - make these configurations configurable by satellite command

    // 1.) Set the accelerometer range
    _lsm.setupAccel(_lsm.LSM9DS1_ACCELRANGE_2G);
    //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
    //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
    //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

    // 2.) Set the magnetometer sensitivity
    _lsm.setupMag(_lsm.LSM9DS1_MAGGAIN_4GAUSS);
    //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
    //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
    //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

    // 3.) Setup the gyroscope
    _lsm.setupGyro(_lsm.LSM9DS1_GYROSCALE_245DPS);
    //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
    //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

LSM9DS1Sensor::~LSM9DS1Sensor()
{
}

void LSM9DS1Sensor::setSensorValueAtBufferLocation(float sensor_value, uint8_t index)
{
    if (index < this->getMeasurementBufferSize()) {
        int16_t int_value = sensor_value;
        hton_int16(int_value, &_buffer[index]);
    }
}

const uint8_t* 
LSM9DS1Sensor::getCurrentMeasurementBuffer(void)
{
    // while the AdaFruuit Library will produce results into convrted units,
    // we will gather the raw sensor measurements in order to keep tranmission
    // concise.

    _lsm.read();

    // Buffer format (TOTAL bytes = 18)
    //      type        value           buffer index
    //      ==============================================
    //      int16_t     accelData.x     0
    //      int16_t     accelData.y     2
    //      int16_t     accelData.z     4
    //      int16_t     gyroData.x      6
    //      int16_t     gyroData.y      8
    //      int16_t     gyroData.z      10
    //      int16_t     magData.x       12
    //      int16_t     magData.y       14
    //      int16_t     magData.z       16
    //
    // While the AdaFruit library stores the above values in floats, they are
    // actually sourced from int16_t values. Converting back to int16_t should
    // have no data loss. 

    setSensorValueAtBufferLocation(_lsm.accelData.x, 0);
    setSensorValueAtBufferLocation(_lsm.accelData.y, 2);
    setSensorValueAtBufferLocation(_lsm.accelData.z, 4);
    setSensorValueAtBufferLocation(_lsm.gyroData.x, 6);
    setSensorValueAtBufferLocation(_lsm.gyroData.y, 8);
    setSensorValueAtBufferLocation(_lsm.gyroData.z, 10);
    setSensorValueAtBufferLocation(_lsm.magData.x, 12);
    setSensorValueAtBufferLocation(_lsm.magData.y, 14);
    setSensorValueAtBufferLocation(_lsm.magData.z, 16);

    return _buffer;
}