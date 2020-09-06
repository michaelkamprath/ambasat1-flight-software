#include "LSM9DS1Sensor.h"
#include "Utilities.h"

//
// LSM9DS1Sensor
//

LSM9DS1Sensor::LSM9DS1Sensor()
    :   _lsm(),
        _accelConfig(ACCELERATION_SENSITIVITY_2G),
        _gyroConfig(GYRO_SENSITIVITY_245DPS),
        _magConfig(MAGNETIC_SENSITIVITY_4GAUSS)
{
    // Try to initialise and warn if we couldn't detect the chip
    if (!_lsm.begin())
    {
        Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
        while (1);
    }
    Serial.println("Found LSM9DS1 9DOF");

 }

LSM9DS1Sensor::~LSM9DS1Sensor()
{
}

void LSM9DS1Sensor::setSensorConfig(void)
{
    // TODO - make these configurations configurable by satellite command

    switch(_accelConfig) {
        case ACCELERATION_SENSITIVITY_2G:
            _lsm.setupAccel(_lsm.LSM9DS1_ACCELRANGE_2G);
            break;
        case ACCELERATION_SENSITIVITY_4G:
             _lsm.setupAccel(_lsm.LSM9DS1_ACCELRANGE_4G);
            break;
        case ACCELERATION_SENSITIVITY_8G:
             _lsm.setupAccel(_lsm.LSM9DS1_ACCELRANGE_8G);
            break;
        case ACCELERATION_SENSITIVITY_16G:
             _lsm.setupAccel(_lsm.LSM9DS1_ACCELRANGE_16G);
            break;
    }
    switch(_gyroConfig) {
        case GYRO_SENSITIVITY_245DPS:
            _lsm.setupGyro(_lsm.LSM9DS1_GYROSCALE_245DPS);
            break;
        case GYRO_SENSITIVITY_500DPS:
            _lsm.setupGyro(_lsm.LSM9DS1_GYROSCALE_500DPS);
            break;
        case GYRO_SENSITIVITY_2000DPS:
            _lsm.setupGyro(_lsm.LSM9DS1_GYROSCALE_2000DPS);
            break;
    }

    switch(_magConfig) {
        case MAGNETIC_SENSITIVITY_4GAUSS:
            _lsm.setupMag(_lsm.LSM9DS1_MAGGAIN_4GAUSS);
            break;
        case MAGNETIC_SENSITIVITY_8GAUSS:
            _lsm.setupMag(_lsm.LSM9DS1_MAGGAIN_8GAUSS);
            break;
        case MAGNETIC_SENSITIVITY_12GAUSS:
            _lsm.setupMag(_lsm.LSM9DS1_MAGGAIN_12GAUSS);
            break;
        case MAGNETIC_SENSITIVITY_16GAUSS:
            _lsm.setupMag(_lsm.LSM9DS1_MAGGAIN_16GAUSS);
            break;
    }
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

    // Buffer format (TOTAL bytes = 21)
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
    //      uint4_t     _accelConfig    18-low
    //      uint4_t     _gyroConfig     18-high
    //      uint4_t     _magConfig      19-low
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
    _buffer[18] = (uint8_t)_accelConfig;
    _buffer[18] |= (uint8_t)_gyroConfig;
    _buffer[19] = (uint8_t)_magConfig;
   
    return _buffer;
}