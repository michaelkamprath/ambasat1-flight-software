#include <Arduino.h>
#include "LSM9DS1Sensor.h"
#include "Utilities.h"

//
// LSM9DS1Sensor
//

//
// The code for driving the sensor was heavily inspired by:
//    https://github.com/FemmeVerbeek/Arduino_LSM9DS1
//
// This is not intended to be a robust driver for the LSM9DS1,
// but instead provide the minimal functionality need for the AmbaSat-1.
// Note that interpretation of the sensor reading is actually done
// by the TTN payload decoder. What get collected here is the raw 
// sensor register readings.

// accelerometer and gyroscope
#define LSM9DS1_ADDRESS            0x6b
#define LSM9DS1_WHO_AM_I           0x0f
#define LSM9DS1_CTRL_REG1_G        0x10
#define LSM9DS1_CTRL_REG3_G        0x12
#define LSM9DS1_STATUS_REG         0x17
#define LSM9DS1_OUT_X_G            0x18
#define LSM9DS1_CTRL_REG6_XL       0x20
#define LSM9DS1_CTRL_REG8          0x22
#define LSM9DS1_OUT_X_XL           0x28

// magnetometer
#define LSM9DS1_ADDRESS_M          0x1e
#define LSM9DS1_CTRL_REG1_M        0x20
#define LSM9DS1_CTRL_REG2_M        0x21
#define LSM9DS1_CTRL_REG3_M        0x22
#define LSM9DS1_CTRL_REG4_M        0x23
#define LSM9DS1_STATUS_REG_M       0x27
#define LSM9DS1_OUT_X_L_M          0x28

LSM9DS1Sensor::LSM9DS1Sensor()
    :   _accelConfig(ACCELERATION_SENSITIVITY_2G),
        _gyroConfig(GYRO_SENSITIVITY_245DPS),
        _magConfig(MAGNETIC_SENSITIVITY_4GAUSS)
{
    // Assumes Wire has been initialized by base class

    // Try to initialise and warn if we couldn't detect the chip
    if (!begin())
    {
        Serial.println(F("Oops ... unable to initialize the LSM9DS1. Check your wiring!"));
        while (1);
    }
    Serial.println(F("Found LSM9DS1 9DOF"));
 }

LSM9DS1Sensor::~LSM9DS1Sensor()
{
    end();
}

bool LSM9DS1Sensor::begin(void)
{
    // reset
    writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG8, 0x05);
    writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG2_M, 0x0c);

    delay(10);

    if (readRegister(LSM9DS1_ADDRESS, LSM9DS1_WHO_AM_I) != 0x68) {
        end();
        return false;
    }

    if (readRegister(LSM9DS1_ADDRESS_M, LSM9DS1_WHO_AM_I) != 0x3d) {
        end();
        return false;
    }
    return true;
}
void LSM9DS1Sensor::end(void)
{
    writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG3_M, 0x03);
    writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG1_G, 0x00);
    writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL, 0x00);

    // There has been a problem with the power usage of the Arduino Nano BLE boards. 
    // Due to a switch in pinnumbers the pull-ups keep drawing current after the call to IMU.end();
    // This shortens battery life. Most likely future updates solve the problem.
    // see https://forum.arduino.cc/index.php?topic=691488.15
    // code for if the old value is used
#if defined(ARDUINO_ARDUINO_NANO33BLE) && PIN_ENABLE_SENSORS_3V3 == 32
    pinMode(PIN_ENABLE_I2C_PULLUP, OUTPUT);    // this restores the energy usage to very low power
    digitalWrite(PIN_ENABLE_I2C_PULLUP, HIGH); 
#endif 
}
void LSM9DS1Sensor::setSensorConfig(void)
{
    // Configure the sensor options   

    // Gyro:
    //      * 59.5 Hz ODR (CTRL_REG1_G | 0b01000000)
    //      * 245 DPS scale (CTRL_REG1_G | 0b00000000)
    //      * Low Power (CTRL_REG3_G bit LP_mode set to 1 ==> 0b10000000 )
    writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG1_G, 0x40);
    writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG3_G, 0x80);

    // Accel:
    //      * 50 Hz ODR (CTRL_REG6_XL | 0b01000000)
    //      * 2G scale (CTRL_REG6_XL | 0b00000000)
     writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL, 0x40);

    //  Magnetic:
    //      * Low power mode (CTRL_REG3_M | 0b00100000) and (CTRL_REG4_M | 0b00000000)
    //      * Continuous conversion (CTRL_REG3_M | 0b00000000)
    //      * 4 gauss (CTRL_REG2_M | 0b00000000)
    //      * Z-axis operative mode selection low power (CTRL_REG4_M | 0b00000000)
    //      * Big Endian (CTRL_REG4_M | 0b00000000)
    writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG3_M, 0x20); 
    writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG2_M, 0x00); 
    writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG4_M, 0x00);

    // now set the sensor sensitivity settings to their configured settings.
    setAccelFS(_accelConfig);
    setGyroFS(_gyroConfig);
    setMagnetFS(_magConfig);
 }

int LSM9DS1Sensor::setAccelFS(AccelerationSensitivitySetting config)
{	
    uint8_t range = 0x00;
    switch(config) {
        case ACCELERATION_SENSITIVITY_2G:
            range = (0b00 << 3);
            break;
        case ACCELERATION_SENSITIVITY_4G:
            range = (0b10 << 3);
            break;
        case ACCELERATION_SENSITIVITY_8G:
            range = (0b11 << 3);
            break;
        case ACCELERATION_SENSITIVITY_16G:
            range = (0b01 << 3);
            break;
        default:
            return 0;
            break;
    }    
	uint8_t setting = ((readRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL) & 0xE7) | range);
	return writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL,setting) ;
}

int LSM9DS1Sensor::setGyroFS(GyroSensitivitySetting config)
{
    uint8_t range = 0x00;
    switch(config) {
        case GYRO_SENSITIVITY_245DPS:
            range = (0b00 << 3);
            break;
        case GYRO_SENSITIVITY_500DPS:
            range = (0b01 << 3);
            break;
        // case GYRO_SENSITIVITY_1000DPS:
        //     range = (0b10 << 3);
        //     break;
        case GYRO_SENSITIVITY_2000DPS:
            range = (0b11 << 3);
            break;
        default:
            return 0;
            break;
    }
    uint8_t setting = ((readRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG1_G) & 0xE7) | range );
    return writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG1_G,setting) ;
}

int LSM9DS1Sensor::setMagnetFS(MagneticSensitivitySetting config) // 0=400.0; 1=800.0; 2=1200.0 , 3=1600.0  (µT)
{
    uint8_t range = 0x00;
    switch(config) {
        case MAGNETIC_SENSITIVITY_4GAUSS:
            range = (0b00 << 5);
            break;
        case MAGNETIC_SENSITIVITY_8GAUSS:
            range = (0b01 << 5);
            break;
        case MAGNETIC_SENSITIVITY_12GAUSS:
            range = (0b10 << 5);
            break;
        case MAGNETIC_SENSITIVITY_16GAUSS:
            range = (0b11 << 5);
            break;
        default:
            return 0;
            break;
    }
    return writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG2_M,range) ;
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
    int16_t accelData[3], gyroData[3], magneticData[3];

    // acceleration
    if (!readRegisters(LSM9DS1_ADDRESS, LSM9DS1_OUT_X_XL, (uint8_t*)accelData, sizeof(accelData))) {
        Serial.println(F("ERROR reading LSM9DS1 accel data"));
        return nullptr;
    }

    // gyro
    if (!readRegisters(LSM9DS1_ADDRESS, LSM9DS1_OUT_X_G, (uint8_t*)gyroData, sizeof(gyroData))) 
    {
        Serial.println(F("ERROR reading LSM9DS1 gyro data"));
        return nullptr;
    }

    // Magnetic 
    if (!readRegisters(LSM9DS1_ADDRESS_M, LSM9DS1_OUT_X_L_M, (uint8_t*)magneticData, sizeof(magneticData))) 
    {
        Serial.println(F("ERROR reading LSM9DS1 magnetic data"));
        return nullptr;
    }

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

    setSensorValueAtBufferLocation(accelData[0], 0);
    setSensorValueAtBufferLocation(accelData[1], 2);
    setSensorValueAtBufferLocation(accelData[2], 4);
    setSensorValueAtBufferLocation(gyroData[0], 6);
    setSensorValueAtBufferLocation(gyroData[1], 8);
    setSensorValueAtBufferLocation(gyroData[2], 10);
    setSensorValueAtBufferLocation(magneticData[0], 12);
    setSensorValueAtBufferLocation(magneticData[1], 14);
    setSensorValueAtBufferLocation(magneticData[2], 16);
    _buffer[18] = (uint8_t)_accelConfig;
    _buffer[18] |= (uint8_t)_gyroConfig;
    _buffer[19] = (uint8_t)_magConfig;
   
    return _buffer;
}