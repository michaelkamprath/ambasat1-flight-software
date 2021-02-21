#include <Arduino.h>
#include "LSM9DS1Sensor.h"
#include "Utilities.h"
#include "PersistedConfiguration.h"
#include "Logging.h"

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

LSM9DS1Sensor::LSM9DS1Sensor(PersistedConfiguration& config)
    :   SensorBase(config)
{
    PRINTLN_DEBUG(F("Starting LSM9DS1 Sensor"));

    // Assumes Wire has been initialized by base class

    // Try to initialise and warn if we couldn't detect the chip
    if (!begin())
    {
        PRINTLN_ERROR(F("ERROR: unable to initialize LSM9DS1."));
        setIsFound(false);
    } else {
        PRINTLN_INFO(F("Found LSM9DS1"));
        setIsFound(true);
    }
 }

LSM9DS1Sensor::~LSM9DS1Sensor()
{
    end();
}

bool LSM9DS1Sensor::begin(void)
{
    uint8_t register_value;

    // reset
    if (!writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG8, 0x05)) {
        return false;
    }
    writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG2_M, 0x0c);

    delay(10);

    if (!readRegister(LSM9DS1_ADDRESS, LSM9DS1_WHO_AM_I, register_value) || (register_value != 0x68)) {
        end();
        return false;
    }

    if (!readRegister(LSM9DS1_ADDRESS_M, LSM9DS1_WHO_AM_I, register_value) || (register_value != 0x3d)) {
        end();
        return false;
    }
    return true;
}
void LSM9DS1Sensor::end(void)
{
    if (!isFound()) {
        return;
    }

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

bool LSM9DS1Sensor::isActive(void) const
{
    return (
        SensorBase::isActive()
        &&(
            (getAcceleratonSensitivitySetting() != ACCELERATION_SENSITIVITY_OFF)
          ||(getGysroSensitivitySetting() != GYRO_SENSITIVITY_OFF)
          ||(getMagneticSensitivitySetting() != MAGNETIC_SENSITIVITY_OFF)
        )
    );
}

void LSM9DS1Sensor::setSensorConfig(void)
{
    if (!isFound()) {
        return;
    }
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
    setAccelFS(getAcceleratonSensitivitySetting());
    setGyroFS(getGysroSensitivitySetting());
    setMagnetFS(getMagneticSensitivitySetting());
 }

void LSM9DS1Sensor::setAccelFS(AccelerationSensitivitySetting config)
{
    if (!isFound()) {
        return;
    }

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
            return;
            break;
    }
    uint8_t register_value = 0;
    readRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL, register_value);
	uint8_t setting = ((register_value & 0xE7) | range);
	writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL,setting) ;
}

void LSM9DS1Sensor::setGyroFS(GyroSensitivitySetting config)
{
    if (!isFound()) {
        return;
    }

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
            return;
            break;
    }
    uint8_t register_value = 0;
    readRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG1_G, register_value);
    uint8_t setting = ((register_value & 0xE7) | range );
    writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG1_G,setting) ;
}

void LSM9DS1Sensor::setMagnetFS(MagneticSensitivitySetting config) // 0=400.0; 1=800.0; 2=1200.0 , 3=1600.0  (µT)
{
    if (!isFound()) {
        return;
    }

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
            return;
            break;
    }
    writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG2_M,range) ;
}

const uint8_t*
LSM9DS1Sensor::getCurrentMeasurementBuffer(void)
{
    if (!isActive()) {
        return nullptr;
    }

    int16_t accelData[3], gyroData[3], magneticData[3];

    // acceleration
    if (!readRegisters(LSM9DS1_ADDRESS, LSM9DS1_OUT_X_XL, (uint8_t*)accelData, sizeof(accelData))) {
        PRINTLN_ERROR(F("ERROR reading LSM9DS1 accel data"));
        return nullptr;
    }

    // gyro
    if (!readRegisters(LSM9DS1_ADDRESS, LSM9DS1_OUT_X_G, (uint8_t*)gyroData, sizeof(gyroData)))
    {
        PRINTLN_ERROR(F("ERROR reading LSM9DS1 gyro data"));
        return nullptr;
    }

    // Magnetic
    if (!readRegisters(LSM9DS1_ADDRESS_M, LSM9DS1_OUT_X_L_M, (uint8_t*)magneticData, sizeof(magneticData)))
    {
        PRINTLN_ERROR(F("ERROR reading LSM9DS1 magnetic data"));
        return nullptr;
    }

    // Buffer format (TOTAL bytes = 20)
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

    hton_int16(accelData[0], &_buffer[0]);
    hton_int16(accelData[1], &_buffer[2]);
    hton_int16(accelData[2], &_buffer[4]);
    hton_int16(gyroData[0], &_buffer[6]);
    hton_int16(gyroData[1], &_buffer[8]);
    hton_int16(gyroData[2], &_buffer[10]);
    hton_int16(magneticData[0], &_buffer[12]);
    hton_int16(magneticData[0], &_buffer[14]);
    hton_int16(magneticData[0], &_buffer[16]);
    _buffer[18] = (uint8_t)getAcceleratonSensitivitySetting();
    _buffer[18] |= (uint8_t)getGysroSensitivitySetting();
    _buffer[19] = (uint8_t)getMagneticSensitivitySetting();

    return _buffer;
}

//
// Sensor Configuration Delegate
//

#define OFFSET_ACCELERATION_SENSITIVITY    0      // 1 byte
#define OFFSET_GYRO_SENSITIVITY            1      // 1 byte
#define OFFSET_MAGNETIC_SENSITIVITY        2     // 1 byte

#define LSM9DS1_CONFIG_BLOCK_SIZE          3

uint8_t LSM9DS1Sensor::configBlockSize( void ) const
{
    return LSM9DS1_CONFIG_BLOCK_SIZE;
}
void LSM9DS1Sensor::setDefaultValues(void)
{
    setAcceleratonSensitivitySetting(ACCELERATION_SENSITIVITY_2G);
    setGysroSensitivitySetting(GYRO_SENSITIVITY_245DPS);
    setMagneticSensitivitySetting(MAGNETIC_SENSITIVITY_4GAUSS);
}
void LSM9DS1Sensor::loadConfigValues(void)
{
    #if LSM9DS1_RESULTS_BUFFER_SIZE < LSM9DS1_CONFIG_BLOCK_SIZE
    #error Cannot reuse LSM9DS1 results buffer to load configuration because it is too small
    #else
    // this approach reduces code size. Reusing payload buffer to conserve memory.
    eeprom_read_block(_buffer, (uint8_t *)getEEPROMBaseAddress(), LSM9DS1_CONFIG_BLOCK_SIZE);
    _accelSensitivity = (AccelerationSensitivitySetting)_buffer[OFFSET_ACCELERATION_SENSITIVITY];
    _gyroSensitivity = (GyroSensitivitySetting)_buffer[OFFSET_GYRO_SENSITIVITY];
    _magneticSensitivity = (MagneticSensitivitySetting)_buffer[OFFSET_MAGNETIC_SENSITIVITY];
    #endif
}

void LSM9DS1Sensor::writeConfigToBuffer(uint8_t* bufferBaseAddress) const
{
    bufferBaseAddress[OFFSET_ACCELERATION_SENSITIVITY] = _accelSensitivity;
    bufferBaseAddress[OFFSET_GYRO_SENSITIVITY] = _gyroSensitivity;
    bufferBaseAddress[OFFSET_MAGNETIC_SENSITIVITY] = _magneticSensitivity;
}

void LSM9DS1Sensor::setAcceleratonSensitivitySetting(AccelerationSensitivitySetting setting) {
    eeprom_update_byte((uint8_t *)(getEEPROMBaseAddress()+OFFSET_ACCELERATION_SENSITIVITY), setting);
    _accelSensitivity = setting;
}

void LSM9DS1Sensor::setGysroSensitivitySetting(GyroSensitivitySetting setting) {
    eeprom_update_byte((uint8_t *)(getEEPROMBaseAddress()+OFFSET_GYRO_SENSITIVITY), setting);
    _gyroSensitivity = setting;
}

void LSM9DS1Sensor::setMagneticSensitivitySetting(MagneticSensitivitySetting setting) {
    eeprom_update_byte((uint8_t *)(getEEPROMBaseAddress()+OFFSET_MAGNETIC_SENSITIVITY), setting);
    _magneticSensitivity = setting;
}

#ifdef ENABLE_AMBASAT_COMMANDS
uint8_t LSM9DS1Sensor::handleCommand(uint16_t cmdSequenceID, uint8_t command, uint8_t* recievedData, uint8_t recievedDataLen){
    if (recievedDataLen != 1) {
        return CMD_STATUS_BAD_DATA_LEN;
    }
    PRINT_DEBUG(F("Set LSM9DS1 config\n"));

    switch (command) {
        case 0x00:
            if (*recievedData > 0x04) return CMD_STATUS_BAD_PARAM;
            PRINT_DEBUG(F("  Accel = "));
            PRINT_DEBUG(*recievedData);
            PRINT_DEBUG(F("\n"));
            setAcceleratonSensitivitySetting((AccelerationSensitivitySetting) *recievedData);
            break;
        case 0x01:
            if (((*recievedData)&0x8F) != 0 ) return CMD_STATUS_BAD_PARAM;
            PRINT_DEBUG(F("  Gyro = "));
            PRINT_DEBUG(*recievedData);
            PRINT_DEBUG(F("\n"));
            setGysroSensitivitySetting((GyroSensitivitySetting)*recievedData);
            break;
        case 0x02:
            if (*recievedData > 0x04) return CMD_STATUS_BAD_PARAM;
            PRINT_DEBUG(F("  Magn = "));
            PRINT_DEBUG(*recievedData);
            PRINT_DEBUG(F("\n"));
            setMagneticSensitivitySetting((MagneticSensitivitySetting) *recievedData);
            break;
        case 0x03:
            PRINT_DEBUG(F("  Set defaults\n"));
            setDefaultValues();
            break;
        default:
            return CMD_STATUS_BAD_PARAM;
    }
    this->_config.updateCRC();
    setSensorConfig(); //cement the changes.
    return CMD_STATUS_SUCCESS;
}
#endif