#include <Arduino.h>
#include "STS21Sensor.h"
#include "Utilities.h"
#include "Logging.h"
#include "PersistedConfiguration.h"

#define STS21_CMD_GET_TEMPERATURE_NOHOLD        0b11110011
#define STS21_CMD_SOFT_RESET                    0b11111110
#define STS21_CMD_READ_REGISTER                 0b11100111
#define STS21_CMD_WRITE_REGISTER                0b11100110


STS21Sensor::STS21Sensor(PersistedConfiguration& config)
    :   SensorBase(config)
{
    if (!begin())
    {
        PRINTLN_ERROR(F("ERROR: unable to initialize the STS21"));
        setIsFound(false);
    } else {
        setIsFound(true);
    }
}

STS21Sensor::~STS21Sensor()
{

}

bool STS21Sensor::begin(void)
{
    uint8_t buffer[8];
    uint8_t serialNumber[8];

    if (!writeRegister(0b11111010, 0b00001111)) {
        return false;
    }
    if (!readData(buffer, 8)) {
        return false;
    }
    serialNumber[5] = buffer[0]; // SNB_3
    serialNumber[4] = buffer[2]; // SNB_2
    serialNumber[3] = buffer[4]; // SNB_1
    serialNumber[2] = buffer[6]; // SNB_0

    if (!writeRegister(0b11111100, 0b11001001)) {
        return false;
    }
    if (!readData(buffer, 6)) {
        return false;
    }
    serialNumber[1] = buffer[0]; // SNC_1
    serialNumber[0] = buffer[1]; // SNC_0
    serialNumber[7] = buffer[3]; // SNA_1
    serialNumber[6] = buffer[4]; // SNA_0

    uint16_t sna = serialNumber[7]*256 + serialNumber[6];
    int32_t snb = serialNumber[5]*256*256*256 + serialNumber[4]*256*256 + serialNumber[3]*256 + serialNumber[2];
    uint16_t snc = serialNumber[1]*256 + serialNumber[0];

    PRINT_INFO(F("Found STS21 sensor with serial number = "));
    PRINT_HEX_INFO(sna);
    PRINT_INFO(F("-"));
    PRINT_HEX_INFO(snb);
    PRINT_INFO(F("-"));
    PRINT_HEX_INFO(snc);
    PRINT_INFO(F("\n"));
    return true;
}

void STS21Sensor::reset(void)
{
    writeData(STS21_CMD_SOFT_RESET);
    delay(15);
}

void STS21Sensor::setup(void)
{
    reset();

}
bool STS21Sensor::isActive(void) const
{
    return SensorBase::isActive();
}
const uint8_t* STS21Sensor::getCurrentMeasurementBuffer(void)
{
    //
    // The buffer format is:
    //      temeprature reading - uint16_t  (with status bits masked)
    //      sensor status byte - uint8_t
    //
    //  a total of 3 bytes
    //

    uint8_t i2cBuffer[3];

    writeData(STS21_CMD_GET_TEMPERATURE_NOHOLD);
    delay(85);
    if (!readData(i2cBuffer,3)) {
        PRINTLN_ERROR(F("ERROR reading temperature from STS21"));
        return nullptr;
    }
    i2cBuffer[1] &= 0b11111100;     // clear sttus bits
    uint16_t tempReading = ((uint16_t)i2cBuffer[0])*256 + i2cBuffer[1];
    float temp = -46.85 + 175.72*((float)tempReading)/65536.0;
    hton_int16(tempReading, &_buffer[0]);

    writeData(STS21_CMD_READ_REGISTER);
    if (!readData(i2cBuffer,1)) {
        PRINTLN_ERROR(F("ERROR reading sensor status from STS21"));
        return nullptr;
    }
    _buffer[2] = i2cBuffer[0];

    PRINT_DEBUG(F("    Temperature reading = "));
    PRINT_DEBUG(temp);
    PRINT_DEBUG(F(" °C ( 0x"));
    PRINT_HEX_DEBUG(tempReading);
    PRINT_DEBUG(F(" ), sensor status = 0x"));
    PRINT_HEX_DEBUG(i2cBuffer[0]);
    PRINT_DEBUG(F("\n"));
    return _buffer;
}


uint8_t STS21Sensor::configBlockSize( void ) const
{
    return 0;
}

void STS21Sensor::setDefaultValues(void)
{
}

void STS21Sensor::loadConfigValues(void)
{
}

void STS21Sensor::writeConfigToBuffer( uint8_t* bufferBaseAddress) const
{
}
