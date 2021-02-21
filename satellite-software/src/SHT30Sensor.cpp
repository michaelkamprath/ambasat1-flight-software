#include <Arduino.h>
#include <Wire.h>
#include "SHT30Sensor.h"
#include "Utilities.h"
#include "Logging.h"
#include "PersistedConfiguration.h"

#define SHT30_CMD_READ_SERIALNBR    0x3780  // read serial number
#define SHT30_CMD_READ_STATUS       0xF32D  // read status register
#define SHT30_CMD_CLEAR_STATUS      0x3041  // clear status register
#define SHT30_CMD_HEATER_ENABLE     0x306D  // enabled heater
#define SHT30_CMD_HEATER_DISABLE    0x3066  // disable heater
#define SHT30_CMD_SOFT_RESET        0x30A2  // soft reset
#define SHT30_CMD_MEAS_CLOCKSTR_H   0x2C06  // measurement: clock stretching, high repeatability
#define SHT30_CMD_MEAS_CLOCKSTR_M   0x2C0D  // measurement: clock stretching, medium repeatability
#define SHT30_CMD_MEAS_CLOCKSTR_L   0x2C10  // measurement: clock stretching, low repeatability
#define SHT30_CMD_MEAS_POLLING_H    0x2400  // measurement: polling, high repeatability
#define SHT30_CMD_MEAS_POLLING_M    0x240B  // measurement: polling, medium repeatability
#define SHT30_CMD_MEAS_POLLING_L    0x2416  // measurement: polling, low repeatability
#define SHT30_CMD_MEAS_PERI_05_H    0x2032  // measurement: periodic 0.5 mps, high repeatability
#define SHT30_CMD_MEAS_PERI_05_M    0x2024  // measurement: periodic 0.5 mps, medium repeatability
#define SHT30_CMD_MEAS_PERI_05_L    0x202F  // measurement: periodic 0.5 mps, low repeatability
#define SHT30_CMD_MEAS_PERI_1_H     0x2130  // measurement: periodic 1 mps, high repeatability
#define SHT30_CMD_MEAS_PERI_1_M     0x2126  // measurement: periodic 1 mps, medium repeatability
#define SHT30_CMD_MEAS_PERI_1_L     0x212D  // measurement: periodic 1 mps, low repeatability
#define SHT30_CMD_MEAS_PERI_2_H     0x2236  // measurement: periodic 2 mps, high repeatability
#define SHT30_CMD_MEAS_PERI_2_M     0x2220  // measurement: periodic 2 mps, medium repeatability
#define SHT30_CMD_MEAS_PERI_2_L     0x222B  // measurement: periodic 2 mps, low repeatability
#define SHT30_CMD_MEAS_PERI_4_H     0x2334  // measurement: periodic 4 mps, high repeatability
#define SHT30_CMD_MEAS_PERI_4_M     0x2322  // measurement: periodic 4 mps, medium repeatability
#define SHT30_CMD_MEAS_PERI_4_L     0x2329  // measurement: periodic 4 mps, low repeatability
#define SHT30_CMD_MEAS_PERI_10_H    0x2737  // measurement: periodic 10 mps, high repeatability
#define SHT30_CMD_MEAS_PERI_10_M    0x2721  // measurement: periodic 10 mps, medium repeatability
#define SHT30_CMD_MEAS_PERI_10_L    0x272A  // measurement: periodic 10 mps, low repeatability
#define SHT30_CMD_FETCH_DATA        0xE000  // readout measurements for periodic mode
#define SHT30_CMD_R_AL_LIM_LS       0xE102  // read alert limits, low set
#define SHT30_CMD_R_AL_LIM_LC       0xE109  // read alert limits, low clear
#define SHT30_CMD_R_AL_LIM_HS       0xE11F  // read alert limits, high set
#define SHT30_CMD_R_AL_LIM_HC       0xE114  // read alert limits, high clear
#define SHT30_CMD_W_AL_LIM_HS       0x611D  // write alert limits, high set
#define SHT30_CMD_W_AL_LIM_HC       0x6116  // write alert limits, high clear
#define SHT30_CMD_W_AL_LIM_LC       0x610B  // write alert limits, low clear
#define SHT30_CMD_W_AL_LIM_LS       0x6100  // write alert limits, low set
#define SHT30_CMD_NO_SLEEP          0x303E

#define SHT30_CRC_POLYNOMIAL        0x131   // P(x) = x^8 + x^5 + x^4 + 1 = 100110001

SHT30Sensor::SHT30Sensor(PersistedConfiguration& config)
    :   SensorBase(config),
        _enableHeater(false)
{
    if (!begin())
    {
        PRINTLN_ERROR(F("ERROR: unable to initialize the SHT30"));
        setIsFound(false);
    } else {
        setIsFound(true);
    }
}

SHT30Sensor::~SHT30Sensor()
{

}

bool SHT30Sensor::sendCommand(uint16_t cmd, bool acceptNACKAtEnd )
{
    // when sending commands without clock stretching, the SHT30 will respond with a NACK rather than
    // and ACK. This generates and error = 3 in the Wire library when ending transimission. The argument
    // acceptNACKAtEnd determines if the we expect an NACK in repsonse, and then tell SensorBase that getting a
    // NACK on data transfer is OK.
    uint8_t buffer[2];
    buffer[0] = cmd >> 8;
    buffer[1] = cmd&0xFF;

    return writeData(i2cDeviceAddress(), buffer, 2, true, acceptNACKAtEnd);
}

bool SHT30Sensor::readTwoBytesAndCRC(uint16_t* outValue)
{
    uint8_t buffer[3];
    if (!readData(buffer, 3)) {
        return false;
    }
    if (calculateCRC(buffer, 2, SHT30_CRC_POLYNOMIAL) != buffer[2]) {
        return false;
    }

    *outValue = (uint16_t)buffer[0]*256 + buffer[1];
    return true;
}

uint16_t  SHT30Sensor::readStatus(void)
{
    uint16_t statusValue = 0xFFFF;
    sendCommand(SHT30_CMD_READ_STATUS);
    if (!readTwoBytesAndCRC(&statusValue)) {
        PRINTLN_ERROR("ERROR: failed CRC check when reading two bytes from SHT30");
    }
    return statusValue;
}

// returns true if the alert status for brownout reboots. In such a
// case, the sensor configuration would need to be reloaded.
bool SHT30Sensor::checkRestartAlertStatus(void)
{
    uint16_t status = readStatus();
    if ((status&0x0010) > 0) {
        // a brownout reboot has occured. Clear alert state.
        PRINTLN_INFO(F("SHT30 has had a brownout reboot. Clearing alerts."));
        sendCommand(SHT30_CMD_CLEAR_STATUS);
        return true;
    }
    return false;
}
bool SHT30Sensor::begin(void)
{
    reset();
    checkRestartAlertStatus();

    sendCommand(SHT30_CMD_READ_SERIALNBR);
    uint8_t buffer[6];
    if (!readData(buffer, 6)) {
        return false;
    }
    if (
        (calculateCRC(&buffer[0], 2, SHT30_CRC_POLYNOMIAL) != buffer[2])
        ||(calculateCRC(&buffer[3], 2, SHT30_CRC_POLYNOMIAL) != buffer[5])
    ) {
        PRINTLN_ERROR(F("ERROR: CRC check failed when reading SHT30 serial number."));
        return false;
    }
    uint16_t serialHigh = (uint16_t)buffer[0]*256 + buffer[1];
    uint16_t serialLow = (uint16_t)buffer[3]*256 + buffer[4];

    PRINT_INFO(F("Found SHT30 sensor with serial number = "));
    PRINT_HEX_INFO(serialHigh);
    PRINT_INFO(F("-"));
    PRINT_HEX_INFO(serialLow);
    PRINT_INFO(F("\n"));
    return true;
}

void SHT30Sensor::reset(void)
{
    if (!sendCommand(SHT30_CMD_SOFT_RESET, true)) {
        Serial.println(F("ERROR: could not reset SHT30"));
        return;
    }
    delay(15);
    // after reset, enable heater as configured
    if (isHeaterEnabled()) {
        if (sendCommand(SHT30_CMD_HEATER_ENABLE, true)) {
            PRINTLN_DEBUG(F("SHT30 heater is enabled"));
        } else {
            PRINTLN_ERROR(F("ERROR unable to enable SHT30 heater"));
        }
    } else {
        if (!sendCommand(SHT30_CMD_HEATER_DISABLE, true)) {
            PRINTLN_ERROR(F("ERROR unable to disable SHT30 heater"));
        }
    }
}

void SHT30Sensor::setup(void)
{
    // The sensor will be used in single shot mode. Not much to set up here.
}
bool SHT30Sensor::isActive(void) const
{
    return SensorBase::isActive();
}

const uint8_t* SHT30Sensor::getCurrentMeasurementBuffer(void)
{
    bool browoutRestartAlert = false;

    if (checkRestartAlertStatus()) {
        setup();
        browoutRestartAlert = true;
    }
    if (!sendCommand(SHT30_CMD_MEAS_CLOCKSTR_H)) {
        PRINTLN_ERROR(F("ERROR: Could not send single shot measurement command to SHT30"));
        return nullptr;
    }
    // need to wait for the measurement to be done.
    delay(500);

    uint8_t localBuffer[6];
    if (!readData(localBuffer, 6)) {
        PRINTLN_ERROR(F("ERROR: Failed to read measurement data from SHT30"));
        return nullptr;
    }
     if (
        (calculateCRC(&localBuffer[0], 2, SHT30_CRC_POLYNOMIAL) != localBuffer[2])
        ||(calculateCRC(&localBuffer[3], 2, SHT30_CRC_POLYNOMIAL) != localBuffer[5])
    ) {
        PRINTLN_ERROR(F("ERROR: CRC check failed when reading SHT30 measurement."));
        return nullptr;
    }
    uint16_t temperatureReading = (uint16_t)localBuffer[0]*256 + localBuffer[1];
    uint16_t humidityReading = (uint16_t)localBuffer[3]*256 + localBuffer[4];

#if LOG_LEVEL >= LOG_LEVEL_DEBUG
    float temperature = -45.0 + 175.0*((float)temperatureReading)/65535.0;
    float humidity = 100.0*((float)humidityReading)/65535.0;
#if LOG_CELSIUS_TEMP == 0
    temperature = (temperature*9.0/5.0) + 32.0;
#endif

    PRINT_DEBUG(F("  SHT30 readings are: temperature = "));
    PRINT_DEBUG(temperature);
#if LOG_CELSIUS_TEMP == 0
    PRINT_DEBUG(F(" °F, humidity = "));
#else
    PRINT_DEBUG(F(" °C, humidity = "));
#endif
    PRINT_DEBUG(humidity);
    PRINT_DEBUG(F("%\n"));
#endif

    //
    // Buffer format is:
    //
    //     uint16_t     temeperature reading
    //     uint16_t     humidity reading
    //     uint8_t      status byte, with this format:
    //          bit 7 = 1 if there was a brown out restart alert at the start of this method
    //          bit 6 = Heater status
    //          bit 5 = RH Tracking Alert
    //          bit 4 = Temp Tracking Alert
    //          bits 3-0 = reserved
    //  Total bytes = 5
    //
    uint16_t status = readStatus();
    hton_int16(temperatureReading, &_buffer[0]);
    hton_int16(humidityReading, &_buffer[2]);
    _buffer[4] = 0;
    // brownout restart alert
    if (browoutRestartAlert) _buffer[4] |= 0b10000000;
    // heater status
    if ((status&0x2000) > 0) _buffer[4] |= 0b01000000;
    // RH Tracking Alert
    if ((status&0x0800) > 0) _buffer[4] |= 0b00100000;
    // Temp Tracking Alert
    if ((status&0x0400) > 0) _buffer[4] |= 0b00010000;
    return _buffer;
}

//
// Sensor Configuration Delegate
//

#define OFFSET_HEATER_ENABLE                0       // 1 byte

#define CONFIG_SHT30_SENSOR_BLOCK_SIZE      1

uint8_t SHT30Sensor::configBlockSize( void ) const
{
    return CONFIG_SHT30_SENSOR_BLOCK_SIZE;
}

void SHT30Sensor::setDefaultValues(void)
{
    setIsHeaterEnabled(false);
}

void SHT30Sensor::loadConfigValues(void)
{
    _enableHeater = (eeprom_read_byte((uint8_t *)(getEEPROMBaseAddress()+OFFSET_HEATER_ENABLE)) > 0);
}

void SHT30Sensor::writeConfigToBuffer( uint8_t* bufferBaseAddress) const
{
    bufferBaseAddress[OFFSET_HEATER_ENABLE] = (_enableHeater ? 1 : 0);
}

void SHT30Sensor::setIsHeaterEnabled(bool setting)
{
    uint8_t eeprom_value = (setting ? 1 : 0);
    eeprom_update_byte((uint8_t *)(getEEPROMBaseAddress()+OFFSET_HEATER_ENABLE), eeprom_value);
    _enableHeater = setting;
}

#ifdef ENABLE_AMBASAT_COMMANDS
uint8_t SHT30Sensor::handleCommand(uint16_t cmdSequenceID, uint8_t command, uint8_t* recievedData, uint8_t recievedDataLen)
{
    if (command == 0x01) {
        // set heater status
        if (recievedDataLen != 1) {
            return CMD_STATUS_BAD_DATA_LEN;
        }
        bool enable = (*recievedData) > 0;

        PRINT_DEBUG(F("  Heater enable = "));
        PRINT_DEBUG(enable);
        PRINT_DEBUG(F("\n"));

        setIsHeaterEnabled(enable);

        this->_config.updateCRC();
        // the heater configuration gets applied in the sensor reset
        reset();
        return CMD_STATUS_SUCCESS;
    }

    return CMD_STATUS_UNKNOWN_CMD;
}
#endif //ENABLE_AMBASAT_COMMANDS