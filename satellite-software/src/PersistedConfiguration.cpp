#include <Arduino.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include "PersistedConfiguration.h"
#include "Utilities.h"

//
// EEPROM Addresses for config Items
//
#define CONFIG_EEPROM_BASE_ADDR            0x0004

#define OFFSET_BOOT_COUNT                  0      // 4 bytes
#define OFFSET_UPLINK_FRAME_COUNT          4      // 4 byte
#define OFFSET_ACCELERATION_SENSITIVITY    8      // 1 byte
#define OFFSET_GYRO_SENSITIVITY            9      // 1 byte
#define OFFSET_MAGNETIC_SENSITIVITY        10     // 1 byte

#define CONFIG_CRC_ADDR                    (E2END-5)    // 4 bytes prior to the last byte

#define CONFIG_DATA_BLOCK_SIZE             11
                       

PersistedConfiguration::PersistedConfiguration()
{
    if (isEEPROMErased()) {
        resetToDefaults();
    } else {
        loadAllCongigurations();
    }

    setRebootCount(_rebootCount+1);
}

uint32_t PersistedConfiguration::getCRC(void) const
{
    // build data buffer
    uint8_t buffer[CONFIG_DATA_BLOCK_SIZE];

    memcpy(&buffer[OFFSET_BOOT_COUNT], (const void *)&_rebootCount, sizeof(_rebootCount));
    memcpy(&buffer[OFFSET_UPLINK_FRAME_COUNT], (const void *)&_uplinkFrameCount, sizeof(_uplinkFrameCount));
    buffer[OFFSET_ACCELERATION_SENSITIVITY] = _accelSensitivity;
    buffer[OFFSET_GYRO_SENSITIVITY] = _gyroSensitivity;
    buffer[OFFSET_MAGNETIC_SENSITIVITY] = _magneticSensitivity;

    return calculateCRC(buffer, CONFIG_DATA_BLOCK_SIZE, 0x0131);
}

void PersistedConfiguration::updateCRC(void)
{
    uint32_t crc_value = getCRC();
    eeprom_update_dword((uint32_t*)CONFIG_CRC_ADDR, crc_value);
}

bool PersistedConfiguration::checkCRC(void) const 
{
    uint32_t saved_crc_value = eeprom_read_dword((uint32_t*)CONFIG_CRC_ADDR);
    return (saved_crc_value == getCRC());
}
bool PersistedConfiguration::isEEPROMErased(void) const
{
    // The first and last byte of EEPROM must be 0x00 to consider the
    // EEPROM as not erased. 
    uint8_t firstByte = eeprom_read_byte((uint8_t*)0x0000);
    uint8_t lastByte = eeprom_read_byte((uint8_t*)E2END);

    return !((firstByte == 0x00)&&(lastByte == 0x00));
}

void PersistedConfiguration::resetToDefaults(void)
{
    Serial.println(F("Setting device configuration to default values."));

    setRebootCount(0, false);
    setUplinkFrameCount(0, false);
    setAcceleratonSensitivitySetting(ACCELERATION_SENSITIVITY_2G, false);
    setGysroSensitivitySetting(GYRO_SENSITIVITY_245DPS, false);
    setMagneticSensitivitySetting(MAGNETIC_SENSITIVITY_4GAUSS, false);

    // set flags to indicate EEPROM is set
    eeprom_update_byte((uint8_t*)0x0000, 0x00);
    eeprom_update_byte((uint8_t*)E2END, 0x00);

    updateCRC();
}

void PersistedConfiguration::loadAllCongigurations(void)
{
    Serial.println(F("Loading device configuration from EEPROM."));

    _rebootCount = eeprom_read_dword((const uint32_t*)(CONFIG_EEPROM_BASE_ADDR+OFFSET_BOOT_COUNT));
    _uplinkFrameCount = eeprom_read_dword((const uint32_t*)(CONFIG_EEPROM_BASE_ADDR+OFFSET_UPLINK_FRAME_COUNT));
    _accelSensitivity = (AccelerationSensitivitySetting)eeprom_read_byte((uint8_t *)(CONFIG_EEPROM_BASE_ADDR+OFFSET_ACCELERATION_SENSITIVITY));
    _gyroSensitivity = (GyroSensitivitySetting)eeprom_read_byte((uint8_t *)(CONFIG_EEPROM_BASE_ADDR+OFFSET_GYRO_SENSITIVITY));
    _magneticSensitivity = (MagneticSensitivitySetting)eeprom_read_byte((uint8_t *)(CONFIG_EEPROM_BASE_ADDR+OFFSET_MAGNETIC_SENSITIVITY));

    if (checkCRC()) {
        Serial.print(F("  Loaded configuration with:\n    reboot count = "));
        Serial.print(_rebootCount);
        Serial.print(F("\n    uplink frame count = "));
        Serial.print(_uplinkFrameCount);
        Serial.print(F("\n    LSM9DS1 accel = 0x"));
        Serial.print(_accelSensitivity, HEX);
        Serial.print(F("\n    LSM9DS1 gyro = 0x"));
        Serial.print(_gyroSensitivity, HEX);
        Serial.print(F("\n    LSM9DS1 magnetic = 0x"));
        Serial.print(_magneticSensitivity, HEX);
        Serial.print(F("\n"));
    } else {
        // failed CRC check. Reset.
        Serial.println(F("  FAILED CRC check. Reseting configuration."));
        resetToDefaults();
    }
}

//
// configuration setters
//

void PersistedConfiguration::setRebootCount(uint32_t rebootCount, bool calculateCRC) {
    eeprom_update_dword((uint32_t*)(CONFIG_EEPROM_BASE_ADDR+OFFSET_BOOT_COUNT), rebootCount);
    _rebootCount = rebootCount;
    if (calculateCRC) {
        updateCRC();
    }
}

void PersistedConfiguration::setUplinkFrameCount(uint32_t frameCount, bool calculateCRC) {
    eeprom_update_dword((uint32_t*)(CONFIG_EEPROM_BASE_ADDR+OFFSET_UPLINK_FRAME_COUNT), frameCount);
    _uplinkFrameCount = frameCount;
    if (calculateCRC) {
        updateCRC();
    }
}

void PersistedConfiguration::setAcceleratonSensitivitySetting(AccelerationSensitivitySetting setting, bool calculateCRC) {
    eeprom_update_byte((uint8_t *)(CONFIG_EEPROM_BASE_ADDR+OFFSET_ACCELERATION_SENSITIVITY), setting);
    _accelSensitivity = setting;
    if (calculateCRC) {
        updateCRC();
    }
}

void PersistedConfiguration::setGysroSensitivitySetting(GyroSensitivitySetting setting, bool calculateCRC) {
    eeprom_update_byte((uint8_t *)(CONFIG_EEPROM_BASE_ADDR+OFFSET_GYRO_SENSITIVITY), setting);
    _gyroSensitivity = setting;
    if (calculateCRC) {
        updateCRC();
    }
}

void PersistedConfiguration::setMagneticSensitivitySetting(MagneticSensitivitySetting setting, bool calculateCRC) {
    eeprom_update_byte((uint8_t *)(CONFIG_EEPROM_BASE_ADDR+OFFSET_MAGNETIC_SENSITIVITY), setting);
    _magneticSensitivity = setting;
    if (calculateCRC) {
        updateCRC();
    }
}