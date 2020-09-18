#include <Arduino.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include "PersistedConfiguration.h"

//
// EEPROM Addresses for config Items
//

#define CONFIG_ADDR_BOOT_COUNT                  0x0004      // 4 bytes
#define CONFIG_ADDR_ACCELERATION_SENSITIVITY    0x0008      // 1 byte
#define CONFIG_ADDR_GYRO_SENSITIVITY            0x0009      // 1 byte
#define CONFIG_ADDR_MAGNETIC_SENSITIVITY        0x000A      // 1 byte


PersistedConfiguration::PersistedConfiguration()
{
    if (isEEPROMErased()) {
        resetToDefaults();
    } else {
        loadAllCongigurations();
    }

    setRebootCount(_rebootCount+1);
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

    setRebootCount(0);
    setAcceleratonSensitivitySetting(ACCELERATION_SENSITIVITY_2G);
    setGysroSensitivitySetting(GYRO_SENSITIVITY_245DPS);
    setMagneticSensitivitySetting(MAGNETIC_SENSITIVITY_4GAUSS);

    // set flags to indicate EEPROM is set
    eeprom_update_byte((uint8_t*)0x0000, 0x00);
    eeprom_update_byte((uint8_t*)E2END, 0x00);
}

void PersistedConfiguration::loadAllCongigurations(void)
{
    Serial.println(F("Loading device configuration from EEPROM."));

    _rebootCount = eeprom_read_dword((const uint32_t*)CONFIG_ADDR_BOOT_COUNT);
    _accelSensitivity = (AccelerationSensitivitySetting)eeprom_read_byte((uint8_t *)CONFIG_ADDR_ACCELERATION_SENSITIVITY);
    _gyroSensitivity = (GyroSensitivitySetting)eeprom_read_byte((uint8_t *)CONFIG_ADDR_GYRO_SENSITIVITY);
    _magneticSensitivity = (MagneticSensitivitySetting)eeprom_read_byte((uint8_t *)CONFIG_ADDR_MAGNETIC_SENSITIVITY);

    Serial.print(F("  Loaded configuration with:\n    reboot count = "));
    Serial.print(_rebootCount);
    Serial.print(F("\n    LSM9DS1 accel = 0x"));
    Serial.print(_accelSensitivity, HEX);
    Serial.print(F("\n    LSM9DS1 gyro = 0x"));
    Serial.print(_gyroSensitivity, HEX);
    Serial.print(F("\n    LSM9DS1 magnetic = 0x"));
    Serial.print(_magneticSensitivity, HEX);
    Serial.print(F("\n"));

}

//
// configuration setters
//

void PersistedConfiguration::setRebootCount(uint32_t rebootCount) {
    eeprom_update_dword((uint32_t*)CONFIG_ADDR_BOOT_COUNT, rebootCount);
    _rebootCount = rebootCount;
}

void PersistedConfiguration::setAcceleratonSensitivitySetting(AccelerationSensitivitySetting setting) {
    eeprom_update_byte((uint8_t *)CONFIG_ADDR_ACCELERATION_SENSITIVITY, setting);
    _accelSensitivity = setting;
}

void PersistedConfiguration::setGysroSensitivitySetting(GyroSensitivitySetting setting) {
    eeprom_update_byte((uint8_t *)CONFIG_ADDR_GYRO_SENSITIVITY, setting);
    _gyroSensitivity = setting;
}

void PersistedConfiguration::setMagneticSensitivitySetting(MagneticSensitivitySetting setting) {
    eeprom_update_byte((uint8_t *)CONFIG_ADDR_MAGNETIC_SENSITIVITY, setting);
    _magneticSensitivity = setting;
}