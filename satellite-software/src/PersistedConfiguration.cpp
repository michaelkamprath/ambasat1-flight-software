#include <Arduino.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include "PersistedConfiguration.h"
#include "Utilities.h"
#include "Logging.h"

//
// EEPROM Addresses for config Items
//
#define CONFIG_EEPROM_BASE_ADDR            0x0004

#define OFFSET_BOOT_COUNT                  0      // 4 bytes
#define OFFSET_UPLINK_FRAME_COUNT          4      // 4 byte

#define CONFIG_SATELLITE_BLOCK_SIZE        8

#define CONFIG_CRC_ADDR                    (E2END-5)    // 4 bytes prior to the last byte

SensorConfigurationDelegate::SensorConfigurationDelegate(PersistedConfiguration& config)
    : _baseEEPROMAddress(0xFFFF),
      _config(config)
{

}

void SensorConfigurationDelegate::setBaseEEPROMAddress(uint16_t address)
{
    // it's a programming error to call this more than once
    if (_baseEEPROMAddress != 0xFFFF) {
        PRINTLN_DEBUG(F("ERROR - called setBaseEEPROMAddress multiple times"));
        return;
    }
    _baseEEPROMAddress = address;
}

PersistedConfiguration::PersistedConfiguration()
    :   _LSM9DS1Delegate(nullptr),
        _missionSensorDelegate(nullptr)
{

}

void PersistedConfiguration::init(void)
{
    // its considered a programming error if this function is called before
    // the sensor delegates are set.
    if ((_LSM9DS1Delegate == nullptr)||(_missionSensorDelegate == nullptr)) {
        PRINTLN_DEBUG(F("ERROR called PersistedConfiguration init too soon"));
        return;
    }

    if (isEEPROMErased()) {
        resetToDefaults();
    } else {
        loadAllCongigurations();
    }

    setRebootCount(_rebootCount+1);
}

void PersistedConfiguration::setSensorConfigDelegates(SensorConfigurationDelegate* LSM9DS1Delegate, SensorConfigurationDelegate* missionSensorDelegate)
{
    // it is a programming error to call this more than once
    if (_LSM9DS1Delegate == nullptr) {
        _LSM9DS1Delegate = LSM9DS1Delegate;
    } else {
        PRINTLN_DEBUG(F("ERROR - callerd setLSM9DS1ConfigDelegate multiple times"));
    }
    if (_missionSensorDelegate == nullptr) {
        _missionSensorDelegate = missionSensorDelegate;
    } else {
        PRINTLN_DEBUG(F("ERROR - callerd setMissionSensorConfigDelegate multiple times"));
    }

    // now set eeprom base addresses
    _LSM9DS1Delegate->setBaseEEPROMAddress(CONFIG_EEPROM_BASE_ADDR+CONFIG_SATELLITE_BLOCK_SIZE);
    _missionSensorDelegate->setBaseEEPROMAddress(CONFIG_EEPROM_BASE_ADDR+CONFIG_SATELLITE_BLOCK_SIZE+_LSM9DS1Delegate->configBlockSize());
}

uint8_t PersistedConfiguration::configBlockSize(void) const
{
    return CONFIG_SATELLITE_BLOCK_SIZE + _LSM9DS1Delegate->configBlockSize() + _missionSensorDelegate->configBlockSize();
}

uint32_t PersistedConfiguration::getCRC(void) const
{
    // build data buffer
    uint8_t buffer[128];

    memcpy(&buffer[OFFSET_BOOT_COUNT], (const void *)&_rebootCount, sizeof(_rebootCount));
    memcpy(&buffer[OFFSET_UPLINK_FRAME_COUNT], (const void *)&_uplinkFrameCount, sizeof(_uplinkFrameCount));

    _LSM9DS1Delegate->writeConfigToBuffer(&buffer[CONFIG_SATELLITE_BLOCK_SIZE]);
    _missionSensorDelegate->writeConfigToBuffer(&buffer[CONFIG_SATELLITE_BLOCK_SIZE + _LSM9DS1Delegate->configBlockSize()]);

    uint32_t crcValue = calculateCRC(buffer, configBlockSize(), 0x0131);
    return crcValue;
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
    PRINTLN_DEBUG(F("Setting device configuration to default values."));

    setRebootCount(0, false);
    setUplinkFrameCount(0, false);

    _LSM9DS1Delegate->setDefaultValues();
    _missionSensorDelegate->setDefaultValues();

    updateCRC();

    // set flags to indicate EEPROM is set
    eeprom_update_byte((uint8_t*)0x0000, 0x00);
    eeprom_update_byte((uint8_t*)E2END, 0x00);
}

void PersistedConfiguration::loadAllCongigurations(void)
{
    PRINTLN_INFO(F("Loading device configuration from EEPROM."));

    _rebootCount = eeprom_read_dword((const uint32_t*)(CONFIG_EEPROM_BASE_ADDR+OFFSET_BOOT_COUNT));
    _uplinkFrameCount = eeprom_read_dword((const uint32_t*)(CONFIG_EEPROM_BASE_ADDR+OFFSET_UPLINK_FRAME_COUNT));

    _LSM9DS1Delegate->loadConfigValues();
    _missionSensorDelegate->loadConfigValues();

    if (checkCRC()) {
        PRINT_DEBUG(F("  Loaded configuration with:\n    reboot count = "));
        PRINT_DEBUG(_rebootCount);
        PRINT_DEBUG(F("\n    uplink frame count = "));
        PRINT_DEBUG(_uplinkFrameCount);
        PRINT_DEBUG(F("\n"));
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