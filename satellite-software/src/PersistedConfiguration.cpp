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
#define OFFSET_UPLINK_PATTERN              8      // 1 byte
#define OFFSET_LAST_PAYLOAD_UPLINKED       9      // 1 byte
#define OFFSET_UPLINK_RATE_VALUE           10     // 1 byte

#define CONFIG_SATELLITE_BLOCK_SIZE        11

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
        PRINTLN_DEBUG(F("ERROR multiple setBaseEEPROMAddress"));
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
    updateCRC();
}

void PersistedConfiguration::setSensorConfigDelegates(SensorConfigurationDelegate* LSM9DS1Delegate, SensorConfigurationDelegate* missionSensorDelegate)
{
    // it is a programming error to call this more than once
    if (_LSM9DS1Delegate == nullptr) {
        _LSM9DS1Delegate = LSM9DS1Delegate;
    } else {
        PRINTLN_DEBUG(F("ERROR multiple setLSM9DS1ConfigDelegate"));
    }
    if (_missionSensorDelegate == nullptr) {
        _missionSensorDelegate = missionSensorDelegate;
    } else {
        PRINTLN_DEBUG(F("ERROR multiple setMissionSensorConfigDelegate"));
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
    buffer[OFFSET_UPLINK_PATTERN] = _uplinkPattern;
    buffer[OFFSET_LAST_PAYLOAD_UPLINKED] = static_cast<uint8_t>(_lastPayloadUplinked);
    buffer[OFFSET_UPLINK_RATE_VALUE] = _uplinkRateValue;

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
    PRINTLN_DEBUG(F("Setting default device configuration"));

    setRebootCount(0);
    setUplinkFrameCount(0);
    setUplinkPattern(0);
    setLastPayloadUplinked(MISSION_SENSOR_PAYLOAD);
    setUplinkSleepCycles(75);

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
    uint8_t buffer[CONFIG_SATELLITE_BLOCK_SIZE];
    eeprom_read_block(buffer, (uint8_t *)CONFIG_EEPROM_BASE_ADDR, CONFIG_SATELLITE_BLOCK_SIZE);

    memcpy(&_rebootCount, &buffer[OFFSET_BOOT_COUNT], sizeof(uint32_t));
    memcpy(&_uplinkFrameCount, &buffer[OFFSET_UPLINK_FRAME_COUNT], sizeof(uint32_t));
    _uplinkPattern = buffer[OFFSET_UPLINK_PATTERN];
    _lastPayloadUplinked = static_cast<UplinkPayloadType>(buffer[OFFSET_LAST_PAYLOAD_UPLINKED]);
    _uplinkRateValue = buffer[OFFSET_UPLINK_RATE_VALUE];

    _LSM9DS1Delegate->loadConfigValues();
    _missionSensorDelegate->loadConfigValues();

    if (checkCRC()) {
        PRINT_DEBUG(F("  Loaded configuration with:\n   reboot count: "));
        PRINT_DEBUG(_rebootCount);
        PRINT_DEBUG(F("\n   uplink frame count: "));
        PRINT_DEBUG(_uplinkFrameCount);
        PRINT_DEBUG(F("\n   sleep cycles: "));
        PRINT_DEBUG(_uplinkRateValue);
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

void PersistedConfiguration::setRebootCount(uint32_t rebootCount) {
    eeprom_update_dword((uint32_t*)(CONFIG_EEPROM_BASE_ADDR+OFFSET_BOOT_COUNT), rebootCount);
    _rebootCount = rebootCount;
}

void PersistedConfiguration::setUplinkFrameCount(uint32_t frameCount) {
    eeprom_update_dword((uint32_t*)(CONFIG_EEPROM_BASE_ADDR+OFFSET_UPLINK_FRAME_COUNT), frameCount);
    _uplinkFrameCount = frameCount;
}

void PersistedConfiguration::setUplinkPattern(uint8_t pattern)
{
    eeprom_update_byte((uint8_t *)(CONFIG_EEPROM_BASE_ADDR+OFFSET_UPLINK_PATTERN), pattern);
    _uplinkPattern = pattern;
}

void PersistedConfiguration::setLastPayloadUplinked(UplinkPayloadType payload)
{
    uint8_t eeprom_value = static_cast<uint8_t>(payload);
    eeprom_update_byte((uint8_t *)(CONFIG_EEPROM_BASE_ADDR+OFFSET_LAST_PAYLOAD_UPLINKED), eeprom_value);
    _lastPayloadUplinked = payload;
}

void PersistedConfiguration::setUplinkSleepCycles(uint8_t rateValue)
{
    eeprom_update_byte((uint8_t *)(CONFIG_EEPROM_BASE_ADDR+OFFSET_UPLINK_RATE_VALUE), rateValue);
    _uplinkRateValue = rateValue;
}