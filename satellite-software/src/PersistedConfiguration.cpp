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
#define OFFSET_ACCELERATION_SENSITIVITY    8      // 1 byte
#define OFFSET_GYRO_SENSITIVITY            9      // 1 byte
#define OFFSET_MAGNETIC_SENSITIVITY        10     // 1 byte

#define CONFIG_SATELLITE_BLOCK_SIZE        11

#define CONFIG_MISSION_SENSOR_EEPROM_BASE_ADDR  (CONFIG_EEPROM_BASE_ADDR+CONFIG_SATELLITE_BLOCK_SIZE)

#if AMBASAT_MISSION_SENSOR == SENSOR_BME680
#define OFFSET_TEMP_OVERSAMPLING            0       // 1 byte
#define OFFSET_HUMIDITY_OVERSAMPLING        1       // 1 byte
#define OFFSET_PRESSURE_OVERSAMPLING        2       // 1 byte
#define OFFSET_IIR_COEF                     3       // 1 byte
#define OFFSET_GAS_HEAT_DURATION            4       // 2 bytes
#define OFFSET_GAS_HEAT_TEMP                6       // 2 bytes

#define CONFIG_MISSION_SENSOR_BLOCK_SIZE    8
#elif AMBASAT_MISSION_SENSOR == SENSOR_SI1132
#define OFFSET_ADC_GAIN_VISIBLE             0       // 1 byte
#define OFFSET_ADC_GAIN_IR                  1       // 1 byte
#define OFFSET_HIGH_SIGNAL_VISIBLE          2       // 1 byte
#define OFFSET_HIGH_SIGNAL_IR               3       // 1 byte

#define CONFIG_MISSION_SENSOR_BLOCK_SIZE    4
#else
#define CONFIG_MISSION_SENSOR_BLOCK_SIZE    0
#endif

#define CONFIG_DATA_BLOCK_SIZE             (CONFIG_SATELLITE_BLOCK_SIZE+CONFIG_MISSION_SENSOR_BLOCK_SIZE)

#define CONFIG_CRC_ADDR                    (E2END-5)    // 4 bytes prior to the last byte


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

    // mission sensor
 #if AMBASAT_MISSION_SENSOR == SENSOR_BME680
    buffer[CONFIG_SATELLITE_BLOCK_SIZE+OFFSET_TEMP_OVERSAMPLING] = _temperatureOversampling;
    buffer[CONFIG_SATELLITE_BLOCK_SIZE+OFFSET_HUMIDITY_OVERSAMPLING] = _humidityOversampling;
    buffer[CONFIG_SATELLITE_BLOCK_SIZE+OFFSET_PRESSURE_OVERSAMPLING] = _pressureOversampling;
    buffer[CONFIG_SATELLITE_BLOCK_SIZE+OFFSET_IIR_COEF] = _iirCoefSetting;
    buffer[CONFIG_SATELLITE_BLOCK_SIZE+OFFSET_GAS_HEAT_DURATION] = _gasProfile0_duration;
    buffer[CONFIG_SATELLITE_BLOCK_SIZE+OFFSET_GAS_HEAT_TEMP] = _gasProfile0_targetTemp;
#elif AMBASAT_MISSION_SENSOR == SENSOR_SI1132
    buffer[CONFIG_SATELLITE_BLOCK_SIZE+OFFSET_ADC_GAIN_VISIBLE] = _adcGainVisible;
    buffer[CONFIG_SATELLITE_BLOCK_SIZE+OFFSET_ADC_GAIN_IR] = _adcGainInfraRed;
    buffer[CONFIG_SATELLITE_BLOCK_SIZE+OFFSET_HIGH_SIGNAL_VISIBLE] = (_highSignalVisible ? 1 : 0);
    buffer[CONFIG_SATELLITE_BLOCK_SIZE+OFFSET_HIGH_SIGNAL_IR] = (_highSignalInfraRed ? 1 : 0);
#endif // AMBASAT_MISSION_SENSOR

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
    PRINTLN_DEBUG(F("Setting device configuration to default values."));

    setRebootCount(0, false);
    setUplinkFrameCount(0, false);
    setAcceleratonSensitivitySetting(ACCELERATION_SENSITIVITY_2G, false);
    setGysroSensitivitySetting(GYRO_SENSITIVITY_245DPS, false);
    setMagneticSensitivitySetting(MAGNETIC_SENSITIVITY_4GAUSS, false);

    // mission sensor
 #if AMBASAT_MISSION_SENSOR == SENSOR_BME680
    setTemperatureOversampling(BME680_OVERSAMPLING_8x, false);
    setHumidityOversampling(BME680_OVERSAMPLING_8x, false);
    setPressureOversampling(BME680_OVERSAMPLING_8x, false);
    setIIRFilterCoef(BME_FILTER_COEF_3, false);
    setGasHeatTemperature(320, false);
    setGasHeatDuration(150, false);
#elif AMBASAT_MISSION_SENSOR == SENSOR_SI1132
    setVisibleADCGain(0, false);
    setInfraRedADCGain(0, false);
    setIsVisibleHighSignalRange(false, false);
    setIsInfraRedHighSignalRange(false, false);
#endif // AMBASAT_MISSION_SENSOR

    // set flags to indicate EEPROM is set
    eeprom_update_byte((uint8_t*)0x0000, 0x00);
    eeprom_update_byte((uint8_t*)E2END, 0x00);

    updateCRC();
}

void PersistedConfiguration::loadAllCongigurations(void)
{
    PRINTLN_INFO(F("Loading device configuration from EEPROM."));

    _rebootCount = eeprom_read_dword((const uint32_t*)(CONFIG_EEPROM_BASE_ADDR+OFFSET_BOOT_COUNT));
    _uplinkFrameCount = eeprom_read_dword((const uint32_t*)(CONFIG_EEPROM_BASE_ADDR+OFFSET_UPLINK_FRAME_COUNT));
    _accelSensitivity = (AccelerationSensitivitySetting)eeprom_read_byte((uint8_t *)(CONFIG_EEPROM_BASE_ADDR+OFFSET_ACCELERATION_SENSITIVITY));
    _gyroSensitivity = (GyroSensitivitySetting)eeprom_read_byte((uint8_t *)(CONFIG_EEPROM_BASE_ADDR+OFFSET_GYRO_SENSITIVITY));
    _magneticSensitivity = (MagneticSensitivitySetting)eeprom_read_byte((uint8_t *)(CONFIG_EEPROM_BASE_ADDR+OFFSET_MAGNETIC_SENSITIVITY));

    // mission sensor
 #if AMBASAT_MISSION_SENSOR == SENSOR_BME680
    _temperatureOversampling = (BME680SensorOversamplingSetting)eeprom_read_byte((uint8_t *)(CONFIG_MISSION_SENSOR_EEPROM_BASE_ADDR+OFFSET_TEMP_OVERSAMPLING));
    _humidityOversampling = (BME680SensorOversamplingSetting)eeprom_read_byte((uint8_t *)(CONFIG_MISSION_SENSOR_EEPROM_BASE_ADDR+OFFSET_HUMIDITY_OVERSAMPLING));
    _pressureOversampling = (BME680SensorOversamplingSetting)eeprom_read_byte((uint8_t *)(CONFIG_MISSION_SENSOR_EEPROM_BASE_ADDR+OFFSET_PRESSURE_OVERSAMPLING));
    _iirCoefSetting = (BME680IIRFilterCoefSetting)eeprom_read_byte((uint8_t *)(CONFIG_MISSION_SENSOR_EEPROM_BASE_ADDR+OFFSET_IIR_COEF));
    _gasProfile0_duration = (uint16_t)eeprom_read_word((uint16_t *)(CONFIG_MISSION_SENSOR_EEPROM_BASE_ADDR+OFFSET_GAS_HEAT_DURATION));
    _gasProfile0_targetTemp = (uint16_t)eeprom_read_word((uint16_t *)(CONFIG_MISSION_SENSOR_EEPROM_BASE_ADDR+OFFSET_GAS_HEAT_TEMP));
#elif AMBASAT_MISSION_SENSOR == SENSOR_SI1132
    _adcGainVisible = eeprom_read_byte((uint8_t *)(CONFIG_MISSION_SENSOR_EEPROM_BASE_ADDR+OFFSET_ADC_GAIN_VISIBLE));
    _adcGainInfraRed = eeprom_read_byte((uint8_t *)(CONFIG_MISSION_SENSOR_EEPROM_BASE_ADDR+OFFSET_ADC_GAIN_IR));
    _highSignalVisible = (eeprom_read_byte((uint8_t *)(CONFIG_MISSION_SENSOR_EEPROM_BASE_ADDR+OFFSET_HIGH_SIGNAL_VISIBLE)) > 0);
    _highSignalInfraRed = (eeprom_read_byte((uint8_t *)(CONFIG_MISSION_SENSOR_EEPROM_BASE_ADDR+OFFSET_HIGH_SIGNAL_IR)) > 0);
#endif // AMBASAT_MISSION_SENSOR

    if (checkCRC()) {
        PRINT_DEBUG(F("  Loaded configuration with:\n    reboot count = "));
        PRINT_DEBUG(_rebootCount);
        PRINT_DEBUG(F("\n    uplink frame count = "));
        PRINT_DEBUG(_uplinkFrameCount);
        PRINT_DEBUG(F("\n    LSM9DS1 accel = 0x"));
        PRINT_HEX_DEBUG(_accelSensitivity);
        PRINT_DEBUG(F("\n    LSM9DS1 gyro = 0x"));
        PRINT_HEX_DEBUG(_gyroSensitivity);
        PRINT_DEBUG(F("\n    LSM9DS1 magnetic = 0x"));
        PRINT_HEX_DEBUG(_magneticSensitivity);
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

//
// Mission Sensor
//


#if AMBASAT_MISSION_SENSOR == SENSOR_BME680
void PersistedConfiguration::setTemperatureOversampling(BME680SensorOversamplingSetting setting, bool calculateCRC)
{
    eeprom_update_byte((uint8_t *)(CONFIG_MISSION_SENSOR_EEPROM_BASE_ADDR+OFFSET_TEMP_OVERSAMPLING), setting);
    _temperatureOversampling = setting;
    if (calculateCRC) {
        updateCRC();
    }    
}

void PersistedConfiguration::setHumidityOversampling(BME680SensorOversamplingSetting setting, bool calculateCRC)
{
    eeprom_update_byte((uint8_t *)(CONFIG_MISSION_SENSOR_EEPROM_BASE_ADDR+OFFSET_HUMIDITY_OVERSAMPLING), setting);
    _humidityOversampling = setting;
    if (calculateCRC) {
        updateCRC();
    }    
}

void PersistedConfiguration::setPressureOversampling(BME680SensorOversamplingSetting setting, bool calculateCRC)
{
    eeprom_update_byte((uint8_t *)(CONFIG_MISSION_SENSOR_EEPROM_BASE_ADDR+OFFSET_PRESSURE_OVERSAMPLING), setting);
    _humidityOversampling = setting;
    if (calculateCRC) {
        updateCRC();
    }  
}

void PersistedConfiguration::setIIRFilterCoef(BME680IIRFilterCoefSetting setting, bool calculateCRC)
{
    eeprom_update_byte((uint8_t *)(CONFIG_MISSION_SENSOR_EEPROM_BASE_ADDR+OFFSET_IIR_COEF), setting);
    _iirCoefSetting = setting;
    if (calculateCRC) {
        updateCRC();
    } 
}

void PersistedConfiguration::setGasHeatDuration(int16_t setting, bool calculateCRC, uint8_t profile)
{
    // for now, profile is ignored
    eeprom_update_word((uint16_t *)(CONFIG_MISSION_SENSOR_EEPROM_BASE_ADDR+OFFSET_GAS_HEAT_DURATION), setting);
    _gasProfile0_duration = setting;
    if (calculateCRC) {
        updateCRC();
    }
}

void PersistedConfiguration::setGasHeatTemperature(int16_t setting, bool calculateCRC, uint8_t profile)
{
    // for now, profile is ignored
    eeprom_update_word((uint16_t *)(CONFIG_MISSION_SENSOR_EEPROM_BASE_ADDR+OFFSET_GAS_HEAT_TEMP), setting);
    _gasProfile0_targetTemp = setting;
    if (calculateCRC) {
        updateCRC();
    }
}
#elif AMBASAT_MISSION_SENSOR == SENSOR_SI1132
// Si1132
void PersistedConfiguration::setVisibleADCGain(uint8_t setting, bool calculateCRC)
{
    // we only keep bottom 3 bits
    uint8_t setting_value = setting&0b00000111;

    eeprom_update_byte((uint8_t *)(CONFIG_MISSION_SENSOR_EEPROM_BASE_ADDR+OFFSET_ADC_GAIN_VISIBLE), setting_value);
    _adcGainVisible = setting_value;
    if (calculateCRC) {
        updateCRC();
    } 
}

void PersistedConfiguration::setInfraRedADCGain(uint8_t setting, bool calculateCRC)
{
    // we only keep bottom 3 bits
    uint8_t setting_value = setting&0b00000111;

    eeprom_update_byte((uint8_t *)(CONFIG_MISSION_SENSOR_EEPROM_BASE_ADDR+OFFSET_ADC_GAIN_IR), setting_value);
    _adcGainInfraRed = setting_value;
    if (calculateCRC) {
        updateCRC();
    } 
}

void PersistedConfiguration::setIsVisibleHighSignalRange(bool setting, bool calculateCRC)
{
    uint8_t eeprom_value = (setting ? 1 : 0);
    eeprom_update_byte((uint8_t *)(CONFIG_MISSION_SENSOR_EEPROM_BASE_ADDR+OFFSET_HIGH_SIGNAL_VISIBLE), eeprom_value);
    _highSignalVisible = setting;
    if (calculateCRC) {
        updateCRC();
    } 
}

void PersistedConfiguration::setIsInfraRedHighSignalRange(bool setting, bool calculateCRC)
{
    uint8_t eeprom_value = (setting ? 1 : 0);
    eeprom_update_byte((uint8_t *)(CONFIG_MISSION_SENSOR_EEPROM_BASE_ADDR+OFFSET_HIGH_SIGNAL_IR), eeprom_value);
    _highSignalVisible = setting;
    if (calculateCRC) {
        updateCRC();
    } 
}

#endif