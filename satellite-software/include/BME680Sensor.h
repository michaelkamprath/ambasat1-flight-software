#ifndef __BME680Sensor__
#define __BME680Sensor__
#include <Arduino.h>
#include "SensorBase.h"

class PersistedConfiguration;

//
// BME680 Sensor
//
#define BME680_I2C_ADDRESS            0x76

#define BME680_RESULTS_BUFFER_SIZE    18

// BME680 Config

typedef enum {
    BME680_OVERSAMPLING_NONE = 0,
    BME680_OVERSAMPLING_1x = 0b001,
    BME680_OVERSAMPLING_2x = 0b010,
    BME680_OVERSAMPLING_4x = 0b011,
    BME680_OVERSAMPLING_8x = 0b100,
    BME680_OVERSAMPLING_16x = 0b101
} BME680SensorOversamplingSetting;

typedef enum {
    BME_FILTER_COEF_0   = 0,
    BME_FILTER_COEF_1   = 0b001,
    BME_FILTER_COEF_3   = 0b010,
    BME_FILTER_COEF_7   = 0b011,
    BME_FILTER_COEF_15  = 0b100,
    BME_FILTER_COEF_31  = 0b101,
    BME_FILTER_COEF_63  = 0b110,
    BME_FILTER_COEF_127 = 0b111
} BME680IIRFilterCoefSetting;


class BME680Sensor : public SensorBase {

private:
    bool _sensorConfigured;
    unsigned long _measurementCompletionMillis;
    uint8_t _buffer[BME680_RESULTS_BUFFER_SIZE];

    BME680SensorOversamplingSetting _temperatureOversampling;
    BME680SensorOversamplingSetting _humidityOversampling;
    BME680SensorOversamplingSetting _pressureOversampling;
    BME680IIRFilterCoefSetting _iirCoefSetting;
    int16_t _gasProfile0_targetTemp;
    int16_t _gasProfile0_duration;

    bool begin(void);
    void reset(void);

    uint8_t calculateHeaterDuration(uint16_t durationMillis);
    bool calculateTemperatureTargetResistance(int16_t target_temp, int16_t amb_temp, uint8_t& out_res_heat_x );
    bool updateTemperatureTargetResistance(int16_t target_temp, int16_t amb_temp);
    uint16_t calculateMeasurmentDuration(void) const;

    int32_t calibratedTemperatureReading(uint8_t temp_adc_msb, uint8_t temp_adc_lsb, uint8_t temp_adc_xlsb, int32_t& t_fine);
    int32_t calibratedPressureReading(uint8_t press_adc_msb, uint8_t press_adc_lsb, uint8_t press_adc_xlsb, int32_t t_fine);
    int32_t calibratedHumidityReading(uint8_t hum_adc_msb, uint8_t hum_adc_lsb, int32_t temp_comp);
    int32_t calibratedGasResistance(uint8_t gas_adc_msb, uint8_t gas_adc_lsb);
    void setSensorConfig(void);

protected:
    virtual uint8_t i2cAutoIncrementBit(void) const             { return 0; }

    virtual uint8_t i2cDeviceAddress(void) const                { return BME680_I2C_ADDRESS; }

public:
    BME680Sensor(PersistedConfiguration& config);
    virtual ~BME680Sensor();

    virtual void setup(void);
    virtual bool isActive(void) const;
    virtual const uint8_t* getCurrentMeasurementBuffer(void);
    virtual uint8_t getMeasurementBufferSize() const            { return BME680_RESULTS_BUFFER_SIZE; }
    virtual uint8_t getPort() const                             { return 5; }

    void startMeasurementProcess(void);

    //
    // Sensor Configuration Delegate
    //

    virtual uint8_t configBlockSize( void ) const;
    virtual void setDefaultValues(void);
    virtual void loadConfigValues(void);
    virtual void writeConfigToBuffer( uint8_t* bufferBaseAddress) const;

    // BME680
    BME680SensorOversamplingSetting getTemperatureOversampling(void) const          { return _temperatureOversampling; }
    void setTemperatureOversampling(BME680SensorOversamplingSetting setting);

    BME680SensorOversamplingSetting getHumidityOversampling(void) const             { return _humidityOversampling; }
    void setHumidityOversampling(BME680SensorOversamplingSetting setting);

    BME680SensorOversamplingSetting getPressureOversampling(void) const             { return _pressureOversampling; }
    void setPressureOversampling(BME680SensorOversamplingSetting setting);

    BME680IIRFilterCoefSetting getIIRFilterCoef(void) const                         { return _iirCoefSetting; }
    void setIIRFilterCoef(BME680IIRFilterCoefSetting setting);

    int16_t getGasHeatDuration(uint8_t profile = 0) const                           { return _gasProfile0_duration; }
    void setGasHeatDuration(int16_t setting, uint8_t profile = 0);

    int16_t getGasHeaterTemperature(uint8_t profile = 0) const                      { return _gasProfile0_targetTemp; }
    void setGasHeatTemperature(int16_t setting, uint8_t profile = 0);

#ifdef ENABLE_AMBASAT_COMMANDS
    // handles a command payload.
    virtual uint8_t handleCommand(uint16_t cmdSequenceID, uint8_t command, uint8_t* recievedData, uint8_t recievedDataLen);

#endif

};

#endif // __BME680Sensor__