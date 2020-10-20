#ifndef __BME680Sensor__
#define __BME680Sensor__
#include <Arduino.h>
#include "SensorBase.h"

class PersistedConfiguration;

//
// BME680 Sensor
//
#define BME680_I2C_ADDRESS            0x76

#define BME680_RESULTS_BUFFER_SIZE    12

class BME680Sensor : public SensorBase {

private:
    bool _sensorConfigured;
    unsigned long _measurementCompletionMillis;
    uint8_t _buffer[BME680_RESULTS_BUFFER_SIZE];

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
};

#endif // __BME680Sensor__