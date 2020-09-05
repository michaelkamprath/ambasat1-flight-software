#ifndef __Sensors__
#define __Sensors__
#include <Arduino.h>

class SensorBase {
    static bool _isI2CSetUp;

public:
    SensorBase();
    virtual ~SensorBase();

    // This functions is expected to get a current sensor measurement
    // the fill a byte buffer that weill be transmitted via the 
    // LoRaWAN radio. It is the sensor's responsibility to define the
    // the buffer format for it's measurement. 
    // IMPORTANT: Returns NULL if sensor read was not successful.
    virtual const uint8_t* getCurrentMeasurementBuffer(void) = 0;

    virtual uint8_t getMeasurementBufferSize() const = 0;
};

//
// Voltage Sensor
//

class VoltageSensor : public SensorBase {
private:
    uint8_t _buffer[2];

public:
    VoltageSensor();
    virtual ~VoltageSensor();

    virtual const uint8_t* getCurrentMeasurementBuffer(void);
    virtual uint8_t getMeasurementBufferSize() const                    { return 2; }
};


//
// LSM9DS1 Sensor
//
#include <Adafruit_LSM9DS1.h>

class LSM9DS1Sensor : public SensorBase {
private:
    Adafruit_LSM9DS1 _lsm;
    uint8_t _buffer[18];

    void setSensorValueAtBufferLocation(float sensor_value, uint8_t index);
public:
    LSM9DS1Sensor();
    virtual ~LSM9DS1Sensor();

    virtual const uint8_t* getCurrentMeasurementBuffer(void);
    virtual uint8_t getMeasurementBufferSize() const                    { return 18; }
};

#endif //__Sensors__