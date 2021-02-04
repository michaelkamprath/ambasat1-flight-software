#ifndef __Si1132Sensor__
#define __Si1132Sensor__
#include "SensorBase.h"

class PersistedConfiguration;

#define SI1132_RESULTS_BUFFER_SIZE    9

class Si1132Sensor : public SensorBase
{
private:
    uint8_t _buffer[SI1132_RESULTS_BUFFER_SIZE];
    uint8_t _MEAS_RATE0;
    uint8_t _MEAS_RATE1;
    bool _sensorConfigured;

    uint8_t _adcGainVisible;
    uint8_t _adcGainInfraRed;
    bool _highSignalVisible;
    bool _highSignalInfraRed;

    bool begin(void);
    void reset(void);
    bool waitUntilSleep(void);
    uint8_t readResponseRegister(void);
    bool sendCommand(uint8_t cmd_value);
    bool setParameter(uint8_t param, uint8_t value);
    uint8_t readParameter(uint8_t param);

protected:
    // The Si1132 auto increment is on by default. Setting
    // bit 6 actually turns off auto incrment. So, set this
    // value to 0 to disable setting autoincrement bit in base
    // class.
    virtual uint8_t i2cAutoIncrementBit(void) const             { return 0; }

    virtual uint8_t i2cDeviceAddress(void) const                { return 0x60; }
public:
    Si1132Sensor(PersistedConfiguration& config);
    virtual ~Si1132Sensor();

    virtual void setup(void);
    virtual bool isActive(void) const;
    virtual const uint8_t* getCurrentMeasurementBuffer(void);
    virtual uint8_t getMeasurementBufferSize() const            { return SI1132_RESULTS_BUFFER_SIZE; }
    virtual uint8_t getPort() const                             { return 8; }

    //
    // Sensor Configuration Delegate
    //

    virtual uint8_t configBlockSize( void ) const;
    virtual void setDefaultValues(void);
    virtual void loadConfigValues(void);
    virtual void writeConfigToBuffer( uint8_t* bufferBaseAddress) const;

    uint8_t getVisibleADCGain(void) const                                           { return _adcGainVisible; }
    void setVisibleADCGain(uint8_t setting);

    uint8_t getInfraRedADCGain(void) const                                          { return _adcGainInfraRed; }
    void setInfraRedADCGain(uint8_t setting);

    bool isVisibleHighSignalRange(void) const                                       { return _highSignalVisible; }
    void setIsVisibleHighSignalRange(bool setting);

    bool isInfraRedHighSignalRange(void) const                                      { return _highSignalInfraRed; }
    void setIsInfraRedHighSignalRange(bool setting);

};



#endif //__Si1132Sensor__