#ifndef __SHT30Sensor__
#define __SHT30Sensor__
#include "SensorBase.h"


class SHT30Sensor : public SensorBase
{
    uint8_t _buffer[5];

    bool _enableHeater;

    bool begin(void);
    void reset(void);

    bool sendCommand(uint16_t cmd, bool acceptNACKAtEnd = false );
    bool readTwoBytesAndCRC(uint16_t* outValue);

    uint16_t readStatus(void);
    bool checkRestartAlertStatus(void);

protected:
    // The SHT30 has no concept of register address autoincrement
    virtual uint8_t i2cAutoIncrementBit(void) const             { return 0; }
    virtual uint8_t i2cDeviceAddress(void) const                { return 0x44; }
public:
    SHT30Sensor(PersistedConfiguration& config);
    virtual ~SHT30Sensor();

    virtual void setup(void);
    virtual bool isActive(void) const;
    virtual const uint8_t* getCurrentMeasurementBuffer(void);
    virtual uint8_t getMeasurementBufferSize() const            { return 5; }
    virtual uint8_t getPort() const                             { return 3; }

    //
    // Sensor Configuration Delegate
    //

    virtual uint8_t configBlockSize( void ) const;
    virtual void setDefaultValues(void);
    virtual void loadConfigValues(void);
    virtual void writeConfigToBuffer( uint8_t* bufferBaseAddress) const;

    bool isHeaterEnabled(void) const        { return _enableHeater; }
    void setIsHeaterEnabled(bool setting);

#ifdef ENABLE_AMBASAT_COMMANDS
    // handles a command payload.
    virtual uint8_t handleCommand(uint16_t cmdSequenceID, uint8_t command, uint8_t* recievedData, uint8_t recievedDataLen);

#endif

};

#endif // __SHT30Sensor__