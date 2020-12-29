#ifndef __AmbaSat1App__
#define __AmbaSat1App__
#include <lmic.h>
#include "AmbaSat1Config.h"
#include "LoRaPayloadBase.h"
#include "Sensors.h"
#include "PersistedConfiguration.h"
#include "LoggerSensor.h"

#define SATELLITE_STATUS_BUFFER_SIZE 7
//defines how many time intervals the satellite will wait between transmission
#define DEFAULT_RATE_VALUE 30
//define the single time interval duration
#define TIME_INTERVAL SLEEP_1S

class AmbaSat1App : public LoRaPayloadBase {
private:
    PersistedConfiguration _config;

    LSM9DS1Sensor   _lsm9DS1Sensor;
#if AMBASAT_MISSION_SENSOR == SENSOR_SI1132
    Si1132Sensor    _missionSensor;
#elif AMBASAT_MISSION_SENSOR == SENSOR_SHT30
    SHT30Sensor     _missionSensor;
#elif AMBASAT_MISSION_SENSOR == SENSOR_STS21
    STS21Sensor     _missionSensor;
#elif AMBASAT_MISSION_SENSOR == SENSOR_BME680
    BME680Sensor    _missionSensor;
#endif  // AMBASAT_MISSION_SENSOR

    #ifdef ENABLE_LOGGER_SENSOR
       LoggerSensor _loggerSensor;
    #endif

    uint8_t _buffer[SATELLITE_STATUS_BUFFER_SIZE];
    bool _sleeping;
     
    void sendSensorPayload(LoRaPayloadBase& sensor);

    friend void onEvent (ev_t ev);
public:
    static AmbaSat1App*  gApp;

    AmbaSat1App();
    virtual ~AmbaSat1App();

    // standard Arduino functions
    void setup();
    void loop();
    
    //
    // Methods for handling the "Statellite Status" payload
    virtual const uint8_t* getCurrentMeasurementBuffer(void);
    virtual uint8_t getMeasurementBufferSize() const                    { return SATELLITE_STATUS_BUFFER_SIZE; }
    virtual uint8_t getPort() const                                     { return 1; }

    int16_t readVccMilliVolts(void) const;


    // 
    // Command Handling (if enabled)
    //
#ifdef ENABLE_AMBASAT_COMMANDS
    void queueCommand(uint8_t port, uint8_t* recievedData, uint8_t recievedDataLen);
    void processQueuedCommand(void);
    virtual uint8_t handleCommand(uint16_t cmdSequenceID, uint8_t command, uint8_t* recievedData, uint8_t recievedDataLen);

    uint8_t executeBlinkCmd(uint8_t* recievedData, uint8_t recievedDataLen);
    uint8_t executeUplinkPatternCmd(uint8_t* recievedData, uint8_t recievedDataLen);
    uint8_t executeUplinkRateCmd(uint8_t* recievedData, uint8_t recievedDataLen);
    uint8_t executeSetFrameCountCmd(uint8_t* recievedData, uint8_t recievedDataLen);
    
private:
    #define QUEUED_COMMAND_BUFFER_SIZE MAX_LEN_FRAME
    uint8_t _queuedCommandPort;
    uint8_t _queuedCommandDataBuffer[QUEUED_COMMAND_BUFFER_SIZE];
    uint8_t _queuedCommandDataLength;
#endif

};

#ifdef __cplusplus
extern "C"{
#endif

void initfunc (osjob_t* j);

#ifdef __cplusplus
} // extern "C"
#endif

#endif //__AmbaSat1App__