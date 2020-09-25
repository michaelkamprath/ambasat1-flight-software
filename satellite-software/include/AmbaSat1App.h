#ifndef __AmbaSat1App__
#define __AmbaSat1App__
#include <lmic.h>
#include "AmbaSat1Config.h"
#include "LoRaPayloadBase.h"
#include "Sensors.h"
#include "PersistedConfiguration.h"


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
#endif  // AMBASAT_MISSION_SENSOR

    uint8_t _buffer[7];
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
    virtual uint8_t getMeasurementBufferSize() const                    { return 7; }
    virtual uint8_t getPort() const                                     { return 1; }

    int16_t readVccMilliVolts(void) const;
};

#ifdef __cplusplus
extern "C"{
#endif

void initfunc (osjob_t* j);

#ifdef __cplusplus
} // extern "C"
#endif

#endif //__AmbaSat1App__