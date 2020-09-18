#ifndef __AmbaSat1App__
#define __AmbaSat1App__
#include <lmic.h>
#include "AmbaSat1Config.h"
#include "Sensors.h"
#include "PersistedConfiguration.h"


class AmbaSat1App {
private:
    PersistedConfiguration _config;

    VoltageSensor   _voltSensor;
    LSM9DS1Sensor   _lsm9DS1Sensor;
#if AMBASAT_MISSION_SENSOR == SENSOR_SI1132
    Si1132Sensor    _missionSensor;
#endif  // AMBASAT_MISSION_SENSOR

    bool _sleeping;
     
    void sendSensorPayload(SensorBase& sensor);

    friend void onEvent (ev_t ev);
public:
    static AmbaSat1App*  gApp;

    AmbaSat1App();
    virtual ~AmbaSat1App();

    // standard Arduino functions
    void setup();
    void loop();

};

#ifdef __cplusplus
extern "C"{
#endif

void initfunc (osjob_t* j);

#ifdef __cplusplus
} // extern "C"
#endif

#endif //__AmbaSat1App__