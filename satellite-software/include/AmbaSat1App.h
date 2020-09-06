#ifndef __AmbaSat1App__
#define __AmbaSat1App__
#include <lmic.h>
#include "Sensors.h"


class AmbaSat1App {
private:

    VoltageSensor   _voltSensor;
    LSM9DS1Sensor   _lsm9DS1Sensor;
    bool _sleeping;

    void sendSensorPayload(SensorBase& sensor, uint8_t port );

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