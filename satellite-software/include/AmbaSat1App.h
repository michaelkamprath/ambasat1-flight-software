#ifndef __AmbaSat1App__
#define __AmbaSat1App__
#include "Sensors.h"


class AmbaSat1App {
private:

    VoltageSensor   _voltSensor;
    LSM9DS1Sensor   _lsm9DS1Sensor;
public:
    AmbaSat1App();
    virtual ~AmbaSat1App();

    // standard Arduino functions
    void setup();
    void loop();

};


#endif //__AmbaSat1App__