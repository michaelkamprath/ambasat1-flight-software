#include <Arduino.h>
#include "Logging.h"
#include "AmbaSat1App.h"

AmbaSat1App* satelliteApp;

void setup()
{
  Serial.begin(9600);

  satelliteApp = new AmbaSat1App();
  satelliteApp->setup();
}

void loop()
{
  satelliteApp->loop();

}