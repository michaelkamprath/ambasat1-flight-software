#ifndef __AmbaSat1Config__
#define __AmbaSat1Config__
#include <Arduino.h>
// IMPORTANT - IMPORTANT - IMPORTANT - IMPORTANT - IMPORTANT - IMPORTANT
// 
// Set the following three values to match your unique AmbaSat-1 satellite.
// DO NOT commit this file with your private device identifiers into source control.
// KEEP your device identifiers priate.
//

// The Network Session Key
static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x00000000 ;  //<< CHANGE


#endif // __AmbaSat1Config__