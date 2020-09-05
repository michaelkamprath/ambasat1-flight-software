#ifndef __Utilities__
#define __Utilities__
#include <Arduino.h>

// converts the int16_t to netork-endianness 
void hton_int16(int16_t x, uint8_t* val_buffer );

// Prints the passed buffer to serial in a human-readable format
void print_buffer( const uint8_t* buffer, uint8_t size);
#endif