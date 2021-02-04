#ifndef __Utilities__
#define __Utilities__
#include <Arduino.h>

// converts the int16_t to netork-endianness
void hton_int16(int16_t x, uint8_t* val_buffer );
// converts the int16_t in the buffer to host-endianness
int16_t ntoh_int16(uint8_t* val_buffer );

// converts the int32_t to netork-endianness
void hton_int32(int32_t x, uint8_t* val_buffer );

// converts the int32_t to netork-endianness
int32_t ntoh_int32(uint8_t* val_buffer);

// Prints the passed buffer to serial in a human-readable format
void print_buffer( const uint8_t* buffer, uint8_t size);

// calculate a CRC on a given block of data using a specified polynomial
uint8_t calculateCRC(const uint8_t* bytes, uint8_t nbrOfBytes, uint16_t polynomial);
#endif