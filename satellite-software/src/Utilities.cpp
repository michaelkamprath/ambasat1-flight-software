#include "Utilities.h"


void hton_int16(int16_t x, uint8_t* val_buffer )
{
#if BYTE_ORDER == LITTLE_ENDIAN
    uint8_t *s = (uint8_t *)&x;
    val_buffer[0] = s[1];
    val_buffer[1] = s[0];
#else
    uint8_t *s = (uint8_t *)&x;
    val_buffer[0] = s[0];
    val_buffer[1] = s[1];
#endif
}

void hton_int32(int32_t x, uint8_t* val_buffer )
{
#if BYTE_ORDER == LITTLE_ENDIAN
    uint8_t *s = (uint8_t *)&x;
    val_buffer[0] = s[3];
    val_buffer[1] = s[2];
    val_buffer[2] = s[1];
    val_buffer[3] = s[0];
#else
    uint8_t *s = (uint8_t *)&x;
    val_buffer[0] = s[0];
    val_buffer[1] = s[1];
    val_buffer[2] = s[2];
    val_buffer[3] = s[3];
#endif
}

void print_buffer( const uint8_t* buffer, uint8_t bufffer_size)
{
    Serial.print("{ ");
    for (int16_t i = 0; i < bufffer_size; i++ ) {
        Serial.print(F("0x"));
        Serial.print(buffer[i], HEX);
        if (i < bufffer_size - 1) {
            Serial.print(F(", "));
        } else {
            Serial.println(F(" }"));
        }
    }
}

uint8_t calculateCRC(const uint8_t* bytes, uint8_t nbrOfBytes, uint16_t polynomial)
{
    uint8_t crc = 0xFF; // calculated checksum
    // calculates 8-Bit checksum with given polynomial
    for(uint8_t byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++)
    {
        crc ^= (bytes[byteCtr]);
        for(uint8_t bit = 8; bit > 0; --bit)
        {
            if(crc & 0x80) crc = (crc << 1) ^ polynomial;
            else           crc = (crc << 1);
        }
    }
    return crc;
}