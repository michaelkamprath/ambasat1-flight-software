#ifndef __Logging__
#define __Logging__

#define LOG_LEVEL_NONE    0
#define LOG_LEVEL_ERRORS  1
#define LOG_LEVEL_INFO    2
#define LOG_LEVEL_DEBUG   3
#define LOG_LEVEL_DEBUG_ONLY   4

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INFO
#endif

#if LOG_LEVEL == LOG_LEVEL_DEBUG

#define PRINT_DEBUG(x)          Serial.print(x)
#define PRINT_HEX_DEBUG(x)      Serial.print(x, HEX)
#define PRINTLN_DEBUG(x)        Serial.println(x)
#define PRINT_INFO(x)           Serial.print(x)
#define PRINT_HEX_INFO(x)       Serial.print(x, HEX)
#define PRINTLN_INFO(x)         Serial.println(x)
#define PRINT_ERROR(x)          Serial.print(x)
#define PRINT_HEX_ERROR(x)      Serial.print(x, HEX)
#define PRINTLN_ERROR(x)        Serial.println(x)

#elif LOG_LEVEL == LOG_LEVEL_INFO

#define PRINT_DEBUG(x)
#define PRINT_HEX_DEBUG(x)
#define PRINTLN_DEBUG(x)
#define PRINT_INFO(x)           Serial.print(x)
#define PRINT_HEX_INFO(x)       Serial.print(x, HEX)
#define PRINTLN_INFO(x)         Serial.println(x)
#define PRINT_ERROR(x)          Serial.print(x)
#define PRINT_HEX_ERROR(x)      Serial.print(x, HEX)
#define PRINTLN_ERROR(x)        Serial.println(x)

#elif LOG_LEVEL == LOG_LEVEL_ERRORS

#define PRINT_DEBUG(x)
#define PRINT_HEX_DEBUG(x)
#define PRINTLN_DEBUG(x)
#define PRINT_INFO(x)
#define PRINT_HEX_INFO(x)
#define PRINTLN_INFO(x)
#define PRINT_ERROR(x)          Serial.print(x)
#define PRINT_HEX_ERROR(x)      Serial.print(x, HEX)
#define PRINTLN_ERROR(x)        Serial.println(x)

#elif LOG_LEVEL == LOG_LEVEL_NONE

#define PRINT_DEBUG(x)
#define PRINT_HEX_DEBUG(x)
#define PRINTLN_DEBUG(x)
#define PRINT_INFO(x)
#define PRINT_HEX_INFO(x)
#define PRINTLN_INFO(x)
#define PRINT_ERROR(x)
#define PRINT_HEX_ERROR(x)
#define PRINTLN_ERROR(x)

#elif LOG_LEVEL == LOG_LEVEL_DEBUG_ONLY

#define PRINT_DEBUG(x)          Serial.print(x)
#define PRINT_HEX_DEBUG(x)      Serial.print(x, HEX)
#define PRINTLN_DEBUG(x)        Serial.println(x)
#define PRINT_INFO(x)
#define PRINT_HEX_INFO(x)
#define PRINTLN_INFO(x)
#define PRINT_ERROR(x)
#define PRINT_HEX_ERROR(x)
#define PRINTLN_ERROR(x)

#endif // LOG_LEVEL

#endif // __Logging__