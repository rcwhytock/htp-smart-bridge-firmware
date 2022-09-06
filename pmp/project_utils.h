#ifndef PROJECT_UTILS_H_
#define PROJECT_UTILS_H_

#include "Arduino.h"

bool check_i2c();
void printHex(byte buffer[], uint8_t size, Uart &out);
#endif /* PROJECT_UTILS_H */
       /*** end of file ***/