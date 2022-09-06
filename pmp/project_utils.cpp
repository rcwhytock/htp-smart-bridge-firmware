#include "project_utils.h"

#include "debug.h"

#ifdef PROJECT_UTILS_DEBUG
#define serial_debug Serial
#endif

/**
 * @brief function to check if i2c pull-up is present, call only once prior to init of Wire library
 *
 * @return bool
 */
bool check_i2c() {
    bool fail = false;
    pinMode(PIN_WIRE_SCL, INPUT);
    pinMode(PIN_WIRE_SDA, INPUT);

    if (LOW == digitalRead(PIN_WIRE_SCL)) {
        // no I2C pull-up detected
#ifdef serial_debug
        serial_debug.print("i2c pull-up error(");
        serial_debug.println("scl)");
#endif
        fail = true;
    }
    if (LOW == digitalRead(PIN_WIRE_SDA)) {
        // no I2C pull-up detected
#ifdef serial_debug
        serial_debug.print("i2c pull-up error(");
        serial_debug.println("sda)");
#endif
        fail = true;
    }
    return fail;
}

void printHex(byte buffer[], uint8_t size, Uart &out) {
    char dataString[2];
    for (uint8_t n = 0; n < size; n++) {
        sprintf(dataString, "%02X", buffer[n]);
        out.print(dataString);
    }
}
/*** end of file ***/