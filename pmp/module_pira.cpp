#include "module_pira.h"

#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>

#include "debug.h"

#ifdef MODULE_PIRA_DEBUG
#define NAME "pira"
#define serial_debug Serial
#endif

Adafruit_BME280 bme;  // comm over I2C

#define SAFETY_POWER_PERIOD 1800  // 20 minutes
#define SAFETY_STARTUP 60
#define SAFETY_SHUTDOWN 15
#define KEEP_ALIVE_TIMEOUT 86400000  // 24 hours

scheduler_result_t MODULE_PIRA::scheduler(void) {
    // If we want to receive UART messages we have to put the system in delay instead of deep sleep, return { ..., true } in that case
    if (global_activate_pira != INACTIVE) {
        if (status_pira_state_machine == IDLE_PIRA) {
            uint16_t voltage = get_voltage_in_mv(MODULE_SYSTEM_BAN_MON_AN);
            if (voltage > MODULE_PIRA_UNDERVOLTAGE_THRESHOLD) {
#ifdef serial_debug
                serial_debug.print(NAME);
                serial_debug.println(": scheduler(activated)");
#endif
                return {5000, false};
            } else {
                global_activate_pira = INACTIVE;
#ifdef serial_debug
                serial_debug.print(NAME);
                serial_debug.print(": scheduler(");
                serial_debug.print("undervoltage lockout!, voltage = ");
                serial_debug.print(voltage);
                serial_debug.println(" mV)");
#endif
                return DONT_RUN;
            }
        } else {
            // In all other states, run with a delay of 5 secs
#ifdef serial_debug
            serial_debug.print(NAME);
            serial_debug.println(": scheduler(active)");
#endif
            switch (status_pira_state_machine) {
                case WAIT_STATUS_ON:
                    return {5000, false};
                default:
                    return {5000, false};
            }
        }
    } else if (KEEP_ALIVE_TIMEOUT > 0 && millis() - last_active > KEEP_ALIVE_TIMEOUT) {
#ifdef serial_debug
        serial_debug.print(NAME);
        serial_debug.println(": scheduler(keep alive)");
#endif
        global_activate_pira = KEEP_ALIVE;
        return {0, false};
    } else {
#ifdef serial_debug
        serial_debug.print(NAME);
        serial_debug.println(": scheduler(inactive)");
#endif
        return DONT_RUN;
    }
}

void status_pin_changed() {
#ifdef serial_debug
    serial_debug.print(NAME);
    serial_debug.println(": status pin change!");
#endif
    STM32L0.wakeup();
}

void MODULE_PIRA::initialize(void) {
    last_active = millis();
    status_pira_state_machine = IDLE_PIRA;
    state_prev = IDLE_PIRA;

    // Configure pins
    pinMode(MODULE_5V_EN, OUTPUT);
    digitalWrite(MODULE_5V_EN, LOW);
    pinMode(MODULE_PIRA_5V, OUTPUT);
    digitalWrite(MODULE_PIRA_5V, LOW);

    // Prepare status pin
    pinMode(MODULE_PIRA_STATUS, INPUT_PULLDOWN);

    attachInterrupt(digitalPinToInterrupt(MODULE_PIRA_STATUS), status_pin_changed, CHANGE);

    // Start Uart communication
    MODULE_PIRA_SERIAL.begin(115200);

    // Enable bme sensor
    uint8_t status = bme.begin();
#ifdef serial_debug
    if (!status) {
        serial_debug.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        serial_debug.print("SensorID was: 0x");
        serial_debug.println(bme.sensorID(), 16);
        serial_debug.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        serial_debug.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        serial_debug.print("        ID of 0x60 represents a BME 280.\n");
        serial_debug.print("        ID of 0x61 represents a BME 680.\n");
    }
#endif
}

void MODULE_PIRA::running(void) {
    // Receive any command from raspberry
    pira_state_machine();
}

/**
 * @brief Sends status values over uart
 *
 * @return none (void)
 */
void MODULE_PIRA::send_status_values(Uart& out, int attempt) {
    out.println("START VALUES");

    out.print("attempt:");
    out.println(attempt);

    out.print("activation:");

    switch (global_activate_pira) {
        case INACTIVE:
            out.println("none");
            break;
        case LORA:
            out.println("lora");
            break;
        case KEEP_ALIVE:
            out.println("alive");
            break;
    }

    out.print("stm32_temp:");
    out.println(serial_values.stm32_temp);

    out.print("bridge_volt:");
    out.println(serial_values.bridge_volt);

    out.print("bridge_temp:");
    out.println(serial_values.bridge_temp);

    out.print("bridge_hum:");
    out.println(serial_values.bridge_hum);

    out.print("bridge_hpa:");
    out.println(serial_values.bridge_hpa);

    out.print("data:");
    printHex(global_relay_payload, global_relay_payload_size, out);
    out.println();

    out.println("END VALUES");
}

/**
 * @brief Transitions to next state and saves the
 * time when the state was entered.
 *
 */
void MODULE_PIRA::pira_state_transition(state_pira_e next) {
    stateTimeoutStart = millis();
    state_prev = status_pira_state_machine;
    status_pira_state_machine = next;
}

/**
 * @brief Check if the state has timed out
 *
 * @return bool
 */
bool MODULE_PIRA::pira_state_check_timeout(void) {
    // stateTimeoutDuration can be disabled
    if (stateTimeoutDuration == 0) {
        return false;
    }

    pira_elapsed = millis() - stateTimeoutStart;

    // All values come in seconds, so we also need pira_elapsed in seconds
    pira_elapsed = pira_elapsed / 1000;

    // check if we have been in the current state too long
    return pira_elapsed >= stateTimeoutDuration;
}
/**
 * @brief Returns state string from given state enum
 *
 * @param[in] status_pira_state_machine
 *
 * @return char*
 */
char* MODULE_PIRA::return_state(state_pira_e status_pira_state_machine) {
    switch (status_pira_state_machine) {  // clang-format off
        case IDLE_PIRA: return "IDLE_PIRA";
        case WAIT_STATUS_ON: return "WAIT_STATUS_ON";
        case WAKEUP: return "WAKEUP";
        case WAIT_SHUTDOWN: return "WAIT_SHUTDOWN";
        case STOP_PIRA: return "STOP_PIRA";
        default: return "ERROR";
    }  // clang-format on
}

/**
 * @brief Finite state machine loop for Raspberry Pi
 *
 * @param[in] safety_power_period
 * @param[in] safety_sleep_period
 * @param[in] operational_wakeup
 * @param[in] safety_reboot
 * @param[in] turnOnRpi
 *
 * @return none (void)
 */
void MODULE_PIRA::pira_state_machine() {
#ifdef serial_debug
    pira_elapsed = millis() - stateTimeoutStart;
    serial_debug.print(NAME);
    serial_debug.print(": fsm(");
    serial_debug.print(return_state(state_prev));
    serial_debug.print(" -> ");
    serial_debug.print(return_state(status_pira_state_machine));
    serial_debug.print(" ");
    serial_debug.print(pira_elapsed / 1000);
    serial_debug.print("/");
    serial_debug.print(stateTimeoutDuration);
    serial_debug.print("s");
    serial_debug.println(")");
#endif

    // update previous state
    state_prev = status_pira_state_machine;

    switch (status_pira_state_machine) {
        case IDLE_PIRA:
            serial_values.stm32_temp = STM32L0.getTemperature();
            serial_values.bridge_volt = get_voltage_in_mv(MODULE_SYSTEM_BAN_MON_AN);
            serial_values.bridge_temp = bme.readTemperature();
            serial_values.bridge_hum = bme.readHumidity();
            serial_values.bridge_hpa = (int)(bme.readPressure() / 100.0F);

            pira_state_transition(WAIT_STATUS_ON);
            break;

        case WAIT_STATUS_ON:
            stateTimeoutDuration = SAFETY_STARTUP;
            state_goto_timeout = STOP_PIRA;

            // WAIT_STATUS_ON state reached, turn on power for raspberry pi
            digitalWrite(MODULE_5V_EN, HIGH);
            digitalWrite(MODULE_PIRA_5V, HIGH);

            // If status pin is read as high go to WAKEUP state
            if (digitalRead(MODULE_PIRA_STATUS)) {
#ifdef serial_debug
                serial_debug.println("SENDING STATUS VALUES");
                send_status_values(serial_debug, 0);
#endif
                for (int x = 0; x < 10; x++) {
                    send_status_values(MODULE_PIRA_SERIAL, x);
                    delay(300);
                }

                pira_state_transition(WAKEUP);
            }
            break;

        case WAKEUP:
            // When woken up by button press, disable timeout
            stateTimeoutDuration = SAFETY_POWER_PERIOD;

            state_goto_timeout = STOP_PIRA;

            if (!digitalRead(MODULE_PIRA_STATUS)) {
                pira_state_transition(WAIT_SHUTDOWN);
            }
            break;

        case WAIT_SHUTDOWN:
            // Give the Pi time to gracefully shutdown
            stateTimeoutDuration = SAFETY_SHUTDOWN;
            state_goto_timeout = STOP_PIRA;

            break;

        case STOP_PIRA:
#ifdef serial_debug
            serial_debug.print(NAME);
            serial_debug.println(": fsm(RPI turned off)");
#endif

            // Turn off Rpi
            digitalWrite(MODULE_5V_EN, LOW);
            digitalWrite(MODULE_PIRA_5V, LOW);

            // Reset global variables connected with
            // HackThePoacher functionality
            global_activate_pira = INACTIVE;
            last_active = millis();
            pira_state_transition(IDLE_PIRA);
            break;

        default:
            status_pira_state_machine = IDLE_PIRA;
            break;
    }

    // check if the existing state has timed out and transition to next state
    // TODO check here if rotating millis functions works as it should
    if (pira_state_check_timeout()) {
        pira_state_transition(state_goto_timeout);
    }
}
/*** end of file ***/
