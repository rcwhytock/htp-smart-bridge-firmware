#include <STM32L0.h>
#include <TimerMillis.h>

#include "adc.h"
#include "debug.h"
#include "project.h"
#include "project_utils.h"

#ifdef MAIN_DEBUG
#define serial_debug Serial
#endif

enum button_state_e {
    NONE,
    PRESSED,
    RELEASED,
};
// Global variables

// Declared externaly in project.h,
// only modules pira and lacuna can access it.
activation_reason_e global_activate_pira = INACTIVE;
uint32_t button_pressed_time = 0;
uint32_t button_released_time = 0;
button_state_e button_state = NONE;

// Declared externaly in project.h,
byte global_relay_payload[255];
uint16_t global_relay_payload_size = 0;

// All possible states in finite state machine
enum state_e {
    INIT,
    MODULE_INIT,
    IDLE,
    HIBERNATION,
    ALWAYS_ON,
    SHOW_POWER_STATE,
};

/**
 * @brief Decode state from enum to string
 *
 * @param uint8_t state
 *
 * @note typecast state_e to uint8_t
 *
 * @returns state in a string
 */
char* decode_state(state_e state) {
    switch (state) {  // clang-format off
        case INIT: return "INIT";
        case MODULE_INIT: return "MODULE_INIT";
        case IDLE: return "IDLE";
        case HIBERNATION: return "HIBERNATION";
        case ALWAYS_ON: return "ALWAYS_ON";
        case SHOW_POWER_STATE: return "SHOW_POWER_STATE";
        default: return "NOT_CORRECT_STATE";
    }  // clang-format on
}
/**
 * @brief Get reset cause
 *
 * @param enumrated reset cause
 *
 * @returns reset cause in a string
 */
char* decode_reset_cause(uint8_t reset_cause) {
    switch (reset_cause) {  // clang-format off
        case 0: return "POWERON";
        case 1: return "EXTERNAL";
        case 2: return "SOFTWARE";
        case 3: return "WATCHDOG";
        case 4: return "FIREWALL";
        case 5: return "OTHER";
        case 6: return "STANDBY";
        default: return "OTHER";
    }  // clang-format on
}

// Variables dealing with FSM
state_e state = INIT;
state_e state_prev = INIT;

/**
 * @brief Changes to next state
 *
 */
void state_transition(state_e next) { state = next; }

void button_triggered() {
    // Ignore button events if button already released in last second or if last button event has not been consumed
    if (button_state == RELEASED || millis() - button_released_time < 1000) {
        return;
    }

    bool button_pressed = !digitalRead(BOARD_BUTTON);
    if (button_pressed && button_state != PRESSED) {
#ifdef serial_debug
        serial_debug.println("BUTTON PRESS");
#endif
        button_state = PRESSED;
        button_pressed_time = millis();
    } else if (!button_pressed && button_state == PRESSED) {
        button_released_time = millis();
        if (button_released_time - button_pressed_time >= 100) {
#ifdef serial_debug
            serial_debug.print("BUTTON RELEASE: ");
            serial_debug.println(button_released_time - button_pressed_time);
#endif
            button_state = RELEASED;
            STM32L0.wakeup();
        } else {
            button_state = NONE;
        }
    }
}

uint32_t consume_button_event() {
    if (button_state == RELEASED) {
        button_state = NONE;
        return button_released_time - button_pressed_time;
    } else {
        return 0;
    }
}

/**
 * @brief Setup function called on boot
 *
 */
void setup() {
    // Serial port debug setup
#ifdef serial_debug
    serial_debug.begin(115200);
    serial_debug.println();  // Empty line for clarity
    serial_debug.println("START OF PROGRAM");
    serial_debug.print("CAUSE OF RESET WAS: ");
    serial_debug.println(decode_reset_cause(STM32L0.resetCause()));
    serial_debug.println();  // Empty line for clarity
#endif

    pinMode(BOARD_BUTTON, INPUT_PULLUP);
    pinMode(BOARD_LED, OUTPUT);
    digitalWrite(BOARD_LED, LOW);

    if (!digitalRead(BOARD_BUTTON) && STM32L0.resetCause() == 1) {
        pinMode(MODULE_5V_EN, OUTPUT);
        digitalWrite(MODULE_5V_EN, HIGH);
        pinMode(MODULE_PIRA_5V, OUTPUT);
        digitalWrite(MODULE_PIRA_5V, HIGH);
        state_transition(ALWAYS_ON);
        return;
    }

    attachInterrupt(digitalPinToInterrupt(BOARD_BUTTON), button_triggered, CHANGE);

    STM32L0.wdtEnable(22000);

    // Prepare gpios, outputs and inputs
    pinMode(MODULE_LACUNA_5V, OUTPUT);
    digitalWrite(MODULE_LACUNA_5V, LOW);

    // Needed to prevent clashes with rtc library
    Wire.end();

    // Starting state
    state_transition(INIT);
}

void power_led_animation() {
    // Show power LED sequence
    digitalWrite(BOARD_LED, HIGH);
    delay(1000);
    digitalWrite(BOARD_LED, LOW);
    delay(100);
    for (int x = 1; x <= 10; x++) {
        digitalWrite(BOARD_LED, HIGH);
        delay(20);
        digitalWrite(BOARD_LED, LOW);
        delay(100);
    }
}

void hibernate() {
    uint32_t sleep_duration = 5000;

    // Show hibernation LED sequence
    power_led_animation();

    while (true) {
        if (consume_button_event() > 750) {
            return;
        }

        STM32L0.wdtReset();
        STM32L0.stop(sleep_duration);

        digitalWrite(BOARD_LED, HIGH);
        delay(50);
        digitalWrite(BOARD_LED, LOW);
        delay(25);
        digitalWrite(BOARD_LED, HIGH);
        delay(50);
        digitalWrite(BOARD_LED, LOW);
    }
}

void show_power_state() {
    uint16_t voltage = get_voltage_in_mv(MODULE_SYSTEM_BAN_MON_AN);

    uint16_t charge = 0;
    if (voltage > MODULE_PIRA_UNDERVOLTAGE_THRESHOLD) {
        charge = voltage - MODULE_PIRA_UNDERVOLTAGE_THRESHOLD;
    }

    uint16_t blink_count = round(charge / 100.0) + 1;

#ifdef serial_debug
    serial_debug.print("voltage: ");
    serial_debug.print(voltage);
    serial_debug.print(", charge: ");
    serial_debug.print(charge);
    serial_debug.print(", blink: ");
    serial_debug.println(blink_count);
#endif

    for (int x = 0; x < blink_count; x++) {
        digitalWrite(BOARD_LED, HIGH);
        delay(600);
        digitalWrite(BOARD_LED, LOW);
        delay(400);
    }

    delay(1000);
}
/**
 * @brief Main system loop running the FSM
 *
 */
void loop() {
    uint32_t button_time = consume_button_event();
#ifdef serial_debug
    serial_debug.print("main: fsm(");
    serial_debug.print(decode_state(state_prev));
    serial_debug.print(" > ");
    serial_debug.print(decode_state(state));
    serial_debug.print(", ");
    serial_debug.print(millis());
    serial_debug.print(", button: ");
    serial_debug.print(button_time);
    serial_debug.print(", volt: ");
    serial_debug.print(get_voltage_in_mv(MODULE_SYSTEM_BAN_MON_AN));
    serial_debug.println("mV)");
    serial_debug.flush();
#endif

    uint32_t sleep = 0;  // reset the sleep after loop, set in every state if required
    bool should_delay = false;
    // update prevous state
    state_prev = state;

    // FSM implementaiton for clarity of process loop
    switch (state) {
        case INIT:
            power_led_animation();
            check_i2c();
            state_transition(MODULE_INIT);
            break;

        case MODULE_INIT:
            for (size_t count = 0; count < N_MODULES; count++) {
                modules[count]->initialize();
            }
            state_transition(IDLE);
            break;

        case IDLE:
            if (button_time > 5000) {
                state_transition(HIBERNATION);
                break;
            } else if (button_time > 100) {
                state_transition(SHOW_POWER_STATE);
                break;
            }

            sleep = 60000;  // Check modules at least every minute for needed activity
            for (size_t count = 0; count < N_MODULES; count++) {
                scheduler_result_t res = modules[count]->scheduler();

                if (res.sleep >= 0) {
                    if (res.sleep < sleep) {
                        sleep = res.sleep;
                    }

                    if (res.should_delay) {
                        should_delay = true;
                    }

                    modules[count]->running();
                }
            }
            break;

        case HIBERNATION: {
            hibernate();
            // Reset the system when coming out of hibernation
            // This also prevents old LoRa messages from being processed
            STM32L0.reset();
            return;
        } break;

        case ALWAYS_ON: {
            sleep = 10000;
            uint16_t voltage = get_voltage_in_mv(MODULE_SYSTEM_BAN_MON_AN);
            if (voltage < MODULE_PIRA_UNDERVOLTAGE_THRESHOLD + 150) {
                // Show low power warning
                for (int x = 0; x < 10; x++) {
                    digitalWrite(BOARD_LED, HIGH);
                    delay(50);
                    digitalWrite(BOARD_LED, LOW);
                    delay(50);
                }
            }
        } break;

        case SHOW_POWER_STATE:
            show_power_state();
            state_transition(IDLE);
            break;

        default:
            state = IDLE;
            break;
    }

    STM32L0.wdtReset();

    // TODO should we always sleep (like 100 ms or something?)
    if (sleep > 0) {
        if (should_delay) {
#ifdef serial_debug
            serial_debug.print("delay(");
            serial_debug.print(sleep);
            serial_debug.println(")");
#endif
            delay(sleep);
        } else {
#ifdef serial_debug
            serial_debug.print("deep sleep(");
            serial_debug.print(sleep);
            serial_debug.println(")");
#endif
            STM32L0.stop(sleep);
        }
    }
}
/*** end of file ***/
