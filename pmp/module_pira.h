#ifndef MODULE_PIRA_H
#define MODULE_PIRA_H

#include <Arduino.h>
#include <STM32L0.h>

#include "Wire.h"
#include "adc.h"
#include "module.h"
#include "project.h"
#include "project_utils.h"

// Size in B, do not change, comunication protocol between
// Pira and RPi depends on this
#define RX_BUFFER_SIZE (7)

class MODULE_PIRA {
   public:
    // functions
    scheduler_result_t scheduler(void);
    void initialize(void);
    void running(void);

   private:
    struct serial_values_t {
        float stm32_temp;
        uint16_t bridge_volt;
        float bridge_temp;
        float bridge_hum;
        uint16_t bridge_hpa;
    } __attribute__((packed));

    enum state_pira_e {
        IDLE_PIRA,
        WAIT_STATUS_ON,
        WAKEUP,
        WAIT_SHUTDOWN,
        STOP_PIRA,
    };

    /**
     * @brief Variables concerning the state of the program
     * @detail
     *      status_pira_state_machine
     *          It keeps the current state of state machine
     *      state_prev
     *          It keeps the previous state of state machine
     *      state_goto_timeout
     *          It keeps state that should be entered in case of time out.
     *          It is set everytime when we enter state.
     *      pira_elapsed
     *          It keeps how much time in ms pira_elapsed since we entered a state
     *      stateTimeoutDuration
     *          If pira_elapsed is larger than stateTimeoutStart then state timeouted.
     *          It is set everytime when we enter state.
     *      stateTimeoutStart
     *          Set everytime we call pira_state_transition funtion.
     */
    serial_values_t serial_values;
    state_pira_e status_pira_state_machine;
    state_pira_e state_prev;
    state_pira_e state_goto_timeout;
    uint32_t pira_elapsed;
    uint32_t last_active;
    uint32_t stateTimeoutDuration;
    uint32_t stateTimeoutStart;

    // Uart related functions
    void send_status_values(Uart &out, int attempt);
    void pira_state_transition(state_pira_e next);
    bool pira_state_check_timeout(void);
    char* return_state(state_pira_e state);
    void pira_state_machine();
};

#endif /* MODULE_PIRA_H */
/*** end of file ***/
