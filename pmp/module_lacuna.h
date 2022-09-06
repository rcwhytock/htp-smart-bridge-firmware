#ifndef MODULE_LACUNA_H
#define MODULE_LACUNA_H

#include <Arduino.h>
#include <LibLacuna.h>
#include <STM32L0.h>

#include "module.h"
#include "project.h"
#include "project_utils.h"

class MODULE_LACUNA {
   public:
    // functions
    scheduler_result_t scheduler(void);
    void initialize(void);
    void running(void);

   private:
    void setup_lacuna();
    void receive_lacuna();

    lsLoraWANParams relay_loraWANParams;
    lsLoraSatTxParams SattxParams;
    lsLoraTxParams txParams;
    lsLoraTxParams relayParams;

    static byte networkKeyLacuna[];
    static byte appKeyLacuna[];
    static byte deviceAddressLacuna[];

    bool lacuna_init_done;
};

#endif /* MODULE_LACUNA_H */
/*** end of file ***/
