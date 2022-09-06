#ifndef MODULE_H
#define MODULE_H

// module includes
#include <Arduino.h>

#include "project_utils.h"
struct scheduler_result_t {
    int32_t sleep;
    bool should_delay;
};

#define DONT_RUN { -1 , false }

// virtual module class
class module {
   public:
    virtual ~module(){};
    virtual scheduler_result_t scheduler(void) { return DONT_RUN; };
    virtual void initialize(void) { return; };
    virtual void running(void){};
};

// Tempalate module class
template <class myModuleType>
class myModule : public module {
   public:
    myModuleType module;
    /**
     * @brief Construct a new Module object
     * @param id - global id of the module
     */
    myModule(void) { }

    /**
     * @brief scheduler object with the purpose of triggering internal functions of the module
     *
     * @return module_flags_e response
     */
    scheduler_result_t scheduler(void) { return module.scheduler(); }

    /**
     * @brief initialize the actions required in this module
     *
     * @return uint8_t
     */
    void initialize(void) { return module.initialize(); }

    /**
     * @brief performs running of the values/sensors
     *
     * @return uint8_t
     */
    void running(void) { module.running(); }
};

#endif /* MODULE_H */
/*** end of file ***/
