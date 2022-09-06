#include "project.h"

module *s_PIRA = new myModule<MODULE_PIRA>();  
module *s_LACUNA = new myModule<MODULE_LACUNA>();

#ifdef PMP_v1
// Array of modules to be loaded - project specific
module *modules[] = {s_PIRA, s_LACUNA};
#endif

#ifdef RHINO_v2_4
// Array of modules to be loaded - project specific
module *modules[] = {s_SYSTEM, s_GPS, s_ACCEL};
#endif

#ifdef LION_v2_3
// Array of modules to be loaded - project specific
module *modules[] = {s_SYSTEM, s_GPS, s_ACCEL};
#endif

/*** end of file ***/
