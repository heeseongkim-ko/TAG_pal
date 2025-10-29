
#ifndef DRV_FAILSAFE_MEMORY_H
#define DRV_FAILSAFE_MEMORY_H

#include <stdint.h>
#include <stdbool.h>
#include "Api_failsafe.h"

bool Drv_failsafe_memory_check_flash(void);

bool Drv_failsafe_memory_check_ram(void);

#endif // DRV_FAILSAFE_MEMORY_H