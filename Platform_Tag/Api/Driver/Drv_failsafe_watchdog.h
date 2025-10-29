

#ifndef DRV_FAILSAFE_WATCHDOG_H
#define DRV_FAILSAFE_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>
#include "Api_failsafe.h"


void failsafe_watchdog_init(void);

bool failsafe_watchdog_check_reset(failsafe_record_t *rec);

void failsafe_watchdog_channel_feed(void);


#endif // DRV_FAILSAFE_WATCHDOG_H

