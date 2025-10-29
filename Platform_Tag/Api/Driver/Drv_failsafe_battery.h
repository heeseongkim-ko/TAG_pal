
#ifndef DRV_FAILSAFE_BATTERY_H
#define DRV_FAILSAFE_BATTERY_H

#include <stdint.h>
#include <stdbool.h>
#include "Api_failsafe.h"

void Drv_failsafe_battery_init(void);

bool Drv_failsafe_battery_update_level(uint16_t level, failsafe_information_t *rec);

void Drv_failsafe_battery_time_update(uint32_t time);

#endif // DRV_FAILSAFE_BATTERY_H