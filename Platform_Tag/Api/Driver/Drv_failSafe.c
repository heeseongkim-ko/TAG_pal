

#include <string.h>
#include <stdio.h>

#include "sdk_config.h"
#include "app_error.h"
#include "nrf52840.h"

#include "Func_UART_LOG.h"

#include "Api_failsafe.h"
#include "Api_uwb.h"
#include "Drv_failsafe.h"

static uint8_t failsafe_uwb_FailCount_ui8 = 0u;

uint8_t Drv_failsafe_uwb_failCountUp(void)
{
	++failsafe_uwb_FailCount_ui8;

	return failsafe_uwb_FailCount_ui8;
}

uint8_t Drv_failsafe_uwb_getFailCount(void)
{
	return failsafe_uwb_FailCount_ui8;
}

void Drv_failsafe_uwb_clearFailCount(void)
{
	failsafe_uwb_FailCount_ui8 = 0;
}

