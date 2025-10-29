

#include "nrf_delay.h"
#include "Func_UART_LOG.h"

/** Magic value stored in Flash for integrity check */
#define API_FAILSAFE_MAGIC_VALUE	0x54415647u  // "TAVG"

/** RAM test pattern */
#define API_FAILSAFE_RAM_TEST_PATTERN	0xDEADBEEFu

volatile const uint32_t Api_failsafe_magic_ui32 = API_FAILSAFE_MAGIC_VALUE;

bool Drv_failsafe_memory_check_flash(void)
{
	// Force Flash READ using volatile pointer to prevent compiler optimization
	volatile uint32_t read_value_ui32 = *(volatile uint32_t*)&Api_failsafe_magic_ui32;

	// Verify magic value
	if (read_value_ui32 != API_FAILSAFE_MAGIC_VALUE)
	{
		LOG_API_FAIL("[FAIL] INTERNAL_MEM: Flash check failed (read=0x%08X, expected=0x%08X)\r\n",
					read_value_ui32, API_FAILSAFE_MAGIC_VALUE);
		return false;
	}

	LOG_API_FAIL("[FAIL] INTERNAL_MEM: Flash check OK\r\n");
	return true;
}

bool Drv_failsafe_memory_check_ram(void)
{
	volatile uint32_t test_value_ui32 = API_FAILSAFE_RAM_TEST_PATTERN;

	// Verify written value
	if (test_value_ui32 != API_FAILSAFE_RAM_TEST_PATTERN)
	{
		LOG_API_FAIL("[FAIL] INTERNAL_MEM: RAM check failed (read=0x%08X, expected=0x%08X)\r\n",
					test_value_ui32, API_FAILSAFE_RAM_TEST_PATTERN);
		return false;
	}

	LOG_API_FAIL("[FAIL] INTERNAL_MEM: RAM check OK\r\n");
	return true;
}

