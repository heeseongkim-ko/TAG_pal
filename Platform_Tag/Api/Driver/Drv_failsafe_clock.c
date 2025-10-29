#include "Drv_failsafe_clock.h"
#include "nrf_drv_clock.h"
#include "nrf_clock.h"
#include "nrf_delay.h"
#include "Func_UART_LOG.h"

#define CLOCK_START_TIMEOUT_MS	10	// Clock start timeout

/**
 * @brief Check HFCLK startup with timeout
 * 
 * @details
 * Waits for HFCLK to start running with timeout protection.
 * This function should be called after nrf_drv_clock_hfclk_request().
 * 
 * Sequence:
 * 1. Check HFCLK running status every 1ms
 * 2. Return true if HFCLK started within timeout
 * 3. Return false if timeout expired
 * 
 * @return true if HFCLK started successfully
 * @return false if timeout occurred
 * 
 * @note This function blocks for up to CLOCK_START_TIMEOUT_MS milliseconds
 * @warning Do not call this before requesting HFCLK start
 */
bool Drv_failsafe_clock_check_hfclk_start(void)
{
	uint32_t timeout_ms_ui32 = CLOCK_START_TIMEOUT_MS;
	
	while (!nrf_drv_clock_hfclk_is_running() && (timeout_ms_ui32 > 0))
	{
		nrf_delay_ms(1);
		timeout_ms_ui32--;
	}
	
	return nrf_drv_clock_hfclk_is_running();
}

/**
 * @brief Check LFCLK startup with timeout
 * 
 * @details
 * Waits for LFCLK to start running with timeout protection.
 * This function should be called after nrf_drv_clock_lfclk_request().
 * 
 * Sequence:
 * 1. Check LFCLK running status every 1ms
 * 2. Return true if LFCLK started within timeout
 * 3. Return false if timeout expired
 * 
 * @return true if LFCLK started successfully
 * @return false if timeout occurred
 * 
 * @note This function blocks for up to CLOCK_START_TIMEOUT_MS milliseconds
 * @warning Do not call this before requesting LFCLK start
 */
bool Drv_failsafe_clock_check_lfclk_start(void)
{
	uint32_t timeout_ms_ui32 = CLOCK_START_TIMEOUT_MS;
	
	while (!nrf_clock_lf_is_running() && (timeout_ms_ui32 > 0))
	{
		nrf_delay_ms(1);
		timeout_ms_ui32--;
	}
	
	return nrf_clock_lf_is_running();
}

/**
 * @brief Recover system clocks (HFCLK + LFCLK)
 * 
 * @details
 * Attempts to recover both HFCLK and LFCLK by stopping and restarting them.
 * 
 * Recovery sequence:
 * 1. Release (stop) both HFCLK and LFCLK
 * 2. Wait 10ms for clocks to stabilize
 * 3. Request (start) LFCLK and check startup with timeout
 * 4. Request (start) HFCLK and check startup with timeout
 * 5. Report success if both clocks started successfully
 * 6. Report failure if either clock failed to start
 * 
 * Result reporting:
 * - Success: Api_failsafe_set_success(FAILSAFE_CODE_16)
 * - Failure: Api_failsafe_set_fail(FAILSAFE_CODE_16)
 * 
 * @param code Failsafe code (must be FAILSAFE_CODE_16)
 * @param retry_count Current retry attempt count (1-5)
 * 
 * @note This function will be called from Api_failsafe_main() recovery loop
 * @warning This function may take up to 300ms to complete
 */
void Drv_failsafe_clock_recover(FAILSAFE_CODE code, uint8_t retry_count)
{
	bool hfclk_ok_b;
	bool lfclk_ok_b;
	
	LOG_DRV_FAIL("[FAIL_CLK] Recovery attempt %d/%d\r\n", 
				retry_count, FAILSAFE_MAX_RETRY_COUNT);
	
	// Stop clocks
	LOG_DRV_FAIL("[FAIL_CLK] Stopping clocks...\r\n");
	nrf_drv_clock_hfclk_release();
	nrf_drv_clock_lfclk_release();
	nrf_delay_ms(10);
	
	// Restart LFCLK
	LOG_DRV_FAIL("[FAIL_CLK] Restarting LFCLK...\r\n");
	nrf_drv_clock_lfclk_request(NULL);
	lfclk_ok_b = Drv_failsafe_clock_check_lfclk_start();
	
	if (lfclk_ok_b)
	{
		LOG_DRV_FAIL("[FAIL_CLK] LFCLK started successfully\r\n");
	}
	else
	{
		LOG_DRV_FAIL("[FAIL_CLK] LFCLK start timeout!\r\n");
	}
	
	// Restart HFCLK
	LOG_DRV_FAIL("[FAIL_CLK] Restarting HFCLK...\r\n");
	nrf_drv_clock_hfclk_request(NULL);
	nrf_delay_ms(100);	// Same delay as in clock_config()
	hfclk_ok_b = Drv_failsafe_clock_check_hfclk_start();
	
	if (hfclk_ok_b)
	{
		LOG_DRV_FAIL("[FAIL_CLK] HFCLK started successfully\r\n");
	}
	else
	{
		LOG_DRV_FAIL("[FAIL_CLK] HFCLK start timeout!\r\n");
	}
	
	// Report result
	if (hfclk_ok_b && lfclk_ok_b)
	{
		LOG_DRV_FAIL("[FAIL_CLK] Clock recovery SUCCESS\r\n");
		Api_failsafe_set_success(FAILSAFE_CODE_16);
	}
	else
	{
		LOG_DRV_FAIL("[FAIL_CLK] Clock recovery FAILED (HFCLK=%d, LFCLK=%d)\r\n",
					hfclk_ok_b, lfclk_ok_b);
		Api_failsafe_set_fail(FAILSAFE_CODE_16);
	}
}

