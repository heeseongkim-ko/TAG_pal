
#ifndef API_BATTERY_H
#define API_BATTERY_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
	API_BATTERY_STATE_IDLE = 0,		/**< Ready for new measurement */
	API_BATTERY_STATE_BUSY,			/**< ADC conversion in progress */
	API_BATTERY_STATE_READY,		/**< Result ready */
	API_BATTERY_STATE_ERROR			/**< Error occurred */
} Api_battery_state_g;
#define API_BATTERY_INVALID_VOLTAGE	0xFFFF
#define API_BATTERY_SAADC_FULL_SCALE_MV		2400
#define API_BATTERY_SAADC_MAX_VALUE			256
#define API_BATTERY_VOLTAGE_DIVIDER_RATIO	2
#define API_BATTERY_MAX_VOLTAGE_MV			5000
#define API_BATTERY_CLAMP_VOLTAGE_MV		5000
bool Api_battery_init(void);
bool Api_battery_unInit(void);
void Api_battery_main(void);
void Api_battery_timer_tick(void);
uint16_t Api_battery_getLevel(void);
bool Api_battery_startRead(void);
Api_battery_state_g Api_battery_state(void);

uint16_t Api_battery_getRawData(void);

#ifdef __cplusplus
}
#endif


#endif /* API_BATTERY_H */

