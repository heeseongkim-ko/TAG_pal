
#ifndef DRV_FAILSAFE_H
#define DRV_FAILSAFE_H

#ifdef __cplusplus
extern "C" {
#endif

uint8_t Drv_failsafe_uwb_failCountUp(void);

uint8_t Drv_failsafe_uwb_getFailCount(void);

void Drv_failsafe_uwb_clearFailCount(void);

#ifdef __cplusplus
}
#endif

#endif /* DRV_FAILSAFE_H */

