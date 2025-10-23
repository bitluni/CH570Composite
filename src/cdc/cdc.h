#pragma once
#include <CH57x_common.h>
#ifdef __cplusplus
extern "C" {
#endif

void initCDC();
void processCDC();
void processCDCData(const uint8_t *p_send_dat, uint16_t send_len);

__INTERRUPT
__HIGH_CODE
void USB_IRQHandler(void);

#ifdef __cplusplus
}
#endif