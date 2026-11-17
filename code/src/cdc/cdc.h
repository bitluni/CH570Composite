#pragma once
#include <CH57x_common.h>
#ifdef __cplusplus
extern "C" {
#endif

void initCDC();
void processCDC();
void processCDCData(const uint8_t *data, uint16_t len);
void sendCDCData(const uint8_t *data, uint16_t len);
__INTERRUPT
__HIGH_CODE
void USB_IRQHandler(void);

#ifdef __cplusplus
}
#endif