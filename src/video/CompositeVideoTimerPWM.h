#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void initVideo();
void updateVideo();
void processCDCData(const uint8_t *data, uint16_t len);

__INTERRUPT
__HIGH_CODE
void TMR_IRQHandler(void);

#ifdef __cplusplus
}
#endif
