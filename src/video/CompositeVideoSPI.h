#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void initVideo();
void updateVideo();

__INTERRUPT
__HIGH_CODE
void SPI_IRQHandler(void);

#ifdef __cplusplus
}
#endif