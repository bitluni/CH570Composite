#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void initVideo();
void updateVideo();

void setCursor(int x, int y);
void scroll(int rows, bool clear = false);
void print(const char *text, bool autoScroll = true, uint32_t len = 0xffffffff);

__INTERRUPT
__HIGH_CODE
void TMR_IRQHandler(void);

#ifdef __cplusplus
}
#endif
