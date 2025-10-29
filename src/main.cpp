#include <CH57x_common.h>
//#include "video/CompositeVideoSPI.h"
#include "video/CompositeVideoTimerPWM.h"
#include "cdc/cdc.h"

__HIGH_CODE
void processCDCData(const uint8_t *data, uint16_t len)
{
    print((const char*)data, true, len);
}

extern "C" 
__HIGH_CODE
int main()
{
    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_100MHz);
//    SetSysClock(CLK_SOURCE_HSE_PLL_100MHz);

    /*sys_safe_access_enable();
    R8_FLASH_CFG = 0X07;
    sys_safe_access_disable();
    sys_safe_access_enable();
    R8_CLK_SYS_CFG = (0x40 | 4);
    sys_safe_access_disable();/**/

    initCDC();
    initVideo();
    while(1)
    {
        updateVideo();
        processCDC();
    }
    return 0;
}
