#include <CH57x_common.h>
//#include "video/CompositeVideoSPI.h"
#include "video/CompositeVideoTimerPWM.h"
#include "cdc/cdc.h"
volatile uint8_t textPos = 0;
volatile uint32_t textCount = 0;
char text[256];

__HIGH_CODE
void processCDCData(const uint8_t *data, uint16_t len)
{
    for(int i = 0; i < len; i++)
        if(textCount < 256)
        {
            text[(textPos + textCount) & 255] = data[i];
            textCount++;
        }
}

#define CLK_SOURCE_HSE_PLL_150MHz (0x40 | 4)

__HIGH_CODE
void SetSysClock150MHz()
{
    sys_safe_access_enable();
    R8_FLASH_CFG = 0X07;
    sys_safe_access_disable();
    sys_safe_access_enable();
    R8_CLK_SYS_CFG = (0x40 | 4);
    sys_safe_access_disable();
}

__HIGH_CODE
void loop()
{
    while(1)
    {
        updateVideo();
        processCDC();
        if(textCount)
        {
            textCount--;
            print(&text[textPos++], true, 1);
        }
    }
}

int main()
{
    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_100MHz);

    initCDC();
    initVideo();
/*   
    print("\n\nPRIORITY ONE\n");
    print("INSURE RETURN OF ORGANISM\n");
    print("FOR ANALYSIS.\n");
    print("ALL OTHER CONSIDERATIONS SECONDARY.\n");
    print("CREW EXPENDABLE.\n"); */
    loop();
    return 0;
}
