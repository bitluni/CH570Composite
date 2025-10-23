#include <CH57x_common.h>
#include "font.h"
#include "CompositeVideoTimerPWM.h"

constexpr int textCols = 37;
constexpr int textRows = 25; //34
//296x272
constexpr int xres = 300;
constexpr int yres = textRows * 8;
constexpr int pixelsPerLine = 400;
constexpr int pixelsPerLineHalf = pixelsPerLine >> 1;
constexpr int pixelsSync = 32;
constexpr int pixelsSyncShort = 16;
constexpr int pixelsSyncLong = pixelsPerLineHalf - pixelsSyncShort;
constexpr int pixelsFront = 50;
constexpr int pixelsBack = pixelsPerLine - pixelsSync - pixelsFront - xres;
constexpr int linesTotal = 312;
constexpr int linesBlankFront = 20;
constexpr int linesSync = 8;
//constexpr int linesBlankBack = linesTotal - linesSync - linesBlankFront - yres;

constexpr int levelSync = 0;
constexpr int levelBlack = 4;
//constexpr int levelGrey = 10;
constexpr int levelWhite = 16;

__attribute__((aligned(4))) uint32_t vram[2][pixelsPerLine];

volatile int currentLine = 0;
volatile uint8_t textBuffer[textRows][textCols];
constexpr int rleBufferSize = 4500;
volatile uint8_t rleBuffer[rleBufferSize];

void initVideo()
{
    GPIOA_ModeCfg(GPIO_Pin_7, GPIO_ModeOut_PP_5mA);

    TMR_PWMCycleCfg(16);
    TMR_PWMInit(High_Level, PWM_Times_1);

    for(int r = 0; r < textRows; r++)
        for(int c = 0; c < textCols; c++)
            textBuffer[r][c] = 0;//'0' + (c % 10) - 32;

    TMR_DMACfg(ENABLE, (uint32_t)vram[0], (uint32_t)vram[1] + pixelsPerLine * sizeof(uint32_t), Mode_LOOP); //two lines
    TMR_PWMEnable();
    TMR_Enable();

    TMR_ClearITFlag(TMR_IT_DMA_END);
    TMR_ITCfg(ENABLE, TMR_IT_DMA_END);
    PFIC_EnableIRQ(TMR_IRQn);
}

#include "rick.h"

volatile uint32_t counter = 0;
const char *hex = "0123456789ABCDEF";
__HIGH_CODE
void updateVideo()
{
    /*static int lastFrame = 0;
    if(lastFrame != (counter >> 2))
    {
        lastFrame = counter >> 2;
        int f = lastFrame % 20;
        for(int i = 0; i < rick_offsets[f + 1] - rick_offsets[f] && i < rleBufferSize; i++)
            rleBuffer[i] = rick_data[rick_offsets[f] + i];
    }/**/
    for(int i = 0; i < 8; i++)
        textBuffer[0][12 - i] = hex[((counter >> (i * 4)) & 15)] - 32;
    return;
}

__HIGH_CODE
void processCDCData(const uint8_t *data, uint16_t len)
{
    for(int i = 0; i < len; i++)
        textBuffer[10][i] = data[i] - 32;
}

//using pragmas to prevent GCC to replace loops by memcpy that is not in SRAM
#pragma GCC push_options
#pragma GCC optimize ("no-tree-loop-distribute-patterns")
__HIGH_CODE
void syncShortShort(uint32_t *line)
{
    uint32_t *l = line;
    for(int i = 0; i < pixelsSyncShort; i++)
    {
        l[i] = levelSync;
        l[i + pixelsPerLineHalf] = levelSync;
    }
    for(int i = pixelsSyncShort; i < pixelsPerLineHalf; i++)
    {
        l[i] = levelBlack;
        l[i + pixelsPerLineHalf] = levelBlack;
    }
}

__HIGH_CODE
void syncLongLong(uint32_t *line)
{
    uint32_t *l = line;
    for(int i = 0; i < pixelsSyncLong; i++)
    {
        l[i] = levelSync;
        l[i + pixelsPerLineHalf] = levelSync;
    }
    for(int i = pixelsSyncLong; i < pixelsPerLineHalf; i++)
    {
        l[i] = levelBlack;
        l[i + pixelsPerLineHalf] = levelBlack;
    }
}

__HIGH_CODE
void syncLongShort(uint32_t *line)
{
    uint32_t *l = line;
    for(int i = 0; i < pixelsSyncShort; i++)
    {
        l[i + pixelsPerLineHalf] = levelSync;
    }
    for(int i = pixelsSyncShort; i < pixelsPerLineHalf; i++)
    {
        l[i + pixelsPerLineHalf] = levelBlack;
    }
}

__HIGH_CODE
void firstBlank(uint32_t *line)
{
    uint32_t *l = line;
    for(int i = 0; i < pixelsSync; i++)
        l[i] = levelSync;
    for(int i = 0; i < pixelsSyncShort; i++)
        l[i + pixelsPerLineHalf] = levelBlack;
}

#pragma GCC pop_options

int rleBufferPos = 0;
int rleLength = 0;
int rleColor = 0;

/*********************************************************************
 * @fn      TMR_IRQHandler
 *
 * @brief   TMR�жϺ���
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void TMR_IRQHandler(void) // TMR0
{
    TMR_ClearITFlag(TMR_IT_DMA_END);

    for(int b = 0; b < 2; b++)
    {
        currentLine++;
        if(currentLine == linesTotal)
        {
            currentLine = 0;
            rleBufferPos = 0;
            rleLength = 0;
            counter++;
        }
        uint32_t *line = vram[b ^ 1];
        uint32_t *pixels = &(line[pixelsSync + pixelsFront]);

        if(b == 1)
            while(R16_TMR_DMA_NOW < ((uint32_t)(&vram[0][pixelsPerLine - pixelsBack]) & 0xffff));

        if(currentLine <= 2 || (currentLine == 6) || (currentLine == 7))
            syncShortShort(line);
        else
        if(currentLine <= 4)
            syncLongLong(line);
        else
        if(currentLine == 5)
            syncLongShort(line);
        else
        if(currentLine < 10)
            firstBlank(line);
        else
        if(currentLine >= linesBlankFront + linesSync) //on screen
        {
            int renderLine = currentLine - linesBlankFront - linesSync;
            if(renderLine < textRows * 8)
            {
                /*int x = 0;
                while(x < 8 * textCols)
                {
                    if(rleLength == 0)
                    {
                        if(rleBufferPos == rleBufferSize) break;
                        rleLength = rleBuffer[rleBufferPos++];
                        rleColor = rleBuffer[rleBufferPos++];
                    }
                    while(rleLength)
                    {
                        pixels[x++] = levelBlack + rleColor * 2;
                        rleLength--;
                        if(x == 8 * textCols) break;
                    }
                }/**/


                int r = renderLine >> 3;
                int y = renderLine & 7;
                {
                    for(int x = 0; x < 8 * textCols; x++)
                    {
                        int ch = textBuffer[r][x >> 3];
                        int bit = (x + 0) & 7;
                        if((font8x8[ch][y] >> bit) & 1)
                            pixels[x] = levelWhite;
                        else
                            pixels[x] = levelBlack;// + (x & 7);
                    }
                }/**/
            }
            else
            if(renderLine < textRows * 8 + 2)
                for(int x = 0; x < xres; x++)
                    pixels[x] = levelBlack;
        }
    }
}
