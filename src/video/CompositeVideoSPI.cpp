#include <CH57x_common.h>
#include "CompositeVideoSPI.h"

__attribute__((aligned(4))) uint8_t spiBuff[12] = {0x01,0x05,0x15,0x55,0x00,0x00,0x00,0x00,0x00,0x0,0x00,0x00,};

void initVideo()
{
    //clock
    R16_PIN_ALTERNATE_H = 1 << 11;
    GPIOA_ModeCfg(GPIO_Pin_3, GPIO_ModeOut_PP_5mA);
    //data
    GPIOA_ModeCfg(GPIO_Pin_7, GPIO_ModeOut_PP_5mA);
    R8_SPI_CLOCK_DIV = 2; 
    R8_SPI_CTRL_MOD = RB_SPI_ALL_CLEAR;
    R8_SPI_CTRL_MOD = RB_SPI_MOSI_OE | RB_SPI_SCK_OE;
    R8_SPI_CTRL_CFG = 
        RB_SPI_DMA_LOOP | RB_SPI_BIT_ORDER | 
        RB_SPI_MST_DLY_EN | RB_SPI_AUTO_IF; // test RB_MST_CLK_SEL  
};

volatile bool done = false;
__HIGH_CODE
void updateVideo()
{
    //SPI_MasterDMATrans(spiBuff, 12);
//    R8_SPI_CTRL_MOD &= ~RB_SPI_FIFO_DIR;
    R16_SPI_DMA_BEG = (uint32_t)spiBuff;
    R16_SPI_DMA_END = (uint32_t)(spiBuff + 12);
    R16_SPI_TOTAL_CNT = 12;
    R8_SPI_INT_FLAG = RB_SPI_IF_CNT_END | RB_SPI_IF_DMA_END;
    //enable DMA
    R8_SPI_INTER_EN = RB_SPI_IE_DMA_END;
    R8_SPI_CTRL_CFG |= RB_SPI_DMA_ENABLE;

    R8_SPI_INT_FLAG = 0xff; //clear all flags;
    //SPI_ClearITFlag(SPI_IT_DMA_END);
    //SPI_ITCfg(ENABLE, SPI_IT_DMA_END);
    PFIC_EnableIRQ(SPI_IRQn);
    done = false;
    while(!done);
};

extern "C"
__INTERRUPT
__HIGH_CODE
void SPI_IRQHandler(void)
{
    static volatile int f, p0, p1, c, p;
    f = R8_SPI_INT_FLAG;
    c = R16_SPI_TOTAL_CNT;
    p0 = R16_SPI_DMA_BEG;
    p1 = R16_SPI_DMA_END;
    p = R16_SPI_DMA_NOW;

    R8_SPI_INT_FLAG = RB_SPI_IF_DMA_END;    //clear flags
    done = true;
//    R8_SPI_CTRL_CFG |= RB_SPI_DMA_ENABLE;

//    R8_SPI_CTRL_MOD &= ~RB_SPI_FIFO_DIR;
//    R16_SPI_DMA_BEG = (uint32_t)spiBuff;
//    R16_SPI_DMA_END = (uint32_t)(spiBuff + 12);
//    R16_SPI_TOTAL_CNT = 32;
//    R8_SPI_CTRL_CFG |= RB_SPI_DMA_ENABLE;
//    R8_SPI_INTER_EN |= RB_SPI_IE_DMA_END;
}