#include "cdc.h"

/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.c
* Author             : WCH
* Version            : V1.0
* Date               : 2020/08/06
* Description        : 
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include "CH57x_common.h"

#define THIS_ENDP0_SIZE         64
#define MAX_PACKET_SIZE         64

#define USB_CDC_MODE      1
#define USB_VENDOR_MODE   0

UINT8 usb_cdc_mode_trans_en = 0;   //cdcģʽ�������ϴ�ʹ��
UINT8 usb_ven_mode_trans_en = 0;   //����ģʽ�������ϴ�ʹ��

const UINT8 TAB_USB_CDC_DEV_DES[18] =
{
	0x12,
	0x01,
	0x10,
	0x01,
	0x02,
	0x00,
	0x00,
	0x40,
	0x86, 0x1a,
	0x40, 0x80,
	0x00, 0x30,
	0x01,
	0x02,
	0x03,
	0x01
};


/* ���������� */
const UINT8 TAB_USB_CDC_CFG_DES[ ] =
{
	0x09,0x02,0x43,0x00,0x02,0x01,0x00,0x80,0x30,

	0x09, 0x04,0x00,0x00,0x01,0x02,0x02,0x01,0x00,

	0x05,0x24,0x00,0x10,0x01,
	0x04,0x24,0x02,0x02,
	0x05,0x24,0x06,0x00,0x01,
	0x05,0x24,0x01,0x01,0x00,

	0x07,0x05,0x84,0x03,0x08,0x00,0x01,
	0x09,0x04,0x01,0x00,0x02,0x0a,0x00,0x00,0x00,

	0x07,0x05,0x01,0x02,0x40,0x00,0x00,
	0x07,0x05,0x81,0x02,0x40,0x00,0x00,
};

const UINT8 My_QueDescr[ ] = { 0x0A, 0x06, 0x00, 0x02, 0xFF, 0x00, 0xFF, 0x40, 0x01, 0x00 };

UINT8 TAB_CDC_LINE_CODING[ ]  =
{
	0x85, /* baud rate*/
	0x20,
	0x00,
	0x00,
	0x00,   /* stop bits-1*/
	0x00,   /* parity - none*/
	0x08    /* no. of bits 8*/
};

const UINT8 TAB_USB_LID_STR_DES[ ] = { 0x04, 0x03, 0x09, 0x04 };

const UINT8 USB_DEV_PARA_CDC_SERIAL_STR[] =     "videoshnipsel";
const UINT8 USB_DEV_PARA_CDC_PRODUCT_STR[] =    "bitluni's CDC";
const UINT8 USB_DEV_PARA_CDC_MANUFACTURE_STR[] = "bituni";

typedef struct DevInfo
{
	UINT8 UsbConfig;
	UINT8 UsbAddress;
	UINT8 gSetupReq;
	UINT8 gSetupLen;
	UINT8 gUsbInterCfg;
	UINT8 gUsbFlag;
}DevInfo_Parm;

DevInfo_Parm  devinf;
UINT8 SetupReqCode, SetupLen;

__aligned(4) UINT8  Ep0Buffer[MAX_PACKET_SIZE];
__aligned(4) UINT8  Ep1Buffer[MAX_PACKET_SIZE];     //IN
__aligned(4) UINT8  Ep2Buffer[2*MAX_PACKET_SIZE];   //OUT & IN
__aligned(4) UINT8  Ep3Buffer[2*MAX_PACKET_SIZE];   //OUT & IN

typedef struct __PACKED _LINE_CODE
{
	UINT32  BaudRate;
	UINT8 StopBits;
	UINT8 ParityType;
	UINT8 DataBits;
}LINE_CODE, *PLINE_CODE;

LINE_CODE Uart0Para;

#define CH341_REG_NUM     10
UINT8 CH341_Reg_Add[CH341_REG_NUM];
UINT8 CH341_Reg_val[CH341_REG_NUM];


UINT8 VENSer0ParaChange = 0;
UINT8 VENSer0SendFlag = 0;

UINT8 UART0_RTS_Val = 0; //��� ��ʾDTE����DCE��������
UINT8 UART0_DTR_Val = 0; //��� �����ն˾���
UINT8 UART0_OUT_Val = 0; //�Զ���modem�źţ�CH340�ֲᣩ

UINT8 UART0_DCD_Val = 0;
UINT8 UART0_DCD_Change = 0;

UINT8 UART0_RI_Val = 0;
UINT8 UART0_RI_Change = 0;

UINT8 UART0_DSR_Val = 0;
UINT8 UART0_DSR_Change = 0;

UINT8 UART0_CTS_Val = 0;
UINT8 UART0_CTS_Change = 0;

/* CDC���õĴ��� */
UINT8 CDCSetSerIdx = 0;
UINT8 CDCSer0ParaChange = 0;

typedef struct _USB_SETUP_REQ_ {
		UINT8 bRequestType;
		UINT8 bRequest;
		UINT8 wValueL;
		UINT8 wValueH;
		UINT8 wIndexL;
		UINT8 wIndexH;
		UINT8 wLengthL;
		UINT8 wLengthH;
} USB_SETUP_REQ_t;

#define UsbSetupBuf     ((USB_SETUP_REQ_t *)Ep0Buffer) //USB_SETUP_REQ_t USB_SETUP_REQ_t

/* USB���涨�壬���ж˵�ȫ������ */
/* �˵�1 -- IN״̬ */
UINT8 Ep1DataINFlag = 0;

/* �˵�1�´����� */
UINT8 Ep1DataOUTFlag = 0;
UINT8 Ep1DataOUTLen = 0;
__aligned(4) UINT8 Ep1OUTDataBuf[MAX_PACKET_SIZE];

/* �˵�2 -- IN״̬ */
UINT8 Ep2DataINFlag = 0;

/* �˵�2�´����� */
UINT8 Ep2DataOUTFlag = 0;
UINT8 Ep2DataOUTLen = 0;
__aligned(4) UINT8 Ep2OUTDataBuf[MAX_PACKET_SIZE];

/* ����USB�жϵ�״̬ ->�ĳɼ���Ĳ�����ʽ */
#define USB_IRQ_FLAG_NUM     4

UINT8 usb_irq_w_idx = 0;
UINT8 usb_irq_r_idx = 0;

volatile UINT8 usb_irq_len[USB_IRQ_FLAG_NUM];
volatile UINT8 usb_irq_pid[USB_IRQ_FLAG_NUM];
volatile UINT8 usb_irq_flag[USB_IRQ_FLAG_NUM];

UINT8 cdc_uart_sta_trans_step = 0;
UINT8 ven_ep1_trans_step = 0;

/* �˵�0ö���ϴ�֡���� */
UINT8 ep0_send_buf[256];

/**********************************************************/
UINT8 DevConfig;
UINT16 SetupReqLen;
const UINT8 *pDescr;


/* Character Size */
#define HAL_UART_5_BITS_PER_CHAR             5
#define HAL_UART_6_BITS_PER_CHAR             6
#define HAL_UART_7_BITS_PER_CHAR             7
#define HAL_UART_8_BITS_PER_CHAR             8

/* Stop Bits */
#define HAL_UART_ONE_STOP_BIT                1
#define HAL_UART_TWO_STOP_BITS               2

/* Parity settings */
#define HAL_UART_NO_PARITY                   0x00                              //��У��
#define HAL_UART_ODD_PARITY                  0x01                              //��У��
#define HAL_UART_EVEN_PARITY                 0x02                              //żУ��
#define HAL_UART_MARK_PARITY                 0x03                              //��1 mark
#define HAL_UART_SPACE_PARITY                0x04                              //�հ�λ space


/* �˵�״̬���ú��� */
void USBDevEPnINSetStatus(UINT8 ep_num, UINT8 type, UINT8 sta);

/* endpoints enumeration */
#define ENDP0                           0x00
#define ENDP1                           0x01
#define ENDP2                           0x02
#define ENDP3                           0x03
#define ENDP4                           0x04

/* ENDP x Type */
#define ENDP_TYPE_IN                    0x00                                    /* ENDP is IN Type */
#define ENDP_TYPE_OUT                   0x01                                    /* ENDP is OUT Type */

/* �˵�Ӧ��״̬���� */
/* OUT */
#define OUT_ACK                         0
#define OUT_TIMOUT                      1
#define OUT_NAK                         2
#define OUT_STALL                       3
/* IN */
#define IN_ACK                          0
#define IN_NORSP                        1
#define IN_NAK                          2
#define IN_STALL                        3

/* USB�豸���ֱ�־λ���� */
#define DEF_BIT_USB_RESET               0x01                                    /* ���߸�λ��־ */
#define DEF_BIT_USB_DEV_DESC            0x02                                    /* ��ȡ���豸��������־ */
#define DEF_BIT_USB_ADDRESS             0x04                                    /* ���ù���ַ��־ */
#define DEF_BIT_USB_CFG_DESC            0x08                                    /* ��ȡ��������������־ */
#define DEF_BIT_USB_SET_CFG             0x10                                    /* ���ù�����ֵ��־ */
#define DEF_BIT_USB_WAKE                0x20                                    /* USB���ѱ�־ */
#define DEF_BIT_USB_SUPD                0x40                                    /* USB���߹����־ */
#define DEF_BIT_USB_HS                  0x80                                    /* USB���١�ȫ�ٱ�־ */

/* �жϴ������� */
__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void USB_IRQHandler(void)
{
	UINT8   j;

	if(R8_USB_INT_FG & RB_UIF_TRANSFER)
	{
		/* ��setup������ */
		if((R8_USB_INT_ST & MASK_UIS_TOKEN) != MASK_UIS_TOKEN){     // �ǿ���
			/* ֱ��д�� */
			usb_irq_flag[usb_irq_w_idx] = 1;
			usb_irq_pid[usb_irq_w_idx]  = R8_USB_INT_ST;  //& 0x3f;//(0x30 | 0x0F);
			usb_irq_len[usb_irq_w_idx]  = R8_USB_RX_LEN;

			switch(usb_irq_pid[usb_irq_w_idx]& 0x3f){   //������ǰ�Ķ˵�
				case UIS_TOKEN_OUT | 2:{
					if( R8_USB_INT_FG & RB_U_TOG_OK ){   //��ͬ�������ݰ�������
						R8_UEP2_CTRL ^=  RB_UEP_R_TOG;
						R8_UEP2_CTRL = (R8_UEP2_CTRL & 0xf3) | 0x08; //OUT_NAK
						/* �������� */
						for(j=0; j<(MAX_PACKET_SIZE/4); j++)
							((UINT32 *)Ep2OUTDataBuf)[j] = ((UINT32 *)Ep2Buffer)[j];
					}
					else usb_irq_flag[usb_irq_w_idx] = 0;
				}break;
				case UIS_TOKEN_IN | 2:{ //endpoint 2# �����˵��ϴ����
					R8_UEP2_CTRL ^=  RB_UEP_T_TOG;
					R8_UEP2_CTRL = (R8_UEP2_CTRL & 0xfc) | IN_NAK; //IN_NAK
				}break;
				case UIS_TOKEN_OUT | 1:{
					if( R8_USB_INT_FG & RB_U_TOG_OK ){   //��ͬ�������ݰ�������
						R8_UEP1_CTRL ^=  RB_UEP_R_TOG;
						R8_UEP1_CTRL = (R8_UEP1_CTRL & 0xf3) | 0x08; //OUT_NAK
						/* �������� */
						for(j=0; j<(MAX_PACKET_SIZE/4); j++)
							((UINT32 *)Ep1OUTDataBuf)[j] = ((UINT32 *)Ep1Buffer)[j];
					}
					else usb_irq_flag[usb_irq_w_idx] = 0;
				}break;
				case UIS_TOKEN_IN | 1:{  //endpoint 1# �����˵��ϴ����
					R8_UEP1_CTRL ^=  RB_UEP_T_TOG;
					R8_UEP1_CTRL = (R8_UEP1_CTRL & 0xfc) | IN_NAK; //IN_NAK
				}break;
				case UIS_TOKEN_OUT | 0:{    // endpoint 0
					if( R8_USB_INT_FG & RB_U_TOG_OK )   //��ͬ�������ݰ�������
						R8_UEP0_CTRL = (R8_UEP0_CTRL & 0xf3) | 0x08; //OUT_NAK
					else usb_irq_flag[usb_irq_w_idx] = 0;
				}break;
				case UIS_TOKEN_IN | 0:{  //endpoint 0
					R8_UEP0_CTRL = (R8_UEP0_CTRL & 0xfc) | IN_NAK; //IN_NAK
				}break;
			}

			/* �е���һ��д��ַ */
			if(usb_irq_flag[usb_irq_w_idx]){
				usb_irq_w_idx++;
				if(usb_irq_w_idx >= USB_IRQ_FLAG_NUM) usb_irq_w_idx = 0;
			}

			R8_USB_INT_FG = RB_UIF_TRANSFER;
		}

		/* setup������ */
		if(R8_USB_INT_ST & RB_UIS_SETUP_ACT){
			/* ��֮ǰ�Ĵ����ӹ� -- UIS_TOKEN_SETUP */
			/* ֱ��д�� */
			usb_irq_flag[usb_irq_w_idx] = 1;
			usb_irq_pid[usb_irq_w_idx]  = UIS_TOKEN_SETUP | 0;  //����֮ǰ�ı�־
			usb_irq_len[usb_irq_w_idx]  = 8;
			/* �е���һ��д��ַ */
			usb_irq_w_idx++;
			if(usb_irq_w_idx >= USB_IRQ_FLAG_NUM) usb_irq_w_idx = 0;

			R8_USB_INT_FG = RB_UIF_TRANSFER;
		}
	}
}

UINT8 Ep4DataINFlag;
UINT8 Ep3DataINFlag;

//UINT8 Ep1DataOUTLen ;
//UINT8 Ep1DataOUTFlag ;

UINT8 Ep3DataOUTFlag = 0;
UINT8 Ep4DataOUTFlag = 0;

/* CH341��ص�����֡ */
#define DEF_VEN_DEBUG_READ              0X95
#define DEF_VEN_DEBUG_WRITE             0X9A
#define DEF_VEN_UART_INIT             0XA1
#define DEF_VEN_UART_M_OUT              0XA4
#define DEF_VEN_BUF_CLEAR             0XB2
#define DEF_VEN_I2C_CMD_X             0X54
#define DEF_VEN_DELAY_MS              0X5E
#define DEF_VEN_GET_VER                 0X5F

//  3.1 Requests---Abstract Control Model
#define DEF_SEND_ENCAPSULATED_COMMAND  0x00
#define DEF_GET_ENCAPSULATED_RESPONSE  0x01
#define DEF_SET_COMM_FEATURE           0x02
#define DEF_GET_COMM_FEATURE           0x03
#define DEF_CLEAR_COMM_FEATURE         0x04
#define DEF_SET_LINE_CODING          0x20   // Configures DTE rate, stop-bits, parity, and number-of-character
#define DEF_GET_LINE_CODING          0x21   // This request allows the host to find out the currently configured line coding.
//#define DEF_SET_CTL_LINE_STE         0X22   // This request generates RS-232/V.24 style control signals.
#define DEF_SET_CONTROL_LINE_STATE     0x22
#define DEF_SEND_BREAK                 0x23

//  3.2 Notifications---Abstract Control Model
#define DEF_NETWORK_CONNECTION         0x00
#define DEF_RESPONSE_AVAILABLE         0x01
#define DEF_SERIAL_STATE               0x20

__HIGH_CODE
void USB_IRQProcessHandler( void )
{
	static  PUINT8  pDescr;
	UINT8 len;
	UINT8   data_dir = 0;
	UINT8   i;

	//for(i=0; i<USB_IRQ_FLAG_NUM; i++)
	{
		i = usb_irq_r_idx;

		if(usb_irq_flag[i])
		{
			usb_irq_r_idx++;
			if(usb_irq_r_idx >= USB_IRQ_FLAG_NUM) usb_irq_r_idx = 0;

			switch ( usb_irq_pid[i] & 0x3f )
			{
				case UIS_TOKEN_IN | 4:  //endpoint 4#
				{
					Ep4DataINFlag = ~0;
					break;
				}
				case UIS_TOKEN_IN | 3:  //endpoint 3#
				{
					Ep3DataINFlag = ~0;
					break;
				}
				case UIS_TOKEN_OUT | 2:    // endpoint 2#
				{
					len = usb_irq_len[i];
					{
						//TODO
						//SendUSBData(Ep2OUTDataBuf, len);  //probably vendor lol idc
						sendCDCData(Ep1OUTDataBuf, len);

						//CH341�������·�
						Ep2DataOUTFlag = 1;
						Ep2DataOUTLen = len;
						VENSer0SendFlag = 1;
						PFIC_DisableIRQ(USB_IRQn);
						R8_UEP2_CTRL = R8_UEP2_CTRL & 0xf3; //OUT_ACK
						PFIC_EnableIRQ(USB_IRQn);
					}
					break;
				}
				case UIS_TOKEN_IN | 2:  //endpoint 2#
				{
					Ep2DataINFlag = 1;
					break;
				}
				case UIS_TOKEN_OUT | 1:    // endpoint 1#
				{
					len = usb_irq_len[i];
					processCDCData(Ep1OUTDataBuf, len);
					sendCDCData(Ep1OUTDataBuf, len);
					//SendUSBData(Ep1OUTDataBuf, len); //TODO this is CDC recv

					//CH341�������·�
					Ep1DataOUTFlag = 1;
					Ep1DataOUTLen = len;
					VENSer0SendFlag = 1;
					PFIC_DisableIRQ(USB_IRQn);
					R8_UEP1_CTRL = R8_UEP1_CTRL & 0xf3; //OUT_ACK
					PFIC_EnableIRQ(USB_IRQn);
					break;
				}
				case UIS_TOKEN_IN | 1:   // endpoint 1#
				{
					Ep1DataINFlag = 1;
					break;
				}
				case UIS_TOKEN_SETUP | 0:    // endpoint 0# SETUP
				{
					len = usb_irq_len[i];
					if(len == sizeof(USB_SETUP_REQ))
					{
						SetupLen = UsbSetupBuf->wLengthL;
						if(UsbSetupBuf->wLengthH) SetupLen = 0xff;

						len = 0;
						SetupReqCode = UsbSetupBuf->bRequest;

						data_dir = USB_REQ_TYP_OUT;
						if(UsbSetupBuf->bRequestType & USB_REQ_TYP_IN) data_dir = USB_REQ_TYP_IN;

						if((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) == USB_REQ_TYP_STANDARD)
						{
							switch( SetupReqCode )
							{
								case USB_GET_DESCRIPTOR:
								{
									switch( UsbSetupBuf->wValueH )
									{
										case 1:
										{
											memcpy(ep0_send_buf,
														&TAB_USB_CDC_DEV_DES[0],
														sizeof( TAB_USB_CDC_DEV_DES ));

											pDescr = ep0_send_buf;
											len = sizeof( TAB_USB_CDC_DEV_DES );
											break;
										}
										case 2:
										{
											memcpy(ep0_send_buf,
														&TAB_USB_CDC_CFG_DES[0],
														sizeof( TAB_USB_CDC_CFG_DES ));
											pDescr = ep0_send_buf;
											len = sizeof( TAB_USB_CDC_CFG_DES );
											break;
										}
										case 3:
										{
											switch(UsbSetupBuf->wValueL)
											{
												case 0:  //����������
												{
													pDescr = (PUINT8)( &TAB_USB_LID_STR_DES[0] );
													len = sizeof( TAB_USB_LID_STR_DES );

													break;
												}
												case 1:  //iManufacturer
												case 2:   //iProduct
												case 3:   //iSerialNumber
												{
													UINT8 ep0_str_len;
													UINT8 *p_send;
													UINT8 *manu_str;
													UINT8 tmp;

													/* ȡ���� */
													if(UsbSetupBuf->wValueL == 1)
														manu_str = (UINT8 *)USB_DEV_PARA_CDC_MANUFACTURE_STR;
													else if(UsbSetupBuf->wValueL == 2)
														manu_str = (UINT8 *)USB_DEV_PARA_CDC_PRODUCT_STR;
													else if(UsbSetupBuf->wValueL == 3)
														manu_str = (UINT8 *)USB_DEV_PARA_CDC_SERIAL_STR;
													ep0_str_len = (UINT8)strlen((char *)manu_str);
													p_send = ep0_send_buf;
													*p_send++ = ep0_str_len*2 + 2;
													*p_send++ = 0x03;
													for(tmp = 0; tmp<ep0_str_len; tmp++)
													{
														*p_send++ = manu_str[tmp];
														*p_send++ = 0x00;
													}

													pDescr = ep0_send_buf;
													len = ep0_send_buf[0];

													break;
												}
												default:
													len = 0xFF;    // ��֧�ֵ�����������
													break;
											}
											break;
										}
										case 6:
										{
											pDescr = (PUINT8)( &My_QueDescr[0] );
											len = sizeof( My_QueDescr );
											break;
										}
										default:
											len = 0xFF;
											break;
									}
									if ( SetupLen > len ) SetupLen = len;
									len = (SetupLen >= THIS_ENDP0_SIZE) ? THIS_ENDP0_SIZE : SetupLen;
									memcpy( Ep0Buffer, pDescr, len );
									SetupLen -= len;
									pDescr += len;

									break;
								}
								case USB_SET_ADDRESS:
								{
									devinf.gUsbFlag |= DEF_BIT_USB_ADDRESS;
									devinf.UsbAddress = UsbSetupBuf->wValueL;

									break;
								}
								case USB_GET_CONFIGURATION:
								{
									Ep0Buffer[0] = devinf.UsbConfig;
									if ( SetupLen >= 1 ) len = 1;

									break;
								}
								case USB_SET_CONFIGURATION:
								{
									devinf.gUsbFlag |= DEF_BIT_USB_SET_CFG;
									devinf.UsbConfig = UsbSetupBuf->wValueL;
									break;
								}
								case USB_CLEAR_FEATURE:
								{
									len = 0;
									if( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
									{
										R8_UEP1_CTRL = (R8_UEP1_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK;
										R8_UEP2_CTRL = (R8_UEP2_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK;
										R8_UEP3_CTRL = (R8_UEP3_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK;
										R8_UEP4_CTRL = (R8_UEP4_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK;

										Ep1DataINFlag = 1;
										Ep2DataINFlag = 1;
										Ep3DataINFlag = 1;
										Ep4DataINFlag = 1;

										Ep1DataOUTFlag = 0;
										Ep2DataOUTFlag = 0;
										Ep3DataOUTFlag = 0;
										Ep4DataOUTFlag = 0;

										cdc_uart_sta_trans_step = 0;
										ven_ep1_trans_step = 0;
									}
									else if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
									{
										switch( UsbSetupBuf->wIndexL )
										{
											case 0x84: R8_UEP4_CTRL = (R8_UEP4_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK; break;
											case 0x04: R8_UEP4_CTRL = (R8_UEP4_CTRL & (~ ( RB_UEP_R_TOG | MASK_UEP_R_RES ))) | UEP_R_RES_ACK; break;
											case 0x83: R8_UEP3_CTRL = (R8_UEP3_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK; break;
											case 0x03: R8_UEP3_CTRL = (R8_UEP3_CTRL & (~ ( RB_UEP_R_TOG | MASK_UEP_R_RES ))) | UEP_R_RES_ACK; break;
											case 0x82: R8_UEP2_CTRL = (R8_UEP2_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK; break;
											case 0x02: R8_UEP2_CTRL = (R8_UEP2_CTRL & (~ ( RB_UEP_R_TOG | MASK_UEP_R_RES ))) | UEP_R_RES_ACK; break;
											case 0x81: R8_UEP1_CTRL = (R8_UEP1_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK; break;
											case 0x01: R8_UEP1_CTRL = (R8_UEP1_CTRL & (~ ( RB_UEP_R_TOG | MASK_UEP_R_RES ))) | UEP_R_RES_ACK; break;
											default: len = 0xFF;  break;
										}
									}
									else len = 0xFF;

									break;
								}
								case USB_GET_INTERFACE:
								{
									Ep0Buffer[0] = 0x00;
									if ( SetupLen >= 1 ) len = 1;
									break;
								}
								case USB_GET_STATUS:
								{
									Ep0Buffer[0] = 0x00;
									Ep0Buffer[1] = 0x00;
									if ( SetupLen >= 2 ) len = 2;
									else len = SetupLen;
									break;
								}
								default:
									len = 0xFF;
									break;
							}
						}
						else if( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_CLASS )
						{
							if(data_dir == USB_REQ_TYP_OUT)
							{
								switch( SetupReqCode )
								{
									case DEF_SET_LINE_CODING: /* SET_LINE_CODING */
									{
										if( Ep0Buffer[ 4 ] == 0x00 )
										{
											CDCSetSerIdx = 0;
											len = 0x00;
										}
										else if( Ep0Buffer[ 4 ] == 0x02 )
										{
											CDCSetSerIdx = 1;
											len = 0x00;
										}
										else len = 0xFF;
										break;
									}
									case DEF_SET_CONTROL_LINE_STATE:  /* SET_CONTROL_LINE_STATE */
									{
										/*UINT8  carrier_sta;
										UINT8  present_sta;
										carrier_sta = Ep0Buffer[2] & (1<<1);   //RTS״̬
										present_sta = Ep0Buffer[2] & (1<<0);   //DTR״̬
										*/
										len = 0;
										break;
									}
									default:
									{
										len = 0xFF;
										break;
									}
								}
							}
							else
							{
								switch( SetupReqCode )
								{
									case DEF_GET_LINE_CODING: /* GET_LINE_CODING */
									{
										pDescr = Ep0Buffer;
										len = sizeof( LINE_CODE );
										( ( PLINE_CODE )Ep0Buffer )->BaudRate   = Uart0Para.BaudRate;
										( ( PLINE_CODE )Ep0Buffer )->StopBits   = Uart0Para.StopBits;
										( ( PLINE_CODE )Ep0Buffer )->ParityType = Uart0Para.ParityType;
										( ( PLINE_CODE )Ep0Buffer )->DataBits   = Uart0Para.DataBits;
										break;
									}
									case DEF_SERIAL_STATE:
									{
										len = 2;
										CDCSetSerIdx = 0;
										Ep0Buffer[0] = 0;
										Ep0Buffer[1] = 0;
										break;
									}
									default:
									{
										//CDC ReqCode SetupReqCode
										len = 0xFF;
										break;
									}
								}
							}
						}
						else len = 0xFF;
					}
					else
					{
						len = 0xFF;
					}
					if (len == 0xFF)
					{
						SetupReqCode = 0xFF;
						PFIC_DisableIRQ(USB_IRQn);
						R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;  // STALL
						PFIC_EnableIRQ(USB_IRQn);
					}
					else if ( len <= THIS_ENDP0_SIZE )
					{
						if( SetupReqCode ==  USB_SET_ADDRESS)
						{
							PFIC_DisableIRQ(USB_IRQn);
							R8_UEP0_T_LEN = len;
							R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;
							PFIC_EnableIRQ(USB_IRQn);
						}
						else if( SetupReqCode ==  USB_SET_CONFIGURATION)  //0x09
						{
							PFIC_DisableIRQ(USB_IRQn);
							R8_UEP0_T_LEN = len;
							R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;
							PFIC_EnableIRQ(USB_IRQn);
						}
						else if( SetupReqCode ==  USB_GET_DESCRIPTOR)  //0x06
						{
							R8_UEP0_T_LEN = len;
							PFIC_DisableIRQ(USB_IRQn);
							R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;
							PFIC_EnableIRQ(USB_IRQn);
						}
						else if( SetupReqCode ==  DEF_VEN_UART_INIT )  //0XA1
						{
							R8_UEP0_T_LEN = len;
							PFIC_DisableIRQ(USB_IRQn);
							R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;
							PFIC_EnableIRQ(USB_IRQn);
						}
						else if( SetupReqCode ==  DEF_VEN_DEBUG_WRITE )  //0X9A
						{
							R8_UEP0_T_LEN = len;
							PFIC_DisableIRQ(USB_IRQn);
							R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;
							PFIC_EnableIRQ(USB_IRQn);
						}
						else if( SetupReqCode ==  DEF_VEN_UART_M_OUT )  //0XA4
						{
							R8_UEP0_T_LEN = len;
							PFIC_DisableIRQ(USB_IRQn);
							R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;
							PFIC_EnableIRQ(USB_IRQn);
						}
						else if( SetupReqCode ==  DEF_SET_CONTROL_LINE_STATE )  //0x22
						{
							PFIC_DisableIRQ(USB_IRQn);
							R8_UEP0_T_LEN = len;
							R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;
							PFIC_EnableIRQ(USB_IRQn);
						}
						else if( SetupReqCode ==  USB_CLEAR_FEATURE )  //0x01
						{
							PFIC_DisableIRQ(USB_IRQn);
							R8_UEP0_T_LEN = len;
							R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;
							PFIC_EnableIRQ(USB_IRQn);
						}
						else
						{
							if(data_dir == USB_REQ_TYP_IN)
							{
								PFIC_DisableIRQ(USB_IRQn);
								R8_UEP0_T_LEN = len;
								R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;
								PFIC_EnableIRQ(USB_IRQn);
							}
							else
							{
								PFIC_DisableIRQ(USB_IRQn);
								R8_UEP0_T_LEN = len;
								R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
								PFIC_EnableIRQ(USB_IRQn);
							}
						}
					}
					else
					{
						R8_UEP0_T_LEN = 0;
						PFIC_DisableIRQ(USB_IRQn);
						R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;
						PFIC_EnableIRQ(USB_IRQn);
					}
					break;
				}
				case UIS_TOKEN_IN | 0:      // endpoint 0# IN  UIS_TOKEN_IN
				{
					switch( SetupReqCode )
					{
						case USB_GET_DESCRIPTOR:  //0x06
						{
							len = (SetupLen >= THIS_ENDP0_SIZE) ? THIS_ENDP0_SIZE : SetupLen;
							memcpy( Ep0Buffer, pDescr, len );
							SetupLen -= len;
							pDescr += len;

							if(len)
							{
								R8_UEP0_T_LEN = len;
								PFIC_DisableIRQ(USB_IRQn);
								R8_UEP0_CTRL ^=  RB_UEP_T_TOG;
								USBDevEPnINSetStatus(ENDP0, ENDP_TYPE_IN, IN_ACK);
								PFIC_EnableIRQ(USB_IRQn);
							}
							else
							{
								R8_UEP0_T_LEN = len;
								PFIC_DisableIRQ(USB_IRQn);
								R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_ACK | UEP_T_RES_NAK;
								PFIC_EnableIRQ(USB_IRQn);
							}
							break;
						}
						case USB_SET_ADDRESS:   //0x05
						{
							R8_USB_DEV_AD = (R8_USB_DEV_AD & RB_UDA_GP_BIT) | (devinf.UsbAddress);
							PFIC_DisableIRQ(USB_IRQn);
							R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK | UEP_T_RES_NAK;
							PFIC_EnableIRQ(USB_IRQn);

							break;
						}
						case DEF_VEN_DEBUG_READ:     //0X95
						case DEF_VEN_GET_VER:         //0X5F
						{
							PFIC_DisableIRQ(USB_IRQn);
							R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_ACK | UEP_T_RES_NAK;
							PFIC_EnableIRQ(USB_IRQn);

							break;
						}
						case DEF_GET_LINE_CODING:  //0x21
						{
							PFIC_DisableIRQ(USB_IRQn);
							R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_ACK | UEP_T_RES_NAK;
							PFIC_EnableIRQ(USB_IRQn);

							break;
						}
						case DEF_SET_LINE_CODING:   //0x20
						{
							PFIC_DisableIRQ(USB_IRQn);
							R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK | UEP_T_RES_NAK;
							PFIC_EnableIRQ(USB_IRQn);
							break;
						}
						default:
						{
							R8_UEP0_T_LEN = 0;
							PFIC_DisableIRQ(USB_IRQn);
							R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK | UEP_T_RES_NAK;
							PFIC_EnableIRQ(USB_IRQn);

							break;
						}
					}
					break;
				}
				case UIS_TOKEN_OUT | 0:      // endpoint 0# OUT
				{
					len = usb_irq_len[i];
					if(len)
					{
						switch(SetupReqCode)
						{
							case DEF_SET_LINE_CODING:
							{
								UINT32 set_bps;
								UINT8  data_bit;
								UINT8  stop_bit;
								UINT8  ver_bit;

								memcpy(&set_bps,Ep0Buffer,4);
								stop_bit = Ep0Buffer[4];
								ver_bit = Ep0Buffer[5];
								data_bit = Ep0Buffer[6];

								Uart0Para.BaudRate = set_bps;
								Uart0Para.StopBits = stop_bit;
								Uart0Para.ParityType = ver_bit;
								Uart0Para.DataBits = data_bit;
								CDCSer0ParaChange = 1;

								PFIC_DisableIRQ(USB_IRQn);
								R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK|UEP_T_RES_ACK;
								PFIC_EnableIRQ(USB_IRQn);
								break;
							}
							default:
								PFIC_DisableIRQ(USB_IRQn);
								R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK | UEP_T_RES_NAK;
								PFIC_EnableIRQ(USB_IRQn);
								break;
						}
					}
					else
					{
						PFIC_DisableIRQ(USB_IRQn);
						R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK|UEP_T_RES_NAK;
						PFIC_EnableIRQ(USB_IRQn);
					}
					break;
				}
				default:
					break;
			}

			PFIC_DisableIRQ(USB_IRQn);
			usb_irq_flag[i] = 0;
			PFIC_EnableIRQ(USB_IRQn);
		}
	}

	if ( R8_USB_INT_FG & RB_UIF_BUS_RST )
	{
		R8_UEP0_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;
		R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
		R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
		R8_UEP3_CTRL = UEP_T_RES_NAK;
		R8_UEP4_CTRL = UEP_T_RES_NAK;

		cdc_uart_sta_trans_step = 0;
		ven_ep1_trans_step = 0;

		R8_USB_DEV_AD = 0x00;
		devinf.UsbAddress = 0;

		R8_USB_INT_FG = RB_UIF_BUS_RST;
	}
	else if (  R8_USB_INT_FG & RB_UIF_SUSPEND )
	{
		if ( R8_USB_MIS_ST & RB_UMS_SUSPEND )
		{
			CDCSer0ParaChange = 1;

			Ep1DataINFlag = 0;
			Ep2DataINFlag = 0;
			Ep3DataINFlag = 0;
			Ep4DataINFlag = 0;

			Ep1DataOUTFlag = 0;
			Ep2DataOUTFlag = 0;
			Ep3DataOUTFlag = 0;
			Ep4DataOUTFlag = 0;

		}
		else
		{
			Ep1DataINFlag = 1;
			Ep2DataINFlag = 1;
			Ep3DataINFlag = 1;
			Ep4DataINFlag = 1;

			Ep1DataOUTFlag = 0;
			Ep2DataOUTFlag = 0;
			Ep3DataOUTFlag = 0;
			Ep4DataOUTFlag = 0;
		}

		cdc_uart_sta_trans_step = 0;
		ven_ep1_trans_step = 0;

		R8_USB_INT_FG = RB_UIF_SUSPEND;
	}
}

void USBDevEPnINSetStatus(UINT8 ep_num, UINT8 type, UINT8 sta)
{
	UINT8 *p_UEPn_CTRL;

	p_UEPn_CTRL = (UINT8 *)(USB_BASE_ADDR + 0x22 + ep_num * 4);
	if(type == ENDP_TYPE_IN) *((PUINT8V)p_UEPn_CTRL) = (*((PUINT8V)p_UEPn_CTRL) & (~(0x03))) | sta;
	else *((PUINT8V)p_UEPn_CTRL) = (*((PUINT8V)p_UEPn_CTRL) & (~(0x03<<2))) | (sta<<2);
}

void USBParaInit(void)
{
	Ep1DataINFlag = 1;
	Ep1DataOUTFlag = 0;
	Ep2DataINFlag = 1;
	Ep2DataOUTFlag = 0;
	Ep3DataINFlag = 1;
	Ep3DataOUTFlag = 0;
	Ep4DataINFlag = 1;
	Ep4DataOUTFlag = 0;
}

void InitCDCDevice(void)
{
	USBParaInit();
	R8_USB_CTRL = 0x00;
	R8_UEP4_1_MOD = RB_UEP4_TX_EN | RB_UEP1_TX_EN | RB_UEP1_RX_EN;
	R8_UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP2_TX_EN | RB_UEP3_TX_EN;

	R16_UEP0_DMA = (UINT32)&Ep0Buffer[0];
	R16_UEP1_DMA = (UINT32)&Ep1Buffer[0];
	R16_UEP2_DMA = (UINT32)&Ep2Buffer[0];
	R16_UEP3_DMA = (UINT32)&Ep3Buffer[0];
	//R16_UEP4_DMA = (UINT16)(UINT32)&Ep2Buffer[0];

	R8_UEP0_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;
	R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
	R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
	R8_UEP3_CTRL = UEP_T_RES_NAK;
	R8_UEP4_CTRL = UEP_T_RES_NAK;
	R8_USB_DEV_AD = 0x00;
	R8_UDEV_CTRL = RB_UD_PD_DIS;
	R8_USB_CTRL = RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN;
	R8_USB_INT_FG = 0xFF;
	//R8_USB_INT_EN = RB_UIE_SUSPEND | RB_UIE_TRANSFER | RB_UIE_BUS_RST;
	R8_USB_INT_EN = RB_UIE_TRANSFER ;
	PFIC_EnableIRQ(USB_IRQn);
	R8_UDEV_CTRL |= RB_UD_PORT_EN;

	devinf.UsbConfig = 0;
	devinf.UsbAddress = 0;
}

void InitUSBDevPara(void)
{
	UINT8 i;

	Uart0Para.BaudRate = 115200;
	Uart0Para.DataBits = HAL_UART_8_BITS_PER_CHAR;
	Uart0Para.ParityType = HAL_UART_NO_PARITY;
	Uart0Para.StopBits = HAL_UART_ONE_STOP_BIT;

	VENSer0ParaChange = 0;
	VENSer0SendFlag = 0;
	CDCSetSerIdx = 0;
	CDCSer0ParaChange = 0;

	for(i=0; i<CH341_REG_NUM; i++)
	{
		CH341_Reg_Add[i] = 0xff;
		CH341_Reg_val[i] = 0x00;
	}

	UART0_DCD_Val = 0;
	UART0_RI_Val = 0;
	UART0_DSR_Val = 0;
	UART0_CTS_Val = 0;

	UART0_RTS_Val = 0;
	UART0_DTR_Val = 0;
	UART0_OUT_Val = 0;

	for(i=0; i<USB_IRQ_FLAG_NUM; i++)
	{
		usb_irq_flag[i] = 0;
	}
}

void sendCDCData(const uint8_t *data, uint16_t len)
{
	memcpy(&Ep1Buffer[MAX_PACKET_SIZE], data, len);

	Ep1DataINFlag = 0;
	R8_UEP1_T_LEN = (UINT8)len;
	PFIC_DisableIRQ(USB_IRQn);
	R8_UEP1_CTRL = R8_UEP1_CTRL & 0xfc; //IN_ACK
	PFIC_EnableIRQ(USB_IRQn);
}

void initCDC()
{
	InitUSBDevPara();
	InitCDCDevice();
	PFIC_EnableIRQ( USB_IRQn );
}
void processCDC()
{
		USB_IRQProcessHandler();
}


