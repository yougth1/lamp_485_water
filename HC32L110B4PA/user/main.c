#include "gpio.h"
#include "adc.h"
#include "uart.h"
#include "bt.h"

static void GpioInit(void);


#define MAX_BUF_LEN 20
uint8_t u8RxData[MAX_BUF_LEN]={0};
uint8_t u8RxFlg=0;
uint8_t uRxDataPos = 0;

static uint16_t crc16_modbus(uint8_t *puch_msg, uint8_t data_len)
{
	uint8_t i = 0, j = 0;
	uint16_t wCRCin = 0xFFFF;
	
	for (; i < data_len; i++) {
		wCRCin = wCRCin ^ *puch_msg++;
		for (j = 0; j < 8; j++) {
			if ((wCRCin & 0x0001) > 0) {
				wCRCin = wCRCin >> 1;
				wCRCin = wCRCin ^ 0xA001;
			}
			else
				wCRCin = wCRCin >> 1;
		}
	}
	
	return (((wCRCin & 0xFF00) >> 8) | ((wCRCin & 0x00FF) << 8));
}

void RxIntCallback(void)
{
    if(uRxDataPos < MAX_BUF_LEN)
    {
        u8RxData[uRxDataPos++] = M0P_UART1->SBUF;
    }
    
    u8RxFlg = 1;
}
void ErrIntCallback(void)
{
  
}

void ClearRxData(void)
{
	uRxDataPos = 0;
	memset(u8RxData, 0, sizeof(u8RxData));
}

uint8_t GetGpioResult(void)
{
    return Gpio_GetIO(3,5);
}

void SendGpioData(void)
{
	uint16_t data_crc;
    uint8_t u8GpioRlt = 0;
	uint8_t GpioData[10] = {0x02, 0x03, 0x02};

	
    u8GpioRlt = GetGpioResult();
	GpioData[3] = 0;
	GpioData[4] = u8GpioRlt;
	data_crc = crc16_modbus(GpioData, 5);
	GpioData[5] = ((data_crc & 0xFF00) >> 8);
	GpioData[6] = (data_crc & 0x00FF);
	
//	Gpio_SetIO(2,5,1);
	Uart1_SendDatas(GpioData, 7);
//	Gpio_SetIO(2,5,0);
//	ClearRxData();
}

int main()
{
    static uint8_t cnt = 0;
	uint16_t data_crc = 0;    
	GpioInit();
    
	
	while(1)
	{	
		if(u8RxFlg)
		{
            //02 03 00 01 00 01 XX YY
			if (uRxDataPos >= 8) {
				if (u8RxData[0] == 0x02 && u8RxData[1] == 0x03) {
                    data_crc = crc16_modbus(u8RxData, 6);
					if (u8RxData[6] == ((data_crc & 0xFF00) >> 8) && u8RxData[7]==(data_crc & 0x00FF)){
                        if (u8RxData[3] == 0x01 && u8RxData[5] == 0x01)
                            SendGpioData();
                    }
                }
                
                cnt = 0;
                u8RxFlg = 0;
                ClearRxData();                
			}
            else{
                if(cnt++ > 0) {
                    cnt = 0;
                    u8RxFlg = 0;
                    ClearRxData();
                }
            }            

		}
		
		delay1ms(20);
	}
}


static void GpioInit(void)
{

	stc_uart_config_t  			stcConfig;
	stc_uart_irq_cb_t 			stcUartIrqCb;
	stc_uart_multimode_t 		stcMulti;
	stc_uart_baud_config_t 	    stcBaud;
	stc_bt_config_t 			stcBtConfig;
	uint32_t timer = 0;
	
	Clk_SwitchTo(ClkRCL);
	Clk_SetRCHFreq(ClkFreq24Mhz);
	Clk_SwitchTo(ClkRCH);
	Clk_SetPeripheralGate(ClkPeripheralGpio, TRUE);
    
//	Gpio_SetAnalog(3, 5, TRUE);
    Gpio_InitIO(3,5,GpioDirIn);

	
	DDL_ZERO_STRUCT(stcUartIrqCb);
	DDL_ZERO_STRUCT(stcMulti);
	DDL_ZERO_STRUCT(stcBaud);
	DDL_ZERO_STRUCT(stcBtConfig);

//	Gpio_InitIO(2,5,GpioDirOut);
//	Gpio_SetIO(2,5,0);
    
	Gpio_InitIOExt(2,3,GpioDirOut,TRUE,FALSE,FALSE,FALSE);
	Gpio_InitIOExt(2,4,GpioDirOut,TRUE,FALSE,FALSE,FALSE);
	Gpio_SetFunc_UART1TX_P23();
	Gpio_SetFunc_UART1RX_P24();
	Clk_SetPeripheralGate(ClkPeripheralBt,TRUE);
	Clk_SetPeripheralGate(ClkPeripheralUart1,TRUE);
	stcUartIrqCb.pfnRxIrqCb = RxIntCallback;
	stcUartIrqCb.pfnTxIrqCb = NULL;
	stcUartIrqCb.pfnRxErrIrqCb = ErrIntCallback;
	stcConfig.pstcIrqCb = &stcUartIrqCb;
	stcConfig.bTouchNvic = TRUE;
	stcConfig.enRunMode = UartMode1;
	stcMulti.enMulti_mode = UartNormal;
	stcConfig.pstcMultiMode = &stcMulti;
	stcBaud.bDbaud = 0u;
	stcBaud.u32Baud = 9600u;
	stcBaud.u8Mode = UartMode1;
	timer = Uart_SetBaudRate(UARTCH1, Clk_GetPClkFreq(), &stcBaud);
    
	stcBtConfig.enMD = BtMode2;
	stcBtConfig.enCT = BtTimer;
	Bt_Init(TIM1, &stcBtConfig);
	Bt_ARRSet(TIM1,timer);
	Bt_Cnt16Set(TIM1,timer);
	Bt_Run(TIM1);
	Uart_Init(UARTCH1, &stcConfig);
    
	Uart_EnableIrq(UARTCH1,UartRxIrq);
	Uart_ClrStatus(UARTCH1,UartRxFull);
	Uart_EnableFunc(UARTCH1,UartRx);
}







