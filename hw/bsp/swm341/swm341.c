#include "bsp/board.h"
#include "SWM341.h"


void USB_Handler(void)
{
#if TUSB_OPT_DEVICE_ENABLED
    tud_int_handler(0);
#endif

#if TUSB_OPT_HOST_ENABLED
    tuh_int_handler(0);
#endif
}


void SerialInit(void)
{
	UART_InitStructure UART_initStruct;
	
	PORT_Init(PORTM, PIN0, PORTM_PIN0_UART0_RX, 1);	//GPIOM.0配置为UART0输入引脚
	PORT_Init(PORTM, PIN1, PORTM_PIN1_UART0_TX, 0);	//GPIOM.1配置为UART0输出引脚
 	
 	UART_initStruct.Baudrate = 57600;
	UART_initStruct.DataBits = UART_DATA_8BIT;
	UART_initStruct.Parity = UART_PARITY_NONE;
	UART_initStruct.StopBits = UART_STOP_1BIT;
	UART_initStruct.RXThreshold = 3;
	UART_initStruct.RXThresholdIEn = 0;
	UART_initStruct.TXThreshold = 3;
	UART_initStruct.TXThresholdIEn = 0;
	UART_initStruct.TimeoutTime = 10;
	UART_initStruct.TimeoutIEn = 0;
 	UART_Init(UART0, &UART_initStruct);
	UART_Open(UART0);
}

void board_init(void)
{
    SystemInit();

    SerialInit();

#if CFG_TUSB_OS  == OPT_OS_NONE
    // 1ms tick timer
    SysTick_Config(SystemCoreClock / 1000);
#endif
}


#if CFG_TUSB_OS  == OPT_OS_NONE
volatile uint32_t system_ticks = 0;
void SysTick_Handler(void)
{
    system_ticks++;
}

uint32_t board_millis(void)
{
    return system_ticks;
}
#endif


//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
    (void) state;
}

uint32_t board_button_read(void)
{
    return 0;
}

int board_uart_read(uint8_t* buf, int len)
{
    uint32_t data;

    for(int i = 0; i < len; i++)
    {
        if(UART_IsRXFIFOEmpty(UART0))
        {
            return i;
        }
        else
        {
            UART_ReadByte(UART0, &data);
            buf[i] = (uint8_t)data;
        }
    }

    return len;
}

int board_uart_write(void const * buf, int len)
{
    for(int i = 0; i < len; i++)
    {
        UART_WriteByte(UART0, ((const uint8_t *)buf)[i]);
	
	    while(UART_IsTXBusy(UART0)) __NOP();
    }

    return len;
}
