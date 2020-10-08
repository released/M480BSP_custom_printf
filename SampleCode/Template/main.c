/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    MP3 player sample plays MP3 files stored on SD memory card
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include<stdarg.h>

#define LED_R					(PH0)
#define LED_Y					(PH1)
#define LED_G					(PH2)

#define BUF_LEN					(1024)

/*
	also need to change #define DEBUG_PORT to target UARTxx , in retarget.c , for printf
*/
#define USE_UART2
//#define USE_UART3
//#define USE_UART4
//#define USE_UART5
//#define USE_UART6

#if defined (USE_UART2)
#define UARTx_TARGET			(UART2)
#define UARTx_IRQHandler			(UART2_IRQHandler)
#define UARTx_TARGET_RST		(UART2_RST)
#define UARTx_TARGET_IRQn		(UART2_IRQn)

#elif defined (USE_UART3)
#define UARTx_TARGET			(UART3)
#define UARTx_IRQHandler			(UART3_IRQHandler)
#define UARTx_TARGET_RST		(UART3_RST)
#define UARTx_TARGET_IRQn		(UART3_IRQn)

#elif defined (USE_UART4)
#define UARTx_TARGET			(UART4)
#define UARTx_IRQHandler			(UART4_IRQHandler)
#define UARTx_TARGET_RST		(UART4_RST)
#define UARTx_TARGET_IRQn		(UART4_IRQn)

#elif defined (USE_UART5)
#define UARTx_TARGET			(UART5)
#define UARTx_IRQHandler			(UART5_IRQHandler)
#define UARTx_TARGET_RST		(UART5_RST)
#define UARTx_TARGET_IRQn		(UART5_IRQn)

#elif defined (USE_UART6)
#define UARTx_TARGET			(UART6)
#define UARTx_IRQHandler			(UART6_IRQHandler)
#define UARTx_TARGET_RST		(UART6_RST)
#define UARTx_TARGET_IRQn		(UART6_IRQn)
#endif



#define FIFO_THRESHOLD 			(4)
#define RX_BUFFER_SIZE 			(256)
#define RX_TIMEOUT_CNT 			(60) //40~255

#define UART_RX_IDEL(uart) (((uart)->FIFOSTS & UART_FIFOSTS_RXIDLE_Msk )>> UART_FIFOSTS_RXIDLE_Pos)

typedef struct {
	uint8_t RX_Buffer[RX_BUFFER_SIZE];
	uint16_t Length;
	uint8_t RDA_Trigger_Cnt;
	uint8_t RXTO_Trigger_Cnt;
	
//	uint8_t end;
}UART_BUF_t;

UART_BUF_t uart0Dev;
UART_BUF_t uartxDev;

typedef enum{
	flag_DEFAULT = 0 ,
		
	flag_UART0_Received_Data ,	
	flag_UARTx_Received_Data ,
	
	flag_END	
}Flag_Index;

volatile uint32_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint32_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))

/*****************************************************************************/

void printf_UART(uint8_t *str, ...);

/*****************************************************************************/

void _putchar(int ch)
{
    while(UART_GET_TX_FULL(UART0));

    if(ch == '\n')
    {
        UART_WRITE(UART0, '\r');
        while(UART_GET_TX_FULL(UART0));
    }
    UART_WRITE(UART0, ch);
}

void _printInteger(uint32_t u32Temp)
{
    uint8_t print_buf[16];
    uint32_t i = 15, j;

    *(print_buf + i) = '\0';
    j = u32Temp >> 31;
    if(j)
        u32Temp = ~u32Temp + 1;
    do
    {
        i--;
        *(print_buf + i) = '0' + u32Temp % 10;
        u32Temp = u32Temp / 10;
    }
    while(u32Temp != 0);
    if(j)
    {
        i--;
        *(print_buf + i) = '-';
    }
    printf_UART(print_buf + i);
}
void _printHex(uint32_t u32Temp)
{
    uint8_t print_buf[16];
    uint32_t i = 15;
    uint32_t temp;

    *(print_buf + i) = '\0';
    do
    {
        i--;
        temp = u32Temp % 16;
        if(temp < 10)
            *(print_buf + i) = '0' + temp;
        else
            *(print_buf + i) = 'a' + (temp - 10) ;
        u32Temp = u32Temp / 16;
    }
    while(u32Temp != 0);
    printf_UART(print_buf + i);
}
void printf_UART(uint8_t *str, ...)
{
    va_list args;
    va_start(args, str);
    while(*str != '\0')
    {
        if(*str == '%')
        {
            str++;
            if(*str == '\0') return;
            if(*str == 'd')
            {
                str++;
                _printInteger(va_arg(args, int));
            }
            else if(*str == 'x')
            {
                str++;
                _printHex(va_arg(args, int));
            }
        }
        _putchar(*str++);
    }
}



void GPIO_Init(void)
{
    GPIO_SetMode(PH, BIT0, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PH, BIT1, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PH, BIT2, GPIO_MODE_OUTPUT);	
}


void TMR0_IRQHandler(void)
{
	static uint32_t LOG = 0;
	static uint16_t CNT = 0;

	uint8_t res = 0;

    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);

		if (CNT++ >= 1000)
		{
			CNT = 0;
			LOG++;
        	printf("%s : %4d\r\n",__FUNCTION__,LOG);
//			printf_UART("UART0 : %d\r\n",LOG);
				
			LED_G ^= 1;	
		}
    }
}

void TIMER0_Init(void)
{
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);	
    TIMER_Start(TIMER0);
}

void UARTx_IRQHandler(void)
{
	uint8_t i;
	static uint16_t u16UART_RX_Buffer_Index = 0;

    if(UART_GET_INT_FLAG(UARTx_TARGET, UART_INTSTS_RDAINT_Msk))    
    {
        /* UART receive data available flag */
        
        /* Record RDA interrupt trigger times */
        uartxDev.RDA_Trigger_Cnt++;
        
        /* Move the data from Rx FIFO to sw buffer (RAM). */
        /* Every time leave 1 byte data in FIFO for Rx timeout */
        for(i = 0 ; i < (FIFO_THRESHOLD - 1) ; i++)
        {
            uartxDev.RX_Buffer[u16UART_RX_Buffer_Index] = UART_READ(UARTx_TARGET);
            u16UART_RX_Buffer_Index ++;

            if (u16UART_RX_Buffer_Index >= RX_BUFFER_SIZE) 
                u16UART_RX_Buffer_Index = 0;
        }
    }
    else if(UART_GET_INT_FLAG(UARTx_TARGET, UART_INTSTS_RXTOINT_Msk)) 
    {
        /* When Rx timeout flag is set to 1, it means there is no data needs to be transmitted. */

        /* Record Timeout times */
        uartxDev.RXTO_Trigger_Cnt++;

        /* Move the last data from Rx FIFO to sw buffer. */
        while(UART_GET_RX_EMPTY(UARTx_TARGET) == 0)
        {
            uartxDev.RX_Buffer[u16UART_RX_Buffer_Index] = UART_READ(UARTx_TARGET);
            u16UART_RX_Buffer_Index ++;
        }

        /* Clear UART RX parameter */
        UART_DISABLE_INT(UARTx_TARGET, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
        u16UART_RX_Buffer_Index = 0;

//        printf("\r\nUARTx Rx Received Data : %s\n",uartxDev.RX_Buffer);
//        printf("UARTx Rx RDA (Fifofull) interrupt times : %d\r\n",uartxDev.RDA_Trigger_Cnt);
//        printf("UARTx Rx RXTO (Timeout) interrupt times : %d\r\n",uartxDev.RXTO_Trigger_Cnt);

        /* Reset UART interrupt parameter */
        UART_EnableInt(UARTx_TARGET, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
		memset(&uartxDev, 0x00, sizeof(UART_BUF_t));

    }
	
}

void UARTx_Init(void)
{
    SYS_ResetModule(UARTx_TARGET_RST);

    /* Configure UARTx and set UARTx baud rate */
    UART_Open(UARTx_TARGET, 115200);

	/* Set UARTx receive time-out */
	UART_SetTimeoutCnt(UARTx_TARGET, RX_TIMEOUT_CNT);

	/* Set UARTx FIFO RX interrupt trigger level to 4-bytes*/
    UARTx_TARGET->FIFO = ((UARTx_TARGET->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_4BYTES);

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UARTx_TARGET, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(UARTx_TARGET_IRQn);
	
	memset(&uartxDev, 0x00, sizeof(UART_BUF_t));

	UART_WAIT_TX_EMPTY(UARTx_TARGET);

	printf("\r\n\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());	
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());

}


void UART0_IRQHandler(void)
{
	uint8_t i;
	uint16_t length;
	
	static uint16_t u16UART_RX_Buffer_Index = 0;

    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk))    
    {
        /* UART receive data available flag */
        
        /* Record RDA interrupt trigger times */
        uart0Dev.RDA_Trigger_Cnt++;
        
        /* Move the data from Rx FIFO to sw buffer (RAM). */
        /* Every time leave 1 byte data in FIFO for Rx timeout */
        for(i = 0 ; i < (FIFO_THRESHOLD - 1) ; i++)
        {
            uart0Dev.RX_Buffer[u16UART_RX_Buffer_Index] = UART_READ(UART0);
            u16UART_RX_Buffer_Index ++;

            if (u16UART_RX_Buffer_Index >= RX_BUFFER_SIZE) 
                u16UART_RX_Buffer_Index = 0;
        }
    }
    else if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RXTOINT_Msk)) 
    {
        /* When Rx timeout flag is set to 1, it means there is no data needs to be transmitted. */

        /* Record Timeout times */
        uart0Dev.RXTO_Trigger_Cnt++;

        /* Move the last data from Rx FIFO to sw buffer. */
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
            uart0Dev.RX_Buffer[u16UART_RX_Buffer_Index] = UART_READ(UART0);
            u16UART_RX_Buffer_Index ++;
        }

        /* Clear UART RX parameter */
        UART_DISABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
        u16UART_RX_Buffer_Index = 0;

	
		LED_R ^= 1;	
//        printf_UART("\nUART0 Rx Received Data : %s\n",uart0Dev.RX_Buffer);

		printf_UART("\r\n");
        for(i = 0 ; i < (uart0Dev.RDA_Trigger_Cnt) ; i++)
        {
			printf_UART("0x%x ," , uart0Dev.RX_Buffer[i]);
        }
		printf_UART("\r\n");
		
        printf_UART("UART0 Rx RDA (Fifofull) interrupt times : %d\r\n",uart0Dev.RDA_Trigger_Cnt);
        printf_UART("UART0 Rx RXTO (Timeout) interrupt times : %d\r\n",uart0Dev.RXTO_Trigger_Cnt);

        /* Reset UART interrupt parameter */
        UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
		memset(&uart0Dev, 0x00, sizeof(UART_BUF_t));

    }
	
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, RX_TIMEOUT_CNT);

	/* Set UART FIFO RX interrupt trigger level to 4-bytes*/
    UART0->FIFO = ((UART0->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_4BYTES);

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(UART0_IRQn);
	
	memset(&uart0Dev, 0x00, sizeof(UART_BUF_t));

	UART_WAIT_TX_EMPTY(UART0);

//	printf_UART("\r\n\r\nUART0 : CLK_GetCPUFreq : %d\r\n",CLK_GetCPUFreq());
//	printf_UART("UART0 : CLK_GetHCLKFreq : %d\r\n",CLK_GetHCLKFreq());	
//	printf_UART("UART0 : CLK_GetHXTFreq : %d\r\n",CLK_GetHXTFreq());
//	printf_UART("UART0 : CLK_GetLXTFreq : %d\r\n",CLK_GetLXTFreq());	
//	printf_UART("UART0 : CLK_GetPCLK0Freq : %d\r\n",CLK_GetPCLK0Freq());
//	printf_UART("UART0 : CLK_GetPCLK1Freq : %d\r\n",CLK_GetPCLK1Freq());

}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2;

    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

	if (UARTx_TARGET == UART6)
	{
	    CLK_EnableModuleClock(UART6_MODULE);
	    CLK_SetModuleClock(UART6_MODULE, CLK_CLKSEL3_UART6SEL_HXT, CLK_CLKDIV4_UART6(1));
	    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA11MFP_Msk | SYS_GPA_MFPH_PA10MFP_Msk);
	    SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA11MFP_UART6_TXD | SYS_GPA_MFPH_PA10MFP_UART6_RXD);
	}
	else if (UARTx_TARGET == UART5)
	{
	    CLK_EnableModuleClock(UART5_MODULE);
	    CLK_SetModuleClock(UART5_MODULE, CLK_CLKSEL3_UART5SEL_HXT, CLK_CLKDIV4_UART5(1));
	    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB5MFP_Msk | SYS_GPB_MFPL_PB4MFP_Msk);
	    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB5MFP_UART5_TXD | SYS_GPB_MFPL_PB4MFP_UART5_RXD);
	}
	else if (UARTx_TARGET == UART4)
	{
	    CLK_EnableModuleClock(UART4_MODULE);
	    CLK_SetModuleClock(UART4_MODULE, CLK_CLKSEL3_UART4SEL_HXT, CLK_CLKDIV4_UART4(1));
	    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC7MFP_Msk | SYS_GPC_MFPL_PC6MFP_Msk);
	    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC7MFP_UART4_TXD | SYS_GPC_MFPL_PC6MFP_UART4_RXD);
	}
	else if (UARTx_TARGET == UART3)
	{
	    CLK_EnableModuleClock(UART3_MODULE);
	    CLK_SetModuleClock(UART3_MODULE, CLK_CLKSEL3_UART3SEL_HXT, CLK_CLKDIV4_UART3(1));
	    SYS->GPC_MFPH &= ~(SYS_GPC_MFPH_PC10MFP_Msk | SYS_GPC_MFPH_PC9MFP_Msk);
	    SYS->GPC_MFPH |= (SYS_GPC_MFPH_PC10MFP_UART3_TXD | SYS_GPC_MFPH_PC9MFP_UART3_RXD);
	}		
	else if (UARTx_TARGET == UART2)
	{
	    CLK_EnableModuleClock(UART2_MODULE);
	    CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HXT, CLK_CLKDIV4_UART2(1));
	    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB1MFP_Msk | SYS_GPB_MFPL_PB0MFP_Msk);
	    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB1MFP_UART2_TXD | SYS_GPB_MFPL_PB0MFP_UART2_RXD);
	}	

    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, MODULE_NoMsk);

	
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();	
	
}

int32_t main (void)
{

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    UART0_Init();	
    UARTx_Init();
	
	GPIO_Init();
	
	TIMER0_Init();	

    while(1)
    {
		LED_Y ^= 1;
    }
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
