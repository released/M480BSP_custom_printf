# M480BSP_custom_printf
 M480BSP_custom_printf

update @ 2020/10/08

1. change #define , to switch specific UART for printf function 

	- #define USE_UART2

	- #define USE_UART3

	- #define USE_UART4

	- #define USE_UART5

	- #define USE_UART6

2. also need to change #define DEBUG_PORT to target UARTxx , in retarget.c , for printf

3. below is UART pin define 
	
	- UART2 TX (PB1) , RX (PB0)
	
	- UART3 TX (PC10) , RX (PC9)	
	
	- UART4 TX (PC7) , RX (PC6)

	- UART5 TX (PB5) , RX (PB4)

	- UART6 TX (PA11) , RX (PA10)
		
		