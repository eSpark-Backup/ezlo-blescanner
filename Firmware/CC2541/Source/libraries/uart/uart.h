#ifndef UART_H
#define UART_H

#include "defines.h"


/***********************************************************************************
* CONSTANTS
*/

// Define size of allocated UART RX/TX buffer (just an example).
#define SIZE_OF_UART_RX_BUFFER   50
#define SIZE_OF_UART_TX_BUFFER   256

// UART test characters.
#define UART_TEST_DATA "Texas Instruments LPRF!\n"

// Test definitions.
//#define UART_TST_MODE_RX
#define UART_TST_MODE_TX

#define B_9600   0x01
#define B_57600  0x02
#define B_115200 0x03

#define UART_BAUDRATE   B_115200

#if (UART_BAUDRATE==B_9600)
// Baudrate = 9.6 kbps (U0BAUD.BAUD_M = 59, U0GCR.BAUD_E = 8), given 32 MHz system clock.
#define UART_BAUD_M  59
#define UART_BAUD_E  8
#elif (UART_BAUDRATE==B_57600)
// Baudrate = 57.6 kbps (U0BAUD.BAUD_M = 216, U0GCR.BAUD_E = 10), given 32 MHz system clock.
#define UART_BAUD_M  216
#define UART_BAUD_E  10
#elif (UART_BAUDRATE==B_115200)
// Baudrate = 115.2 kbps (U0BAUD.BAUD_M = 216, U0GCR.BAUD_E = 11), given 32 MHz system clock.
#define UART_BAUD_M  216
#define UART_BAUD_E  11
#endif

/***********************************************************************************
* LOCAL VARIABLES
*/

// Buffer for UART RX/TX.
static uint8 __xdata uartRxBuffer[SIZE_OF_UART_RX_BUFFER];
static uint8 __xdata uartTxBuffer[SIZE_OF_UART_TX_BUFFER];// = UART_TEST_DATA;


/***********************************************************************************
* LOCAL FUNCTIONS
*/

void uart_init();
void uart0Send(uint8* uartTxBuf, uint16 uartTxBufLength);
void uart0Receive(uint8* uartRxBuf, uint16 uartRxBufLength);


#endif   // UART_H