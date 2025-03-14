#include "uart.h"

/***********************************************************************************
* @fn          uart_init
*
* @brief       Function which initializes UART peripheral
*
* @param       
*
* @return      void
*/
void uart_init()
{
    /***************************************************************************
    * Setup I/O ports
    *
    * Port and pins used by USART0 operating in UART-mode, at the Alternative 1
    * location are:
    * RX     : P0_2
    * TX     : P0_3
    * CT/CTS : P0_4
    * RT/RTS : P0_5
    *
    * These pins must be set to function as peripheral I/O to be used by UART0.
    * The TX pin on the transmitter must be connected to the RX pin on the receiver.
    * If enabling hardware flow control (U0UCR.FLOW = 1) the CT/CTS (Clear-To-Send)
    * on the transmitter must be connected to the RS/RTS (Ready-To-Send) pin on the
    * receiver.
    */
    // Configure USART0 for Alternative 1 => Port P0 (PERCFG.U0CFG = 0).
    PERCFG = (PERCFG & ~PERCFG_U0CFG) | PERCFG_U0CFG_ALT1;    
        
    // Give priority to USART 0 over Timer 1 for port 0 pins.
    P2DIR &= P2DIR_PRIP0_USART0;

    // Set pins 2, 3 and 5 as peripheral I/O and pin 4 as GPIO output.
    P0SEL |= BIT5 | BIT4 | BIT3 | BIT2;
     
    // // Initialize P0_1 for SRF05EB S1 button.
    // P0SEL &= ~BIT1;           // Function as General Purpose I/O.
    // P0DIR &= ~BIT1;           // Input.
  
    /***************************************************************************
    * Configure UART
    *
    */
  
    // Initialise bitrate
    U0BAUD = UART_BAUD_M;
    U0GCR = (U0GCR & ~U0GCR_BAUD_E) | UART_BAUD_E;

    // Initialise UART protocol (start/stop bit, data bits, parity, etc.):
    // USART mode = UART (U0CSR.MODE = 1)
    U0CSR |= U0CSR_MODE;

    // Start bit level = low => Idle level = high  (U0UCR.START = 0).
    U0UCR &= ~U0UCR_START;

    // Stop bit level = high (U0UCR.STOP = 1).
    U0UCR |= U0UCR_STOP;

    // Number of stop bits = 1 (U0UCR.SPB = 0).
    U0UCR &= ~U0UCR_SPB;

    // Parity = disabled (U0UCR.PARITY = 0).
    U0UCR &= ~U0UCR_PARITY;

    // 9-bit data enable = 8 bits transfer (U0UCR.BIT9 = 0).
    U0UCR &= ~U0UCR_BIT9;

    // Level of bit 9 = 0 (U0UCR.D9 = 0), used when U0UCR.BIT9 = 1.
    // Level of bit 9 = 1 (U0UCR.D9 = 1), used when U0UCR.BIT9 = 1.
    // Parity = Even (U0UCR.D9 = 0), used when U0UCR.PARITY = 1.
    // Parity = Odd (U0UCR.D9 = 1), used when U0UCR.PARITY = 1.
    U0UCR &= ~U0UCR_D9;

    // Flow control = disabled (U0UCR.FLOW = 0).
    U0UCR &= ~U0UCR_FLOW;

    // Bit order = LSB first (U0GCR.ORDER = 0).
    U0GCR &= ~U0GCR_ORDER;
}


/***********************************************************************************
* @fn          uart0Send
*
* @brief       Function which sends a requested number of bytes on UART0
*
* @param       uint8* uartTxBuf - address of allocated UART TX buffer
*              uint16 uartTxBufLength - size of allocated UART TX buffer
*
* @return      void
*/
void uart0Send(uint8* uartTxBuf, uint16 uartTxBufLength)
{
    uint16 uartTxIndex;

    // Clear any pending TX interrupt request (set U0CSR.TX_BYTE = 0).
    U0CSR &= ~U0CSR_TX_BYTE;

    // Loop: send each UART0 sample on the UART0 TX line.
    for (uartTxIndex = 0; uartTxIndex < uartTxBufLength; uartTxIndex++)
    {
        U0DBUF = uartTxBuf[uartTxIndex];
        while(! (U0CSR & U0CSR_TX_BYTE) );
        U0CSR &= ~U0CSR_TX_BYTE;
    }
}



/***********************************************************************************
* @fn          uart0Receive
*
* @brief       Function which receives a requested number of bytes on UART0
*
* @param       uint8* uartRxBuf - address of allocated UART RX buffer
*              uint16 uartRxBufLength - size of allocated UART RX buffer
*
* @return      void
*/
void uart0Receive(uint8* uartRxBuf, uint16 uartRxBufLength)
{
    uint16 uartRxIndex;

    // Enable UART0 RX (U0CSR.RE = 1).
    U0CSR |= U0CSR_RE;

    // Clear any pending RX interrupt request (set U0CSR.RX_BYTE = 0).
    U0CSR &= ~U0CSR_RX_BYTE;

    // Loop: receive each UART0 sample from the UART0 RX line.
    for (uartRxIndex = 0; uartRxIndex < uartRxBufLength; uartRxIndex++)
    {
        // Wait until data received (U0CSR.RX_BYTE = 1).
        while( !(U0CSR & U0CSR_RX_BYTE) );

        // Read UART0 RX buffer.
        uartRxBuf[uartRxIndex] = U0DBUF;
    }
}