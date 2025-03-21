#include "print.h"
#include <string.h>
#include <stdarg.h>  // For va_list, va_start, va_end
#include <stdio.h>   // For vsnprintf


void Printf(const char *format, ...)
{
    uint8   txBuffer[NPI_UART_TX_BUF_SIZE];
    va_list args;                    // Argument list for format specifiers

    va_start(args, format);          // Initialize the argument list
    int transmitLength = vsprintf((char*)txBuffer, format, args);  // Format the string
    va_end(args);                    // Clean up the argument list

    // int transmitLength = strlen((char*)txBuffer);

    if (transmitLength < NPI_UART_TX_BUF_SIZE - 1)
    {
        txBuffer[transmitLength] = '\n';
        txBuffer[++transmitLength] = '\0';  // Null-terminate the string
    }
    
    // HalUARTWrite(HAL_UART_PORT_0, txBuffer, transmitLength);
    NPI_WriteTransport(txBuffer, transmitLength);
}
