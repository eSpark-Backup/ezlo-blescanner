#include "print.h"
#include <string.h>
#include <stdarg.h>  // For va_list, va_start, va_end
#include <stdio.h>   // For vsnprintf


void Printf(const char *format, ...)
{
    uartTxIndex = 15;
    va_list args;                    // Argument list for format specifiers

    va_start(args, format);          // Initialize the argument list
    vsprintf((char*)uartTxBuffer, format, args);  // Format the string
    va_end(args);                    // Clean up the argument list

    transmitLength = strlen((char*)uartTxBuffer);
    uart0StartTxForIsr();
    
    // uart0Send(uartTxBuffer, strlen((char*)uartTxBuffer));  // Send character via UART
}
