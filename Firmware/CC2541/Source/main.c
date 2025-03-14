// /***********************************************************************************
//   Filename:     main.c

//   Description:  This example uses a slave to receive data using I2C.

//   Comments:     To execute this example, use the IAR project i2c_master_send
//                 as the master. The slave's code must be executed before executing
//                 the master's code, since the slave is clocked by the Master.
//                 When a stop condition received, the LED1 will be set on the
//                 SmartRF05EB and the buffer index will be reset.

//   Note:         On the SmartRF05EB, P0_6 and P0_7 is shared by the 
//                 EM_JOY_LEVEL and EM_LCD_CS signals. These have to be disconnected 
//                 by removing jumpers 7-8 and 3-4 on P10 I/O Connector. Thus the I2C 
//                 and SRF05EB LCD can't be used simultaneously.

//                 The I2C pins also have external pullups on the EM, at the as 
//                 required by the I2C standard.

// ***********************************************************************************/

#include "defines.h"
#include "print.h"
#include "uart.h"
#include "i2c.h"


/***********************************************************************************
* CONSTANTS
*/


/***********************************************************************************
* LOCAL VARIABLES
*/


/***********************************************************************************
* @fn          main
*
* @brief       Send/receive data on UART0, using polling.
*
* @param       void
*
* @return      void
*/
void main (void)
{
    /****************************************************************************
    * Clock setup
    * See basic software example "clk_xosc_cc254x"
    */
  
    // Set system clock source to HS XOSC, with no pre-scaling.
    CLKCONCMD = (CLKCONCMD & ~(CLKCON_OSC | CLKCON_CLKSPD)) | CLKCON_CLKSPD_32M;
    // Wait until clock source has changed.
    while (CLKCONSTA & CLKCON_OSC);
  
    /* Note the 32 kHz RCOSC starts calibrating, if not disabled. */

    
    uart_init();
    i2c_init();

    while(1)
    {

        for(int i = 0; i<9000; i++)
        {
          for(int y = 0; y<90; y++)
          {}
        }
        Printf(UART_TEST_DATA);
    }
}


