#include "i2c.h"
#include "print.h"

/***********************************************************************************
* @fn          I2C_ISR
*
* @brief       Function which sends the next I2C data byte.
*
* @param       none
*
* @return      0
*/
#pragma vector = I2C_VECTOR
__interrupt void I2C_ISR(void)
{ 
    // Clear I2C CPU interrupt flag.
    P2IF = 0;
  
    // If previously addressed with own Slave address, and a Data byte has been
    // received and acknowledged ...  
    if (I2CSTAT == 0x80)
    { 
        // Read the I2C byte.
        buffer[bufferIndex++] = I2CDATA;
    }

    // If general call address received
    if (I2CSTAT == 0x70)
    { 
        // Read the I2C byte.
        buffer[bufferIndex++] = I2CDATA;
    }

    // If a Stop condition is detected ...
    else if (I2CSTAT == 0xA0)
    {    
        // Reset buffer index.
        bufferIndex = 0;
    }

    // I2CCFG.SI flag must be cleared by software at the end of the ISR.
    I2CCFG &= ~I2CCFG_SI;
}

void i2c_init()
{
    /***************************************************************************
    * Setup I/O ports
    *
    * CC2541 has dedicated I2C ports
    * I2C SCL:   Pin 2   (Debug Connector P18_5)
    * I2C SDA:   Pin 3   (Debug Connector P18_3)
    */
  
    // Enable I2C on CC2541.
    I2CWC &= ~0x80;


  /***************************************************************************
  * Setup interrupt
  */

    // Clear I2C CPU interrupt flag.
    P2IF = 0;

    // Enable interrupt from I2C by setting [IEN2.I2CIE = 1].
    IEN2 |= IEN2_P2IE;

    // Enable global interrupts.
    EA = 1;


    /***************************************************************************
    * Configure I2C
    */
  
    // Set device address, with General-call address acknowledge.
    I2CADDR = (slaveAddr << 1) | I2CADDR_GC;
  
    // Enable the I2C module, the slave is clocked by the Master.
    // Enable Assert Acknowledge (AA bit).
    I2CCFG |= I2CCFG_ENS1 | I2CCFG_AA;
}