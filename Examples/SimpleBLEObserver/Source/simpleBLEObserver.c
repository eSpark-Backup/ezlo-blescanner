/******************************************************************************

 @file  simpleBLEObserver.c

 @brief This file contains the Simple BLE Observer sample application for use
        with the CC2540 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 
 Copyright (c) 2011-2021, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com.

 ******************************************************************************
 Release Name: ble_sdk_1.5.2.0
 Release Date: 2021-03-10 17:38:59
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "ll.h"
#include "hci.h"

#include <hal_types.h>
#include "ioCC2541.h"
#include "ioCC254x_bitdef.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "observer.h"

#include "simpleBLEObserver.h"

/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  24

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 4000

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         FALSE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * UART RELATED
 */
#define SIZE_OF_UART_RX_BUFFER   50
#define SIZE_OF_UART_TX_BUFFER   SIZE_OF_UART_RX_BUFFER

// UART test characters.
#define UART_TEST_DATA "Texas Instruments LPRF!\n"

//#define UART_TST_MODE_RX
#define UART_TST_MODE_TX

// Baudrate = 57.6 kbps (U0BAUD.BAUD_M = 216, U0GCR.BAUD_E = 10), given 32 MHz system clock.
#define UART_BAUD_M  216
#define UART_BAUD_E  10

/*********************************************************************
 * LOCAL VARIABLES
 */
// Buffer for UART RX/TX.
static uint8 __xdata uartRxBuffer[SIZE_OF_UART_RX_BUFFER];
static uint8 __xdata uartTxBuffer[SIZE_OF_UART_TX_BUFFER];// = UART_TEST_DATA;

// Prototype for local functions.
void uart0Send(uint8* uartTxBuf, uint16 uartTxBufLength);
void uart0Receive(uint8* uartRxBuf, uint16 uartRxBufLength);
void uartInit(void);
// char* uartStringValue( char *title, uint16 value, uint8 format );
// int insertNullTerminator(char *str, size_t maxLen);


// Task ID for internal task/event processing
static uint8 simpleBLETaskId;

// GAP GATT Attributes
//static const uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Observer";

// Number of scan results and scan result index
static uint8 simpleBLEScanRes;
static uint8 simpleBLEScanIdx;

// Scan result list
static gapDevRec_t simpleBLEDevList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static uint8 simpleBLEScanning = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 simpleBLEObserverEventCB( gapObserverRoleEvent_t *pEvent );
static void simpleBLEObserver_HandleKeys( uint8 shift, uint8 keys );
static void simpleBLEObserver_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType );
char *bdAddr2Str ( uint8 *pAddr );




/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapObserverRoleCB_t simpleBLERoleCB =
{
  NULL,                     // RSSI callback
  simpleBLEObserverEventCB  // Event callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEObserver_Init
 *
 * @brief   Initialization function for the Simple BLE Observer App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLEObserver_Init( uint8 task_id )
{
  simpleBLETaskId = task_id;

  // Setup Observer Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPObserverRole_SetParameter ( GAPOBSERVERROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLETaskId );
  
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );

  // initialize UART
  uartInit();
  
  // Setup a delayed profile startup
  osal_set_event( simpleBLETaskId, START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      SimpleBLEObserver_ProcessEvent
 *
 * @brief   Simple BLE Observer Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEObserver_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  char buffer[SIZE_OF_UART_TX_BUFFER];
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLETaskId )) != NULL )
    {
      strcpy(buffer, "Message Received.\n");
      uart0Send((uint8*)buffer, osal_strlen(buffer));

      simpleBLEObserver_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPObserverRole_StartDevice( (gapObserverRoleCB_t *) &simpleBLERoleCB );

    strcpy(buffer, "Device Started.\n");
    uart0Send((uint8*)buffer, osal_strlen(buffer));

    if ( !simpleBLEScanning )
    {
      simpleBLEScanning = TRUE;
      simpleBLEScanRes = 0;

      strcpy(buffer, "Discovering...\n");
      uart0Send((uint8*)buffer, osal_strlen(buffer));
      
      GAPObserverRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                      DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                      DEFAULT_DISCOVERY_WHITE_LIST );      
    }

    return ( events ^ START_DEVICE_EVT );
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLEObserver_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEObserver_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  char buffer[SIZE_OF_UART_TX_BUFFER];

  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      strcpy(buffer, "Processing Message.\n");
      // buffer = "Processing Message.\n";
      uart0Send((uint8*)buffer, osal_strlen(buffer));
      simpleBLEObserver_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
      
    default:
      strcpy(buffer, "Invalid Message.\n");
      // buffer = "Invalid Message.\n";
      uart0Send((uint8*)buffer, osal_strlen(buffer));
      break;
  }
}

/*********************************************************************
 * @fn      simpleBLEObserver_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
uint8 gStatus;
static void simpleBLEObserver_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter

  char buffer[SIZE_OF_UART_TX_BUFFER];

  if ( keys & HAL_KEY_UP )
  {
    strcpy(buffer, "HAL_KEY_UP.\n");
    uart0Send((uint8*)buffer, osal_strlen(buffer));
    // Start or stop discovery
    if ( !simpleBLEScanning )
    {
      simpleBLEScanning = TRUE;
      simpleBLEScanRes = 0;
      
      // LCD_WRITE_STRING( "Discovering...", HAL_LCD_LINE_1 );
      // LCD_WRITE_STRING( "", HAL_LCD_LINE_2 );

      strcpy(buffer, "Discovering...\n");
      uart0Send((uint8*)buffer, osal_strlen(buffer));
      
      GAPObserverRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                      DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                      DEFAULT_DISCOVERY_WHITE_LIST );      
    }
    else
    {
      GAPObserverRole_CancelDiscovery();
    }
  }

  if ( keys & HAL_KEY_LEFT )
  {
    strcpy(buffer, "HAL_KEY_LEFT.\n");
    uart0Send((uint8*)buffer, osal_strlen(buffer));
    // Display discovery results
    if ( !simpleBLEScanning && simpleBLEScanRes > 0 )
    {
        // Increment index of current result (with wraparound)
        simpleBLEScanIdx++;
        if ( simpleBLEScanIdx >= simpleBLEScanRes )
        {
          simpleBLEScanIdx = 0;
        }
        
        strcpy(buffer, "Device.\n");
        uart0Send((uint8*)buffer, osal_strlen(buffer));
        strcpy(buffer, bdAddr2Str( simpleBLEDevList[simpleBLEScanIdx].addr ));
        uart0Send((uint8*)buffer, osal_strlen(buffer));

        // LCD_WRITE_STRING_VALUE( "Device", simpleBLEScanIdx + 1,
        //                         10, HAL_LCD_LINE_1 );
        // LCD_WRITE_STRING( bdAddr2Str( simpleBLEDevList[simpleBLEScanIdx].addr ),
        //                   HAL_LCD_LINE_2 );
    }
  }

  if ( keys & HAL_KEY_RIGHT )
  {
  }
  
  if ( keys & HAL_KEY_CENTER )
  {
  }
  
  if ( keys & HAL_KEY_DOWN )
  {
  }
}

/*********************************************************************
 * @fn      simpleBLEObserverEventCB
 *
 * @brief   Observer event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8 simpleBLEObserverEventCB( gapObserverRoleEvent_t *pEvent )
{
  char buffer[SIZE_OF_UART_TX_BUFFER];
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  
      {
        strcpy(buffer, "BLE Observer.\n");
        uart0Send((uint8*)buffer, osal_strlen(buffer));
        sprintf(buffer,"%s\n", bdAddr2Str( pEvent->initDone.devAddr ));
        uart0Send((uint8*)buffer, osal_strlen(buffer));

        // LCD_WRITE_STRING( "BLE Observer", HAL_LCD_LINE_1 );
        // LCD_WRITE_STRING( bdAddr2Str( pEvent->initDone.devAddr ),  HAL_LCD_LINE_2 );
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        simpleBLEAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType );
      }
      break;
      
    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
        simpleBLEScanning = FALSE;

        // Copy results
        simpleBLEScanRes = pEvent->discCmpl.numDevs;
        osal_memcpy( simpleBLEDevList, pEvent->discCmpl.pDevList,
                     (sizeof( gapDevRec_t ) * pEvent->discCmpl.numDevs) );
        
        // strcpy(buffer, "Devices Found. ");
        // uart0Send((uint8*)buffer, osal_strlen(buffer));
        // _ltoa(simpleBLEScanRes, (uint8*)buffer, 10);
        // // strcpy(buffer, devicesFound);
        // // strcpy(buffer, uartStringValue("Devices Found", simpleBLEScanRes, 10 ));
        // uart0Send((uint8*)buffer, osal_strlen(buffer));

        sprintf(buffer, "Devices Found. %d\n", simpleBLEScanRes);
        uart0Send((uint8*)buffer, osal_strlen(buffer));

        for(uint8 i=0; i<simpleBLEScanRes; i++)
        {
          sprintf(buffer, "%d, %s\n", (i+1), bdAddr2Str(simpleBLEDevList[i].addr));
          uart0Send((uint8*)buffer, osal_strlen(buffer));
        }

        // LCD_WRITE_STRING_VALUE( "Devices Found", simpleBLEScanRes,
        //                         10, HAL_LCD_LINE_1 );
        // if ( simpleBLEScanRes > 0 )
        // {
        //   LCD_WRITE_STRING( "<- To Select", HAL_LCD_LINE_2 );
        // }

        // initialize scan index to last device
        simpleBLEScanIdx = simpleBLEScanRes;
      }
      break;
      
    default:
      break;
  }
  
  return ( TRUE );
}

/*********************************************************************
 * @fn      simpleBLEAddDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType )
{
  uint8 i;
  
  // If result count not at max
  if ( simpleBLEScanRes < DEFAULT_MAX_SCAN_RES )
  {
    // Check if device is already in scan results
    for ( i = 0; i < simpleBLEScanRes; i++ )
    {
      if ( osal_memcmp( pAddr, simpleBLEDevList[i].addr , B_ADDR_LEN ) )
      {
        return;
      }
    }
    
    // Add addr to scan result list
    osal_memcpy( simpleBLEDevList[simpleBLEScanRes].addr, pAddr, B_ADDR_LEN );
    simpleBLEDevList[simpleBLEScanRes].addrType = addrType;
    
    // Increment scan result count
    simpleBLEScanRes++;
  }
}

/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += B_ADDR_LEN;
  
  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  
  *pStr = 0;
  
  return str;
}

/*********************************************************************
 * @fn      uartInit
 *
 * @brief   Convert Bluetooth address to string
 *
 * @return  none
 */
void uartInit( void )
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
  // Initialize P0_1 for SRF05EB S1 button.
  P0SEL &= ~BIT1;           // Function as General Purpose I/O.
  P0DIR &= ~BIT1;           // Input.

  /***************************************************************************
  * Configure UART
  *
  */

  // Initialise bitrate = 57.6 kbps.
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

// char* uartStringValue( char *title, uint16 value, uint8 format )
// {
  // uint8 tmpLen;
  // uint8 buf[SIZE_OF_UART_TX_BUFFER];
  // uint32 err;

  // tmpLen = (uint8)osal_strlen( (char*)title );
  // osal_memcpy( buf, title, tmpLen );
  // buf[tmpLen] = ' ';
  // err = (uint32)(value);
  // _ltoa( err, &buf[tmpLen+1], format );
  // char* buffer = malloc(10);

  // return (char*)uartTxBuffer;
  // // HalLcdWriteString( (char*)buf, line );
// }


// int insertNullTerminator(char *str, size_t maxLen)
// {
//     if (str == NULL || maxLen == 0) {
//         return -1; // Invalid input
//     }
    
//     // Find the current length of the string
//     size_t i;
//     for (i = 0; i < maxLen; i++) {
//         if (str[i] == '\0') {
//             return 0; // Already null-terminated
//         }
//     }

//     // If no null terminator is found, add one at the last position
//     str[maxLen - 1] = '\0';
//     return 0; // Success
// }

/*********************************************************************
*********************************************************************/
