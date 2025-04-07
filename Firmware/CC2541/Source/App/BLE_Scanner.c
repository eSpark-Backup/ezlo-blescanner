/******************************************************************************

 @file  simpleBLEObserver.c

 @brief This file contains the Simple BLE Observer sample application for use
        with the CC2540 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 
 Copyright (c) 2011-2016, Texas Instruments Incorporated
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
 Release Name: ble_sdk_1.4.2.2
 Release Date: 2016-06-09 06:57:10
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

#include "observer.h"

#include "BLE_Scanner.h"

#include "print.h"

/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

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
 * LOCAL VARIABLES
 */

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
static void simpleBLEObserver_HandleCommand( uint8 shift, uint8 cmd );
// static void simpleBLEObserver_HandleKeys( uint8 shift, uint8 keys );
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
  LOG("Initializing App.\n");
  simpleBLETaskId = task_id;
  
/* I2C */
#if (defined HAL_I2C) && (HAL_I2C == TRUE)
  LOG("Setting up i2c driver.\n");
  HalI2CInit(HAL_I2C_SLAVE_ADDR_DEF, I2C_Callback_Func);
#endif
  
  LOG("Setting up BLE profile.\n");
  // Setup Observer Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPObserverRole_SetParameter ( GAPOBSERVERROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );

  // // Register for all key events - This app will handle all key events
  // RegisterForKeys( simpleBLETaskId );
  
  // // makes sure LEDs are off
  // HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
  
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
  LOG("executing SimpleBLEObserver_ProcessEvent\n");
  
  if ( events & SYS_EVENT_MSG )
  {
    LOG("Processing SYS_EVENT_MSG.\n");
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLETaskId )) != NULL )
    {
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
    LOG("Processing START_DEVICE_EVT.\n");
    // uint8 macAddr[6];
    // osal_memcpy(macAddr, (uint8*)0x7800, 6);
    // LOG("macAddr[]: ");
    // for(uint8 i=0; i<6; i++)
    // {
    //   LOG("%X ", macAddr[i]);
    // }
    // LOG("\n");

    VOID GAPObserverRole_StartDevice( (gapObserverRoleCB_t *) &simpleBLERoleCB );

    // if ( !simpleBLEScanning )
    // {
    //   simpleBLEScanning = TRUE;
    //   simpleBLEScanRes = 0;

    //   // strcpy(buffer, "Discovering...\n");
    //   // uart0Send((uint8*)buffer, osal_strlen(buffer));
      
    //   GAPObserverRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
    //                                   DEFAULT_DISCOVERY_ACTIVE_SCAN,
    //                                   DEFAULT_DISCOVERY_WHITE_LIST );      
    // }

    return ( events ^ START_DEVICE_EVT );
  }

  if ( events & BLE_SCAN_START_EVT )
  {
    LOG("BLE_SCAN_START_EVT: \n");

    simpleBLEObserver_HandleCommand(0, START_SCAN);
    
    return ( events ^ BLE_SCAN_START_EVT );
  }

  if ( events & BLE_SCAN_STOP_EVT )
  {
    LOG("BLE_SCAN_STOP_EVT: \n");

    simpleBLEObserver_HandleCommand(0, STOP_SCAN);

    return ( events ^ BLE_SCAN_STOP_EVT );
  }

  if ( events & WHO_AM_I_EVT )
  {
    LOG("WHO_AM_I_EVT:  sending CHIPID:0x%X and CHVER:%d.\n", CHIPID, CHVER);

    uint8 var[] = {CHIPID, CHVER};
    HalI2CWrite(2, var);
    
    return ( events ^ WHO_AM_I_EVT );
  }

  if ( events & RESET_EVT )
  {
    LOG("RESET_EVT:\n");
    while(HalUARTBusy(NPI_UART_PORT));

    Onboard_soft_reset();
    
    return ( events ^ RESET_EVT );
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
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      // simpleBLEObserver_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
      
    default:
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
static void simpleBLEObserver_HandleCommand( uint8 shift, uint8 cmd )
// static void simpleBLEObserver_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter

  if ( cmd & START_SCAN )
  {
    // Start or stop discovery
    if ( !simpleBLEScanning )
    {
      simpleBLEScanning = TRUE;
      simpleBLEScanRes = 0;
      
      LOG( "Discovering..." );
      LOG( "\n" );
      // LCD_WRITE_STRING( "Discovering...", HAL_LCD_LINE_1 );
      // LCD_WRITE_STRING( "", HAL_LCD_LINE_2 );
      
      GAPObserverRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                      DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                      DEFAULT_DISCOVERY_WHITE_LIST );      
    }
    else
    {
      GAPObserverRole_CancelDiscovery();
    }
  }

  if ( cmd & STOP_SCAN )
  {
    // Display discovery results
    if ( !simpleBLEScanning && simpleBLEScanRes > 0 )
    {
      printf("cmd & STOP_SCAN");
        // Increment index of current result (with wraparound)
        simpleBLEScanIdx++;
        if ( simpleBLEScanIdx >= simpleBLEScanRes )
        {
          simpleBLEScanIdx = 0;
        }
        
        LOG( "Device: %d", simpleBLEScanIdx + 1 );
        LOG( bdAddr2Str( simpleBLEDevList[simpleBLEScanIdx].addr ) );
        // LCD_WRITE_STRING_VALUE( "Device", simpleBLEScanIdx + 1,
        //                         10, HAL_LCD_LINE_1 );
        // LCD_WRITE_STRING( bdAddr2Str( simpleBLEDevList[simpleBLEScanIdx].addr ),
        //                   HAL_LCD_LINE_2 );
    }
  }

  // if ( keys & HAL_KEY_RIGHT )
  // {
  // }
  
  // if ( keys & HAL_KEY_CENTER )
  // {
  // }
  
  // if ( keys & HAL_KEY_DOWN )
  // {
  // }
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
  printf("here\n");
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  
      {
        LOG( "BLE Observer" );
        LOG( bdAddr2Str( pEvent->initDone.devAddr ) );
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
        
        LOG( "Devices Found: %d", simpleBLEScanRes );
        // LCD_WRITE_STRING_VALUE( "Devices Found", simpleBLEScanRes,
        //                         10, HAL_LCD_LINE_1 );
        if ( simpleBLEScanRes > 0 )
        {
          // LCD_WRITE_STRING( "<- To Select", HAL_LCD_LINE_2 );
        }

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

#if (defined HAL_I2C) && (HAL_I2C == TRUE)
uint8 buffer[HAL_I2C_BUF_MAX] = {0};
uint8 I2C_Callback_Func(uint8 cnt, uint8 src)
{
  static uint8 indx = 0;
  // LOG("I2C cnt value is %x\n", cnt);

  HalI2CRead(cnt, buffer);
  // LOG("I2C buffer: ");
  // for(int i = 0; i<cnt; i++)
  // {
  //   LOG("buffer[%d]: %x, ", i, buffer[i]);
  // }
  // LOG("\n");

  if(cnt!=0)
  {
    if(buffer[0] == WHO_AM_I)
    {
      osal_set_event( simpleBLETaskId, WHO_AM_I_EVT );
    }
    else if(buffer[0] == DEV_RESET)
    {
      osal_set_event( simpleBLETaskId, RESET_EVT );
    }
    else if(buffer[0] == BLE_SCAN)
    {
      osal_set_event( simpleBLETaskId, BLE_SCAN_START_EVT );
    }
    else if(buffer[0] == BLE_PRNT)
    {
      osal_set_event( simpleBLETaskId, BLE_SCAN_STOP_EVT );
    }
    return TRUE;
  }
  else{
    // if(src == 0xA8)
    // {
    //   osal_set_event( simpleBLETaskId, I2C_DEBUG_PRINT_1 );
    // }
    // else
    // {
    //   osal_set_event( simpleBLETaskId, I2C_DEBUG_PRINT_2 );
    // }
    return TRUE;
  }

}
#endif // (defined HAL_I2C) && (HAL_I2C == TRUE)

/*********************************************************************
*********************************************************************/
