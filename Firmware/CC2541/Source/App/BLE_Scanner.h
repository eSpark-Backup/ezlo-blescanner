/******************************************************************************

 @file  simpleBLEObserver.h

 @brief This file contains the Simple BLE Observer sample application
        definitions and prototypes.

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

#ifndef SIMPLEBLEOBSERVER_H
#define SIMPLEBLEOBSERVER_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

#if (defined HAL_I2C) && (HAL_I2C == TRUE)
#include "hal_i2c.h"
#endif

/*********************************************************************
 * CONSTANTS
 */

// 15 events can be registerred, SYS_EVENT_MSG(0x8000) is already used
// Simple BLE Observer Task Events
#define START_DEVICE_EVT                              0x0001
#define START_DISCOVERY_EVT                           0x0002
#define WHO_AM_I_EVT                                  0x0004
#define RESET_EVT                                     0x0008
 
#define BLE_REPORT_EVT                                0x0010
#define BLE_SCAN_START_EVT                            0x0020
#define BLE_SCAN_STOP_EVT                             0x0040

#define I2C_DEBUG_PRINT_1                             0x1000
#define I2C_DEBUG_PRINT_2                             0x2000

#define START_SCAN                                      0x01
#define STOP_SCAN                                       0x08

// I2C registers/commands
#define WHO_AM_I                                        0xEE
#define DEV_RESET                                       0x50
#define BLE_SCAN                                        0x12
#define BLE_PRNT                                        0x22

/*********************************************************************
 * MACROS
 */

// LCD macros
#if HAL_LCD == TRUE
#define LCD_WRITE_STRING(str, option)                       HalLcdWriteString( (str), (option))
#define LCD_WRITE_SCREEN(line1, line2)                      HalLcdWriteScreen( (line1), (line2) )
#define LCD_WRITE_STRING_VALUE(title, value, format, line)  HalLcdWriteStringValue( (title), (value), (format), (line) )
#else
#define LCD_WRITE_STRING(str, option)                     
#define LCD_WRITE_SCREEN(line1, line2)                    
#define LCD_WRITE_STRING_VALUE(title, value, format, line)
#endif


#if ( DEBUG_MSGS ) && ( DEBUG_MSGS == TRUE)
#define LOG(...)       printf(__VA_ARGS__)
#else
#define LOG(...)
#endif

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void SimpleBLEObserver_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 SimpleBLEObserver_ProcessEvent( uint8 task_id, uint16 events );


#if (defined HAL_I2C) && (HAL_I2C == TRUE)
extern uint8 I2C_Callback_Func(uint8 cnt, uint8 src);
#endif

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLEOBSERVER_H */
