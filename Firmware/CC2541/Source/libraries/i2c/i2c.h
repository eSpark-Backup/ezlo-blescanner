#ifndef I2C_H
#define I2C_H

#include "defines.h"

/***********************************************************************************
* CONSTANTS
*/

// Define size of buffer and number of bytes to send
#define BUFFER_SIZE 0xFF

// Unique address range according to I2C protocol
// 0x08 - 0x77 (8 to 119)
#define SLAVE_ADDRESS 0x77    // 7-bit addressing

#define READ_FROM_SLAVE 0x01
#define WRITE_TO_SLAVE 0x00

// // Commands issued from master/slave
// enum
// {
//     SLAVE_INIT = 0x01,
//     SCAN_BLE_ADV_PACKETS,
// }I2C_COMMANDS_e;


/***********************************************************************************
* LOCAL VARIABLES
*/

static uint8 slaveAddr = SLAVE_ADDRESS;     // Will be adjusted by master
static uint8 buffer[BUFFER_SIZE];
static uint8 bufferIndex = 0;

/***********************************************************************************
* LOCAL FUNCTIONS
*/

void i2c_init();
void i2c_ProcessEvent( uint8 task_id, uint16 events );

#endif // I2C_H