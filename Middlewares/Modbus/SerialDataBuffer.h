/*------------------------------------------------------------------------------
 *
 * $Workfile:   SerialDataBuffer.h  $ $Modtime:   Jun 04 2019 10:27:50  $
 * Description: This file contains all the definitions for the serial data buffer.
 *
 * Copyright (C) 2021 Banner Engineering Corp. All rights reserved.
 * Developed by Sensonix Incorporated
 *
 *------------------------------------------------------------------------------
 */



#ifndef SERIALDATABUFFER_H
#define SERIALDATABUFFER_H

#include <stdint.h>
#include "sxi_std/DeviceDefines.h"

// RX buffer size in bytes
#define SERIAL_RX_BUFFER_SIZE             MODBUS_MESSAGE_SIZE_MAX
// TX buffer size in bytes
#define SERIAL_TX_BUFFER_SIZE             MODBUS_MESSAGE_SIZE_MAX

typedef struct GenericSerialTxDef
{
  uint8_t                   CommandCode;
  uint8_t                   DataCode;
  uint8_t                   Data1;
  uint8_t                   Data0;
  uint8_t                   XOrCheck;
  uint8_t                   SerialTxBuffer[SERIAL_TX_BUFFER_SIZE-5];
} GenericSerialTxDefType;

typedef struct GenericSerialTxDef32Bit
{
  uint8_t                   CommandCode;
  uint8_t                   DataCode;
  uint8_t                   Data3;
  uint8_t                   Data2;
  uint8_t                   Data1;
  uint8_t                   Data0;
  uint8_t                   XOrCheck;
  uint8_t                   SerialTxBuffer[SERIAL_RX_BUFFER_SIZE-7];
} GenericSerialTxDef32BitType;

typedef struct GenericSerialRxDef
{
  uint8_t                   CommandCode;
  uint8_t                   DataCode;
  uint8_t                   Data1;
  uint8_t                   Data0;
  uint8_t                   XOrCheck;
  uint8_t                   SerialRxBuffer[SERIAL_RX_BUFFER_SIZE-5];
} GenericSerialRxDefType;

typedef union SerialTxBuffer_u
{
  GenericSerialTxDefType      GenericSerialTxDef;
  GenericSerialTxDef32BitType GenericSerialTxDef32Bit;
  uint8_t           SerialTxBuffer[SERIAL_TX_BUFFER_SIZE];
} SerialTxBufferType;

typedef union SerialRxBuffer_u 
{
  GenericSerialRxDefType GenericSerialRxDef;
  uint8_t               SerialRxBuffer[SERIAL_RX_BUFFER_SIZE];
} SerialRxBufferType;

#endif /* SERIALDATABUFFER_H */

