/*------------------------------------------------------------------------------
 *
 * $Workfile:   DeviceEnums.h  $ $Modtime:   Feb 19 2018 16:03:50  $
 *
 * Copyright (C) 2021 Banner Engineering Corp. All rights reserved.
 * Developed by Sensonix Incorporated
 *
 *------------------------------------------------------------------------------
 */



#ifndef DEVICE_ENUMS_H_
#define DEVICE_ENUMS_H_

/////////////////////////////////
// Device Specific Enumeration //
/////////////////////////////////

typedef enum ReadWritAction
{
  NULL_ACTION = 0,
  READ           ,
  WRITE          ,
} ReadWritActionType;

typedef enum WaitSleepMode_e
{
  WAIT_SLEEP_MODE__STANDARD  = 0,
  WAIT_SLEEP_MODE__ACTIVE       ,
  WAIT_SLEEP_MODE__IDLE         ,
  WAIT_SLEEP_MODE__BACKUP       ,
} WaitSleepModeType;

typedef enum MuxAddressIndex
{
  MUX_HEADER_INDEX__ADDRRESS                = 0,
  MUX_HEADER_INDEX__MUX_MODBUS_ADDRESS         ,
  MUX_HEADER_INDEX__MUX_MODBUS_CHANNEL         ,
  MUX_HEADER_INDEX__32BIT_LABEL_HIHI           ,
  MUX_HEADER_INDEX__32BIT_LABEL_HILO           ,
  MUX_HEADER_INDEX__32BIT_LABEL_LOHI           ,
  MUX_HEADER_INDEX__32BIT_LABEL_LOLO           ,
  MUX_WITH_32BIT_LABEL_MODBUS_ADDRESS_SIZE     ,
} MuxAddressIndexType;

#endif
