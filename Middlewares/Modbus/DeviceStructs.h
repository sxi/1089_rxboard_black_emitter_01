/*
 *****************************************************************************
 *
 * $Workfile:   DeviceStructs.h  $ $Modtime:   Mar 02 2015 11:24:06  $
 * Description:
 *
 * Copyright (C) 2012 Banner Engineering Corp. All rights reserved.
 * Developed by Sensonix Incorporated
 *
 *****************************************************************************
*/

#ifndef DEVICESTRUCTS_H
#define DEVICESTRUCTS_H

#include <stdint.h>
#include <arm_bf16.h>
// #include <arm_math.h>

typedef union ByteToNybble
{
  struct
  {
    uint8_t   LowNybble  :4;
    uint8_t   HighNybble :4;
  };
  uint8_t   Byte;
} ByteToNybbleType;

typedef union WordToByte
{
  struct
  {
    uint8_t   LowByte  ;
    uint8_t   HighByte ;
  };
  struct
  {
    ByteToNybbleType   LowByteDetail;
    ByteToNybbleType   HighByteDetail;
  };
  uint16_t   Word;
  int16_t    WordSigned;
} WordToByteType;

typedef union LongToWordToByte
{
  struct
  {
    WordToByteType   LowWord  ;
    WordToByteType   HighWord ;
  };
  uint32_t   uLong;
  struct
  {
    uint32_t LowerThreeBytes   :24;
    uint32_t Unused            :8;
  };
  struct
  {
    uint8_t  LowerThreeByteArray[3];
    uint8_t  UnusedArray;
  };
  uint8_t    Array[4];
  float32_t    Float;
} LongToWordToByteType;

typedef struct ModbusLookUp
{
  uint16_t ModbusStartAddress;
  uint16_t ModbusStopAddress;
  uint16_t EepromAddress;
  uint8_t  BitMask;          // NOTE: Used for 8 bit only
  uint8_t  ParameterSize;
  uint8_t* RamPointer;
} ModbusLookUpType;

typedef union SleepOrIdleBitField
{
  struct
  {
    uint8_t   SerialEnable           :1;
    uint8_t   VibrationReadEnable    :1;
    uint8_t   ProgramDevice          :1;
    uint8_t   UnusedBits             :5;
  };
  uint8_t   Byte;
} SleepOrIdleBitFieldType;
#endif
