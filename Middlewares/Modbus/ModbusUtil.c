/*
* ModbusUtilities.c
*
 * Copyright (C) 2021 Banner Engineering Corp. All rights reserved.
 * Developed by Sensonix Incorporated
*
 *
* Created: 4/23/2021 10:41:47 AM
*  Author: pbalyeat
*/

#include "ModbusUtil.h"
#include <stdbool.h>

TickType_t ModbusUtil_CalculateLineSilence(ModbusPhy_t media, uint8_t u8BitsPerChar, uint8_t u8Parity, StopBits_t StopBit, uint8_t u8BaudRate)
{
  uint32_t u32BaudRate = comms_common_baudrate_lookup(u8BaudRate);

  // add at least 1 start bit.
  if(ModbusPhy_Serial ==  media)
  {
      u8BitsPerChar++;

      switch(StopBit)
      {
        case Stop_Bits_1_5:
        Assert(false);
        break;

        case Stop_Bits_2:
        u8BitsPerChar += 2;
        break;

        case Stop_Bits_1:
        default:
        u8BitsPerChar++;
        break;
      }
  }
  if(u8Parity != 0)
    u8BitsPerChar++;

  float f32SecondsPerSymbol = (1.0 * u8BitsPerChar ) / (u32BaudRate * 1.0);
  uint32_t u32usPerSymbol = (uint32_t) (1000000.0 * f32SecondsPerSymbol);
  // add 1us for a guard value.  Cover possible math under run.
  u32usPerSymbol+=1;
  // RTU line silence is 3.5 chars
  // should check for rollver values  here.
  uint32_t u32ls = 3 * u32usPerSymbol;
  u32ls += (u32usPerSymbol/ 2);
  u32ls /= 1000;
  // realistically this should clamp at 5mS minimum
  // Just moving this out for safety's sake.
  if(u32ls < 5)
    u32ls = 5;

  return pdMS_TO_TICKS(u32ls);
}

TickType_t ModbusUtil_CalculateTicksFromSymbols(ModbusPhy_t media, uint8_t u8BitsPerChar, uint8_t u8Parity, StopBits_t StopBit, uint8_t u8BaudRate, uint8_t char_cnt)
{
  uint32_t u32BaudRate = comms_common_baudrate_lookup(u8BaudRate);
  // interbyte delay in bits.
  uint32_t u32interbytedelay = 0;
  if(ModbusPhy_Serial ==  media)
  {
    // add at least 1 start bit.
    u8BitsPerChar++;

    switch(StopBit)
    {
      case Stop_Bits_1_5:
      Assert(false);
      break;

      case Stop_Bits_2:
      u8BitsPerChar += 2;
      break;

      case Stop_Bits_1:
      default:
      u8BitsPerChar++;
      break;
    }
    // .5 bit delay between symbols.
    u32interbytedelay = char_cnt;
  }
  if(u8Parity != 0)
    u8BitsPerChar++;

  float f32timperbit = 1.0 / (u32BaudRate * 1.0);
  float f32totaltime = (1.0 * ((u8BitsPerChar * char_cnt)  + u32interbytedelay) ) * f32timperbit ;
  uint32_t u32TotalTime = (uint32_t) (1000000.0 * f32totaltime);
  // add 1us for a guard value.  Cover possible math under run.
  u32TotalTime+=1;

  // capture interbyte delays.
  uint32_t roundcheck = u32TotalTime;
  u32TotalTime /= 1000;
  if( (roundcheck - (u32TotalTime * 1000)  ) >= 500  )
  {
    u32TotalTime++;
  }
  return pdMS_TO_TICKS(u32TotalTime);
}
