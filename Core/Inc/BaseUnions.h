/*
 * BaseUnions.h
 *
 *  Created on: Oct 3, 2023
 *      Author: pbalyeat
 */

#ifndef INC_BASEUNIONS_H_
#define INC_BASEUNIONS_H_

#include <stdint.h>
#include <arm_bf16.h>

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

#endif /* INC_BASEUNIONS_H_ */
