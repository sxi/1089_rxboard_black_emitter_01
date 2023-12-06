/*
*****************************************************************************
*
* $Workfile:   ModbusParser.c  $ $Modtime:   01 Jun 2020 16:19:20  $
*
*
* Description:
*
* Copyright (C) 2012 Banner Engineering Corp. All rights reserved.
* Developed by Sensonix Incorporated
*
*****************************************************************************
*/

#include <stdlib.h>
#include <stdbool.h>
#include "CompilerTypeDefs.h"
#include "DeviceDefines.h"
#include "DeviceEnums.h"
#include "DeviceStructs.h"
#include "CRC16.h"
#include "ModbusParserSensor.h"
#include "ModbusTableDefines.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "serialtask.h"

extern SemaphoreHandle_t sema_configHandle;
extern TaskHandle_t imager_taskHandle;

extern const ModbusLookUpType fModbusAddress[];
extern const uint8_t EnableModbusBroadcast;
extern const uint8_t myModbusAddress;
extern const uint16_t NUMBER_OF_MODBUS_INDEX;

#define               MODBUS_REGISTER_START_ADDRESS   40000
///* PRIVATE METHODS */

/*
* \brief Handles Modbus read and write messages.
* \param[in] rAction Action to take.
* \param[in, out] rpData On reads, the output buffer. On writes, the input buffer.
* \param[in, out] rpNumberOfRegistersByteCount Number of registers to read/write
* Gets overwritten with byte count of buffer.
* \brief [in] rNumberOfRegisters Number of registers to read/write.
* \brief [in] rRegisterOffset Used for reading non-16 bit register values.
* \brief [in] rParameterReservedRegisterSize size of register to read.
* \brief [in] rEepromAddress EEPROM address (in shadowram) to read/write.
* \brief [in,out] rpRamPointer ?
* \brief [in] rParameterSize ?
* \brief [in] rBitMask to read or write only masked bits of a value.
* \retval Pointer to response.
*/
static uint8_t*         ModbusParser__ReadWriteAction(ReadWritActionType rAction,
uint8_t* rpData,
uint8_t* rpNumberOfRegistersByteCount,
uint16_t rNumberOfRegisters,
uint8_t rRegisterOffset,
uint8_t rParameterReservedRegisterSize,
uint16_t rEepromAddress,
uint8_t* rpRamPointer,
uint8_t rParameterSize,
uint8_t rBitMask);

/*
* \brief Does extra actions after modbus reads/writes.
* \param [in] rAction Read or write action
* \param [in] rRegister To operate on
* \param [in] rNumberOfRegisters number of registers to operate on.
* \param [in] rNewNumberOfRegisters optional?
*/
static void           ModbusParser__CheckExtraRegisterActions(ReadWritActionType rAction,
uint16_t rRegister,
uint16_t rNumberOfRegisters,
uint16_t rNewNumberOfRegisters);


bool ModbusParser__CheckForModbusTableError(void)
{
  volatile uint16_t _Index          = 0;
  volatile bool  _BadFlashDefine = FALSE;

  while((_Index < NUMBER_OF_MODBUS_INDEX) && (!_BadFlashDefine))
  {
    // Make sure ModbusStartAddress is less than or equal to ModbusStopAddress
    // If not _BadFlashDefine = TRUE
    if(fModbusAddress[_Index].ModbusStartAddress > fModbusAddress[_Index].ModbusStopAddress)
    {
      _BadFlashDefine = TRUE;
    }

    // Check next index
    if((_Index+1) < NUMBER_OF_MODBUS_INDEX)
    {
      // Make sure defines do not overlap
      if(fModbusAddress[_Index].ModbusStopAddress >= fModbusAddress[(_Index+1)].ModbusStartAddress)
      {
        _BadFlashDefine = TRUE;
      }
    }

    // Increment index
    _Index++;
  }

  return _BadFlashDefine;
}

bool ModbusParser__CheckIfMessageIsValid(uint8_t* rpData, uint8_t rNumberOfBytes)
{
  volatile bool          _ValidCrc = FALSE;
  WordToByteType _CRC;

  _CRC.Word = CRC16b(rpData, (rNumberOfBytes - MODBUS_CRC_SIZE));

  rpData += (rNumberOfBytes - MODBUS_CRC_SIZE);

  if(*rpData == _CRC.LowByte)
  {
    rpData++;

    if(*rpData == _CRC.HighByte)
    {
      _ValidCrc = TRUE;
    }
  }

  return _ValidCrc;
}

uint16_t ModbusParser__GetModbusTableIndex(uint16_t rRegister)
{
  uint16_t _LowIndex  = 1; // Index 0 is used for undefined modbus values
  uint16_t _HighIndex = NUMBER_OF_MODBUS_INDEX;
  // MidIndex is initialized to _LowIndex + ((_HighIndex - _LowIndex) >> 1)
  uint16_t _MidIndex  = (1 + (( NUMBER_OF_MODBUS_INDEX - 1) / 2) );

  ModbusLookUpType *_fpModbusAddress;
  ModbusLookUpType _ModbusAddress;

  _fpModbusAddress = (ModbusLookUpType *) &fModbusAddress[_MidIndex];

  while(_LowIndex < _HighIndex)
  {
    _ModbusAddress = *_fpModbusAddress;

    if(_ModbusAddress.ModbusStartAddress  > rRegister)
    {
      _HighIndex = _MidIndex;
    }
    else if(_ModbusAddress.ModbusStopAddress < rRegister)
    {
      _LowIndex = (_MidIndex + 1);
    }
    else
    {
      return(_MidIndex);
    }
    _MidIndex = _LowIndex + ((_HighIndex - _LowIndex) >> 1);

    _fpModbusAddress = (ModbusLookUpType *) &fModbusAddress[_MidIndex];
  }
  return 0;
}

uint8_t ModbusParser__ProcessModbusMessage(uint8_t* rpData, uint8_t rNumberOfBytes)
{
  WordToByteType     _RegisterValue;
  WordToByteType     _NumberOfRegisters ;
  uint8_t              _SizeOfModbusAddress = STANDARD_MODBUS_ADDRESS_SIZE;
  uint8_t              _ByteCount           = 0;
  uint8_t              _ResponceByteCount   = 0;

  // Initialize values
  _RegisterValue.Word = 0;
  _NumberOfRegisters.Word = 0;

  // Check of extended modbus addressing and moves pointer to the funcion pointer;
  if(*rpData == EXTENTED_DEVICE_NUMBER_MODBUS_ADDRESS)
  {
    // Add Serial number check here
    _SizeOfModbusAddress = DEVICE_NUMBER_MODBUS_ADDRESS_SIZE;

    // Move to Function code
    rpData += DEVICE_NUMBER_MODBUS_ADDRESS_SIZE;
  }
  else if(*rpData == MASTER_PING_MODBUS_ADDRESS)
  {
    // Add Serial number check here
    _SizeOfModbusAddress = MASTER_PING_MODBUS_ADDRESS_SIZE;

    // Move to Function code
    rpData += MASTER_PING_MODBUS_ADDRESS_SIZE;
  }
  else if(*rpData == MUX_WITH_32BIT_LABEL_MODBUS_ADDRESS)
  {
    // Add Serial number check here
    _SizeOfModbusAddress = MUX_WITH_32BIT_LABEL_MODBUS_ADDRESS_SIZE;

    // Move to Function code
    rpData += MUX_WITH_32BIT_LABEL_MODBUS_ADDRESS_SIZE;
  }
  else if(*rpData == SERIAL_NUMBER_MODBUS_ADDRESS)
  {
    // Add Serial number check here
    _SizeOfModbusAddress = SERIAL_NUMBER_MODBUS_ADDRESS_SIZE;

    // Move to Function code
    rpData += SERIAL_NUMBER_MODBUS_ADDRESS_SIZE;
  }
  else
  {
    // Move to Function code
    rpData += STANDARD_MODBUS_ADDRESS_SIZE;
  }

  // Check Function Code
  switch(*rpData)
  {
    case MODBUS_READ_MULTIPLE_REGISTERS:
    // Check data size
    if(rNumberOfBytes == (READ_MULTIPLE_REGISTERS_DATA_SIZE_IN + _SizeOfModbusAddress))
    {
      // Get starting address
      rpData++;
      _RegisterValue.HighByte = *rpData;
      rpData++;
      _RegisterValue.LowByte = *rpData;

      // Get number of registers
      rpData++;
      _NumberOfRegisters.HighByte = *rpData;
      rpData++;
      _NumberOfRegisters.LowByte = *rpData;

      // Check if NumberOfRegisters is within spec
      if(_NumberOfRegisters.Word > MODBUS_READ_MULTIPLE_MAX_REGISTERS)
      {
        _NumberOfRegisters.Word = 0;
      }

      // Set byte count
      _ByteCount = (uint8_t)(_NumberOfRegisters.Word * MODBUS_REGISTER_SIZE);

      // Check if there is a byte count
      if(_ByteCount)
      {
        // Make sure that the response is with in the MODBUS_MESSAGE_SIZE_MAX
        // Equation: _SizeOfModbusAddress + READ_MULTIPLE_REGISTERS_DATA_SIZE_OUT + _ByteCount <= MODBUS_MESSAGE_SIZE_MAX
        //          Simplify by moving all constants to one side:
        //          _SizeOfModbusAddress + _ByteCount <= MODBUS_MESSAGE_SIZE_MAX - READ_MULTIPLE_REGISTERS_DATA_SIZE_OUT
        if((_SizeOfModbusAddress + _ByteCount) <= (MODBUS_MESSAGE_SIZE_MAX - READ_MULTIPLE_REGISTERS_DATA_SIZE_OUT))
        {
          // Move pointer back to byte after Function Code Ready to populate the byte count
          rpData -= (sizeof(_RegisterValue) + sizeof(_NumberOfRegisters) - 1);

          // Set byte count
          *rpData = _ByteCount;

          // Move pointer to byte after Byte Count
          rpData++;

          // If ModbusParser__ReadWriteModbusRegisters is successful then set _ResponceByteCount for a response
          if(ModbusParser__ReadWriteModbusRegisters(READ, rpData, _RegisterValue.Word, _NumberOfRegisters.Word))
          {
            // Calculate the total number of bytes in the responce message including the CRC
            _ResponceByteCount = _SizeOfModbusAddress + _ByteCount + READ_MULTIPLE_REGISTERS_DATA_SIZE_OUT;

          }

          // Move pointer back to beginning of message
          // The pointer is currntly at the byte after the Function Code (Byte Count)
          rpData -= (_SizeOfModbusAddress + (READ_MULTIPLE_REGISTERS_DATA_SIZE_OUT - MODBUS_CRC_SIZE) );
        }
      }
    }
    break;

    case MODBUS_WRITE_SINGLE_REGISTER:
    // Check data size
    if(rNumberOfBytes == (WRITE_SINGLE_REGISTER_DATA_SIZE_IN_OUT + _SizeOfModbusAddress))
    {
      // Get starting address
      rpData++;
      _RegisterValue.HighByte = *rpData;
      rpData++;
      _RegisterValue.LowByte = *rpData;

      // Move pointer to Register Value
      rpData++;

      // If ModbusParser__ReadWriteModbusRegisters is successful then set _ResponceByteCount for a responce
      if(ModbusParser__ReadWriteModbusRegisters(WRITE, rpData, _RegisterValue.Word, 1))
      {
        // Calculate the total number of bytes in the responce message including the CRC
        _ResponceByteCount = _SizeOfModbusAddress  + WRITE_SINGLE_REGISTER_DATA_SIZE_IN_OUT;
      }

      // Move pointer back to beginning of message
      // The pointer is currntly at the first Register Value
      rpData -= (_SizeOfModbusAddress + (WRITE_SINGLE_REGISTER_DATA_SIZE_IN_OUT - MODBUS_CRC_SIZE - sizeof(_RegisterValue)) );
    }
    break;

    case MODBUS_WRITE_MULTIPLE_REGISTERS:
    // Check Data size in
    // MODBUS_WRITE_MULTIPLE_PROCESS_TIME_FROM_BYTES is need for processing big messages
    // Only allow messages minus MODBUS_WRITE_MULTIPLE_PROCESS_TIME_FROM_BYTES
    //   if (rNumberOfBytes <= (MAXIMUM_DATA_PACKET_IN_FRAMELET_SIZE*DATA_FRAMELETS_PER_SLOT - MODBUS_WRITE_MULTIPLE_PROCESS_TIME_FROM_BYTES))
    {
      // Get starting address
      rpData++;
      _RegisterValue.HighByte = *rpData;
      rpData++;
      _RegisterValue.LowByte = *rpData;

      // Get number of registers
      rpData++;
      _NumberOfRegisters.HighByte = *rpData;
      rpData++;
      _NumberOfRegisters.LowByte = *rpData;

      // Check if NumberOfRegisters is within spec
      if(_NumberOfRegisters.Word > MODBUS_WRITE_MULTIPLE_MAX_REGISTERS)
      {
        _NumberOfRegisters.Word = 0;
      }

      // Set byte count
      _ByteCount = (uint8_t)(_NumberOfRegisters.Word * MODBUS_REGISTER_SIZE);

      // move pointer to Byte Count
      rpData++;

      // Check data size and register size definition is good
      if((rNumberOfBytes == (WRITE_MULTIPLE_REGISTERS_DATA_SIZE_IN + _SizeOfModbusAddress + _ByteCount)) &&
      ((_ByteCount) && (_ByteCount == *rpData)))
      {
        // Move pointer to the beginning of the Register Values
        rpData++;

        // If ModbusParser__ReadWriteModbusRegisters is successful then set _ResponceByteCount for a responce
        if(ModbusParser__ReadWriteModbusRegisters(WRITE, rpData, _RegisterValue.Word, _NumberOfRegisters.Word))
        {
          // Calculate the total number of bytes in the responce message including the CRC
          _ResponceByteCount = _SizeOfModbusAddress + WRITE_MULTIPLE_REGISTERS_DATA_SIZE_OUT;
        }

        // Move pointer back to beginning of message
        // The pointer is currntly at the beginning of the Register Values
        rpData -= (_SizeOfModbusAddress + (WRITE_MULTIPLE_REGISTERS_DATA_SIZE_IN - MODBUS_CRC_SIZE) );
      }
    }
    break;
  }

  if(_ResponceByteCount)
  {
    WordToByteType _CRC;

    _CRC.Word = CRC16b(rpData, (_ResponceByteCount - MODBUS_CRC_SIZE));

    rpData += (_ResponceByteCount - MODBUS_CRC_SIZE);

    *rpData = _CRC.LowByte;

    rpData++;

    *rpData = _CRC.HighByte;
  }

  return _ResponceByteCount;
}

uint8_t ModbusParser__ModbusAddressMatch(uint8_t* rpData, bool rInvertPointerDirection)
{
  uint8_t _AddressPointerOffset = 0;
  //  uint8_t _LocalModbusAddress = /*enter modbus address here*/;
  uint8_t _LocalModbusAddress = myModbusAddress;

  // Set default modbus address to 1
  if (_LocalModbusAddress == 0)
  {
    _LocalModbusAddress = 1;
  }

  if(*rpData == EXTENTED_DEVICE_NUMBER_MODBUS_ADDRESS)
  {
    WordToByteType _DeviceAddress;
    WordToByteType _SN_LoWord;
    _SN_LoWord.LowByte = 0;
    _SN_LoWord.HighByte = 0;

    if(rInvertPointerDirection)
    {
      rpData--;
    }
    else
    {
      rpData++;
    }
    _DeviceAddress.HighByte = *rpData;

    if(rInvertPointerDirection)
    {
      rpData--;
    }
    else
    {
      rpData++;
    }
    _DeviceAddress.LowByte  = *rpData;

    //    if (_DeviceAddress.Word == /*enter device address here*/) Extended part
    if(_DeviceAddress.Word == _SN_LoWord.Word)
    {
      _AddressPointerOffset = DEVICE_NUMBER_MODBUS_ADDRESS_SIZE;
    }
  }
  else if(*rpData == MASTER_PING_MODBUS_ADDRESS)
  {
    if(rInvertPointerDirection)
    {
      rpData--;
    }
    else
    {
      rpData++;
    }

  }
  else if(*rpData == MUX_WITH_32BIT_LABEL_MODBUS_ADDRESS)
  {
    LongToWordToByteType _MuxLabel;

    // Ignor first two bytes for Mux address and channel
    if(rInvertPointerDirection)
    {
      rpData--;
      rpData--;
    }
    else
    {
      rpData++;
      rpData++;
    }

    if(rInvertPointerDirection)
    {
      rpData--;
    }
    else
    {
      rpData++;
    }
    _MuxLabel.HighWord.HighByte = *rpData;

    if(rInvertPointerDirection)
    {
      rpData--;
    }
    else
    {
      rpData++;
    }
    _MuxLabel.HighWord.LowByte  = *rpData;

    if(rInvertPointerDirection)
    {
      rpData--;
    }
    else
    {
      rpData++;
    }
    _MuxLabel.LowWord.HighByte = *rpData;

    if(rInvertPointerDirection)
    {
      rpData--;
    }
    else
    {
      rpData++;
    }
    _MuxLabel.LowWord.LowByte  = *rpData;

  }
  else if ( (*rpData == BROADCAST_MODBUS_ADDRESS)                                                      &&
  (1 == EnableModbusBroadcast) )
  {
    _AddressPointerOffset = STANDARD_MODBUS_ADDRESS_SIZE;
  }
  #if defined(MODBUS_ACTION_REGISTER_ENABLED)
  else if ( (ShadowRamData.ConfigurableParameters.UartParameters.ActionModBusAddress)            &&
  (*rpData == ShadowRamData.ConfigurableParameters.UartParameters.ActionModBusAddress) )
  {
    if (MicrocontrollerSensor__ActionModBusAddressCheck())
    {
      _AddressPointerOffset = STANDARD_MODBUS_ADDRESS_SIZE;
    }
  }
  #endif
  else if(*rpData == SERIAL_NUMBER_MODBUS_ADDRESS)
  {
    LongToWordToByteType _SerialNumber;

    if(rInvertPointerDirection)
    {
      rpData--;
    }
    else
    {
      rpData++;
    }
    _SerialNumber.HighWord.HighByte = *rpData;

    if(rInvertPointerDirection)
    {
      rpData--;
    }
    else
    {
      rpData++;
    }
    _SerialNumber.HighWord.LowByte  = *rpData;

    if(rInvertPointerDirection)
    {
      rpData--;
    }
    else
    {
      rpData++;
    }
    _SerialNumber.LowWord.HighByte = *rpData;

    if(rInvertPointerDirection)
    {
      rpData--;
    }
    else
    {
      rpData++;
    }
    _SerialNumber.LowWord.LowByte  = *rpData;

    LongToWordToByteType _SerialNumberShadowRam;
    _SerialNumberShadowRam.LowWord.LowByte   = 3;
    _SerialNumberShadowRam.LowWord.HighByte  = 2;
    _SerialNumberShadowRam.HighWord.LowByte  = 1;
    _SerialNumberShadowRam.HighWord.HighByte = 0;

    if(_SerialNumber.uLong == _SerialNumberShadowRam.uLong)
    {
      _AddressPointerOffset = SERIAL_NUMBER_MODBUS_ADDRESS_SIZE;
    }
  }
  // Make sure _LocalModbusAddress is not 0
  else if(_LocalModbusAddress)
  {
    if(*rpData == _LocalModbusAddress)
    {
      _AddressPointerOffset = STANDARD_MODBUS_ADDRESS_SIZE;
    }
  }

  // Check that Fuction Code is supported
  {
    if(rInvertPointerDirection)
    {
      rpData--;
    }
    else
    {
      rpData++;
    }

    switch (*rpData)
    {
      case MODBUS_READ_MULTIPLE_REGISTERS:
      case MODBUS_WRITE_SINGLE_REGISTER:
      case MODBUS_WRITE_MULTIPLE_REGISTERS:
      {
        // Keep existing _AddressPointerOffset
      }
      break;

      default:
      {
        _AddressPointerOffset = 0;
      }
    }
  }

  return _AddressPointerOffset;
}

uint8_t ModbusParser__BuildNackResponce(uint8_t* rpData, uint8_t rErrorCode)
{
  WordToByteType _CRC;
  uint8_t          _NackResponceSize = MODBUS_NACK_DATA_SIZE_OUT;

  // Check of extended modbus addressing and moves pointer to the funcion pointer;
  if(*rpData == EXTENTED_DEVICE_NUMBER_MODBUS_ADDRESS)
  {
    // Move to Function code
    rpData += DEVICE_NUMBER_MODBUS_ADDRESS_SIZE;
    _NackResponceSize += DEVICE_NUMBER_MODBUS_ADDRESS_SIZE;
  }
  else if(*rpData == MASTER_PING_MODBUS_ADDRESS)
  {
    // Move to Function code
    rpData += MASTER_PING_MODBUS_ADDRESS_SIZE;
    _NackResponceSize += MASTER_PING_MODBUS_ADDRESS_SIZE;
  }
  else if(*rpData == MUX_WITH_32BIT_LABEL_MODBUS_ADDRESS)
  {
    // Move to Function code
    rpData += MUX_WITH_32BIT_LABEL_MODBUS_ADDRESS_SIZE;
    _NackResponceSize += MUX_WITH_32BIT_LABEL_MODBUS_ADDRESS_SIZE;
  }
  else if(*rpData == SERIAL_NUMBER_MODBUS_ADDRESS)
  {
    // Move to Function code
    rpData += SERIAL_NUMBER_MODBUS_ADDRESS_SIZE;
    _NackResponceSize += SERIAL_NUMBER_MODBUS_ADDRESS_SIZE;
  }
  else
  {
    // Move to Function code
    rpData += STANDARD_MODBUS_ADDRESS_SIZE;
    _NackResponceSize += STANDARD_MODBUS_ADDRESS_SIZE;
  }

  // Set MSB of the function Code
  *rpData |= MODBUS_NACK_BIT_MASK;
  rpData++;

  // Populate Error Code
  *rpData = rErrorCode;
  rpData++;

  // Move data pointer back to start
  rpData -= (_NackResponceSize - MODBUS_CRC_SIZE);

  // Calulate CRC16
  _CRC.Word = CRC16b(rpData, (_NackResponceSize - MODBUS_CRC_SIZE));

  // Move data pointer to CRC
  rpData += (_NackResponceSize - MODBUS_CRC_SIZE);

  // Set CRC
  *rpData = _CRC.LowByte;
  rpData++;
  *rpData = _CRC.HighByte;

  return _NackResponceSize;
}

bool ModbusParser__ReadWriteModbusRegisters(ReadWritActionType rAction, uint8_t* rpData, uint16_t rRegister, uint16_t rNumberOfRegisters)
{
  uint16_t             _EepromAddress;
  uint8_t*             _pRamPointer;
  uint8_t              _ParameterSize;
  uint8_t              _RegisterOffset;
  uint8_t              _ParameterReservedRegisterSize;
  uint8_t              _NumberOfRegistersByteCount;
  uint16_t             _Index;
  uint16_t             _NextIndex;
  uint8_t              _BitMask;
  bool              _FindFirstModbusIndex = TRUE;
  ReadWritActionType _Action;

  uint16_t _Register = 0;
  if(rRegister >= MODBUS_REGISTER_START_ADDRESS)
  {
    _Register = rRegister;
  }
  else
  {
    _Register = (rRegister + MODBUS_REGISTER_START_ADDRESS);
  }


  while(rNumberOfRegisters)
  {
    // Set up what to read or write

    // Find the first modbus index
    if(_FindFirstModbusIndex)
    {
      // Get index from ModbusTable
      _Index     = ModbusParser__GetModbusTableIndex(_Register);
      _NextIndex = _Index+1;
    }
    else
    {
      // If the first index has been found the look at the next index and
      // check if it is in the next entry and for index overflow
      if((_Register >= fModbusAddress[_NextIndex].ModbusStartAddress) &&
      (_Register <= fModbusAddress[_NextIndex].ModbusStopAddress)  &&
      (_NextIndex < (NUMBER_OF_MODBUS_INDEX)))
      {
        _Index = _NextIndex;
        _NextIndex++;
      }
      else
      {
        _Index = 0;
      }
    }

    // Index is not 0 then the modbus register is defined
    if(0 != _Index)
    {
      _RegisterOffset       = (_Register - fModbusAddress[_Index].ModbusStartAddress);
      _FindFirstModbusIndex = FALSE;
    }
    // If the modbus register is not defined then set _RegisterOffset to 0
    else
    {
      _RegisterOffset = 0;
    }
    _ParameterReservedRegisterSize = ((fModbusAddress[_Index].ModbusStopAddress - fModbusAddress[_Index].ModbusStartAddress) + 1);
    _EepromAddress                 = fModbusAddress[_Index].EepromAddress;
    _pRamPointer                   = fModbusAddress[_Index].RamPointer;
    _BitMask                       = fModbusAddress[_Index].BitMask;
    // If Bitfield is enabled then _ParameterSize MUST be equal to 1 byte
    if(0 != _BitMask)
    {
      _ParameterSize = 1;
    }
    else
    {
      _ParameterSize               = fModbusAddress[_Index].ParameterSize;
    }

    // Check for READ_ONLY Modbus Table definiton
    if((rAction == WRITE) && (_EepromAddress == READ_ONLY))
    {
      _Action = NULL_ACTION;
    }
    else
    {
      // Define Action
      _Action = rAction;
    }

    // Read Ram or Write Eeprom while taking care of modbus offsets and parameter size conditions

	  rpData = ModbusParser__ReadWriteAction(_Action,
	  rpData,
	  &_NumberOfRegistersByteCount,
	  rNumberOfRegisters,
	  _RegisterOffset,
	  _ParameterReservedRegisterSize,
	  _EepromAddress,
	  _pRamPointer,
	  _ParameterSize,
	  _BitMask);


    // Decrement _Register by the number of registers handled above
    // Define new register count
    uint16_t _NewNumberOfRegisters = (((uint16_t)(_NumberOfRegistersByteCount)) / MODBUS_REGISTER_SIZE);

    ModbusParser__CheckExtraRegisterActions(_Action, _Register, rNumberOfRegisters, _NewNumberOfRegisters);

    // Move _Register location
    _Register += (rNumberOfRegisters - _NewNumberOfRegisters);

    // Set rNumberOfRegisters to _NewNumberOfRegisters
    rNumberOfRegisters = _NewNumberOfRegisters;
  }

  return TRUE;
}

static uint8_t* ModbusParser__ReadWriteAction(ReadWritActionType rAction,
uint8_t* rpData,
uint8_t* rpNumberOfRegistersByteCount,
uint16_t rNumberOfRegisters,
uint8_t rRegisterOffset,
uint8_t rParameterReservedRegisterSize,
uint16_t rEepromAddress,
uint8_t* rpRamPointer,
uint8_t rParameterSize,
uint8_t rBitMask)
{
  // Need both semaphores here.  May be non-contiguous reads/writes.

  xSemaphoreTake(sema_configHandle, pdMS_TO_TICKS(2));

  // Take a snapshot of the start of rpRamPointer and _ParameterSize
  // because it is used later for writing the eeprom
  uint8_t* _pRamPointerStart   = rpRamPointer;
  uint8_t  _ParameterSizeStart = rParameterSize;

  // Initialize Parameters return
  uint8_t  _NumberOfRegistersByteCount = *rpNumberOfRegistersByteCount;

  // Set up rpRamPointer so it points to one byte past the most significant
  // byte to fix the "end to end" mode from the message to the register.
  rpRamPointer += rParameterSize;

  // Make rParameterReservedRegisterSize equal to the total number of bytes reserved in the modbus table
  rParameterReservedRegisterSize *= MODBUS_REGISTER_SIZE;

  // Make _NumberOfRegistersByteCount equal to the total number of bytes to read or write
  _NumberOfRegistersByteCount = (uint8_t)(rNumberOfRegisters * MODBUS_REGISTER_SIZE);

  // Take care of register offset (used if reading 16-bit fraction values bigger than 16-bit )
  // AND
  // Take care of register error if rParameterReservedRegisterSize is less than rParameterSize
  // this will take only the lower bytes of the parameter to be read or written.
  while((rRegisterOffset) || (rParameterReservedRegisterSize < rParameterSize))
  {
    // Only take one byte off if value does not fit in 16-bit increments
    if(rParameterSize | 0x01)
    {
      // If the rParameterReservedRegisterSize minus the unused upper byte is equal to or less than
      // the rParameterSize adjust the Ram pointer to cut off the upper byte
      if((rParameterReservedRegisterSize-1) <= rParameterSize)
      {
        rParameterSize--;
        rpRamPointer--;
      }
    }
    // Take out Top two bytes
    else if(rParameterSize >= MODBUS_REGISTER_SIZE)
    {
      // If the rParameterReservedRegisterSize is equal to or less than the rParameterSize
      // adjust the Ram pointer to cut off the upper two bytes
      if(rParameterReservedRegisterSize <= rParameterSize)
      {
        rParameterSize -= MODBUS_REGISTER_SIZE;
        rpRamPointer   -= MODBUS_REGISTER_SIZE;
      }
    }

    // Decrement rRegisterOffset
    if(rRegisterOffset)
    {
      rRegisterOffset--;

      // Decrement rParameterReservedRegisterSize
      if(rParameterReservedRegisterSize >= MODBUS_REGISTER_SIZE)
      {
        rParameterReservedRegisterSize -= MODBUS_REGISTER_SIZE;
      }
    }
  }

  // If rParameterReservedRegisterSize is ZERO this is a ERROR case
  // that may be due to rParameterReservedRegisterSize NOT matching
  // the number of Modbus Registers defined for a parameter.
  // To recover from this set rParameterReservedRegisterSize to
  // MODBUS_REGISTER_SIZE and Clear rParameterSize
  // NOTE: The Modbus Registers will be offset by the error
  if(!rParameterReservedRegisterSize)
  {
    rParameterReservedRegisterSize = MODBUS_REGISTER_SIZE;
    rParameterSize                 = 0;
  }

  // Fill bytes with 0 where the rParameterSize does not fill in rParameterReservedRegisterSize
  // NOTE: rParameterReservedRegisterSize must have a value if it does not exit while
  while((rParameterSize < rParameterReservedRegisterSize) &&
        (rParameterReservedRegisterSize) &&
        (_NumberOfRegistersByteCount))
  {
    // If bytes don't match fill responce with 0x00 if rAction is READ and increment rpData
    if(rAction == READ)
    {
      *rpData = 0;
    }
    rpData++;

    // Decrement _NumberOfRegistersByteCount and rParameterReservedRegisterSize
    _NumberOfRegistersByteCount--;
    rParameterReservedRegisterSize--;
  }

  // Fill rpData OR rpRamPointer
  // Block interrupts to avoid Ram corruption - Semaphor should cover you.
  // Check rBitMask to only read or write bits of bytes
  // For safty make sure rParameterReservedRegisterSize and _NumberOfRegistersByteCount are no-zero
  if((rBitMask) && (rParameterReservedRegisterSize) && (_NumberOfRegistersByteCount))
  {
    rpRamPointer--;

    uint8_t _ShiftedBitMask = rBitMask;
    uint8_t _BitShiftCount  = 0;
    uint8_t _Value          = 0;

    // Move bit mask so the smallest bottom bit is shifted to the LSB
    while(!(_ShiftedBitMask & 0x01))
    {
      _ShiftedBitMask >>= 1;
      _BitShiftCount++;
    }

    if(rAction == READ)
    {
      // Populate _Value with RamPointer Value
      _Value = *rpRamPointer;
      // Mask-out the bits defined
      _Value &= rBitMask;
      // Shift the bits Down
      _Value >>= _BitShiftCount;

      // Write value to buffer
      *rpData = _Value;
    }
    else if(rAction == WRITE)
    {
      // Populate _Value with buffer Value
      _Value = *rpData;
      // Mask-out bits defined
      _Value &= _ShiftedBitMask;
      // Shift the bits Up
      _Value <<= _BitShiftCount;
      // Clear the rBitMask bits of the RamPointer Value
      *rpRamPointer &= ~(rBitMask);
      // Write the rBitMask bits of the RamPointer Value to the new Value
      *rpRamPointer |= _Value;
    }

    rpData++;

    // Decrement _NumberOfRegistersByteCount and rParameterReservedRegisterSize
    _NumberOfRegistersByteCount--;
    rParameterReservedRegisterSize--;
  }
  else
  {
    while((rParameterReservedRegisterSize) && (_NumberOfRegistersByteCount))
    {
      // Decrement Ram pointer
      // Check if action is a READ or WRITE
      // If READ poplulate rpData for the modbus responce
      // If WRITE poplulate rpRamPointer with data
      // Then increment rpData
      rpRamPointer--;
      if(rAction == READ)
      {
        *rpData = *rpRamPointer;
      }
      else if(rAction == WRITE)
      {
        *rpRamPointer = *rpData;
      }
      rpData++;

      // Decrement _NumberOfRegistersByteCount and rParameterReservedRegisterSize
      _NumberOfRegistersByteCount--;
      rParameterReservedRegisterSize--;
    }
  }
  xSemaphoreGive(sema_configHandle);

  // Update return varables to pointer return
  *rpNumberOfRegistersByteCount = _NumberOfRegistersByteCount;
  // Return pointer location
  return rpData;
}

void platform_common_ModbusRegisterReadAction(uint16_t rRegister,
		uint16_t rNumberOfRegisters, uint16_t rNewNumberOfRegisters)
{

}

void platform_common_ModbusRegisterWriteAction(uint16_t rRegister,
		uint16_t rNumberOfRegisters, uint16_t rNewNumberOfRegisters)
{
	uint32_t holder = 0;
	// need to hit ref calc first.
	if ((rRegister <= REF_KICK)
			&& ((rRegister + rNumberOfRegisters) >= REF_KICK))
	{
		xTaskNotify(imager_taskHandle, (uint32_t ) CMD_REF_REMOVAL,
				eSetValueWithOverwrite);
		holder = ulTaskNotifyTake(0, pdMS_TO_TICKS(150));
	}
	if ((rRegister <= READ_KICK)
			&& ((rRegister + rNumberOfRegisters) >= READ_KICK))
	{

		xTaskNotify(imager_taskHandle, (uint32_t ) CMD_LINEAR_ARRAY_WALK,
				eSetValueWithOverwrite);
		holder = ulTaskNotifyTake(0, pdMS_TO_TICKS(150));
	}
}

static void ModbusParser__CheckExtraRegisterActions(ReadWritActionType rAction,
		uint16_t rRegister, uint16_t rNumberOfRegisters,
		uint16_t rNewNumberOfRegisters)
{
	if (rAction == WRITE)
	{
		platform_common_ModbusRegisterWriteAction(rRegister, rNumberOfRegisters,
				rNewNumberOfRegisters);

		/* Example ----------------
		 // Check mOutputModbusData
		 if ( (rRegister >= MODBUS_OUTPUT_REGISTER_START)                      &&
		 (rRegister < (MODBUS_OUTPUT_REGISTER_START + NUMBER_OF_OUTPUTS)) )
		 {
		 // Initialize _SetBits to the number of modbus registers written
		 // in a bit field form
		 // Example: 4 register is 0x000F
		 uint16_t _SetBits = ((1 << (rNumberOfRegisters - rNewNumberOfRegisters)) - 1);

		 // Move bits to by MODBUS_OUTPUT_REGISTER_START offset
		 _SetBits = (_SetBits << (rRegister - MODBUS_OUTPUT_REGISTER_START));

		 // Set Service Output Bits (mServiceOutput)
		 InputAndOutputControl__SetServiceOutputBits(_SetBits);
		 }
		 -----------------------------*/
	}
	else if (rAction == READ)
	{
		platform_common_ModbusRegisterReadAction(rRegister, rNumberOfRegisters,
				rNewNumberOfRegisters);
	}
}

