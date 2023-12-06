/*------------------------------------------------------------------------------
*
* $Workfile:   ModbusParserSensor.h
*
* $Author:   skawalec  $
*
* Copyright (C) 2014 Banner Engineering Corp. All rights reserved.
* Developed by Sensonix Incorporated
*
*------------------------------------------------------------------------------
*/

#ifndef MODBUSPARSERSENSOR_H
#define MODBUSPARSERSENSOR_H

/* PUBLIC METHODS */

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "DeviceStructs.h"
#include "DeviceEnums.h"

// Moddbus Write Logic


//read only
#define MODBUS_DIFF_VALUES_START            ((uint16_t) 42001 )
#define MODBUS_DIFF_VALUES_END              ((uint16_t) ( MODBUS_DIFF_VALUES_START + 128 ) )
#define MODBUS_SUM_VALUES_START            ((uint16_t) 43001 )
#define MODBUS_SUM_VALUES_END              ((uint16_t) ( MODBUS_SUM_VALUES_START + 128 ) )

/*
* \brief Checks fModbusAddress validity.
* \retval False = no problem
* For each entry in fModbusAddress, verifies that start address <= stop
* address.  Failures indicates an overlapping field.
* Depends on NUMBER_OF_MODBUS_INDEX
*/
bool          ModbusParser__CheckForModbusTableError(void);

/*
* \brief Checks message CRC
* \param [in] rpData Pointer to message buffer
* \param [in] rNumberOfBytes Number of bytes in message
* \retval True if message CRC is valid.
*/
bool             ModbusParser__CheckIfMessageIsValid(uint8_t* rpData, uint8_t rNumberOfBytes);

/**
* \brief Parses a modbus message and returns a response in the incoming buffer.
*
* \details Given a message and number of bytes, attempt to parse the message.
*		    If the message is valid, overwrite the incoming message with the response
*			and return the byte count.
*
* \param[in, out]	rpData			Incoming message.  Becomes the output message
* \param[in]		rNumberOfBytes  Length of input message
*
* \return	Byte length of response
* \retval non-zero denotes success
*/
uint8_t          ModbusParser__ProcessModbusMessage(uint8_t* rpData, uint8_t rNumberOfBytes);

/*
* \brief Given a register ID determine it's index in modbus table.
* \param [in] rRegister Modbus register ID
* \retval 0 on fails, non-zero is modbus register index.
*/
uint16_t         ModbusParser__GetModbusTableIndex(uint16_t rRegister);

/*
* \brief
* \param [in, out] rpData buffer containing modbus message.  Advanced or decremented
*        by address length
* \param [in] rInvertPointerDirection pointer to the head of the message buffer.
* \retval Non-zero on success, denotes offset from rpData for the message start.
*/
uint8_t          ModbusParser__ModbusAddressMatch(uint8_t* rpData, bool rInvertPointerDirection);

/*
* \retval Generates a NACK message.
* \param[in, out] rData buffer to build nack message in.
* \param[in] rErrorCode error code to place in NACK message.
* \retval byte count of NACK
*/
uint8_t          ModbusParser__BuildNackResponce(uint8_t* rData, uint8_t rErrorCode);

/*
* \brief Reads and writes Modbus registers from the register table
* \param[in] rAction should be READ|WRITE
* \param[in] rpData pointer to message data
* \param[in] rRegister start register to operate on
* \param[in] rNumberOfRegisters number of registers to operate on.
* \retval TRUE on success FALSE on failure.
*/
bool          ModbusParser__ReadWriteModbusRegisters(ReadWritActionType rAction, uint8_t* rpData, uint16_t rRegister, uint16_t rNumberOfRegisters);

/* VARIABLE DEFINITION */




#endif // MODBUSPARSERSENSOR_H
