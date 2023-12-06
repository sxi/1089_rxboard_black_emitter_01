/*
 * ModbusUtilities.h
 *
  * Copyright (C) 2021 Banner Engineering Corp. All rights reserved.
  * Developed by Sensonix Incorporated
  *
 * Created: 4/23/2021 10:41:05 AM
 *  Author: pbalyeat
 */


#ifndef MODBUSUTIL_H_
#define MODBUSUTIL_H_
#include <stdint.h>
#include "FreeRTOS.h"

/*
 * \enum ModbusPhy_t modbus connection type
 */
typedef enum ModbusPhy {
  ModbusPhy_Serial = 0,
  ModbusPhy_TCP,
} ModbusPhy_t;

/*
 * \enum StopBits_t number of serial stop bits. 
 */
typedef enum StopBits_e {
  Stop_Bits_1 = 0,
  Stop_Bits_1_5,
  Stop_Bits_2,
} StopBits_t;


/*
 * \brief Calculates Modbus line silence
 * \param [in] media physical layer type.
 * \param [in] u8BitsPerChar number of bits per char
 * \param [in] u8Parity parity settings from eep.
 * \param [in] u8StopBitCount number of stop bits
 * \param [in] u8BaudRate baudrate eep setting.
 * \retval Modbus timeout in Ticks.
 */
extern TickType_t ModbusUtil_CalculateLineSilence(ModbusPhy_t media, uint8_t u8BitsPerChar, uint8_t u8Parity, StopBits_t StopBit, uint8_t u8BaudRate);

/*
 * \brief Calculates Ticks to send a number of modbus symbols
 * \param [in] media physical layer type.
 * \param [in] u8BitsPerChar number of bits per char
 * \param [in] u8Parity parity settings from eep.
 * \param [in] u8StopBitCount number of stop bits
 * \param [in] u8BaudRate baudrate eep setting.
 * \param [in] char_cnt Number of characters to send.
 * \retval Number of Ticks to send a number of symbols.
 */
extern TickType_t ModbusUtil_CalculateTicksFromSymbols(ModbusPhy_t media, uint8_t u8BitsPerChar, uint8_t u8Parity, StopBits_t StopBit, uint8_t u8BaudRate, uint8_t char_cnt);
/*
 * \brief Looks up EEPROM values and returns baud rate
 * \param [in] value EEPROM value
 * \param [in] periph 0 = uart, 1 = usart
 * \retval Returns baud rate for a peripheral
 */
extern uint32_t ModbusUtil_BaudRateLookup(uint8_t value, uint8_t periph);

/*
 * \brief Looks up EEPROM values and returns parity
 * \param [in] value EEPROM value
 * \param [in] periph 0 = uart, 1 = usart
 * \retval Returns parity for a peripheral
 */
extern uint32_t ModbusUtil_ParityLookup(uint8_t value, uint8_t periph);


#endif /* MODBUSUTILITIES_H_ */
