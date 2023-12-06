/*
 * serialtask.h
 *
 *  Created on: Aug 21, 2023
 *      Author: pbalyeat
 */

#ifndef SRC_SERIALTASK_H_
#define SRC_SERIALTASK_H_

#include <stdint.h>
#include "main.h"
#include "FreeRTOS.h"
#include "timers.h"

// Length of serial buffer
#define  SERIAL_BUFFER_SIZE  ( (uint16_t)  520)

#define ARRAY_WALK_HEADER ((uint8_t) 4)

typedef enum {
	CMD_NONE = 0,
	CMD_GET_VERSION = 0x01,
	CMD_WRITE_SINGLE_PARM = 0x02,
	CMD_READ_SINGLE_PARM = 0x03,
	CMD_LINEAR_ARRAY_WALK = 0x04,
	CMD_REF_REMOVAL	= 0x05,
#ifdef HAS_DAC_OUT
	CMD_LINEAR_ARRAY_WALK_DAC = 0xFF,
#endif
} cmd_enum_t;

/*
 * \brief Primary serial task loop
 * \param [in] argument ignored.
 */
void serial_task_func(void *argument);

/*
 * \brief Callback for line silence timer
 * \param [in] argument ignored.
 */
void line_silence_cb(TimerHandle_t argument);

/*
 * \brief user handler for lpuart irq
 * \param [in] caller lpuart calling this function.
 */
void user_secondary_usart_handler(USART_TypeDef * caller);

/*
 * \brief user handler for usart irq
 * \param [in] caller usart calling this function
 */
void user_primary_usart_handler(USART_TypeDef * caller);


#endif /* SRC_SERIALTASK_H_ */
