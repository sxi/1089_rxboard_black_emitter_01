/*
 * serialtask.c
 *
 *  Created on: Aug 21, 2023
 *      Author: pbalyeat
 */

#include "serialtask.h"
#include "1089E.h"
#include "main.h"
#include <string.h>
#include "ModbusParserSensor.h"
#include "DeviceStructs.h"
#include "defaults.h"
#include "FreeRTOS.h"
#include "semphr.h"

#ifdef HAS_DAC_OUT
extern config_and_arrays_dac_t current_config;
#else
extern config_and_arrays_t current_config;
#endif

extern uint16_t interleaved_adc_samples[];
extern uint16_t SUM_REF;
extern uint16_t DIFF_REF;
extern uint16_t ref_kick;

extern uint8_t generic_buffer[];
extern pixel_t g_pixel_cfg;
extern SemaphoreHandle_t sema_configHandle;
extern TaskHandle_t serial_taskHandle;
extern TaskHandle_t imager_taskHandle;
extern TimerHandle_t line_silence_timerHandle;

static const uint8_t ACK = 0x01;
static const uint8_t NACK = 0x02;

typedef enum {
	None,
	PRIMARY,
	SECONDARY,
}comms_source_t;

static comms_source_t interface;
/*
 * \brief dumps bytes out the UART
 * \param [in] byte_count Number of bytes to print
 * \param [in] buffer buffer to print
 * \param [in] interface to print
 */
static void print_driver(uint16_t byte_count, uint8_t * buffer, comms_source_t ci);

#ifdef HAS_DAC_OUT

/*
 * \brief Given a single parameter read/write verify it's accurate
 * \param [in] parm parameter to validate
 * \retval 0 == success, non-zero failure.
 */
static uint8_t validate_single_param_dac(uint8_t parm);

/*
 * \brief Gets a single config parameter from global config section
 * \param [in] cfg source for config data.
 * \param [in] param to get. Must be a valid config address
 * \param [in|out] out output buffer
 * \retval returns byte count.
 */
static uint16_t get_single_param_dac(config_and_arrays_dac_t * cfg , uint8_t parm, uint8_t * out);

/*
 * \brief sets a single config parameter in global config section
 * \param [in] param to set.  Must be a valid config address
 */
static void set_single_param_dac(config_and_arrays_dac_t * cfg, uint8_t parm, uint16_t value);

#else

/*
 * \brief Given a single parameter read/write verify it's accurate
 * \param [in] parm parameter to validate
 * \retval 0 == success, non-zero failure.
 */
static uint8_t validate_single_param(uint8_t parm);

/*
 * \brief Gets a single config parameter from global config section
 * \param [in] cfg source for config data.
 * \param [in] param to get. Must be a valid config address
 * \param [in|out] out output buffer
 * \retval returns byte count.
 */
static uint16_t get_single_param(config_and_arrays_t * cfg , uint8_t parm, uint8_t * out);

/*
 * \brief sets a single config parameter in global config section
 * \param [in] param to set.  Must be a valid config address
 */
static void set_single_param(config_and_arrays_t * cfg, uint8_t parm, uint16_t value);

#endif

/*
 * \brief loads a linear array walk packet in to config structs
 * \param [in] buffer inbound inear array walk message.
 * \param [in|out] pixelcfg pixel configuration
 * \param [in|out] sensorcfg fixed sensor parameters
 * \param [in|out] array0cfg array 0 data
 * \param [in|out] array1cfg array 1 data.
 */
static void parse_linear_array_walk(uint8_t * buffer,
			pixel_t * pixelcfg,
			all_config_u * sensorcfg,
			array_u * array0cfg,
			array_u * array1cfg);
/*
 * \brief compose a response to linear array walk command.
 * \param [in] header message header
 * \param [in] adc interleaved adc data
 * \param [in | out ] buffer buffer to store response
 * \retval number of bytes to send
 */
static uint16_t compose_linear_array_walk_rx(uint8_t * header,
		uint16_t * adc,
		uint8_t * buffer);

/*
 * \brief composes the response to reference removal command
 * \param [in] resolution adc resolution
 * \param [in] diff diff average
 * \param [in] sum sum average
 * \param [in |out ] buffer rx buffer
 * \retval number of bytes to send.
 */
static uint16_t compose_ref_removal_rx(uint8_t resolution,
		uint16_t diff,
		uint16_t sum,
		uint8_t *buffer);

/*
 * \brief processes serial buffer for linear imager serial protocol
 * \param [in| out] buffer Buffer contains input, and output command.
 * \retval  0 on failure, Number of bytes to print on success.
 */
static uint16_t process_linear(uint8_t * buffer);;

// number of bytes rx'd, number of bytes to tx
static uint8_t  serial_byte_count = 0;

// buffer for serial RX/tx shared with serial IRQ and serial task.
uint8_t serial_buffer[SERIAL_BUFFER_SIZE];

// Board specific defines.
static USART_TypeDef * PRIMARY_USART	= USART1;
static IRQn_Type	PRIMARY_US_IRQN = USART1_IRQn;

static USART_TypeDef * SECONDARY_USART  = USART2; // USART1;
static IRQn_Type	SECONDARY_US_IRQN = USART2_IRQn;


/* USER CODE END Header_SerialTaskFunc */
void serial_task_func(void *argument)
{
	/* USER CODE BEGIN SerialTaskFunc */
	NVIC_DisableIRQ(PRIMARY_US_IRQN);
	if(SECONDARY_USART != NULL)
	{
		NVIC_DisableIRQ(SECONDARY_US_IRQN);
	}

	if (&serial_buffer[0]
			!= memset(serial_buffer, 0x00, (size_t) SERIAL_BUFFER_SIZE))
	{
		while (1)
		{
		}
	}

	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	interface = None;
	BaseType_t ret = pdFALSE;
	uint32_t task_notif = 0;
	uint8_t scrap = 0;

	LL_GPIO_SetOutputPin(BB_POL_GPIO_Port, BB_POL_Pin);
	LL_GPIO_SetOutputPin(PRI_EN_GPIO_Port, PRI_EN_Pin);
	if(SECONDARY_USART != NULL)
	{
		LL_GPIO_ResetOutputPin(SEC_EN_GPIO_Port, SEC_EN_Pin);
	}
	/* Infinite loop */
	for (;;)
	{


		LL_USART_DisableDirectionTx(PRIMARY_USART);
		LL_GPIO_ResetOutputPin(PRI_EN_GPIO_Port, PRI_EN_Pin);
		// Turning on USART may leave a junk byte in the RX buffer
		while(LL_USART_IsActiveFlag_RXNE(PRIMARY_USART) == 1)
		{
			scrap = LL_USART_ReceiveData8(PRIMARY_USART);
		}
		LL_USART_EnableDirectionRx(PRIMARY_USART);
		LL_USART_EnableIT_RXNE(PRIMARY_USART);
		NVIC_EnableIRQ(PRIMARY_US_IRQN);

		if(NULL != SECONDARY_USART)
		{
			LL_USART_DisableDirectionTx(SECONDARY_USART);
			LL_GPIO_ResetOutputPin(SEC_EN_GPIO_Port, SEC_EN_Pin);
			while(LL_USART_IsActiveFlag_RXNE(PRIMARY_USART) == 1)
			{
				scrap = LL_USART_ReceiveData8(PRIMARY_USART);
			}
			LL_USART_EnableDirectionRx(SECONDARY_USART);
			LL_USART_EnableIT_RXNE(SECONDARY_USART);
			NVIC_EnableIRQ(SECONDARY_US_IRQN);
		}

		task_notif = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if (serial_byte_count != 0)
		{
			uint16_t ret = ModbusParser__ProcessModbusMessage( &(serial_buffer[0]),
					serial_byte_count);
			if(0 == ret)
			{
				ret = process_linear(&(serial_buffer[0]));
			}

			if(ret == 0)
			{
				ret = 1;
				serial_buffer[0] = NACK;
			}
			print_driver(ret, &(serial_buffer[0]), interface);
		}
		serial_buffer[0] = 0;
		serial_byte_count = 0;
		interface = None;
	}
	/* USER CODE END SerialTaskFunc */
}

void line_silence_cb(TimerHandle_t argument)
{
	LL_USART_DisableIT_RXNE(PRIMARY_USART);
	if(NULL != SECONDARY_USART)
		LL_USART_DisableIT_RXNE(SECONDARY_USART);
	xTaskNotify(serial_taskHandle, 1UL, eSetValueWithOverwrite);
}
void user_secondary_usart_handler(USART_TypeDef * caller)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (LL_USART_IsActiveFlag_RXNE(caller) == 1)
	{
		LL_USART_DisableIT_RXNE(caller);
		serial_buffer[serial_byte_count++] = LL_USART_ReceiveData8(caller);
		if (1 == serial_byte_count)
		{
			LL_USART_DisableIT_RXNE(PRIMARY_USART);
			LL_GPIO_SetOutputPin(PRI_EN_GPIO_Port, PRI_EN_Pin);

			interface = SECONDARY;
			xTimerStartFromISR(line_silence_timerHandle,
					&xHigherPriorityTaskWoken);
		}
		else if (serial_byte_count < SERIAL_BUFFER_SIZE)
		{
			xTimerResetFromISR(line_silence_timerHandle,
					&xHigherPriorityTaskWoken);
		}
		LL_USART_EnableIT_RXNE(caller);
	}
}

void user_primary_usart_handler(USART_TypeDef * caller)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (LL_USART_IsActiveFlag_RXNE(caller) == 1)
	{
		LL_USART_DisableIT_RXNE(caller);
		LL_GPIO_SetOutputPin(SEC_EN_GPIO_Port, SEC_EN_Pin);
		serial_buffer[serial_byte_count++] = LL_USART_ReceiveData8(caller);
		if (1 == serial_byte_count)
		{
			if(NULL != SECONDARY_USART)
				LL_USART_DisableIT_RXNE(SECONDARY_USART);

			interface = PRIMARY;
			xTimerStartFromISR(line_silence_timerHandle,
					&xHigherPriorityTaskWoken);
		}
		else if (serial_byte_count < SERIAL_BUFFER_SIZE)
		{
			xTimerResetFromISR(line_silence_timerHandle,
					&xHigherPriorityTaskWoken);
		}
		LL_USART_EnableIT_RXNE(caller);
	}
}


static void print_driver(uint16_t byte_count, uint8_t * buffer, comms_source_t ci)
{
	// by the time we get here, any non-used interface is disabled.
	bool good2tx = false;
	USART_TypeDef * active = NULL;
	if(PRIMARY == interface)
	{
		NVIC_DisableIRQ(PRIMARY_US_IRQN);
		LL_USART_DisableDirectionRx(PRIMARY_USART);
		LL_USART_EnableDirectionTx(PRIMARY_USART);
		LL_USART_ClearFlag_TC(PRIMARY_USART);
		active = PRIMARY_USART;
		good2tx = true;
	}
	else if( (NULL != SECONDARY_USART ) &&
			 (SECONDARY == interface) )
	{
		NVIC_DisableIRQ(SECONDARY_US_IRQN);
		LL_USART_DisableDirectionRx(SECONDARY_USART);
		LL_USART_EnableDirectionTx(SECONDARY_USART);
		LL_USART_ClearFlag_TC(SECONDARY_USART);
		active = SECONDARY_USART;
		good2tx = true;
	}

	if(true == good2tx )
	{
		uint8_t clear = 0;
		for(uint16_t i = 0 ; i < byte_count; i++)
		{
			LL_USART_TransmitData8(active, buffer[i]);
			while( 0 == LL_USART_IsActiveFlag_TXE(active) ) {}
			while( 0 == LL_USART_IsActiveFlag_TC(active) ) {}
			LL_USART_ClearFlag_TC(active);
			clear = LL_USART_ReceiveData8(active);
		}
	}
}

#ifdef HAS_DAC_OUT
static uint8_t validate_single_param_dac(uint8_t parm)
{

	return ( ( ( parm == TIA_C) ||
				( parm == TIA_S) ||
				( parm == SUM) ||
				( parm == DIFF) ||
				( parm == REIS) ||
				( parm == REIC) ||
				( parm == DAC_PARM) ) ? 0 : 1);
}

static uint16_t get_single_param_dac( config_and_arrays_dac_t * cfg , uint8_t parm, uint8_t * out)
{
	uint16_t retval = 0;
	if(DAC_PARM == parm)
	{
		retval = 2;
		WordToByteType wtb = {0};
		wtb.Word = cfg->dac;
		out[0] = wtb.LowByte;
		out[1] = wtb.HighByte;
	}
	else
	{
		retval = 1;
		out[0] = cfg->cfg.raw[parm / 2];
	}
	return retval;
}

static void set_single_param_dac (config_and_arrays_dac_t * cfg, uint8_t parm, uint16_t value)
{
	if(DAC_PARM == parm)
	{
		cfg->dac = value;
	}
	else
	{
		cfg->cfg.raw[parm / 2] = (uint8_t) value;
	}

}

#else

static uint8_t validate_single_param(uint8_t parm)
{

	return ( ( ( parm == TIA_C) ||
				( parm == TIA_S) ||
				( parm == SUM) ||
				( parm == DIFF) ||
				( parm == REIS) ||
				( parm == REIC) ) ? 0 : 1);
}

static uint16_t get_single_param( config_and_arrays_t * cfg , uint8_t parm, uint8_t * out)
{
	uint16_t retval = 1;
	out[0] = NACK;
	if(parm <= REIC)
	{
		out[0] = cfg->cfg.raw[parm / 2];
	}
	return retval;
}

static void set_single_param(config_and_arrays_t * cfg, uint8_t parm, uint16_t value)
{
	cfg->cfg.raw[parm / 2] = (uint8_t) value;
}

#endif

static void parse_linear_array_walk(uint8_t * buffer,
			pixel_t * pixelcfg,
			all_config_u * sensorcfg,
			array_u * array0cfg,
			array_u * array1cfg)
{
	uint8_t index = 1;
	pixelcfg->width = buffer[index++];
	pixelcfg->iterations = buffer[index++];
	pixelcfg->start_array = buffer[index++];
	pixelcfg->start_location = buffer[index++];

 	sensorcfg->bits.tia_c.raw =  buffer[index++];
	sensorcfg->bits.tia_s.raw =  buffer[index++];
	sensorcfg->bits.sum.raw =  buffer[index++];
	sensorcfg->bits.diff.raw = buffer[index++];
	sensorcfg->bits.reis.value =  buffer[index++];
	sensorcfg->bits.reic.value =  buffer[index++];

	for(uint8_t i = 0; i < sizeof(array_u); i++ )
	{
		array0cfg->value[i] = buffer[index++];
	}
	for(uint8_t i = 0; i < sizeof(array_u); i++ )
	{
		array1cfg->value[i] = buffer[index++];
	}
}

static uint16_t compose_linear_array_walk_rx(uint8_t * header,
		uint16_t * adc,
		uint8_t * buffer)
{
	uint16_t retval = 0;
	for(uint8_t i = 0; i < ARRAY_WALK_HEADER ; i++)
	{
		buffer[retval++] = header[i];
	}
	for(uint16_t ctr = 0; ctr < header[3] * 4; ctr++)
	{
		buffer[retval++] = ((uint8_t *)adc)[ctr];
	}
	return retval;
}

static uint16_t compose_ref_removal_rx(uint8_t resolution,
		uint16_t diff,
		uint16_t sum,
		uint8_t * dest)
{
	WordToByteType wtb = {0};
	uint16_t retval = 0;
	dest[retval++] = 0;
	dest[retval++] = resolution;

	wtb.Word = diff;
	dest[retval++] = wtb.HighByte;
	dest[retval++] = wtb.LowByte;

	wtb.Word = sum;
	dest[retval++] = wtb.HighByte;
	dest[retval++] = wtb.LowByte;
	return retval;
}

static uint16_t process_linear(uint8_t * buffer)
{
	uint16_t retval = 0;
	BaseType_t ret = pdFALSE;
	uint32_t task_notif = 0;
	switch (buffer[0])
	{
		case (uint8_t) CMD_GET_VERSION:
		{
			buffer[0] = version_buffer[0];
			buffer[1] = version_buffer[1];
			buffer[2] = version_buffer[2];
			retval = 3;
		}
		break;

		case (uint8_t) CMD_WRITE_SINGLE_PARM:
		{
			if( (serial_byte_count >= 3 ) &&
			#ifdef HAS_DAC_OUT
				(0 == validate_single_param_dac(buffer[1])) )
			#else
				(0 == validate_single_param(serial_buffer[1])) )
			#endif  
			{
				ret = xSemaphoreTake(sema_configHandle, pdMS_TO_TICKS(50));
				#ifdef HAS_DAC_OUT
				set_single_param_dac(&current_config,
						buffer[1],
						buffer[2]);
				#else
				set_single_param(&current_config,
						buffer[1],
						buffer[2]);
				#endif
				xSemaphoreGive(sema_configHandle);
				xTaskNotify(imager_taskHandle,
						(uint32_t) CMD_WRITE_SINGLE_PARM,
						eSetValueWithOverwrite);
				task_notif = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(150));
				buffer[0] = ACK;
				retval = 1;
			}
		}
		break;

		case (uint8_t) CMD_READ_SINGLE_PARM:
		{
			if( (serial_byte_count >= 2 ) &&
			#ifdef HAS_DAC_OUT
				(0 == validate_single_param_dac(buffer[1])) )
			#else
				(0 == validate_single_param(serial_buffer[1])) )
			#endif
			{
				ret = xSemaphoreTake(sema_configHandle, pdMS_TO_TICKS(50));
				#ifdef HAS_DAC_OUT
					retval = get_single_param_dac(&current_config, buffer[1] , &(buffer[0] ));
				#else
					retval = get_single_param(&current_config, buffer[1] , &(buffer[0] ));
				#endif
				xSemaphoreGive(sema_configHandle);
			}
		}
		break;

		case (uint8_t) CMD_LINEAR_ARRAY_WALK:
		{
			if(serial_byte_count >=  (uint16_t) ( sizeof(pixel_t) +
												  sizeof(all_config_u) +
												  2 * sizeof(array_u ) +1 ) )
			{
				// validate pixel_t

				ret = xSemaphoreTake(sema_configHandle, pdMS_TO_TICKS(50));
				parse_linear_array_walk(buffer,
						&g_pixel_cfg,
						&current_config.cfg,
						&current_config.array_0,
						&current_config.array_1);
				xSemaphoreGive(sema_configHandle);
				xTaskNotify(imager_taskHandle,
						(uint32_t) CMD_LINEAR_ARRAY_WALK,
						eSetValueWithOverwrite);
				task_notif = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(150));
				if(1 == task_notif)
				{
					retval = compose_linear_array_walk_rx( &(generic_buffer[0]),
							&(interleaved_adc_samples[0]),
							&(buffer[0]));
					memset(&(interleaved_adc_samples[0]),
							0x00,
							ADC_SAMPLE_COUNT * sizeof(uint16_t));
					memset(&(generic_buffer[0]), 0x00, 4);
				}
			}
		}
		break;

		case (uint8_t) CMD_REF_REMOVAL:
		{
			if(buffer[1] <= 2)
			{
				// 0 out reference
				if(0 == buffer[1])
				{
					SUM_REF = 0;
					DIFF_REF = 0;
					buffer[0] = ACK;
					retval = 1;
				}
				else {
					ret = xSemaphoreTake(sema_configHandle, pdMS_TO_TICKS(50));
					ref_kick = buffer[1];
					xSemaphoreGive(sema_configHandle);
					xTaskNotify(imager_taskHandle,
							(uint32_t) CMD_REF_REMOVAL,
							eSetValueWithOverwrite);
					task_notif = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(150));
					if(1 == task_notif)
					{
						ret = xSemaphoreTake(sema_configHandle, pdMS_TO_TICKS(50));
						retval = compose_ref_removal_rx( generic_buffer[0],
								DIFF_REF,
								SUM_REF,
								&(buffer[0]));
						xSemaphoreGive(sema_configHandle);
					}

				}
			}
			else
			{
				buffer[0] = NACK;
				retval =1;
			}
		}
		break;

#ifdef HAS_DAC_OUT
		case (uint8_t) CMD_LINEAR_ARRAY_WALK_DAC:
		{

		}
		break;
#endif

		default :
		{
			retval = 1;
			buffer[0] = NACK;
			break;
		}
	}
	return retval;
}
