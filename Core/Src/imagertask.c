/*
 * imagertask.c
 *
 *  Created on: Aug 22, 2023
 *      Author: pbalyeat
 */

#include "imagertask.h"
#include <string.h>
#include "1089E.h"
#include "main.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <stddef.h>
#include "serialtask.h"
#include "BaseUnions.h"
#include "defaults.h"

#ifdef HAS_DAC_OUT
	extern config_and_arrays_dac_t current_config;
#else
	extern config_and_arrays_t current_config;
#endif

extern SemaphoreHandle_t sema_configHandle;
extern TaskHandle_t serial_taskHandle;
extern uint16_t interleaved_adc_samples[];
extern uint8_t generic_buffer[];
extern pixel_t g_pixel_cfg;
extern uint16_t imager_count;
extern uint16_t DIFF_REF;
extern uint16_t SUM_REF;
extern uint16_t ref_kick;

/*
 * \brief Formats analog data and header after array walk
 * \param [in] t_dbg  debug monitor
 * \param [in] analogs current analog settings
 * Uses globals generic_buffer, g_pixel_cfg, interleaved_adc_samples.
 */
static void compose_array_walk_response(timer_dbg_t * t_dbg,
		analog_values_interleaved_t analogs);

/*
 * \brief Handles reference level removal.
 * \param [in] cmd 0,1,2 CMD to to execute
 * \param [in] handle 1089 pin collection
 * \param [in] dma comms dma interface.
 * \param [in] tmr 1us timer peripheral
 * \param [in|out] analogs read and stores analog values.
 * \param [in] final_pins final pin state after all writes are done
 */
static void imager_calculate_reference_values(uint16_t cmd,
		spike_t * handle,
		spike_dma_t *dma,
		timer_1_us_t * tmr,
		analog_values_interleaved_t *analogs,
		mission_pin_state_t * final_pins);

/*
 * \brief Reads ADC, returns resolution
 * \param [in] adc ADC
 * \retval ADC resolution (in bits) as a uint8_t
 */
static uint8_t get_ADC_resolution(ADC_TypeDef * adc);

void imager_task_func(void *argument)
{

	LL_Pin miso =
	{ .gpio = GPIOA,
			.pin = LL_GPIO_PIN_11 };
	LL_Pin sck =
	{ .gpio = GPIOA,
			.pin = LL_GPIO_PIN_1 };
	LL_Pin ncs =
	{ .gpio = SoftCS_GPIO_Port,
			.pin = SoftCS_Pin };
	LL_Pin dth_sdi =
	{ .gpio = GPIOA,
			.pin = LL_GPIO_PIN_12 };
	LL_Pin orb =
	{ .gpio = ORB_GPIO_Port,
			.pin = ORB_Pin };
	LL_Pin oth =
	{ .gpio = OTH_GPIO_Port,
			.pin = OTH_Pin };
	LL_Pin led_pulse =
	{ .gpio = LED_Pulse_GPIO_Port,
			.pin = LED_Pulse_Pin };
	spike_t handle =
			{ .tbm_sck = sck,
					.dth_sdi = dth_sdi,
					.ncs = ncs,
					.npor_sdo = miso,
					.spike = SPI1,
					.orb = orb,
					.oth = oth,
					.LED_out = led_pulse,
					.LL_AF = LL_GPIO_AF_5,
					.LL_BAUDRATEPRESCALER =
					LL_SPI_BAUDRATEPRESCALER_DIV4, };
	spike_dma_t dma =
	{ .dma = DMA1,
			.LL_rx_channel = LL_DMA_CHANNEL_2,
			.LL_rx_extra = LL_DMA_REQUEST_1,
			.LL_tx_channel = LL_DMA_CHANNEL_3,
			.LL_tx_extra =
			LL_DMA_REQUEST_1, };
	timer_1_us_t tmr =
	{ .timer = TIM2,
			.ll_channel = LL_TIM_CHANNEL_CH1 };

	analog_values_interleaved_t analogs = {
			.adc = ADC1,
			.interleaved = &(interleaved_adc_samples[0]),
	};
#ifdef HAS_DAC_OUT

#endif

//	LL_Pin gpio_tmr = {
//			.gpio = SamplingActive_GPIO_Port,
//			.pin = SamplingActive_Pin
//	};
//	timer_dbg_t t_dbg = {
//			.timer = TIM6,
//			.setup_time_us = 0,
//			.run_time_us = 0,
//			.dbg_pin = gpio_tmr,
//			.SumSample = sum_sample,
//			.DiffSample = diff_sample
//	};


	LL_TIM_InitTypeDef TIM_InitStruct =
	{ 0 };
	TIM_InitStruct.Prescaler = 79;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	LL_TIM_Init(tmr.timer, &TIM_InitStruct);

	current_config.cfg.bits = default_1089;
	current_config.array_0.large[0] = 0;
	current_config.array_0.large[1] = 0;
	current_config.array_0.large[2] = 0;
	current_config.array_0.large[3] = 0;

	current_config.array_1.large[0] = 0xFFFFFFFFUL;
	current_config.array_1.large[1] = 0xFFFFFFFFUL;
	current_config.array_1.large[2] = 0xFFFFFFFFUL;
	current_config.array_1.large[3] = 0xFFFFFFFFUL;

	g_pixel_cfg.iterations = 128;
	g_pixel_cfg.start_array = 1;
	g_pixel_cfg.start_location = 0;
	g_pixel_cfg.width = 1;

	driver_init(&handle);

	LL_GPIO_SetOutputPin(SoftCS_GPIO_Port, SoftCS_Pin);
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
	mission_pin_state_t mps =
	{ .dth = 0,
			.orb = 0,
			.tbm = 1,
			.oth = 1 };

	while(0 != write_all_config_1089(&handle, &(current_config.cfg), 1ul) ) {}
	while(0 != read_all_config_1089(&handle, &(current_config.cfg), 1ul) ) {}

	xTaskNotify(serial_taskHandle,
			1UL,
			eSetValueWithOverwrite);
	uint32_t task_notification = 0;
	/* Infinite loop */
	BaseType_t ret = pdFALSE;

	for (;;)
	{
		task_notification = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(portMAX_DELAY));
		switch(task_notification)
		{
			case (uint32_t) CMD_WRITE_SINGLE_PARM:
			{
				ret = xSemaphoreTake(sema_configHandle,
							pdMS_TO_TICKS(50));
				write_all_config_1089(&handle,
							&(current_config.cfg),
							1ul);
				xSemaphoreGive(sema_configHandle);
				task_notification = 0;
				xTaskNotify(serial_taskHandle,
						(uint32_t) 0x01,
						eSetValueWithOverwrite);
			}
			break;

			case (uint32_t) CMD_LINEAR_ARRAY_WALK:
			{
				ret = xSemaphoreTake(sema_configHandle, pdMS_TO_TICKS(50));

				linear_pixel_walk_1089(&handle, &dma, &tmr, &analogs, &g_pixel_cfg,
						&current_config, &mps, NULL);

				compose_array_walk_response(NULL, analogs);

				xSemaphoreGive(sema_configHandle);
				task_notification = 0;
				xTaskNotify(serial_taskHandle, (uint32_t ) 0x01, eSetValueWithOverwrite);
			}
			break;

			case (uint32_t) CMD_REF_REMOVAL:
			{
				ret = xSemaphoreTake(sema_configHandle, pdMS_TO_TICKS(50));
				imager_calculate_reference_values(ref_kick,
						&handle,
						&dma,
						&tmr,
						&analogs,
						&mps);

				xSemaphoreGive(sema_configHandle);
				task_notification = 0;
				xTaskNotify(serial_taskHandle, (uint32_t ) 0x01, eSetValueWithOverwrite);

			}
			break;

			default:
			case 0:
			{
				break;
			}
		}
	}
}


static void imager_calculate_reference_values(uint16_t cmd,
		spike_t * handle,
		spike_dma_t *dma,
		timer_1_us_t * tmr,
		analog_values_interleaved_t *analogs,
		mission_pin_state_t * final_pins)
{
	if( (1 == cmd) ||
		(2 == cmd) )

	{
		pixel_t pxl_cfg = {0};
		pxl_cfg.iterations = 1;
		pxl_cfg.start_array = 1;
		pxl_cfg.start_location = 0;
		pxl_cfg.width = 1;

		config_and_arrays_t ref = { 0};
		ref.cfg.bits.tia_c.raw = background_remove_settings.tia_c.raw;
		ref.cfg.bits.tia_s.raw = background_remove_settings.tia_s.raw;
		ref.cfg.bits.sum.raw = background_remove_settings.sum.raw;
		ref.cfg.bits.diff.raw = background_remove_settings.diff.raw;
		ref.cfg.bits.reis.value = background_remove_settings.reis.value;
		ref.cfg.bits.reic.value = background_remove_settings.reic.value;

		ref.array_0.large[0] = 0;
		ref.array_0.large[1] = 0;
		ref.array_0.large[2] = 0;
		ref.array_0.large[3] = 0;
		ref.array_1.large[0] = 0xFFFFFFFF;
		ref.array_1.large[1] = 0xFFFFFFFF;
		ref.array_1.large[2] = 0xFFFFFFFF;
		ref.array_1.large[3] = 0xFFFFFFFF;


		// 1 read to flush output caps.
		linear_pixel_walk_1089(handle,
				dma,
				tmr,
				analogs,
				&pxl_cfg,
				 &ref,
				final_pins,
				NULL);

		if(2 == cmd)
		{
			pxl_cfg.start_location = 0;
		}
		else if(1 == cmd)
		{
			pxl_cfg.start_location = 120;
		}
		pxl_cfg.iterations = 8;
		linear_pixel_walk_1089(handle,
				dma,
				tmr,
				analogs,
				&pxl_cfg,
				 &ref,
				final_pins,
				NULL);
		uint32_t diff_tmp = 0;
		uint32_t sum_tmp = 0;
		for(uint8_t i = 0; i < 16; i++)
		{
			if((i % 2) == 0 )
			{
				diff_tmp += analogs->interleaved[i];
			}
			else
			{
				sum_tmp += analogs->interleaved[i];
			}
		}
		diff_tmp = (diff_tmp / 8);
		sum_tmp = (sum_tmp / 8);
		DIFF_REF = diff_tmp;
		SUM_REF = sum_tmp;
		generic_buffer[0] =  get_ADC_resolution(analogs->adc);
	}
	else if(0 == cmd)
	{
		DIFF_REF = 0;
		SUM_REF = 0;
	}
}

static void compose_array_walk_response(timer_dbg_t * t_dbg,
		analog_values_interleaved_t analogs)
{
	if(t_dbg != NULL)
	{
		generic_buffer[0] = (uint8_t) (t_dbg->setup_time_us);
		generic_buffer[1] = (uint8_t) (t_dbg->run_time_us);
	}
	else
	{
		generic_buffer[0] = (uint8_t) 0;
		generic_buffer[1] = (uint8_t) 0;
	}
	// this doesn't account for oversampling.
	generic_buffer[2] = get_ADC_resolution(analogs.adc);
	// ref subtraction
	if (DIFF_REF != 0 || SUM_REF != 0)
	{
		for (uint16_t j = 0; j < (g_pixel_cfg.iterations * 2); j++)
		{
			if (j % 2 == 0)
			{
				if (interleaved_adc_samples[j] < DIFF_REF)
					interleaved_adc_samples[j] = 0;
				else
					interleaved_adc_samples[j] -= DIFF_REF;
			}
			else
			{
				if (interleaved_adc_samples[j] < SUM_REF)
					interleaved_adc_samples[j] = 0;
				else
					interleaved_adc_samples[j] -= SUM_REF;
			}
		}
	}
	imager_count++;
	generic_buffer[3] = g_pixel_cfg.iterations;
}

static uint8_t get_ADC_resolution(ADC_TypeDef * adc)
{
	uint8_t retval = 0;
	switch (LL_ADC_GetResolution(adc))
	{
		case LL_ADC_RESOLUTION_6B:
		{
			retval = 6;
		}
			break;

		case LL_ADC_RESOLUTION_8B:
		{
			retval = 8;
		}
			break;

		case LL_ADC_RESOLUTION_10B:
		{
			retval = 10;
		}
			break;

		case LL_ADC_RESOLUTION_12B:
		default:
		{
			retval = 12;
		}
			break;
	}
	return retval;
}
