#include "1089E.h"
#include "string.h"

/*
 * \brief Disables SPI, and sets all pins to logic high
 * \param [in] t spike to disable.
 * This is the library entry point.  Call this to ensure the spike interface
 * is in a consistent state before doing any spike comms.
 */
static void de_init_comms(spike_t * t);

/*
 * \brief Initializes SPI and sets up pins
 * \param [in|out] t spike interface to configure.
 */
static void init_comms(spike_t *t);

/*
 * \brief configures SPI and sets all pin props except AF]
 * \param [in|out] t spike interface to configure.
 * Does not enable any peripherals or set AF.,
 */
static void configure_comms(spike_t *t);

/*
 * \brief Sets a pin to output and clears AF
 * \param [in] t pin to output.
 * \param [in] io 0 = output, 1= input pin.
 */
static void de_init_pin(const LL_Pin *t, uint8_t io);

/*
 * \brief range checks and fixes pixel settings
 * \param [in|out] pixel pixel settings to fix.
 */
static void fix_pixel_settings(pixel_t *pixel);

/*
 * \brief TX only SPI DMA write.
 * \param [in] handle Spike interface
 * \param [in] dma dma information
 * \param [in] txbuff data sent
 * \param [in | out] rxbuff filled in
 * \param [in] byte_count number of bytes to send.
 *
 */
static uint8_t spike_dma_read_write(spike_t * handle,
		spike_dma_t * dma,
		uint8_t * txbuff,
		uint8_t * rxbuff,
		uint16_t byte_count);

/*
 * \brief given a direction read xor write byte_count starting at 0x00
 * \param [in] handle pin collection to use
 * \param [in] run_mode non-zero turn on run mode. 0 = run_mode off
 * \param [in] dir SPIKE_READ or SPIKE_WRITE
 * \param [in | out] src_dest if read - data is written here
 * 		if write, data is sourced from ehre
 * 	\param [in] byte_count bytes to read/write/.
 * 	\retval 0 on success
 */
static uint8_t spike_polled_read_write(spike_t * handle,
		uint32_t run_mode,
		uint8_t dir,
		uint8_t *src_dest,
		uint16_t byte_count);

/*
 * \brief Given a DMA channel, clears all irq status bits
 * \param [in] channel channel status to clear.
 */
static void clear_dma_status(DMA_TypeDef * dma, uint32_t channel);

/*
 * \brief configures DMA read/write
 * \param [in] byte_count number of bytes to read/write
 * \param [in] dma dma configuration
 * \param [in] txbuff buffer to send
 * \param [in] handle spike pin collection
 * \param [in] rxbuff rx buffer.
 */
static void spike_dma_setup(uint16_t byte_count,
		spike_dma_t *dma,
		uint8_t *txbuff,
		spike_t *handle,
		uint8_t *rxbuff);

uint8_t driver_init(spike_t * t)
{
	uint8_t retval = 0;
	if( (t->dth_sdi.gpio == t->ncs.gpio)	&&
		(t->ncs.gpio == t->npor_sdo.gpio)	&&
		(t->npor_sdo.gpio == t->orb.gpio)	&&
		(t->orb.gpio == t->oth.gpio)		&&
		(t->oth.gpio == t->tbm_sck.gpio) )
	{
		retval = 0;
		de_init_comms(t);
	}
	else
	{
		retval = 1;
	}
	return retval;
}

uint8_t read_all_config_1089(spike_t * handle,
		all_config_u * cfg,
		uint32_t run_mode) {
	return spike_polled_read_write(handle,
			run_mode,
			SPIKE_READ,
			cfg->raw,
			(uint16_t) sizeof(all_config_u));
}

uint8_t write_all_config_1089(spike_t * handle,
		all_config_u * cfg,
		uint32_t run_mode) {
	return spike_polled_read_write(handle,
			run_mode,
			SPIKE_WRITE,
			cfg->raw,
			(uint16_t) sizeof(all_config_u));
}

uint8_t linear_pixel_walk_1089(spike_t * handle,
		spike_dma_t *dma,
		timer_1_us_t * tmr,
		analog_values_interleaved_t *analogs,
		pixel_t *pixel_setup,
		config_and_arrays_t *cfg,
		mission_pin_state_t * final_pins,
		timer_dbg_t *dbg )
{
	// dma_buffer for main tx/rx data.
	// DMA transmit buff
	//1 + (uint16_t) ( (2 * sizeof(array_u)) + sizeof(all_config_u) )
	uint8_t dma_buff [40];
	// DMA read buff, usually scrap data.
	//1 + (uint16_t) ( (2 * sizeof(array_u)) + sizeof(all_config_u) )
	uint8_t scrap_buff [ 40 ];
	// number of adc reads.
	uint16_t adccount = 0;
	// marks the start of pixel writes in the dma buff.
	uint8_t dma_buff_index = 0;
	if(NULL != dbg)
	{
		LL_GPIO_ResetOutputPin(dbg->dbg_pin.gpio, dbg->dbg_pin.pin);
		LL_TIM_SetAutoReload(dbg->timer, 0xFFFF);
		LL_TIM_EnableCounter(dbg->timer);
		dbg->run_time_us = 0;
		dbg->setup_time_us = 0;
	}
	// used to load ~5us delay into our counter
	const uint32_t delay_5us = 4;
	const uint32_t final = ( (1 == final_pins->dth ?
			handle->dth_sdi.pin : (handle->dth_sdi.pin << 16UL )) |
		 (1 == final_pins->orb ?
				 handle->orb.pin : (handle->orb.pin << 16UL )) |
		 (1 == final_pins->tbm ?
				 handle->tbm_sck.pin : (handle->tbm_sck.pin << 16UL )) |
		 (1 == final_pins->oth ?
				 handle->oth.pin : (handle->oth.pin << 16UL )) );

	// tracks the number of bytes for the dma write
	uint16_t byte_count = 0;
	fix_pixel_settings(pixel_setup);

	addr_1089_u start = {0};
	start.bits.RW = 0;
	start.bits.run_mode = 1;
	start.bits.addr = TIA_C;
	LL_GPIO_ResetOutputPin(handle->LED_out.gpio, handle->LED_out.pin);
	dma_buff[byte_count++] = start.raw;

	for(uint8_t i = 0; i < sizeof(all_config_u); i++)
	{
		dma_buff[byte_count++] = cfg->cfg.raw[i];
	}

	array_u * array_ptr = &(cfg->array_0);
	if(0 == pixel_setup->start_array )
	{
		dma_buff_index = byte_count;
		array_ptr = &(cfg->array_0);
	}

	for(uint8_t i = 0; i < sizeof(array_u); i++)
	{
		dma_buff[byte_count++] = cfg->array_0.value[i];
	}

	if(1 == pixel_setup->start_array )
	{
		dma_buff_index = byte_count;
	}

	for(uint8_t i = 0; i < sizeof(array_u); i++)
	{
		dma_buff[byte_count++] = cfg->array_1.value[i];
		array_ptr = &(cfg->array_1);
	}

	clear_dma_status(dma->dma ,dma->LL_tx_channel);

	// tracks which array index we're in
	uint8_t pixel_word = pixel_setup->start_location / 8;
	// current pixel mask value.
	uint8_t pixel = 0x80;
	pixel = pixel >> (pixel_setup->start_location % 8);

	// Tracks number of adc reads
	uint8_t adc_read_count = 0;

	// Used for rapid pin init/de-init.
	LL_Pin af_pin_buffer [3] = {
				handle->dth_sdi,
				handle->tbm_sck,
				handle->npor_sdo,};

	// drop the analogs on the 1089
	LL_GPIO_ResetOutputPin(handle->orb.gpio,
				(handle->orb.pin | handle->oth.pin ) );

	// setup comms.
	configure_comms(handle);
	LL_SPI_InitTypeDef SPI_InitStruct = {0};
	SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
	SPI_InitStruct.BaudRate = handle->LL_BAUDRATEPRESCALER;
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	SPI_InitStruct.CRCPoly = 7;

	// clear timer stuff
	LL_TIM_DisableCounter(tmr->timer);
	LL_TIM_DisableARRPreload(tmr->timer);
	LL_TIM_ClearFlag_UPDATE(tmr->timer);

	if(0 == LL_ADC_IsEnabled(analogs->adc))
	{
		LL_ADC_Enable(analogs->adc);
		while(0 == LL_ADC_IsActiveFlag_ADRDY(analogs->adc) ) {}
	}

	if(NULL != dbg)
	{
		dbg->setup_time_us = (uint16_t) LL_TIM_GetCounter(dbg->timer);
		LL_TIM_DisableCounter(dbg->timer);
		LL_GPIO_SetOutputPin(dbg->dbg_pin.gpio, dbg->dbg_pin.pin);
		dbg->timer->CNT = 0;
	}
	uint8_t first_pass = 0;

	for(uint8_t i = 0; i < pixel_setup->iterations; i++)
	{
		if(NULL != dbg)
		{
			if(dbg->run_time_us == 0)
			{
				LL_TIM_SetAutoReload(dbg->timer, 0xFFFF);
				LL_TIM_EnableCounter(dbg->timer);
			}
			LL_GPIO_ResetOutputPin(dbg->dbg_pin.gpio, dbg->dbg_pin.pin);
		}

		// rapid pin init.
		for(uint8_t j = 0 ; j < 3; j++)
		{
			LL_GPIO_SetPinMode(af_pin_buffer[j].gpio,
						af_pin_buffer[j].pin,
						LL_GPIO_MODE_ALTERNATE);

			if(af_pin_buffer[j].pin > 7)
			{
				LL_GPIO_SetAFPin_8_15(af_pin_buffer[j].gpio,
						af_pin_buffer[j].pin,
						handle->LL_AF);
			}
			else
			{
				LL_GPIO_SetAFPin_0_7(af_pin_buffer[j].gpio,
						af_pin_buffer[j].pin,
						handle->LL_AF);
			}
		}
		LL_SPI_Init(handle->spike, &SPI_InitStruct);
		LL_SPI_SetStandard(handle->spike, LL_SPI_PROTOCOL_MOTOROLA);
		LL_SPI_DisableNSSPulseMgt(handle->spike);
		LL_SPI_Enable(handle->spike);

		// pixel walking time
		if(i != 0)
		{
			if(0 ==  ( ( pixel_setup->start_location +  i) % 8) )
			{
				dma_buff[dma_buff_index + pixel_word] = array_ptr->value[pixel_word];
				pixel_word =  (uint8_t) ( (pixel_setup->start_location + i) / 8);
				pixel = 0x80;
			}
			else
			{
				pixel = pixel >> 1;
			}
		}
		dma_buff[dma_buff_index + pixel_word] =\
				array_ptr->value[pixel_word] & ~pixel;

		spike_dma_setup(byte_count,
				dma,
				dma_buff,
				handle,
				scrap_buff);

		LL_DMA_EnableChannel(dma->dma,
				dma->LL_tx_channel);

		LL_GPIO_ResetOutputPin(handle->ncs.gpio,
				( handle->oth.pin |
				  handle->ncs.pin |
				  handle->orb.pin) );

		LL_SPI_EnableDMAReq_TX(handle->spike);

		// Must use direct register reads here.
		// -O1 was removing some LL calls.
		switch(dma->LL_tx_channel)
		{
			case LL_DMA_CHANNEL_7:
				while( (dma->dma->ISR & DMA_ISR_TCIF7) != (DMA_ISR_TCIF7) ) {}
				break;
			case LL_DMA_CHANNEL_6:
				while( (dma->dma->ISR & DMA_ISR_TCIF6) != (DMA_ISR_TCIF6) ) {}
				break;
			case LL_DMA_CHANNEL_5:
				while( (dma->dma->ISR & DMA_ISR_TCIF5) != (DMA_ISR_TCIF5) ) {}
				break;
			case LL_DMA_CHANNEL_4:
				while( (dma->dma->ISR & DMA_ISR_TCIF4) != (DMA_ISR_TCIF4) ) {}
				break;
			case LL_DMA_CHANNEL_3:
				while( (dma->dma->ISR & DMA_ISR_TCIF3) != (DMA_ISR_TCIF3) ) {}
				break;
			case LL_DMA_CHANNEL_2:
				while( (dma->dma->ISR & DMA_ISR_TCIF2) != (DMA_ISR_TCIF2) ) { }
				break;
			case LL_DMA_CHANNEL_1:
				while( (dma->dma->ISR & DMA_ISR_TCIF1) != (DMA_ISR_TCIF1) ) {}
				break;
		}

		// ensure SPI is flushed.
		// direct register reads req'd;  -O1 sometimes removes this.
		while ( ( handle->spike->SR &  SPI_SR_BSY ) != 0 ) {}
		LL_SPI_Disable(handle->spike);
		LL_SPI_DisableDMAReq_TX(handle->spike);

		// rapid pin de-init
		for(uint8_t j = 0 ; j < 3; j++)
		{
			LL_GPIO_SetPinMode(af_pin_buffer[j].gpio,
					af_pin_buffer[j].pin,
					LL_GPIO_MODE_OUTPUT);
			if(af_pin_buffer[j].pin > 7)
			{
				LL_GPIO_SetAFPin_8_15(af_pin_buffer[j].gpio,
						af_pin_buffer[j].pin,
						LL_GPIO_AF_0);
			}
			else
			{
				LL_GPIO_SetAFPin_0_7(af_pin_buffer[j].gpio,
						af_pin_buffer[j].pin,
						LL_GPIO_AF_0);
			}
		}

		LL_GPIO_SetOutputPin(handle->dth_sdi.gpio,
				( handle->dth_sdi.pin |
					handle->tbm_sck.pin |
					handle->ncs.pin ));

		LL_TIM_ClearFlag_UPDATE(tmr->timer);
		LL_TIM_SetAutoReload(tmr->timer, delay_5us);
		LL_TIM_EnableCounter(tmr->timer);
		// need under 1us and bigger than .5us delay between NCS
		// going high and ORB going high.
		uint32_t tval  = 0;
		while(1 != (LL_TIM_IsActiveFlag_UPDATE(tmr->timer)) )
		{
			if(1 == tval)
			{
				LL_GPIO_SetOutputPin(handle->orb.gpio, handle->orb.pin);
			}
			tval++;
		}

		LL_GPIO_ResetOutputPin(handle->dth_sdi.gpio, handle->dth_sdi.pin);
		LL_TIM_DisableCounter(tmr->timer);
		LL_TIM_ClearFlag_UPDATE(tmr->timer);
		LL_TIM_SetAutoReload(tmr->timer, delay_5us);
		LL_TIM_EnableCounter(tmr->timer);

		while (1 != (LL_TIM_IsActiveFlag_UPDATE(tmr->timer)) ) {}
		// OTH MUST precede ORB and TBM going low.
		LL_GPIO_SetOutputPin(handle->oth.gpio, handle->oth.pin);

		// Now we're good to trip the led.
		handle->tbm_sck.gpio->BSRR = (
				( (handle->tbm_sck.pin | handle->orb.pin) << 16UL ) |
				  ( handle->LED_out.pin ) );

		LL_TIM_DisableCounter(tmr->timer);
		LL_TIM_ClearFlag_UPDATE(tmr->timer);
		LL_TIM_SetAutoReload(tmr->timer, delay_5us);
		LL_TIM_EnableCounter(tmr->timer);
		// clear adc and begin ADC sampling.
		LL_ADC_ClearFlag_EOC(analogs->adc);
		LL_ADC_ClearFlag_EOS(analogs->adc);
		while(0 == LL_ADC_IsActiveFlag_ADRDY(analogs->adc)) {}
		LL_ADC_REG_StartConversion(analogs->adc);

		if( (NULL != dbg) &&
			(NULL != dbg->DiffSample.gpio) )
		{
			LL_GPIO_SetOutputPin(dbg->DiffSample.gpio, dbg->DiffSample.pin);
		}

		while (1 != (LL_TIM_IsActiveFlag_UPDATE(tmr->timer)) )
		{
			// Giant ADC sampling block in case of short ADC sampling.
			if( (1 == LL_ADC_IsActiveFlag_EOC(analogs->adc) ) ||
				(1 == LL_ADC_IsActiveFlag_OVR(analogs->adc) ) ||
				(1 == LL_ADC_IsActiveFlag_EOS(analogs->adc) ) )
			{
				analogs->interleaved[adccount++] = LL_ADC_REG_ReadConversionData12(analogs->adc);
				if(0 == adc_read_count)
				{
					if( (NULL != dbg) &&
						(NULL != dbg->DiffSample.gpio ) )
					{
						LL_GPIO_ResetOutputPin(dbg->DiffSample.gpio, dbg->DiffSample.pin);
						LL_GPIO_SetOutputPin(dbg->SumSample.gpio, dbg->SumSample.pin);
					}
					LL_ADC_ClearFlag_EOS(analogs->adc);
					adc_read_count =1;
				}
				else if(1 == adc_read_count)
				{
					if( (NULL != dbg) &&
						(NULL != dbg->SumSample.gpio) )
					{
						LL_GPIO_ResetOutputPin(dbg->SumSample.gpio, dbg->SumSample.pin);
					}
					LL_ADC_REG_StopConversion(analogs->adc);
					LL_ADC_ClearFlag_EOS(analogs->adc);
					LL_ADC_ClearFlag_EOC(analogs->adc);
					adc_read_count = 2;
				}
			}
		}
		// This forces the 1089 to hold SUM and DIFF output levels.
		LL_GPIO_ResetOutputPin(handle->oth.gpio, handle->oth.pin);
		LL_TIM_DisableCounter(tmr->timer);
		LL_TIM_ClearFlag_UPDATE(tmr->timer);
		LL_GPIO_ResetOutputPin(handle->LED_out.gpio, handle->LED_out.pin);

		while(2 != adc_read_count )
		{
			// by hitting this point, we're sampling longer then the led.
			if( (1 == LL_ADC_IsActiveFlag_EOC(analogs->adc) ) ||
				(1 == LL_ADC_IsActiveFlag_OVR(analogs->adc) ) ||
				(1 == LL_ADC_IsActiveFlag_EOS(analogs->adc) ) )
			{
				analogs->interleaved[adccount++] = LL_ADC_REG_ReadConversionData12(analogs->adc);
				if(0 == adc_read_count)
				{
					if( (NULL != dbg) &&
						(NULL != dbg->DiffSample.gpio ) )
					{
						LL_GPIO_ResetOutputPin(dbg->DiffSample.gpio, dbg->DiffSample.pin);
						LL_GPIO_SetOutputPin(dbg->SumSample.gpio, dbg->SumSample.pin);
					}
					LL_ADC_ClearFlag_EOS(analogs->adc);
					adc_read_count =1;
				}
				else if(1 == adc_read_count)
				{
					if( (NULL != dbg) &&
						(NULL != dbg->SumSample.gpio) )
					{
						LL_GPIO_ResetOutputPin(dbg->SumSample.gpio, dbg->SumSample.pin);
					}
					LL_ADC_REG_StopConversion(analogs->adc);
					LL_ADC_ClearFlag_EOS(analogs->adc);
					LL_ADC_ClearFlag_EOC(analogs->adc);
					adc_read_count = 2;
				}
			}
		}
		adc_read_count = 0;

		if(NULL != dbg)
		{
			if(dbg->run_time_us == 0)
			{
				dbg->run_time_us = (uint16_t) LL_TIM_GetCounter(dbg->timer);
				LL_TIM_DisableCounter(dbg->timer);
			}
			LL_GPIO_SetOutputPin(dbg->dbg_pin.gpio, dbg->dbg_pin.pin);
		}
	}
	// End main loop

	// de_init_comms default pins.
	handle->ncs.gpio->BSRR = final;
	LL_GPIO_SetAFPin_8_15(handle->npor_sdo.gpio,
			handle->npor_sdo.pin,
			LL_GPIO_AF_0);
	LL_TIM_DisableCounter(tmr->timer);
	LL_TIM_ClearFlag_UPDATE(tmr->timer);
	LL_TIM_SetCounter(tmr->timer, 0);
	handle->LED_out.gpio->BSRR =  ( handle->LED_out.pin << 16UL);
	LL_DMA_DisableChannel(dma->dma,
			dma->LL_tx_channel);
	return 0;
}

static void clear_dma_status(DMA_TypeDef * dma, uint32_t channel)
{
	switch (channel)
	{
	case LL_DMA_CHANNEL_7:
		LL_DMA_ClearFlag_GI7(dma);
		LL_DMA_ClearFlag_HT7(dma);
		LL_DMA_ClearFlag_TC7(dma);
		LL_DMA_ClearFlag_TE7(dma);
		break;
	case LL_DMA_CHANNEL_6:
		LL_DMA_ClearFlag_GI6(dma);
		LL_DMA_ClearFlag_HT6(dma);
		LL_DMA_ClearFlag_TC6(dma);
		LL_DMA_ClearFlag_TE6(dma);
		break;
	case LL_DMA_CHANNEL_5:
		LL_DMA_ClearFlag_GI5(dma);
		LL_DMA_ClearFlag_HT5(dma);
		LL_DMA_ClearFlag_TC5(dma);
		LL_DMA_ClearFlag_TE5(dma);
		break;
	case LL_DMA_CHANNEL_4:
		LL_DMA_ClearFlag_GI4(dma);
		LL_DMA_ClearFlag_HT4(dma);
		LL_DMA_ClearFlag_TC4(dma);
		LL_DMA_ClearFlag_TE4(dma);
		break;
	case LL_DMA_CHANNEL_3:
		LL_DMA_ClearFlag_GI3(dma);
		LL_DMA_ClearFlag_HT3(dma);
		LL_DMA_ClearFlag_TC3(dma);
		LL_DMA_ClearFlag_TE3(dma);
		break;
	case LL_DMA_CHANNEL_2:
		LL_DMA_ClearFlag_GI2(dma);
		LL_DMA_ClearFlag_HT2(dma);
		LL_DMA_ClearFlag_TC2(dma);
		LL_DMA_ClearFlag_TE2(dma);
		break;
	case LL_DMA_CHANNEL_1:
		LL_DMA_ClearFlag_GI1(dma);
		LL_DMA_ClearFlag_HT1(dma);
		LL_DMA_ClearFlag_TC1(dma);
		LL_DMA_ClearFlag_TE1(dma);
		break;
	}
}

static void de_init_comms(spike_t *t)
{
	uint8_t scrap = 0;
	// flush the rx buffer.
	while(LL_SPI_IsActiveFlag_RXNE(t->spike)) {
		scrap = LL_SPI_ReceiveData8(t->spike);
	}
	LL_SPI_Disable(t->spike);
	de_init_pin( &(t->dth_sdi), 0 );

	// npor should be an input ... ?!
	de_init_pin( &(t->npor_sdo), 1 );
	de_init_pin( &(t->tbm_sck), 0);
}

static uint8_t spike_polled_read_write(spike_t * handle,
		uint32_t run_mode,
		spike_a_t dir,
		uint8_t *src_dest,
		uint16_t byte_count)
{
	addr_1089_u start = {0};
	if( (SPIKE_READ != dir) && (SPIKE_WRITE != dir) )
	{
		dir = SPIKE_READ;
	}
	start.bits.RW = (uint8_t) dir;
	start.bits.addr = TIA_C;
	start.bits.run_mode = (uint8_t) (run_mode > 0 ? 1 : 0);
	uint8_t retval = 0;
	uint16_t scrap = 0;

	init_comms(handle);
	LL_GPIO_ResetOutputPin(handle->ncs.gpio, handle->ncs.pin);
	// set address and flush rx
	while(!LL_SPI_IsActiveFlag_TXE(handle->spike)) {}
	LL_SPI_TransmitData8(handle->spike, start.raw);

	while(!LL_SPI_IsActiveFlag_RXNE(handle->spike)) {}
	scrap =  LL_SPI_ReceiveData8(handle->spike);


	for(uint16_t i = 0; i < byte_count; i++)
	{
		while(!LL_SPI_IsActiveFlag_TXE(handle->spike)) {}

		if(SPIKE_WRITE == dir)
		{
			LL_SPI_TransmitData8(handle->spike, src_dest[i]);
		}
		else
		{
			LL_SPI_TransmitData8(handle->spike, 0x00);
		}

		while(!LL_SPI_IsActiveFlag_RXNE(handle->spike)) {}

		if(SPIKE_READ == dir)
		{

			src_dest[i] = LL_SPI_ReceiveData8(handle->spike);
		}
		else
		{
			scrap =  LL_SPI_ReceiveData8(handle->spike);
		}
	}
	de_init_comms(handle);
	LL_GPIO_SetOutputPin( handle->ncs.gpio, handle->ncs.pin );
	return retval;
}

static void spike_dma_setup(uint16_t byte_count,
		spike_dma_t *dma,
		uint8_t *txbuff,
		spike_t *handle,
		uint8_t *rxbuff)
{
	// tx config
	if (LL_DMA_IsEnabledChannel(dma->dma, dma->LL_tx_channel))
	{
		LL_DMA_DisableChannel(dma->dma, dma->LL_tx_channel);
	}
	LL_DMA_SetPeriphRequest(dma->dma, dma->LL_tx_channel, dma->LL_tx_extra);
	LL_DMA_ConfigAddresses(dma->dma, dma->LL_tx_channel, (uint32_t) txbuff,
			(uint32_t) &(handle->spike->DR), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetDataLength(dma->dma, dma->LL_tx_channel, byte_count);
	uint32_t txflags = LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_MODE_NORMAL
			| LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT
			| LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE
			| LL_DMA_PRIORITY_MEDIUM;
	LL_DMA_ConfigTransfer(dma->dma, dma->LL_tx_channel, txflags);
	// RX config
	if (LL_DMA_IsEnabledChannel(dma->dma, dma->LL_rx_channel))
	{
		LL_DMA_DisableChannel(dma->dma, dma->LL_rx_channel);
	}
	LL_DMA_SetPeriphRequest(dma->dma, dma->LL_rx_channel, dma->LL_rx_extra);
	LL_DMA_ConfigAddresses(dma->dma, dma->LL_rx_channel,
			(uint32_t) &(handle->spike->DR), (uint32_t) rxbuff,
			LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(dma->dma, dma->LL_rx_channel, (byte_count));
	uint32_t rxflags = LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_MODE_NORMAL
			| LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT
			| LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE
			| LL_DMA_PRIORITY_MEDIUM;
	LL_DMA_ConfigTransfer(dma->dma, dma->LL_rx_channel, rxflags);
}

static void init_comms(spike_t *t)
{
	// SPI and GPIO port should already be clocked.

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = t->LL_AF;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pin = t->dth_sdi.pin | t->npor_sdo.pin | t->tbm_sck.pin;
	LL_GPIO_Init(t->dth_sdi.gpio, &GPIO_InitStruct);

	LL_SPI_InitTypeDef SPI_InitStruct = {0};
	SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
	SPI_InitStruct.BaudRate = t->LL_BAUDRATEPRESCALER;
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	SPI_InitStruct.CRCPoly = 7;
	LL_SPI_Init(t->spike, &SPI_InitStruct);
	LL_SPI_SetStandard(t->spike, LL_SPI_PROTOCOL_MOTOROLA);
	LL_SPI_DisableNSSPulseMgt(t->spike);
	LL_SPI_Enable(t->spike);
}

static void configure_comms(spike_t *t)
{
	// SPI and GPIO port should already be clocked.

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pin = t->dth_sdi.pin | t->npor_sdo.pin | t->tbm_sck.pin;
	LL_GPIO_Init(t->dth_sdi.gpio, &GPIO_InitStruct);

	LL_SPI_InitTypeDef SPI_InitStruct = {0};
	SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
	SPI_InitStruct.BaudRate = t->LL_BAUDRATEPRESCALER;
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	SPI_InitStruct.CRCPoly = 7;
	LL_SPI_Init(t->spike, &SPI_InitStruct);
	LL_SPI_SetStandard(t->spike, LL_SPI_PROTOCOL_MOTOROLA);
	LL_SPI_DisableNSSPulseMgt(t->spike);
}

static void de_init_pin(const LL_Pin *t, uint8_t io)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };
	LL_GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.Pin = t->pin;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	if(0 == io)
	{
		GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
		GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	}
	else
	{
		GPIO_InitStruct.Mode =LL_GPIO_MODE_INPUT;
	}
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;

	LL_GPIO_Init(t->gpio,&GPIO_InitStruct);
}

static void fix_pixel_settings(pixel_t *pixel)
{
	if(pixel->width == 0)
		pixel->width = 1;
	else if(pixel->width > 129)
		pixel->width = 128;

	if(pixel->start_array > 1)
		pixel->start_array = 1;

	if(pixel->start_location > 127)
		pixel->start_location = 127;

	if(pixel->iterations > 128)
		pixel->iterations = 128;
}
