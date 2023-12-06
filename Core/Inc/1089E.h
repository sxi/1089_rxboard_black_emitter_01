/*
 * 1089E.h
 *
 *	Provides a LL interface to the 1089 ASIC.
 *
 *  Created on: May 30, 2023
 *      Author: pbalyeat
 */

#ifndef SRC_1089E_H_
#define SRC_1089E_H_

#include <stdint.h>
// needed for spi and gpio defs.
#include "main.h"

// start APPLICATION DEFINES
const static uint8_t TIA_C = 	( (uint8_t) 0x00 );
const static uint8_t TIA_S =	( (uint8_t) 0x02 );
const static uint8_t SUM  =	   ( (uint8_t) 0x04 );
const static uint8_t DIFF	=  ( (uint8_t) 0x06 );
const static uint8_t REIS	=  ( (uint8_t) 0x08 );
const static uint8_t REIC	=  ( (uint8_t) 0x0A );
const static uint8_t ARRAY_0 = ( (uint8_t) 0x10 );
const static uint8_t ARRAY_1 = ( (uint8_t) 0x12 );
const static uint8_t ARRAY_2 = ( (uint8_t) 0x14 );
const static uint8_t TEST_1	= ( (uint8_t) 0x20 );
const static uint8_t TEST_2	= ( (uint8_t) 0x22 );

#ifdef HAS_DAC_OUT
const static uint8_t DAC_PARM = ( (uint8_t) 0x0B);
#endif

/*
 * \struct pixel_t pixel settings for array walk.
 */
typedef struct {
	// pixel width 1-128
	uint8_t width;
	// number of cycles to run [0-127]
	uint8_t iterations;
	// 0 = array_0, 1 = array_1
	uint8_t start_array;
	// start index for pixel.
	uint8_t start_location;
} pixel_t;

// end APPLICATION DEFINES

// start ASIC registers

/*
 * \enum spike_a_t spike action
 */
typedef  enum {
	SPIKE_WRITE	=  	0x00,
	SPIKE_READ 	=	0x01,
}spike_a_t;

/*
 * \enum run_mode_t sets the run after a read/write.
 */
typedef enum {
	STANDBY = 0,
	RUN = 1,
} run_mode_t;

/*
 * \struct addr_1089_t 1089 address struct
 */
typedef struct
{
	// 0 = standby, 1 = run
	uint8_t run_mode :1;
	// address to read/write
	uint8_t addr :6;
	//  0 = write 1 = read
	uint8_t RW :1;
} addr_1089_t;

/*
 * \union addr_1089_u individual register addresses.
 */
typedef union
{
	addr_1089_t bits;
	uint8_t raw;
} addr_1089_u;

/*
 * \struct tia_t represents a tia register.
 */
typedef struct
{
	// tia gain setting
	// 'b111 = 540k ohm
	// 'b011 = 180k ohm
	// 'bX01 = 60k ohm
	// 'bxx0 = 20k ohm
	// x = don't care , 0 preferred
	uint8_t gain :3;
	// 0000 = 0 pF
	// 0001 = 0.125 pF
	// 0010 = 0.25 pF
	// 0100 = 0.5 pF
	// 1000 = 1.0 pf
	uint8_t comp : 4;
	// disable gain up/down steps of register
	uint8_t lock :1;
} tia_t;

/*
 * \union tia_u tia + byte access to tia struct.
 */
typedef union
{
	tia_t bits;
	uint8_t raw;
} tia_u;

/*
 * \struc sum_t represents sum register settings
 */
typedef struct {
	// bits	 R		gain
	// 111 = 1024k 4.5
	// 110 = 896k
	// 101 = 768k
	// 100 = 640k
	// 011 = 512k
	// 010 = 384k
	// 001 = 256k
	// 000 = 128k 1.0
	// gain is ( rf<2,0>  + 2) / 2
	uint8_t gain_rf :3;
	// 0000 0pF
	// xxx1 .25pF
	// xx1x .5pF
	// x1xx	1.0pf
	// 1xxx 2.0pF
	// 1111 (15) = 3.75pF
	uint8_t binary_weighted_caps_cf : 4;
	uint8_t :1 ;
} sum_t;

/*
 * \union sum_u sum_t + byte access.
 */
typedef union {
	sum_t bits;
	uint8_t raw;
} sum_u;

/*
 * \struct diff_t represents a diff registers.
 */
typedef struct {
	// bits	 R		gain
	// 111 = 1024k	32
	// 110 = 896k
	// 101 = 768k
	// 100 = 640k	20
	// 011 = 512k
	// 010 = 384k
	// 001 = 256k
	// 000 = 128k	4
	// gain is 4 * ( rf<2,0>  + 1 )
	uint8_t gain_rf : 3;
	// 0000 0pF
	// xxx1 .25pF
	// xx1x .5pF
	// x1xx	1.0pf
	// 1xxx 2.0pF
	// 1111 (15) = 3.75pF
	uint8_t binary_weighted_caps_cf : 4;
	uint8_t :1 ;
} diff_t;

/*
 * \union diff_u overlays diff_t over a byte.
 */
typedef union {
	diff_t bits;
	uint8_t raw;
} diff_u;

/*
 * \struct reis_t represents a ReIS register
 */
typedef struct {
	// 0 = 1.5
	// 64 = 1.25
	// 255 = .504
	// 1.5 -(  3.906x10^-3 * value )
	uint8_t value;
} reis_t;

/*
 * \struct reic_t represents a ReIC register
 */
typedef struct {
	// 0 = .504
	// 64 = .754
	// 255 = 1.5
	// ( 3.906x10^-3 * value ) + 0.504
	uint8_t value;
}reic_t;

/*
 * \union array_u  photo array values.
 */
typedef union {
	uint8_t value[16];
	uint32_t large[4];
} array_u;

/*
 * \struct test_1_t test_1 register
 */
typedef struct {
	// 0 = enable, 1 = disable
	uint8_t sns_tia_enable 	: 1;
	// 0 = sns tia out tref, 1 = sns tia out track
	uint8_t sns_tia_out  	: 1;
	// 0 = cut tia enable, 1 = cut tia enable
	uint8_t cut_tia_enable 	: 1;
	// 0 = cut tia out tref, 1 = cut tia out track
	uint8_t cut_tia_out 	: 1;
	uint8_t : 2;
	// 0 =  sum no conn, 1 = sum to output
	uint8_t sum_no_con		: 1;
	// 0 = ref sh dt h cntrl, 1 = ref sh continuous
	uint8_t ref_sh_dt		: 1;
} test_1_t;

/*
 * \union test_1_u wraps a test_1 with byte access
 */
typedef union {
	test_1_t bits;
	uint8_t raw;
} test_1_u;

/*
 * \struct test_2_t test 2 register
 */
typedef struct {
	// 0 = sns tia no conn, 1 =  sns tia to sum
	uint8_t sns_tia 	: 1;
	// 0 = tref enabled, 1 = tref disabled
	uint8_t tref_enable : 1;
	// 0 = sum enable, 1 = sum disable
	uint8_t sum_enable	: 1;
	// 0 = diff enable, 1 = diff disable
	uint8_t diff_enable	: 1;
	// 0 = cut tia no conn, 1 = cut tia to diff
	uint8_t cut_tia_con	: 1;
	// 0 = diff no con, 1 = diff to output
	uint8_t diff_con	: 1;
	// 0 = pdt per array_ax, 1 = pdt to all pds
	uint8_t pdt			: 1;
	// 0 = pad disable, 1 = pdt pad enable
	uint8_t pdt_pad		: 1;
} test_2_t;

/*
 * \union test_2_u wraps a test_2_t with a byte
 */
typedef union {
	test_2_t bits;
	uint8_t raw;
} test_2_u;

/*
 * \struct all_config_t contains contiguous cfg regs.
 */
typedef struct {
	tia_u tia_c;
	tia_u tia_s;
	sum_u sum;
	diff_u diff;
	reis_t reis;
	reic_t reic;
} all_config_t;

/*
 * \union all_config_u wraps config info with a byte array;
 */
typedef union {
	all_config_t bits;
	uint8_t raw[ (uint16_t) sizeof(all_config_t)];
} all_config_u;

/*
 * \struct config_and_arrays_t contains all config and array data
 * There is no union here because cfg picks up 2 padding bytes.
 */
typedef struct {
	all_config_u cfg;
	// array 0 data
	array_u array_0;
	// array 1 data
	array_u array_1;
} config_and_arrays_t;

#ifdef HAS_DAC_OUT
/*
 * \struct config_and_arrays_dac_t contains all config and array data along with
 * a DAC output value.
 */
typedef struct {
	// dac output value for this run.
	uint16_t dac;
	all_config_u cfg;
	// array 0 data
	array_u array_0;
	// array 1 data
	array_u array_1;
}config_and_arrays_dac_t;
#endif

// end ASIC registers

// start LOW LEVEL DRIVER hardware

/*
 * \struct LL_pin associates a port and a pin
 */
typedef struct
{
	GPIO_TypeDef *gpio;
	uint32_t pin;
} LL_Pin;

/*
 * \struct spike_t collection of pins and comms to communicate over spike.
 * Note, this library will handle all init/de-init operations.
 * BIG NOTE: all of these pins MUST be on the same port.
 */
typedef struct
{
	// Not power on ready pin / data out
	LL_Pin npor_sdo;
	// not chipselect
	LL_Pin ncs;
	// data in
	LL_Pin dth_sdi;
	// clock in / tia adjust
	LL_Pin tbm_sck;
	// orb
	LL_Pin orb;
	// oth
	LL_Pin oth;
	// GPIO to pulse LED
	LL_Pin LED_out;
	// Alternate function for spi periph.
	uint32_t LL_AF;
	// baud rate pre scalar.
	uint32_t LL_BAUDRATEPRESCALER;
	// spi interface
	SPI_TypeDef * spike;
} spike_t;

/*
 * \struct spike_dma_t handles LL driver DMA config.
 */
typedef struct
{
	// DMA controller
	DMA_TypeDef * 	dma;
	// LL TX dma channel
	uint32_t 		LL_tx_channel;
	// LL DMA MUX data source / request # (varies as mcu)
	uint32_t 		LL_tx_extra;
	// RX dma channel
	uint32_t 		LL_rx_channel;
	// LL DMA MUX data dest / request # (varies as mcu)
	uint32_t 		LL_rx_extra;
} spike_dma_t;

/*
 * \struct timer_1_us_t a TC with a 1us base.
 */
typedef struct
{
	// timer, set to a 1uS, count up.
	TIM_TypeDef * 	timer;
	// timer channel
	uint32_t		ll_channel;

} timer_1_us_t;

/*
 * \struct timer_dbg used to profile setup and run time
 */
typedef struct
{
	// 1us timer.
	TIM_TypeDef * 	timer;
	// debug pin, gets set high and toggles.
	LL_Pin dbg_pin;
	// High when sum is sampling
	LL_Pin SumSample;
	// high when diff is sampling.
	LL_Pin DiffSample;
	// number of us to setup function
	uint16_t setup_time_us;
	// average runtime for read in us
	uint16_t run_time_us;

} timer_dbg_t;

/*
 * \struct mission_pin_state_t Sets pin state
 * 1 = pin logic high. 0 = pin logic low.
 */
typedef struct
{
	uint8_t orb : 1;
	uint8_t oth : 1;
	uint8_t tbm : 1;
	uint8_t dth : 1;
	uint8_t : 4;
} mission_pin_state_t;

/*
 * \struct analog_values_split_t analog data sources and storage locations
 */
typedef struct {
	// diff channel
	ADC_TypeDef * adc;
	// storage location for diff readings
	uint16_t * diff_values;
	// storage location for sum readings
	uint16_t  * sum_values;
} analog_values_split_t;

/*
 * \struct analog_values_split_t analog data sources and storage locations
 */
typedef struct {
	// diff channel
	ADC_TypeDef * adc;
	// Storage location for inteleaved values.
	uint16_t * interleaved;
} analog_values_interleaved_t;

// end LOW LEVEL DRIVER hardware

/*
 * \brief reads all config data
 * \param [in] handle 1089 pin collection
 * \param [in|out] cfg config data.
 * \param [in] run_mode non-zero turn on run mode. 0 = run_mode off
 * \retval 0 on success, non-zero on fails.
 */
uint8_t read_all_config_1089(spike_t * handle,
		all_config_u *cfg,
		uint32_t run_mode);

/*
 * \brief polled - writes all config data
 * \param [in] handle 1089 pin collection
 * \param [in] cfg config data.
 * \param [in] run_mode non-zero turn on run mode. 0 = run_mode off
 * \retval 0 on success, non-zero on fails.
 */
uint8_t write_all_config_1089(spike_t * handle,
		all_config_u *cfg,
		uint32_t run_mode);
/*
 * \brief Given initial state in cfg, walk a pixel down 1 array
 * Further stick shifts some GPIO to force an analog out.
 * \param [in] handle 1089 pin collection
 * \param [in] dma comms dma interface.
 * \param [in] tmr 1us timer peripheral
 * \param [in] cfg device cfg and initial array state.
 * 				Only array 0 and 1 are used.
 * \param [in] pixel_setup pixel configuration
 * \param [in|out] analogs read and stores analog values.
 * \param [in] final_pins final pin state after all writes are done
 * \retval 0 on success, non-zero on fails.
 */
uint8_t linear_pixel_walk_1089(spike_t * handle,
		spike_dma_t *dma,
		timer_1_us_t * tmr,
		analog_values_interleaved_t *analogs,
		pixel_t *pixel_setup,
		config_and_arrays_t *cfg,
		mission_pin_state_t * final_pins,
		timer_dbg_t *dbg );

#ifdef HAS_DAC_OUT
/*
 * \brief Given initial state in cfg, walk a pixel down 1 array
 * Further stick shifts some GPIO to force an analog out.
 * \param [in] handle 1089 pin collection
 * \param [in] dma comms dma interface.
 * \param [in] tmr 1us timer peripheral
 * \param [in] cfg device cfg and initial array state.
 * 				Only array 0 and 1 are used.
 * \param [in] dac DAC instance to use
 * \param [in] pixel_setup pixel configuration
 * \param [in|out] analogs read and stores analog values.
 * \param [in] final_pins final pin state after all writes are done
 * \retval 0 on success, non-zero on fails.
 */
uint8_t linear_pixel_walk_dac_1089(spike_t * handle,
		spike_dma_t *dma,
		timer_1_us_t * tmr,
		analog_values_interleaved_t *analogs,
		pixel_t *pixel_setup,
		config_and_arrays_dac_t *cfg,
		DAC_TypeDef * dac,
		mission_pin_state_t * final_pins,
		timer_dbg_t *dbg );
#endif

/*
 * \brief Takes a spike_t and configures all pins appropriately.
 * \param [in] t Spike pin collection to configure.
 * Note all LL_Pins in t must be on the same port!
 * \retval 0 on success, 1 on fails.
 */
uint8_t driver_init(spike_t * t);

#endif /* SRC_1089E_H_ */
