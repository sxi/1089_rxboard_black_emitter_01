/*
 * defaults.h
 *
 *  Created on: Nov 7, 2023
 *      Author: pbalyeat
 * Portable declarations for application defaults.
 * see defaults.c to change values.
 */

#ifndef INC_DEFAULTS_H_
#define INC_DEFAULTS_H_

#include "1089E.h"

// firmware version
extern const uint8_t version_buffer[3];

// default 1089 config
extern const all_config_t default_1089;

extern const all_config_t background_remove_settings;

#define ADC_SAMPLE_COUNT 	( (uint16_t)  256 )

#endif /* INC_DEFAULTS_H_ */
