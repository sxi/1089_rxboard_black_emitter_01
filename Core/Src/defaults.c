#include <stdint.h>
#include "1089E.h"

/*
 * Implements Default, global data structures.
 */

const uint8_t version_buffer[3] = { 0, 0, 35 };

// default 1089 config
const all_config_t default_1089 = {
	//  AMP "A"
    // .750pF, 32x gain
	.diff.raw = 0x1f,
	// AMP "B"
	// .750pF, 4.5x gain
	.sum.raw = 0x1f,
	// .5pF, 540k gain
	.tia_c.raw = 0x27,
	// .5pF, 540k gain
	.tia_s.raw = 0x27,
	// approximately balanced
	.reis.value = 0x80,
	.reic.value = 0x80
};

// used to remove reference values.
const all_config_t background_remove_settings = {
		.tia_c.raw = 0x27,
		.tia_s.raw = 0x27,
		.sum.raw = 0x18,
		.diff.raw = 0x18,
		.reis.value = 0x80,
		.reic.value = 0x80
};

const uint8_t EnableModbusBroadcast = 0;
const uint8_t myModbusAddress = 1;
