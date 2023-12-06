/*------------------------------------------------------------------------------
 *
 * $Workfile:   CompilerTypeDefs.h  $ $Modtime:   Jun 04 2019 10:27:10  $
 * Copyright (C) 2021 Banner Engineering Corp. All rights reserved.
 * Developed by Sensonix Incorporated
 *------------------------------------------------------------------------------
 */

#ifndef COMPILER_TYPE_DEFS_H_
#define COMPILER_TYPE_DEFS_H_

#include <stdint.h>

#define     LO_BYTE(x)      *((uint8_t*)&x+0)
#define     HI_BYTE(x)      *((uint8_t*)&x+1)
#define     LO_WORD(x)      *((uint16_t*)&x+0)
#define     HI_WORD(x)      *((uint16_t*)&x+1)

#endif // COMPILER_TYPE_DEFS_H_
