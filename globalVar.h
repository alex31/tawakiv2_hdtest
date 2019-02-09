#pragma once

#include "ch.h"
#include "hal.h"



/*===========================================================================*/
/* uniq id of stm32 processor                                                        */
/*===========================================================================*/

extern const uint8_t *UniqProcessorId ;
extern const uint8_t UniqProcessorIdLen ;



/*===========================================================================*/
/* USB related stuff.                                                        */
/*===========================================================================*/

/*
 * USB Driver structure.
 */

#if HAL_USE_SERIAL_USB
extern SerialUSBDriver SDU1;
#endif 

extern BaseSequentialStream *chp;

