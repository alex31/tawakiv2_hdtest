#include <ch.h>
#include <hal.h>
#include <stdnoreturn.h>
#include <math.h>
#include "hal_stm32_dma.h"

#define DMA_DATA_LEN 128

extern mailbox_t mb;
extern uint16_t samples[DMA_DATA_LEN];
extern volatile uint32_t sumOk, sum17;
void initSpy(void);
