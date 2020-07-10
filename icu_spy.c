#include "icu_spy.h"
#include "stdutil.h"
#include "esc_dshot_config.h"
#include <string.h>


#define DCR_DBL			(1 << 8) // 2 transfert
// first register to get is CCR1
#define DCR_DBA			(((uint32_t *) (&ICUD1.tim->CCR) - ((uint32_t *) ICUD1.tim))) 

static void initIcu(void);
static void initDma(void);
static bool startDma(void);
static void startDmaAcquisition(uint16_t *widthOrPeriod,
				size_t depth);
static void error_cb(DMADriver *dmap, dmaerrormask_t err);
static void end_cb(DMADriver *dmap, volatile void *buffer, const size_t num);

static volatile dmaerrormask_t last_err = 0;

#define MB_SIZE 256
static msg_t mb_buffer[MB_SIZE];
MAILBOX_DECL(mb, mb_buffer, MB_SIZE);

static const DMAConfig dmaConfig = {
  .stream = STM32_ICU1_CH1_DMA_STREAM,
  .channel = STM32_ICU1_CH1_DMA_CHANNEL,
  .dma_priority = STM32_ICU1_CH1_DMA_PRIORITY,
  .irq_priority = STM32_ICU1_CH1_DMA_IRQ_PRIORITY,
  //  .periph_addr = &TIM1->DMAR,
  .direction = DMA_DIR_P2M,
  .psize = 2,
  .msize = 2,
  .inc_peripheral_addr = false,
  .inc_memory_addr = true,
  .circular = true,
  .error_cb = &error_cb,
#if STM32_DMA_USE_ASYNC_TIMOUT
  .timeout = TIME_MS2I(100),
#endif
  .end_cb = &end_cb,
  .pburst = 0,
  .mburst = 8,
  .fifo = 4
};

static DMADriver dmap;

static const ICUConfig icu1ch1_cfg = {
  .mode = ICU_INPUT_ACTIVE_HIGH,
  .frequency = (STM32_SYSCLK / (1200 / DSHOT_SPEED_KHZ)),    /* 108Mhz ICU clock frequency.   */
  .width_cb = NULL,
  .period_cb = NULL,
  .overflow_cb = NULL,
  .channel = ICU_CHANNEL_1,
  .dier = TIM_DIER_CC1DE | TIM_DIER_TDE
};




uint16_t samples[DMA_DATA_LEN] __attribute__((section(DMA_SECTION "_init"), aligned(16))) = {0}; 

volatile uint32_t sumOk=0U, sum17=0U;
volatile uint32_t currentBitIdx=0U;
volatile msg_t dshotVal=0;

#define MAX_WIDTH 182U
#define PULSE_WIDTH 80U


void initSpy(void)
{
  memset(samples, 0, sizeof(samples));
  initDma();
  initIcu();
  startDma();
  startDmaAcquisition(samples, DMA_DATA_LEN);
  icuStartCapture(&ICUD1);
  icuEnableNotifications(&ICUD1);
}

static void initDma(void)
{
   dmaObjectInit(&dmap);
}



static void initIcu(void)
{
  icuStart(&ICUD1, &icu1ch1_cfg);
  ICUD1.tim->DCR = DCR_DBL | DCR_DBA;
}

static bool startDma(void)
{
   return dmaStart(&dmap, &dmaConfig);
}


static void startDmaAcquisition(uint16_t *widthsAndPeriods,
				const size_t depth)
{
  dmaAcquireBus(&dmap);
  dmaStartTransfert(&dmap, &TIM1->DMAR, widthsAndPeriods, depth);
}



static void error_cb(DMADriver *_dmap, dmaerrormask_t err)
{
  (void) _dmap;
  last_err = err;
  chSysHalt("error_cb");
}

static void end_cb(DMADriver *_dmap, volatile void *buffer, size_t num)
{
  (void) _dmap;
  uint16_t * const smpls = (uint16_t *) buffer;

  chSysLockFromISR();
  for (size_t i=0; i<num; i+=2) {
    const uint16_t w=smpls[i];
    const uint16_t p=smpls[i+1];
    if (w < MAX_WIDTH) {
      if (p > PULSE_WIDTH)
	dshotVal |= (1U << currentBitIdx);
      currentBitIdx++;
    } else {
      if (currentBitIdx == 15U) {
	sumOk++;
	chMBPostI(&mb, revbit( (uint16_t)dshotVal) >> 5);
      } else {
	sum17++;
      }
      dshotVal = 0U;
      currentBitIdx=0U;
    }
  }
    chSysUnlockFromISR();
}
