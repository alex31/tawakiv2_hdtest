

#define STRESS 1

#include <ch.h>
#include <hal.h>
#include <stdnoreturn.h>
#include <math.h>
#include <string.h>
#include "globalVar.h"
#include "stdutil.h"
#include "ttyConsole.h"
#include "esc_dshot.h"
#include "icu_spy.h"
#if STRESS
#include "adc_stress.h"
#endif


/*

  ° connecter A08, A09
  ° connecter B10 (uart3_tx) sur ftdi rx :  shell
  ° connecter B11 (uart3_rx) sur ftdi tx :  shell
  ° connecter C00 sur led1 
*/

#define BENCH_TELEMETRY_BAUD		115200



#if STRESS
static const UARTConfig uartConfig =  {
					 .txend1_cb =NULL,
					 .txend2_cb = NULL,
					 .rxend_cb = NULL,
					 .rxchar_cb = NULL,
					 .rxerr_cb = NULL,
					 .speed = 1000 * 1000,
					 .cr1 = 0,
					 .cr2 = USART_CR2_STOP2_BITS,
					 .cr3 = 0
  };
#endif



static DshotDmaBuffer dshotd3DmaBuffer __attribute__ ((section(".ram2"), aligned(8)));
static const DSHOTConfig dshotConfig3 = {
  .dma_stream = STM32_PWM3_UP_DMA_STREAM,
  .dma_channel = STM32_PWM3_UP_DMA_CHANNEL,
  .pwmp = &PWMD3,
  .tlm_sd = NULL,
  .uncached_dma_buf = &dshotd3DmaBuffer
};
static DSHOTDriver dshotd3;

static THD_WORKING_AREA(waBlinker, 512);
static noreturn void blinker (void *arg);
#if STRESS
static THD_WORKING_AREA(waMemoryStress, 512);
static noreturn void memoryStress (void *arg);
static THD_WORKING_AREA(waDmaStress, 512);
static noreturn void dmaStress (void *arg);
#endif
static THD_WORKING_AREA(waPrinter, 512);
static noreturn void printer (void *arg);

static void printSamples(void);

int main(void)
{
/*
 * System initializations.
 * - HAL initialization, this also initializes the configured device drivers
 *   and performs the board-specific initializations.
 * - Kernel initialization, the main() function becomes a thread and the
 *   RTOS is active.
 */

  // attempt to give priority to peripheral when acceding sram
  /* uint32_t ahbscr = SCB->AHBSCR; */
  /* ahbscr &= ~SCB_AHBSCR_CTL_Msk; */
  /* ahbscr |= 0b11; */
  /* SCB->AHBSCR = ahbscr; */

  halInit();
  chSysInit();
  initHeap();
  
  //  SCB_DisableDCache();
  
  consoleInit();
  consoleLaunch();
  dshotStart(&dshotd3, &dshotConfig3);
  initSpy();
  
  chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, blinker, NULL);
#if STRESS
  chThdCreateStatic(waMemoryStress, sizeof(waMemoryStress), NORMALPRIO, memoryStress, NULL);
  chThdCreateStatic(waDmaStress, sizeof(waDmaStress), NORMALPRIO, dmaStress, NULL);
  adcStressInit();
#endif

  chThdCreateStatic(waPrinter, sizeof(waPrinter), NORMALPRIO, printer, NULL);
  
  if ((((uint32_t)&dshotd3.config->uncached_dma_buf % 16)) == 0) {
    DebugTrace("dshotd3.config->uncached_dma_buf aligned 16");
  } else if ((((uint32_t)&dshotd3.config->uncached_dma_buf % 8)) == 0) {
    DebugTrace("dshotd3.config->uncached_dma_buf aligned 8");
  } else {
    DebugTrace("dshotd3.config->uncached_dma_buf NOT aligned");
  }

  int32_t throttle = 50;
  while (true) {
    for (size_t i=0; i<DSHOT_CHANNELS; i++) {
      dshotSetThrottle(&dshotd3, i, throttle+i);
    }
    dshotSendFrame(&dshotd3);

    // test dma buffer coherency
    /* for (int j=16; j<20; j++) */
    /*   for (int c=0; c<DSHOT_CHANNELS; c++) { */
    /* 	if (dshotd3.config->uncached_dma_buf.widths32[j][c] != 0) { */
    /* 	  DebugTrace("w32[%d][%d] = %u", j, c, dshotd3.config->uncached_dma_buf.widths32[j][c]); */
    /* 	} */
    /*   } */

    //    printSamples();
    //    DebugTrace ("%u", throttle);
    //    chprintf(chp, "\r\n\r\n\r\n\r\n\r\n");
    if (++throttle > 2000)
      throttle = 50;
    chThdSleepMicroseconds(500);
  } 
}


static void printSamples(void)
{
  for (int k=0; k<16; k++) {
    for (int l=0; l<8; l++) {
      chprintf(chp, "s[%d]=%u, ", k*8+l, samples[k*8+l]);
    }
    chprintf(chp, "\r\n");
  }
}

/*
#                                _    _    _                      _            
#                               | |  | |  | |                    | |           
#                  ___    __ _  | |  | |  | |__     __ _    ___  | | _         
#                 / __|  / _` | | |  | |  | '_ \   / _` |  / __| | |/ /        
#                | (__  | (_| | | |  | |  | |_) | | (_| | | (__  |   <         
#                 \___|  \__,_| |_|  |_|  |_.__/   \__,_|  \___| |_|\_\        
*/





/*
#                 _      _                                 _                 
#                | |    | |                               | |                
#                | |_   | |__    _ __    ___    __ _    __| |   ___          
#                | __|  | '_ \  | '__|  / _ \  / _` |  / _` |  / __|         
#                \ |_   | | | | | |    |  __/ | (_| | | (_| |  \__ \         
#                 \__|  |_| |_| |_|     \___|  \__,_|  \__,_|  |___/         
*/
static noreturn void blinker (void *arg)
{

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palToggleLine(LINE_C00_LED1); 	
    chThdSleepMilliseconds(500);
    DebugTrace("Ok:%lu Ko:%lu [%.2f %%] Ter=%u  DMer=%u Fer=%u FEmp=%u FFu=%u",
	       sumOk, sum17, sum17*100.0f/(sumOk+sum17),
	       dshotd3.dmap.nbTransferError,
	       dshotd3.dmap.nbDirectModeError,
	       dshotd3.dmap.nbFifoError,
	       dshotd3.dmap.nbFifoEmpty,
	       dshotd3.dmap.nbFifoFull
	       );
  }
}

#if STRESS
static noreturn void memoryStress (void *arg)
{
  static volatile uint16_t IN_DMA_SECTION(stress[1024]);
  
  uint32_t cnt = 0;
  (void)arg;
  chRegSetThreadName("memory stress");
  DebugTrace("memory stress addr = %p", stress);
  while (true) {
    stress[cnt % ARRAY_LEN(stress)] = stress[(cnt+200 ) % ARRAY_LEN(stress)] +1;
    cnt++;
  }
}

static noreturn void dmaStress (void *arg)
{
  static uint8_t IN_DMA_SECTION(stress[128]);
  
  uint32_t cnt = 0;
  (void)arg;
  chRegSetThreadName("dma stress");
  DebugTrace("dma stress addr = %p", stress);
  size_t nb=sizeof(stress);
  
  uartStart(&UARTD2, &uartConfig);
  
  while (true) {
    stress[cnt % ARRAY_LEN(stress)] = stress[(cnt+200 ) % ARRAY_LEN(stress)] +1;
    //    chThdSleepMilliseconds(10);
    uartSendTimeout(&UARTD2, &nb, &stress, TIME_INFINITE);
  }
}
#endif
static noreturn void printer (void *arg)
{
  (void)arg;
  msg_t recThrottle=0;
  msg_t last=0;
  chRegSetThreadName("printer");

  while (true) {
    chMBFetchTimeout(&mb, &recThrottle, TIME_MS2I(1000));
    if ((recThrottle - last) > 2000) {
      DebugTrace("last = %ld, rec = %ld", last, recThrottle);
    }
    last = recThrottle;
  }
}


