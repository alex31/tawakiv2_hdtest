

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
#define USE_DSHOT2 0
#define USE_DSHOT3 1
#define USE_DSHOT4 1


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


#if USE_DSHOT2
static DshotDmaBuffer IN_DMA_SECTION_NOINIT(dshotd2DmaBuffer);
#endif

#if USE_DSHOT3
static DshotDmaBuffer IN_DMA_SECTION_NOINIT(dshotd3DmaBuffer);
#endif

#if USE_DSHOT4
static DshotDmaBuffer IN_DMA_SECTION_NOINIT(dshotd4DmaBuffer);
#endif

#if USE_DSHOT2
static const DSHOTConfig dshotConfig2 = {
  .dma_stream = STM32_PWM2_UP_DMA_STREAM,
  .dma_channel = STM32_PWM2_UP_DMA_CHANNEL,
  .pwmp = &PWMD2,
  .tlm_sd = NULL,
  .dma_buf = &dshotd2DmaBuffer,
  .dcache_memory_in_use = false
};
 static DSHOTDriver dshotd2;
#endif

#if USE_DSHOT3
static const DSHOTConfig dshotConfig3 = {
  .dma_stream = STM32_PWM3_UP_DMA_STREAM,
  .dma_channel = STM32_PWM3_UP_DMA_CHANNEL,
  .pwmp = &PWMD3,
  .tlm_sd = NULL,
  .dma_buf = &dshotd3DmaBuffer,
  .dcache_memory_in_use = false
};
static DSHOTDriver dshotd3;
#endif

#if USE_DSHOT4
static const DSHOTConfig dshotConfig4 = {
  .dma_stream = STM32_PWM4_UP_DMA_STREAM,
  .dma_channel = STM32_PWM4_UP_DMA_CHANNEL,
  .pwmp = &PWMD4,
  .tlm_sd = NULL,
  .dma_buf = &dshotd4DmaBuffer,
  .dcache_memory_in_use = false
};
static DSHOTDriver dshotd4;
#endif




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
  chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, blinker, NULL);  
  consoleInit();
  consoleLaunch();

#if USE_DSHOT2
  dshotStart(&dshotd2, &dshotConfig2);
#endif

#if USE_DSHOT3
  dshotStart(&dshotd3, &dshotConfig3);
#endif

#if USE_DSHOT4
  dshotStart(&dshotd4, &dshotConfig4);
#endif
   
  initSpy();

  

#if STRESS
  chThdCreateStatic(waMemoryStress, sizeof(waMemoryStress), NORMALPRIO, memoryStress, NULL);
  chThdCreateStatic(waDmaStress, sizeof(waDmaStress), NORMALPRIO, dmaStress, NULL);
  adcStressInit();
#endif

  chThdCreateStatic(waPrinter, sizeof(waPrinter), NORMALPRIO, printer, NULL);

#if USE_DSHOT2
  if ((((uint32_t) dshotd2.config->dma_buf % 32)) == 0) {
    DebugTrace("dshotd2.config->dma_buf aligned 32");
  }  else if ((((uint32_t) dshotd2.config->dma_buf % 16)) == 0) {
    DebugTrace("dshotd2.config->dma_buf aligned 16");
  } else if ((((uint32_t) dshotd2.config->dma_buf % 8)) == 0) {
    DebugTrace("dshotd2.config->dma_buf aligned 8");
  } else {
    DebugTrace("dshotd2.config->dma_buf NOT aligned");
  }
#endif
#if USE_DSHOT3
   if ((((uint32_t) dshotd3.config->dma_buf % 32)) == 0) {
    DebugTrace("dshotd3.config->dma_buf aligned 32");
  }  else if ((((uint32_t) dshotd3.config->dma_buf % 16)) == 0) {
    DebugTrace("dshotd3.config->dma_buf aligned 16");
  } else if ((((uint32_t) dshotd3.config->dma_buf % 8)) == 0) {
    DebugTrace("dshotd3.config->dma_buf aligned 8");
  } else {
    DebugTrace("dshotd3.config->dma_buf NOT aligned");
  }
#endif
#if USE_DSHOT4
   if ((((uint32_t) dshotd4.config->dma_buf % 32)) == 0) {
    DebugTrace("dshotd4.config->dma_buf aligned 32");
  }  else if ((((uint32_t) dshotd4.config->dma_buf % 16)) == 0) {
    DebugTrace("dshotd4.config->dma_buf aligned 16");
  } else if ((((uint32_t) dshotd4.config->dma_buf % 8)) == 0) {
    DebugTrace("dshotd4.config->dma_buf aligned 8");
  } else {
    DebugTrace("dshotd4.config->dma_buf NOT aligned");
  }
#endif



   
  int32_t throttle = 50;
  while (true) {
    for (size_t i=0; i<DSHOT_CHANNELS; i++) {
#if USE_DSHOT2
      dshotSetThrottle(&dshotd2, i, throttle+i);
#endif
#if USE_DSHOT3
      dshotSetThrottle(&dshotd3, i, throttle+i);
#endif
#if USE_DSHOT4
      dshotSetThrottle(&dshotd4, i, throttle+i);
#endif
    }
#if USE_DSHOT2
    dshotSendFrame(&dshotd2);
#endif
#if USE_DSHOT3    
    dshotSendFrame(&dshotd3);
#endif
#if USE_DSHOT4    
    dshotSendFrame(&dshotd4);
#endif
     // test dma buffer coherency
    /* for (int j=16; j<20; j++) */
    /*   for (int c=0; c<DSHOT_CHANNELS; c++) { */
    /* 	if (dshotd2.config->dma_buf.widths32[j][c] != 0) { */
    /* 	  DebugTrace("w32[%d][%d] = %u", j, c, dshotd2.config->dma_buf.widths32[j][c]); */
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
#if USE_DSHOT2
    DebugTrace("D2 Ok:%lu Ko:%lu [%.2f %%] Ter=%u  DMer=%u Fer=%u FEmp=%u FFu=%u",
	       sumOk, sum17, sum17*100.0f/(sumOk+sum17),
	       dshotd2.dmap.nbTransferError,
	       dshotd2.dmap.nbDirectModeError,
	       dshotd2.dmap.nbFifoError,
	       dshotd2.dmap.nbFifoEmpty,
	       dshotd2.dmap.nbFifoFull
	       );
#endif
#if USE_DSHOT3   
    DebugTrace("D3 Ok:%lu Ko:%lu [%.2f %%] Ter=%u  DMer=%u Fer=%u FEmp=%u FFu=%u",
	       sumOk, sum17, sum17*100.0f/(sumOk+sum17),
	       dshotd3.dmap.nbTransferError,
	       dshotd3.dmap.nbDirectModeError,
	       dshotd3.dmap.nbFifoError,
	       dshotd3.dmap.nbFifoEmpty,
	       dshotd3.dmap.nbFifoFull
	       );
#endif 
#if USE_DSHOT4   
    DebugTrace("D4 Ok:%lu Ko:%lu [%.2f %%] Ter=%u  DMer=%u Fer=%u FEmp=%u FFu=%u",
	       sumOk, sum17, sum17*100.0f/(sumOk+sum17),
	       dshotd4.dmap.nbTransferError,
	       dshotd4.dmap.nbDirectModeError,
	       dshotd4.dmap.nbFifoError,
	       dshotd4.dmap.nbFifoEmpty,
	       dshotd4.dmap.nbFifoFull
	       );
#endif 
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
  
  uartStart(&UARTD3, &uartConfig);
  
  while (true) {
    stress[cnt % ARRAY_LEN(stress)] = stress[(cnt+200 ) % ARRAY_LEN(stress)] +1;
    //    chThdSleepMilliseconds(10);
    uartSendTimeout(&UARTD3, &nb, &stress, TIME_INFINITE);
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


