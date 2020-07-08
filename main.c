/*
  Nom(s), prénom(s) du ou des élèves : 
 */

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


/*

  ° connecter A08, A09
  ° connecter B10 (uart3_tx) sur ftdi rx :  shell
  ° connecter B11 (uart3_rx) sur ftdi tx :  shell
  ° connecter C00 sur led1 
*/

#define BENCH_TELEMETRY_BAUD		115200


static const DSHOTConfig dshotConfig3 = {
  .dma_stream = STM32_PWM3_UP_DMA_STREAM,
  .dma_channel = STM32_PWM3_UP_DMA_CHANNEL,
  .pwmp = &PWMD3,
  .tlm_sd = NULL
};


DSHOTDriver IN_DMA_SECTION_CLEAR(dshotd3);

static THD_WORKING_AREA(waBlinker, 512);
static noreturn void blinker (void *arg);
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

  halInit();
  chSysInit();
  initHeap();
  
  consoleInit();
  consoleLaunch();
  dshotStart(&dshotd3, &dshotConfig3);
  initSpy();
  
  chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, blinker, NULL);

  if ((((uint32_t)&dshotd3.dsdb % 16)) == 0) {
    DebugTrace("dshotd3.dsdb aligned 16");
  } else if ((((uint32_t)&dshotd3.dsdb % 8)) == 0) {
    DebugTrace("dshotd3.dsdb aligned 8");
  } else {
    DebugTrace("dshotd3.dsdb NOT aligned");
  }

  int throttle = 50;
  while (true) {
    
    for (size_t i=0; i<DSHOT_CHANNELS; i++) {
      dshotSetThrottle(&dshotd3, i, throttle+i);
    }
    dshotSendFrame(&dshotd3);

    // test dma buffer coherency
    /* for (int j=16; j<20; j++) */
    /*   for (int c=0; c<DSHOT_CHANNELS; c++) { */
    /* 	if (dshotd3.dsdb.widths32[j][c] != 0) { */
    /* 	  DebugTrace("w32[%d][%d] = %u", j, c, dshotd3.dsdb.widths32[j][c]); */
    /* 	} */
    /*   } */

    //    printSamples();
    //    DebugTrace ("%u", throttle);
    //    chprintf(chp, "\r\n\r\n\r\n\r\n\r\n");
    throttle += 20;
    if (throttle > 2000)
      throttle = 50;
    chThdSleepMilliseconds(2);
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
    DebugTrace("Ok:%lu Ko:%lu", sumOk, sum17);
  }
}


