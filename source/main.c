#include <ch.h>
#include <hal.h>
#include <stdnoreturn.h>
#include <stdlib.h>
#include "stdutil.h"
#include "ttyConsole.h"
#include "microrl/microrlShell.h"


/*
  ° connecter C00 sur led1 
*/

static volatile systime_t waitled = TIME_MS2I(500);

static THD_WORKING_AREA(waBlinker, 512);
static noreturn void blinker (void *arg);

static void cmd_period1(BaseSequentialStream *lchp, int argc,const char * const argv[]);
static void cmd_period2(BaseSequentialStream *lchp, int argc,const char * const argv[]);

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

  chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, &blinker, NULL);
  
  consoleInit();
  consoleLaunch();
  shellAddEntry((ShellCommand){"period", &cmd_period1});
  shellAddEntry((ShellCommand){"period", &cmd_period2});
  
  chThdSleep(TIME_INFINITE);
}


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
    chThdSleep(waitled);
  }
}



static void cmd_period1(BaseSequentialStream *lchp, int argc,const char * const argv[]) {
  (void)argv;
  if (argc < 1) {
    chprintf (lchp, "period 1 arg (milliseconds)\r\n");
  } else {
    const systime_t tm = TIME_MS2I(atoi(argv[0]));
    if (tm != 0) {
    waitled = tm;
    } else {
      DebugTrace("null wait period forbidden");
    }
  }
}

static void cmd_period2(BaseSequentialStream *lchp, int argc,const char * const argv[]) {
  (void)argv;
  if (argc < 1) {
    chprintf (lchp, "period 2 arg (milliseconds)\r\n");
  } else {
    const systime_t tm = TIME_MS2I(atoi(argv[0]));
    if (tm != 0) {
    waitled = tm;
    } else {
      DebugTrace("null wait period forbidden");
    }
  }
}
