#include <ch.h>
#include <hal.h>
#include <stdnoreturn.h>
#include <stdlib.h>
#include "stdutil.h"
#include "microrl/microrlShell.h"


/*
  ° connecter C00 sur led1 
*/

static volatile systime_t waitled = TIME_MS2I(500);

static THD_WORKING_AREA(waBlinker, 512);
static noreturn void blinker (void *arg);
static shellcmd_f cmd_mem, cmd_period,  cmd_period2;

static const SerialConfig ftdiConfig =  {
  115200,
  0,
  USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  0
};

static const ShellCommand commands[] = {
  {"mem", cmd_mem},
  {NULL, NULL}
};

static const ShellConfig shell_cfg = {
  (BaseSequentialStream *) &CONSOLE_DEV_SD,
  commands
};

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
  
  sdStart(&CONSOLE_DEV_SD, &ftdiConfig);
  shellInit();
  thread_t * shelltp = shellCreateFromHeap(&shell_cfg, 2048U, NORMALPRIO);
  chprintf((BaseSequentialStream *) &CONSOLE_DEV_SD, "shell launched ptr = %p\r\n",
	   shelltp);
  // dynamic entry
  shellAddEntry((ShellCommand){"period", &cmd_period});
  
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



static void cmd_period(BaseSequentialStream *lchp, int argc,const char * const argv[]) {
  (void)argv;
  if (argc < 1) {
    chprintf (lchp, "period arg (milliseconds)\r\n");
  } else {
    const systime_t tm = TIME_MS2I(atoi(argv[0]));
    if (tm != 0) {
    waitled = tm;
    shellAddEntry((ShellCommand){"period2", &cmd_period2});
    } else {
      DebugTrace("null wait period forbidden");
    }
  }
}

static void cmd_period2(BaseSequentialStream *lchp, int argc,const char * const argv[]) {
  (void)argv;
  if (argc < 1) {
    chprintf (lchp, "period arg (milliseconds)\r\n");
  } else {
    const systime_t tm = TIME_MS2I(atoi(argv[0]));
    if (tm != 0) {
    waitled = tm;
    shellAddEntry((ShellCommand){"period2", NULL});
    } else {
      DebugTrace("null wait period forbidden");
    }
  }
}


static void cmd_mem(BaseSequentialStream *lchp, int argc,const char * const argv[]) {
  (void)argv;
  if (argc > 0) {
    chprintf (lchp, "Usage: mem\r\n");
    return;
  }

  chprintf (lchp, "core free memory : %u bytes\r\n", chCoreGetStatusX());
  chprintf (lchp, "heap free memory : %u bytes\r\n", getHeapFree());

  void * ptr1 = malloc_m (100);
  void * ptr2 = malloc_m (100);

  chprintf (lchp, "(2x) malloc_m(1000) = %p ;; %p\r\n", ptr1, ptr2);
  chprintf (lchp, "heap free memory : %d bytes\r\n", getHeapFree());

  free_m (ptr1);
  free_m (ptr2);
}
