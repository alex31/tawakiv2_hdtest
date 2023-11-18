#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <cstring>
#include "ttyConsole.hpp"
#include "ch.h"
#include "hal.h"
#include "microrl/microrlShell.h"
#include "stdutil.h"
#include "printf.h"
#include "etl/string.h"
#include <etl/vector.h>
#include <etl/string_utilities.h>

#if CONSOLE_DEV_USB
#include "usb_serial.h"
#endif

#ifdef CONSOLE_DEV_SD

/*===========================================================================*/
/* START OF EDITABLE SECTION                                           */
/*===========================================================================*/

// declaration des prototypes de fonction
// ces declarations sont necessaires pour remplir le tableau commands[] ci-dessous
using cmd_func_t =  void  (BaseSequentialStream *lchp, int argc,const char * const argv[]);
using shortStr_t = etl::string<6>;
static cmd_func_t cmd_mem, cmd_uid, cmd_restart, cmd_param, cmd_sd, cmd_sdc;
#if CH_DBG_STATISTICS
static cmd_func_t cmd_threads;
#endif



static const ShellCommand commands[] = {
  {"mem", cmd_mem},		// affiche la mémoire libre/occupée
#if  CH_DBG_STATISTICS
  {"threads", cmd_threads},	// affiche pour chaque thread le taux d'utilisation de la pile et du CPU
#endif
  {"sd", cmd_sd},		// affiche le numéro d'identification unique du MCU
  {"sdc", cmd_sdc},		// affiche le numéro d'identification unique du MCU
  {"uid", cmd_uid},		// affiche le numéro d'identification unique du MCU
  {"param", cmd_param},		// fonction à but pedagogique qui affiche les
				//   paramètres qui lui sont passés

  {"restart", cmd_restart},	// reboot MCU
  {NULL, NULL}			// marqueur de fin de tableau
};



/*
  definition de la fonction cmd_param asociée à la commande param (cf. commands[])
  cette fonction a but pédagogique affiche juste les paramètres fournis, et tente
  de convertir les paramètres en entier et en flottant, et affiche le resultat de
  cette conversion. 
  une fois le programme chargé dans la carte, essayer de rentrer 
  param toto 10 10.5 0x10
  dans le terminal d'eclipse pour voir le résultat 
 */
static void cmd_param(BaseSequentialStream *lchp, int argc,const char* const argv[])
{
  if (argc == 0) {  // si aucun paramètre n'a été passé à la commande param 
    chprintf(lchp, "pas de paramètre en entrée\r\n");
  } else { // sinon (un ou plusieurs pararamètres passés à la commande param 
    for (int argn=0; argn<argc; argn++) { // pour tous les paramètres
      chprintf(lchp, "le parametre %d/%d est %s\r\n", argn, argc-1, argv[argn]); // afficher

      // tentative pour voir si la chaine peut être convertie en nombre entier et en nombre flottant
      int entier = atoi (argv[argn]); // atoi converti si c'est possible une chaine en entier
      float flottant = atof (argv[argn]); // atof converti si c'est possible une chaine en flottant

      chprintf(lchp, "atoi(%s) = %d ;; atof(%s) = %.3f\r\n",
		argv[argn], entier, argv[argn], flottant);
    }
  }
}


using pGetFunc_t = uint32_t (*) (void);
using pSetFunc_t  = void (*) (uint32_t);


static void cmd_restart(BaseSequentialStream *lchp, int argc,const char* const argv[])
{
  (void) lchp;
  (void) argc;
  (void) argv;
  systemReset();
}

static void cmd_sd(BaseSequentialStream *lchp, int argc,const char * const argv[]) {
  shortStr_t pins, cmd;
  etl::vector<ioline_t, 6> sdLines;
  if (argc == 1) {
    sdLines = {LINE_SDMMC1_D0, LINE_SDMMC1_D1, LINE_SDMMC1_D2, LINE_SDMMC1_D3,
      LINE_SDMMC1_CK, LINE_SDMMC1_CMD};
    pins = "all";
    cmd = argv[0];
  } else if (argc == 2) {
    pins = argv[0];
    cmd = argv[1];
  } else {
    chprintf (lchp, "Usage: sd [all|d0..3|ck|cmd] hiz|l|h|u|d\r\n");
    return;
  }
  etl::to_lower_case(pins);
  etl::to_lower_case(cmd);
  
  if (pins == "all") {
    sdLines = {LINE_SDMMC1_D0, LINE_SDMMC1_D1, LINE_SDMMC1_D2, LINE_SDMMC1_D3,
      LINE_SDMMC1_CK, LINE_SDMMC1_CMD}; 
  } else if (pins == "d0") {
    sdLines = {LINE_SDMMC1_D0}; 
  } else if (pins == "d1") {
    sdLines = {LINE_SDMMC1_D1}; 
  } else if (pins == "d2") {
    sdLines = {LINE_SDMMC1_D2}; 
  } else if (pins == "d3") {
    sdLines = {LINE_SDMMC1_D3}; 
  } else if (pins == "ck") {
    sdLines = {LINE_SDMMC1_CK}; 
  } else if (pins == "cmd") {
    sdLines = {LINE_SDMMC1_CMD}; 
  } else {
    chprintf (lchp, "unknown pins %s Usage: sd [all|d0..3|ck|cmd] hiz|l|h|u|d\r\n",
	      pins.c_str());
    return;
  }
  
  for (const auto l : sdLines) {
    if (cmd == "hiz") {
      palSetLineMode(l, PAL_MODE_INPUT);
    } else if (cmd == "l") {
      palSetLineMode(l, PAL_MODE_OUTPUT_PUSHPULL);
      palClearLine(l);
    } else if (cmd == "h") {
      palSetLineMode(l, PAL_MODE_OUTPUT_PUSHPULL);
      palSetLine(l);
    } else if (cmd == "u") {
      palSetLineMode(l, PAL_MODE_INPUT_PULLUP);
    } else if (cmd == "d") {
      palSetLineMode(l, PAL_MODE_INPUT_PULLDOWN);
    } else {
      chprintf (lchp, "unknown command %s Usage: sd [all|d0..3|ck|cmd] h|l|h|u|d\r\n",
		cmd.c_str());
      return;
    }
  }
  
}



/*
  
 */


/*===========================================================================*/
/* START OF PRIVATE SECTION  : DO NOT CHANGE ANYTHING BELOW THIS LINE        */
/*===========================================================================*/

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/


#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2000)




#ifndef CONSOLE_DEV_USB
#define  CONSOLE_DEV_USB 0
#endif

#if CONSOLE_DEV_USB == 0
static const SerialConfig ftdiConfig =  {
  115200,
  0,
  USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  0
};
#endif


#define MAX_CPU_INFO_ENTRIES 20

typedef struct _ThreadCpuInfo {
  float    ticks[MAX_CPU_INFO_ENTRIES];
  float    cpu[MAX_CPU_INFO_ENTRIES];
  float    totalTicks;
  float    totalISRTicks;
  _ThreadCpuInfo () {
    for (auto i=0; i< MAX_CPU_INFO_ENTRIES; i++) {
      ticks[i] = 0.0f;
      cpu[i] = -1.0f;
    }
    totalTicks = 0.0f;
    totalISRTicks = 0.0f;
  }
} ThreadCpuInfo ;
  
#if CH_DBG_STATISTICS
static void stampThreadCpuInfo (ThreadCpuInfo *ti);
static float stampThreadGetCpuPercent (const ThreadCpuInfo *ti, const uint32_t idx);
static float stampISRGetCpuPercent (const ThreadCpuInfo *ti);
#endif

static void cmd_uid(BaseSequentialStream *lchp, int argc,const char* const argv[]) {
  (void)argv;
  if (argc > 0) {
     chprintf(lchp, "Usage: uid\r\n");
    return;
  }

  for (uint32_t i=0; i< UniqProcessorIdLen; i++)
    chprintf(lchp, "[%x] ", UniqProcessorId[i]);
  chprintf(lchp, "\r\n");
}



static void cmd_mem(BaseSequentialStream *lchp, int argc,const char* const argv[]) {
  (void)argv;
  if (argc > 0) {
    chprintf(lchp, "Usage: mem\r\n");
    return;
  }

  chprintf(lchp, "core free memory : %u bytes\r\n", chCoreGetStatusX());

#if CH_HEAP_SIZE != 0
  chprintf(lchp, "heap free memory : %u bytes\r\n", getHeapFree());
  
  void * ptr1 = malloc_m (100);
  void * ptr2 = malloc_m (100);
  
  chprintf(lchp, "(2x) malloc_m(1000) = %p ;; %p\r\n", ptr1, ptr2);
  chprintf(lchp, "heap free memory : %d bytes\r\n", getHeapFree());
  
  free_m (ptr1);
  free_m (ptr2);
#endif
  
}



#if  CH_DBG_STATISTICS
static void cmd_threads(BaseSequentialStream *lchp, int argc,const char * const argv[]) {
  static const char *states[] = {CH_STATE_NAMES};
  thread_t *tp = chRegFirstThread();
  (void)argv;
  (void)argc;
  float totalTicks=0;
  float idleTicks=0;

  static ThreadCpuInfo threadCpuInfo;
  
  stampThreadCpuInfo (&threadCpuInfo);
  
  chprintf (lchp, "    addr    stack  frestk prio refs  state        time \t percent        name\r\n");
  uint32_t idx=0;
  do {
    chprintf (lchp, "%.8lx %.8lx %6lu %4lu %4lu %9s %9lu   %.2f%%    \t%s\r\n",
	      (uint32_t)tp, (uint32_t)tp->ctx.sp,
	      get_stack_free(tp),
	      (uint32_t)tp->hdr.pqueue.prio, (uint32_t)(tp->refs - 1),
	      states[tp->state],
	      (uint32_t)RTC2MS(STM32_SYS_CK, tp->stats.cumulative),
	      stampThreadGetCpuPercent (&threadCpuInfo, idx),
	      chRegGetThreadNameX(tp));

    totalTicks+= (float)tp->stats.cumulative;
    if (strcmp(chRegGetThreadNameX(tp), "idle") == 0)
    idleTicks = (float)tp->stats.cumulative;
    tp = chRegNextThread((thread_t *)tp);
    idx++;
  } while (tp != NULL);

  const float idlePercent = (idleTicks*100.f)/totalTicks;
  const float cpuPercent = 100.f - idlePercent;
  chprintf (lchp, "Interrupt Service Routine \t\t     %9lu   %.2f%%    \tISR\r\n",
	    (uint32_t)RTC2MS(STM32_SYS_CK,threadCpuInfo.totalISRTicks),
	    stampISRGetCpuPercent(&threadCpuInfo));
  chprintf (lchp, "\r\ncpu load = %.2f%%\r\n", cpuPercent);
}
#endif

static const ShellConfig shell_cfg1 = {
#if CONSOLE_DEV_USB == 0
  (BaseSequentialStream *) &CONSOLE_DEV_SD,
#else
  (BaseSequentialStream *) &SDU1,
#endif
  commands
};

#define SDC_BURST_SIZE      16
/* Buffer for block read/write operations, note that extra bytes are
   allocated in order to support unaligned operations.*/
static uint8_t buf[MMCSD_BLOCK_SIZE * SDC_BURST_SIZE + 4]
__attribute__ ((section(SDMMC_SECTION), aligned(32)));;

/* Additional buffer for sdcErase() test */
static uint8_t buf2[MMCSD_BLOCK_SIZE * SDC_BURST_SIZE ]
__attribute__ ((section(SDMMC_SECTION), aligned(32)));;

static void cmd_sdc(BaseSequentialStream *lchp, int argc,const char * const argv[]) {
  static const char *mode[] = {"SDV11", "SDV20", "MMC", NULL};
  systime_t start, end;
  uint32_t n, startblk;

  if (argc != 1) {
    chprintf(lchp, "Usage: sdiotest read|write|erase|all\r\n");
    return;
  }

  /* Card presence check.*/
  if (!blkIsInserted(&SDCD1)) {
    chprintf(lchp, "Card not inserted, aborting.\r\n");
    return;
  }
  sdcStart(&SDCD1, NULL);
  /* Connection to the card.*/
  chprintf(lchp, "Connecting... ");
  if (sdcConnect(&SDCD1)) {
    chprintf(lchp, "failed\r\n");
    return;
  }

  chprintf(lchp, "OK\r\n\r\nCard Info\r\n");
  chprintf(lchp, "CSD      : %08lx %08lx %08lx %08lx \r\n",
           SDCD1.csd[3], SDCD1.csd[2], SDCD1.csd[1], SDCD1.csd[0]);
  chprintf(lchp, "CID      : %08lx %08lx %08lx %08lx \r\n",
           SDCD1.cid[3], SDCD1.cid[2], SDCD1.cid[1], SDCD1.cid[0]);
  chprintf(lchp, "Mode     : %s\r\n", mode[SDCD1.cardmode & 3U]);
  chprintf(lchp, "Capacity : %ldMB\r\n", SDCD1.capacity / 2048);

  /* The test is performed in the middle of the flash area.*/
  startblk = (SDCD1.capacity / MMCSD_BLOCK_SIZE) / 2;
  startblk = 0;
  
  if ((strcmp(argv[0], "read") == 0) ||
      (strcmp(argv[0], "all") == 0)) {

    /* Single block read performance, aligned.*/
    chprintf(lchp, "Single block aligned read performance:           ");
    start = chVTGetSystemTime();
    end = chTimeAddX(start, TIME_MS2I(1000));
    n = 0;
    do {
      if (blkRead(&SDCD1, startblk, buf, 1)) {
        chprintf(lchp, "failed\r\n");
        goto exittest;
      }
      n++;
    } while (chVTIsSystemTimeWithin(start, end));
    chprintf(lchp, "%ld blocks/S, %ld bytes/S\r\n", n, n * MMCSD_BLOCK_SIZE);

    /* Multiple sequential blocks read performance, aligned.*/
    chprintf(lchp, "16 sequential blocks aligned read performance:   ");
    start = chVTGetSystemTime();
    end = chTimeAddX(start, TIME_MS2I(1000));
    n = 0;
    do {
      if (blkRead(&SDCD1, startblk, buf, SDC_BURST_SIZE)) {
        chprintf(lchp, "failed\r\n");
        goto exittest;
      }
      n += SDC_BURST_SIZE;
    } while (chVTIsSystemTimeWithin(start, end));
    chprintf(lchp, "%ld blocks/S, %ld bytes/S\r\n", n, n * MMCSD_BLOCK_SIZE);

#if STM32_SDC_SDMMC_UNALIGNED_SUPPORT
    /* Single block read performance, unaligned.*/
    chprintf(lchp, "Single block unaligned read performance:         ");
    start = chVTGetSystemTime();
    end = chTimeAddX(start, TIME_MS2I(1000));
    n = 0;
    do {
      if (blkRead(&SDCD1, startblk, buf + 1, 1)) {
        chprintf(lchp, "failed\r\n");
        goto exittest;
      }
      n++;
    } while (chVTIsSystemTimeWithin(start, end));
    chprintf(lchp, "%ld blocks/S, %ld bytes/S\r\n", n, n * MMCSD_BLOCK_SIZE);

    /* Multiple sequential blocks read performance, unaligned.*/
    chprintf(lchp, "16 sequential blocks unaligned read performance: ");
    start = chVTGetSystemTime();
    end = chTimeAddX(start, TIME_MS2I(1000));
    n = 0;
    do {
      if (blkRead(&SDCD1, startblk, buf + 1, SDC_BURST_SIZE)) {
        chprintf(lchp, "failed\r\n");
        goto exittest;
      }
      n += SDC_BURST_SIZE;
    } while (chVTIsSystemTimeWithin(start, end));
    chprintf(lchp, "%ld blocks/S, %ld bytes/S\r\n", n, n * MMCSD_BLOCK_SIZE);
#endif /* STM32_SDC_SDIO_UNALIGNED_SUPPORT */
  }

#define BLK_SIZE_TEST 1U
  if ((strcmp(argv[0], "write") == 0) ||
      (strcmp(argv[0], "all") == 0)) {
    unsigned i;

    memset(buf, 0xAA, MMCSD_BLOCK_SIZE * BLK_SIZE_TEST);
    memset(buf2, 0x0, MMCSD_BLOCK_SIZE * BLK_SIZE_TEST);
    for (startblk = 1000; startblk<1e6; startblk +=BLK_SIZE_TEST) {
      chprintf(lchp, "Writing %ld...", startblk);
      if(sdcWrite(&SDCD1, startblk, buf, BLK_SIZE_TEST)) {
	chprintf(lchp, "failed\r\n");
	goto exittest;
      }
      chprintf(lchp, "Ok\r\nReRead...");
      if(sdcRead(&SDCD1, startblk, buf2, BLK_SIZE_TEST)) {
	chprintf(lchp, "failed\r\n");
	goto exittest;
      }
      chprintf(lchp, "OK\r\n");
      if (memcmp(buf, buf2,  MMCSD_BLOCK_SIZE * BLK_SIZE_TEST) == 0) {
	chprintf(lchp, "compare...Ok\r\n");
      } else {
	chprintf(lchp, "compare...Failed\r\n");
	for (uint32_t j=0; j<MMCSD_BLOCK_SIZE * BLK_SIZE_TEST; j++) {
	  if (buf[j] != buf2[j])
	    chprintf(lchp, "index %lu %u vs %u\r\n", j, buf[j], buf2[j]);
	}
	goto exittest;
      }


    }

    memset(buf, 0x55, MMCSD_BLOCK_SIZE * 2);
    chprintf(lchp, "Reading...");
    if (blkRead(&SDCD1, startblk, buf, 1)) {
      chprintf(lchp, "failed\r\n");
      goto exittest;
    }
    chprintf(lchp, "OK\r\n");

    for (i = 0; i < MMCSD_BLOCK_SIZE; i++)
      buf[i] = i + 8;
    chprintf(lchp, "Writing...");
    if(sdcWrite(&SDCD1, startblk, buf, 2)) {
      chprintf(lchp, "failed\r\n");
      goto exittest;
    }
    chprintf(lchp, "OK\r\n");

    memset(buf, 0, MMCSD_BLOCK_SIZE * 2);
    chprintf(lchp, "Reading...");
    if (blkRead(&SDCD1, startblk, buf, 1)) {
      chprintf(lchp, "failed\r\n");
      goto exittest;
    }
    chprintf(lchp, "OK\r\n");
  }

  if ((strcmp(argv[0], "erase") == 0) ||
      (strcmp(argv[0], "all") == 0)) {
    /**
     * Test sdcErase()
     * Strategy:
     *   1. Fill two blocks with non-constant data
     *   2. Write two blocks starting at startblk
     *   3. Erase the second of the two blocks
     *      3.1. First block should be equal to the data written
     *      3.2. Second block should NOT be equal too the data written (i.e. erased).
     *   4. Erase both first and second block
     *      4.1 Both blocks should not be equal to the data initially written
     * Precondition: SDC_BURST_SIZE >= 2
     */
    memset(buf, 0, MMCSD_BLOCK_SIZE * 2);
    memset(buf2, 0, MMCSD_BLOCK_SIZE * 2);
    /* 1. */
    unsigned int i = 0;
    for (; i < MMCSD_BLOCK_SIZE * 2; ++i) {
      buf[i] = (i + 7) % 'T'; //Ensure block 1/2 are not equal
    }
    /* 2. */
    if(sdcWrite(&SDCD1, startblk, buf, 2)) {
      chprintf(lchp, "sdcErase() test write failed\r\n");
      goto exittest;
    }
    /* 3. (erase) */
    if(sdcErase(&SDCD1, startblk + 1, startblk + 2)) {
      chprintf(lchp, "sdcErase() failed\r\n");
      goto exittest;
    }
    sdcflags_t errflags = sdcGetAndClearErrors(&SDCD1);
    if(errflags) {
      chprintf(lchp, "sdcErase() yielded error flags: %ld\r\n", errflags);
      goto exittest;
    }
    if(sdcRead(&SDCD1, startblk, buf2, 2)) {
      chprintf(lchp, "single-block sdcErase() failed\r\n");
      goto exittest;
    }
    /* 3.1. */
    if(memcmp(buf, buf2, MMCSD_BLOCK_SIZE) != 0) {
      chprintf(lchp, "sdcErase() non-erased block compare failed\r\n");
      goto exittest;
    }
    /* 3.2. */
    if(memcmp(buf + MMCSD_BLOCK_SIZE,
              buf2 + MMCSD_BLOCK_SIZE, MMCSD_BLOCK_SIZE) == 0) {
      chprintf(lchp, "sdcErase() erased block compare failed\r\n");
      goto exittest;
    }
    /* 4. */
    if(sdcErase(&SDCD1, startblk, startblk + 2)) {
      chprintf(lchp, "multi-block sdcErase() failed\r\n");
      goto exittest;
    }
    if(sdcRead(&SDCD1, startblk, buf2, 2)) {
      chprintf(lchp, "single-block sdcErase() failed\r\n");
      goto exittest;
    }
    /* 4.1 */
    if(memcmp(buf, buf2, MMCSD_BLOCK_SIZE) == 0) {
      chprintf(lchp, "multi-block sdcErase() erased block compare failed\r\n");
      goto exittest;
    }
    if(memcmp(buf + MMCSD_BLOCK_SIZE,
              buf2 + MMCSD_BLOCK_SIZE, MMCSD_BLOCK_SIZE) == 0) {
      chprintf(lchp, "multi-block sdcErase() erased block compare failed\r\n");
      goto exittest;
    }
    /* END of sdcErase() test */
  }
  
  /* Card disconnect and command end.*/
exittest:
  sdcDisconnect(&SDCD1);
}


void consoleInit (void)
{
  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * USBD1 : FS, USBD2 : HS
   */

#if CONSOLE_DEV_USB != 0
  usbSerialInit(&SDU1, &USBD1); 
  chp = (BaseSequentialStream *) &SDU1;
#else
  sdStart(&CONSOLE_DEV_SD, &ftdiConfig);
  chp = (BaseSequentialStream *) &CONSOLE_DEV_SD;
#endif
  /*
   * Shell manager initialization.
   */
  shellInit();
}


void consoleLaunch (void)
{
  thread_t *shelltp = NULL;

 
#if CONSOLE_DEV_USB != 0
  if (!shelltp) {
    while (usbGetDriver()->state != USB_ACTIVE) {
      chThdSleepMilliseconds(10);
    }
    
    // activate driver, giovani workaround
    chnGetTimeout(&SDU1, TIME_IMMEDIATE);
    while (!isUsbConnected()) {
      chThdSleepMilliseconds(10);
    }
    shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO - 1);
    //    palSetLine(LINE_USB_LED);
  } else if (shelltp && (chThdTerminatedX(shelltp))) {
    chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
    shelltp = NULL;           /* Triggers spawning of a new shell.        */
  }

#else // CONSOLE_DEV_USB == 0

   if (!shelltp) {
     shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO - 1);
   } else if (chThdTerminatedX(shelltp)) {
     chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
     shelltp = NULL;           /* Triggers spawning of a new shell.        */
   }
   chThdSleepMilliseconds(100);
   
#endif //CONSOLE_DEV_USB

}



#if CH_DBG_STATISTICS
static void stampThreadCpuInfo (ThreadCpuInfo *ti)
{
  const thread_t *tp =  chRegFirstThread();
  uint32_t idx=0;
  
  ti->totalTicks =0;
  do {
    ti->ticks[idx] = (float) tp->stats.cumulative;
    ti->totalTicks += ti->ticks[idx];
    tp = chRegNextThread ((thread_t *)tp);
    idx++;
  } while ((tp != NULL) && (idx < MAX_CPU_INFO_ENTRIES));
  ti->totalISRTicks = currcore->kernel_stats.m_crit_isr.cumulative;
  ti->totalTicks += ti->totalISRTicks;
  tp =  chRegFirstThread();
  idx=0;
  do {
    ti->cpu[idx] =  (ti->ticks[idx]*100.f) / ti->totalTicks;
    tp = chRegNextThread ((thread_t *)tp);
    idx++;
  } while ((tp != NULL) && (idx < MAX_CPU_INFO_ENTRIES));
}

static float stampThreadGetCpuPercent (const ThreadCpuInfo *ti, const uint32_t idx)
{
  if (idx >= MAX_CPU_INFO_ENTRIES) 
    return -1.f;

  return ti->cpu[idx];
}

static float stampISRGetCpuPercent (const ThreadCpuInfo *ti)
{
  return ti->totalISRTicks * 100.0f / ti->totalTicks;
}
#endif // CH_DBG_STATISTICS
#endif // CONSOLE_DEV_SD
