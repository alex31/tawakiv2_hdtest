#include <ch.h>
#include <hal.h>
#include "ramArch.h"
#include "stdutil.h"		
#include "ttyConsole.hpp"	
#include "leds.h"		
#include "internalSensors.hpp"
#include <array>
#include <algorithm>
#include "sdioTest.h"

/*
  TODO :
  
  * test sdLog
  * test ADC (tension batterie)
  * test/debug usbserie 
  
 */

/*
  ° cf ChibiOS_21.11_stable/os/common/startup/ARMCMx/compilers/GCC/ld/STM32H743xI.ld
  ° cf RM0433 page 104/3353 : memory map
 
  *  ADC3, I2C4, SPI6  : BDMA -> .RAM4 -> DMAMUX2
  *  SDMMC1, SPI2, I2C2, TIM1, TIM3, TIM4  : DMA -> .RAM0NC -> DMAMUX1 
  *  FDCAN : non connecté DMA

  * solution retenue : 
    + pour l'adc, buffer statique ou l'on specifie section .ram4
    + pour le BMP390 et le LIS3MDL en I²C, l'ICM40605, les librairies allouent un buffer
      aligné sur la pile et force la coherence de cache : il suffit que tous les appels
      se fassent depuis des thread ou la pile soit allouée sur dans la section ram4
      ->TODO : thread qui utilisent .ram4 et initialisation des periphériques 
           faite dans un thread plutot que dans main

    + pour le SDMMC : connecté à AXI (.ram0) 
      dans les fichiers linker script, nocache correspond à ram0nc pour les buffers
      internes du driver SDIO (hardwired)
    + driver sdLog modifié

 */


/*
  Câbler une LED sur la broche C0


  ° connecter B6 (uart1_tx) sur PROBE+SERIAL Rx AVEC UN JUMPER
  ° connecter B7 (uart1_rx) sur PROBE+SERIAL Tx AVEC UN JUMPER
  ° connecter C0 sur led0

 */

// see board.cfg for detail about LINE_CONTINUITY_GROUP
static const std::array sdLines = {LINE_CONTINUITY_GROUP};

__attribute__ ((section(FAST_SECTION), aligned(8)))
static THD_WORKING_AREA(waGpioPulse, 304);
static void gpioPulse (void *)
{
  chRegSetThreadName("gpioPulse");

  for (const auto &l : sdLines) {
    palSetLineMode(l, PAL_MODE_INPUT_PULLDOWN);
    palSetLine(l);
  }
  
  while (true) {
    const auto ts = chVTGetSystemTime();
    const auto limit = ts + TIME_MS2I(50); // a complete cycle every 50 milliseconds
    
    for (const auto &output : sdLines) {

      // all other line are input pulldown
      for (const auto &input : sdLines) {
	if (input != output) {
	  palSetLineMode(input, PAL_MODE_INPUT_PULLDOWN);
	}
      }

      // the one which output is output pushpull 
      palClearLine(output);
      palSetLineMode(output, PAL_MODE_OUTPUT_PUSHPULL);
      chThdSleepMicroseconds(100);		// pulse negatif de 100 µs
      palSetLine(output);
      chThdSleepMilliseconds(1);		// pulse de 1 ms
      palClearLine(output);
      chThdSleepMicroseconds(100);
    }

    chThdSleepUntil(limit); // frequence de 20hz (50ms)
   }
}

// static void testSpi6(void);
// static void testSpi2(void);

int main (void)
{
  halInit();
  mpuConfigureNonCachedRam();
  chSysInit();
  initHeap();		// initialisation du "tas" pour permettre l'allocation mémoire dynamique 

  ledRegisterLine(LINE_LED1, LED_BLINKSLOW);
  ledRegisterLine(LINE_LED2, LED_BLINKSLOW);
  ledRegisterLine(LINE_LED3, LED_BLINKSLOW);
  ledRegisterLine(LINE_LED4, LED_BLINKSLOW);
							    
  
  consoleInit();	// initialisation des objets liés au shell
  consoleLaunch();      // lancement du shell

  // DebugTrace("test SPI2");
  // chThdSleepMilliseconds(100);

  // DebugTrace("test SPI6");
  // chThdSleepMilliseconds(100);
  //testSpi6();
  
  ledSet(LINE_LED4, LED_BLINKFAST);
  chThdSleepSeconds(5);
  ledSet(LINE_LED3, LED_BLINKFAST);
  //  chThdCreateStatic(waGpioPulse, sizeof(waGpioPulse), NORMALPRIO+8, &gpioPulse, NULL); 
  launchSensorsThd();
  
  chThdSleep(TIME_INFINITE);
}


// static void testSpi6(void)
// {
//   static const SPIConfig spiCfg = {
//     .circular         = false,
//      .slave	    = false,
//      .data_cb          = nullptr,
//      .error_cb         = [](hal_spi_driver*) {chSysHalt("spi cb error");},
//     .ssline	    = LINE_SPI6_CS_INTERNAL,
//     .cfg1             = SPI_CFG1_MBR_DIV16 | SPI_CFG1_DSIZE_VALUE(7),
//     .cfg2             = SPI_CFG2_CPOL | SPI_CFG2_CPHA
//   };

//   spiStart(&SPID6, &spiCfg);
//   __attribute__ ((section(BDMA_SECTION "_init"), aligned(8)))
//     static uint8_t w_array[] = {255, 0};
//   __attribute__ ((section(BDMA_SECTION), aligned(8)))
//     static uint8_t r_array[2];
//   spiSelect(&SPID6);
//   spiSend(&SPID6, sizeof(w_array), w_array);
//   spiUnselect(&SPID6);
// }
